"""Tests for ``gate_leg`` -- the state machine every phase flies through.

Before these existed, 344 lines containing the approach state machine, the
three-rung recovery ladder and the open-loop crossing had been exercised exactly
once, by the happy path of a single offline run.  Every branch below is a
decision that can end with an aircraft in a net.

Each test names the state or rung it covers.  Regression tests carry the bug's
severity and date in the docstring so a future reader knows the assertion is
load-bearing rather than incidental.
"""

import math
import unittest
from unittest.mock import patch

from support import (  # noqa: E402
    FakeClock,
    FakeNav,
    RecordingLog,
    ScriptedMission,
    detection,
)

from navigation.missions import gate_leg  # noqa: E402
from navigation.missions.contracts import (  # noqa: E402
    AltitudeEnvelope,
    GateFix,
    GateLegConfig,
    GateResult,
)


def leg_config(**overrides) -> GateLegConfig:
    fields = dict(
        commit_distance_m=1.0,
        step_size_m=0.25,
        approach_speed_m_s=0.35,
        observation_duration_s=0.5,
        max_approach_attempts=14,
        approach_timeout_s=90.0,
        commit_confirm_frames=1,
        max_align_attempts=4,
        max_observe_retries=3,
        max_scan_sweeps=2,
        max_backoffs=2,
        altitude=AltitudeEnvelope(),
    )
    fields.update(overrides)
    return GateLegConfig(**fields)


def run_leg(nav, mission, cfg, clock, log=None):
    """Run one leg with the module's clock replaced by a virtual one."""
    log = log or RecordingLog()
    with patch.object(gate_leg, "time", clock):
        outcome = gate_leg.approach_and_cross_one_gate(
            nav, mission, cfg, gate_index=0, log=log
        )
    return outcome, log


def gate_fix(n, e, d=-1.5, *, normal=(1.0, 0.0)) -> GateFix:
    """A minimal localized gate, for the crossed-gate guard tests."""
    return GateFix(
        n=n, e=e, d=d, normal_n=normal[0], normal_e=normal[1],
        yaw_rad=math.atan2(normal[1], normal[0]), range_m=1.0,
        lateral_body_m=0.0, vertical_body_m=0.0, lateral_axis_m=0.0,
        cone_angle_deg=0.0, normal_flipped=False, altitude_clamped=False,
        low_confidence=False, reason="",
    )


# A gate close enough and centred enough that the commit gate passes.
COMMITTABLE = detection(forward=0.9, dist=0.9)
# In range (hypot(0.7, 0.4) = 0.81 m <= 1.0 m) but 0.40 m off the boresight,
# against a 0.20 m lateral tolerance. This is the geometry that range-only
# commit logic gets wrong: the slant range looks fine while the drone is crabbed
# far enough off axis to clip a gate leg on an open-loop pass.
IN_RANGE_UNALIGNED = detection(forward=0.7, right=0.4, dist=0.806)
# Far away and centred: a normal approach step.
FAR = detection(forward=3.0, dist=3.0)


class CommitAndCrossTests(unittest.TestCase):
    """COMMIT -> EXIT: the part where the drone stops trusting vision."""

    def setUp(self):
        self.clock = FakeClock()
        self.nav = FakeNav(self.clock)
        self.nav.integrate_velocity = True

    def test_a_centred_in_range_gate_is_committed_and_crossed(self):
        outcome, log = run_leg(
            self.nav, ScriptedMission([COMMITTABLE]), leg_config(), self.clock
        )

        self.assertIs(outcome.result, GateResult.CROSSED)
        self.assertTrue(outcome.ok)
        self.assertTrue(log.contains("COMMITTING"))
        self.assertTrue(log.contains("CROSSED"))

    def test_the_commit_gate_must_hold_for_the_configured_consecutive_frames(self):
        # One good frame is not a commit when three are required. The gate is
        # the single most important decision in the mission, and a one-frame
        # PnP glitch at 1 m is exactly the input it has to survive.
        mission = ScriptedMission([COMMITTABLE, COMMITTABLE, COMMITTABLE])
        outcome, log = run_leg(
            self.nav, mission, leg_config(commit_confirm_frames=3), self.clock
        )

        self.assertIs(outcome.result, GateResult.CROSSED)
        self.assertTrue(log.contains("held 1/3"))
        self.assertTrue(log.contains("held 2/3"))
        self.assertTrue(log.contains("held 3/3"))

    def test_a_failed_frame_resets_the_consecutive_commit_counter(self):
        # good, good, BAD, good, good, good -> the run of three only completes
        # after the bad frame, never across it.
        mission = ScriptedMission(
            [COMMITTABLE, COMMITTABLE, IN_RANGE_UNALIGNED,
             COMMITTABLE, COMMITTABLE, COMMITTABLE]
        )
        outcome, log = run_leg(
            self.nav, mission, leg_config(commit_confirm_frames=3), self.clock
        )

        self.assertIs(outcome.result, GateResult.CROSSED)
        # "held 1/3" appears twice: once before the reset and once after it.
        self.assertEqual(log.text.count("held 1/3"), 2)

    def test_crossing_is_judged_on_the_gate_plane_not_on_arrival(self):
        # A pass-through cannot satisfy a position tolerance while still moving,
        # so the completion test is a signed projection onto the gate normal.
        # The drone finishes PAST the gate, not AT the pass-through point.
        cfg = leg_config()
        outcome, _ = run_leg(self.nav, ScriptedMission([COMMITTABLE]), cfg, self.clock)

        self.assertIs(outcome.result, GateResult.CROSSED)
        along = outcome.exit_state.n - outcome.fix.n
        self.assertGreaterEqual(along, cfg.exit_clearance_m)
        self.assertLess(along, cfg.pass_distance_m)

    def test_a_crossing_that_never_clears_the_plane_times_out(self):
        # The vehicle is told to fly but never moves (no integration), so the
        # crossing budget expires. This must be reported, not swallowed: the old
        # course mission announced success from an ignored False return.
        self.nav.integrate_velocity = False
        outcome, log = run_leg(
            self.nav, ScriptedMission([COMMITTABLE]), leg_config(), self.clock
        )

        self.assertIs(outcome.result, GateResult.TIMED_OUT)
        self.assertTrue(log.contains("Crossing timed out"))

    def test_the_crossing_leg_leaves_a_hold_behind(self):
        run_leg(self.nav, ScriptedMission([COMMITTABLE]), leg_config(), self.clock)
        self.assertTrue(any("after gate 0" in label for label in self.nav.holds))


class ApproachAndAlignTests(unittest.TestCase):
    """APPROACH and ALIGN: the two ways a leg makes progress."""

    def setUp(self):
        self.clock = FakeClock()
        self.nav = FakeNav(self.clock)

    def test_a_distant_gate_produces_a_step_limited_standoff_move(self):
        # 3 m out with a 0.25 m step: the commanded move is a step, not a lunge.
        mission = ScriptedMission([FAR, FAR, FAR, COMMITTABLE])
        self.nav.integrate_velocity = True
        run_leg(self.nav, mission, leg_config(), self.clock)

        self.assertTrue(self.nav.moves)
        label, target, speed, tolerance = self.nav.moves[0]
        self.assertIn("standoff", label)
        self.assertAlmostEqual(math.hypot(target.n, target.e), 0.25, places=6)
        self.assertAlmostEqual(speed, 0.35, places=9)

    def test_approach_moves_use_the_configured_arrival_tolerance(self):
        """REGRESSION -- MED, 2026-08-10: tolerance/timeout/deadline disagreed.

        A 0.10 m arrival tolerance sits inside the noise floor of a flow-aided
        position estimate, so a settled vehicle failed the arrival test and
        spent its whole move budget on every step. The leg now passes its own
        configured tolerance down rather than inheriting a hard-coded one.
        """
        cfg = leg_config(arrival_tolerance_m=0.18)
        self.nav.integrate_velocity = True
        run_leg(self.nav, ScriptedMission([FAR, COMMITTABLE]), cfg, self.clock)

        self.assertTrue(self.nav.moves)
        for label, _target, _speed, tolerance in self.nav.moves:
            self.assertAlmostEqual(tolerance, 0.18, places=9, msg=label)

    def test_a_gate_in_range_but_off_axis_produces_an_align_move(self):
        mission = ScriptedMission([IN_RANGE_UNALIGNED, COMMITTABLE])
        self.nav.integrate_velocity = True
        run_leg(self.nav, mission, leg_config(), self.clock)

        self.assertTrue(any("align" in label for label, *_ in self.nav.moves))

    def test_align_attempts_are_bounded(self):
        # In range and never alignable: the leg must give up with a named
        # outcome rather than nudging sideways until the battery runs out.
        outcome, _ = run_leg(
            self.nav,
            ScriptedMission([IN_RANGE_UNALIGNED]),
            leg_config(max_align_attempts=2),
            self.clock,
        )

        self.assertIs(outcome.result, GateResult.NO_COMMIT)
        self.assertIn("never aligned", outcome.reason)

    def test_approach_attempts_are_bounded(self):
        outcome, _ = run_leg(
            self.nav,
            ScriptedMission([FAR]),
            leg_config(max_approach_attempts=3),
            self.clock,
        )

        self.assertIs(outcome.result, GateResult.NO_COMMIT)
        self.assertIn("all 3 approach attempts", outcome.reason)
        self.assertEqual(outcome.attempts, 3)

    def test_a_move_that_does_not_complete_re_observes_rather_than_aborting(self):
        # A timed-out step is information, not a failure: the gate is measured
        # again from wherever the vehicle actually got to.
        self.nav.move_results = [False, False]
        self.nav.integrate_velocity = True
        mission = ScriptedMission([FAR, FAR, COMMITTABLE])
        outcome, log = run_leg(self.nav, mission, leg_config(), self.clock)

        self.assertIs(outcome.result, GateResult.CROSSED)
        self.assertTrue(log.contains("did not complete"))


class RecoveryLadderTests(unittest.TestCase):
    """SEARCH: the three rungs, in order, each bounded."""

    def setUp(self):
        self.clock = FakeClock()
        self.nav = FakeNav(self.clock)

    def test_rung_one_re_observes_without_escalating(self):
        # Three misses inside max_observe_retries=3 must not move the aircraft.
        mission = ScriptedMission([None, None, None, COMMITTABLE])
        self.nav.integrate_velocity = True
        outcome, log = run_leg(
            self.nav, mission, leg_config(max_observe_retries=3), self.clock
        )

        self.assertIs(outcome.result, GateResult.CROSSED)
        self.assertEqual(self.nav.slews, [])
        self.assertTrue(log.contains("not seen (miss 3)"))

    def test_rung_two_is_a_bounded_yaw_sweep(self):
        mission = ScriptedMission([None, None, None, None, COMMITTABLE])
        self.nav.integrate_velocity = True
        outcome, log = run_leg(
            self.nav, mission, leg_config(max_observe_retries=3), self.clock
        )

        self.assertIs(outcome.result, GateResult.CROSSED)
        self.assertTrue(log.contains("Recovery rung 2: yaw sweep 1/2"))
        # The sweep OBSERVES at each heading and stops at the one where it sees
        # the gate. Here the gate is visible from the very first heading, so a
        # single slew is issued and the aircraft stays pointed at it.
        self.assertEqual(len(self.nav.slews), 1)
        self.assertTrue(log.contains("Gate re-acquired"))
        self.assertTrue(log.contains("staying here"))

    def test_a_sweep_that_finds_nothing_returns_to_the_search_heading(self):
        # +45, -45, then back to where it started -- three commanded headings.
        mission = ScriptedMission([None])
        outcome, log = run_leg(
            self.nav,
            mission,
            leg_config(max_observe_retries=1, max_scan_sweeps=1, max_backoffs=0),
            self.clock,
        )

        self.assertIs(outcome.result, GateResult.NOT_FOUND)
        self.assertEqual(len(self.nav.slews), 3)
        self.assertAlmostEqual(self.nav.slews[-1][1], 0.0, places=6)
        self.assertTrue(log.contains("returning to the search heading"))

    def test_the_sweep_actually_looks_rather_than_just_turning(self):
        """REGRESSION -- multi-gate prep, 2026-08-13.

        `_yaw_sweep` used to slew to +45, then -45, then back to 0 and never
        call observe_gate once. It rotated, came back, and the caller
        re-observed at exactly the heading that had already failed. Recovery
        rung 2 could only ever succeed by coincidence -- it was incapable of
        finding a gate that was not already in front of the aircraft. Harmless
        on a straight-line course, useless on any course with a turn in it.
        """
        mission = ScriptedMission([None, None, COMMITTABLE])
        before = mission.observations
        run_leg(
            self.nav,
            mission,
            leg_config(max_observe_retries=1, max_scan_sweeps=1),
            self.clock,
        )
        # Observations happened DURING the sweep, not only around it.
        self.assertGreater(mission.observations, before + 2)

    def test_the_second_sweep_looks_twice_as_wide(self):
        # A zig-zag puts the next gate inside the first band; a box course turns
        # about 90 degrees, which only the widened second band reaches.
        mission = ScriptedMission([None])
        run_leg(
            self.nav,
            mission,
            leg_config(
                max_observe_retries=1, max_scan_sweeps=2, max_backoffs=0,
                scan_half_angle_deg=45.0,
            ),
            self.clock,
        )

        headings = [abs(math.degrees(heading)) for _label, heading, _rate in self.nav.slews]
        self.assertTrue(any(abs(h - 45.0) < 1e-6 for h in headings), headings)
        self.assertTrue(any(abs(h - 90.0) < 1e-6 for h in headings), headings)

    def test_the_yaw_sweep_is_rate_limited_by_this_code(self):
        """REGRESSION -- PROJECT_STATE gap 3: yaw-rate limiting was absent.

        The sweep used to issue each +/-45 degree step as one absolute-yaw
        move, i.e. "turn there as fast as PX4 will let you", relying on
        MPC_YAWRAUTO_MAX still being at its firmware default. Fast yaw injects
        rotation-induced optical flow, which degrades the position estimate --
        during a recovery, which is when the estimate matters most.

        Note the offline model rate-limits yaw internally, which is exactly why
        a green offline run never surfaced this.
        """
        mission = ScriptedMission([None, None, None, None, COMMITTABLE])
        self.nav.integrate_velocity = True
        run_leg(
            self.nav,
            mission,
            leg_config(max_observe_retries=3, max_yaw_rate_deg_s=20.0),
            self.clock,
        )

        self.assertTrue(self.nav.slews)
        for _label, _heading, rate in self.nav.slews:
            self.assertAlmostEqual(rate, 20.0, places=9)
        # And nothing reached the aircraft as a bare yaw step.
        self.assertFalse(any("scan" in label for label, *_ in self.nav.moves))

    def test_rung_three_backs_off_only_when_a_fix_exists(self):
        # Rung 3 widens the field of view by retreating along the gate normal,
        # which is only meaningful if a gate was ever localized.
        mission = ScriptedMission([FAR] + [None] * 30)
        outcome, log = run_leg(
            self.nav,
            mission,
            leg_config(max_observe_retries=1, max_scan_sweeps=1, max_backoffs=1),
            self.clock,
        )

        self.assertTrue(log.contains("Recovery rung 3: backing off"))
        self.assertTrue(any("back-off" in label for label, *_ in self.nav.moves))
        self.assertIs(outcome.result, GateResult.LOST)

    def test_rung_three_backs_off_even_when_the_gate_was_never_localized(self):
        """REGRESSION -- multi-gate prep, 2026-08-13.

        Rung 3 was guarded on `last_fix is not None`, so a gate that had never
        been localized could not be backed away from. That is exactly backwards:
        the most likely reason a gate is invisible is that the aircraft is too
        close for it to fit in frame -- the vision process refuses to solve a
        pose for any box touching the image edge -- and on a multi-gate course
        you can finish a crossing sitting right on top of the next gate.
        Retreating is the only useful thing left to try, and it was the one
        thing forbidden.
        """
        outcome, log = run_leg(
            self.nav,
            ScriptedMission([None]),
            leg_config(max_observe_retries=1, max_scan_sweeps=1, max_backoffs=1),
            self.clock,
        )

        self.assertTrue(log.contains("gate never localized, backing straight off"))
        self.assertTrue(any("back-off" in label for label, *_ in self.nav.moves))
        self.assertIs(outcome.result, GateResult.NOT_FOUND)

    def test_the_blind_backoff_retreats_along_the_drones_own_heading(self):
        self.nav.set_state(n=0.0, e=0.0, d=-1.5, yaw_rad=0.0)   # pointing north
        run_leg(
            self.nav,
            ScriptedMission([None]),
            leg_config(
                max_observe_retries=1, max_scan_sweeps=0, max_backoffs=1,
                backoff_distance_m=1.0,
            ),
            self.clock,
        )

        backoffs = [t for label, t, *_ in self.nav.moves if "back-off" in label]
        self.assertTrue(backoffs)
        # Straight backwards: one metre south of where it was, same altitude.
        self.assertAlmostEqual(backoffs[0].n, -1.0, places=6)
        self.assertAlmostEqual(backoffs[0].e, 0.0, places=6)

    def test_a_gate_never_seen_at_all_is_not_found_not_lost(self):
        # NOT_FOUND and LOST are different diagnoses: one says the search was
        # wrong, the other says the approach was.
        outcome, log = run_leg(
            self.nav,
            ScriptedMission([None]),
            leg_config(max_observe_retries=1, max_scan_sweeps=1, max_backoffs=1),
            self.clock,
        )

        self.assertIs(outcome.result, GateResult.NOT_FOUND)
        self.assertIn("recovery ladder exhausted", outcome.reason)
        # No fix ever existed, so rung 3 must not have run.
        self.assertFalse(log.contains("backing off"))

    def test_the_ladder_never_runs_forever(self):
        # Every rung is bounded, so an entirely blind mission still terminates.
        outcome, _ = run_leg(
            self.nav,
            ScriptedMission([None]),
            leg_config(max_observe_retries=2, max_scan_sweeps=2, max_backoffs=2),
            self.clock,
        )
        self.assertIn(outcome.result, (GateResult.NOT_FOUND, GateResult.LOST))
        self.assertLessEqual(len(self.nav.slews), 3 * 2)


class CrossedGateGuardTests(unittest.TestCase):
    """Multi-gate: a gate must not be counted, or flown, twice."""

    def setUp(self):
        self.clock = FakeClock()
        self.nav = FakeNav(self.clock)
        self.nav.integrate_velocity = True

    def _run(self, avoid_gate, script, **cfg_overrides):
        log = RecordingLog()
        with patch.object(gate_leg, "time", self.clock):
            outcome = gate_leg.approach_and_cross_one_gate(
                self.nav,
                ScriptedMission(script),
                leg_config(**cfg_overrides),
                gate_index=1,
                log=log,
                avoid_gate=avoid_gate,
            )
        return outcome, log

    def test_re_detecting_the_gate_just_crossed_is_refused(self):
        # COMMITTABLE localizes to (0.9, 0, -1.5) from a drone at the origin.
        # Tell the leg that IS the gate it already flew through.
        outcome, log = self._run(
            gate_fix(0.9, 0.0), [COMMITTABLE], crossed_gate_avoid_m=0.8
        )

        self.assertIs(outcome.result, GateResult.NOT_FOUND)
        self.assertTrue(log.contains("almost certainly the SAME gate"))
        self.assertIn("only ever re-detected the gate already crossed", outcome.reason)

    def test_a_genuinely_different_gate_is_accepted(self):
        # The previous gate is 5 m away; this fix is nothing to do with it.
        outcome, _ = self._run(
            gate_fix(-5.0, 0.0), [COMMITTABLE], crossed_gate_avoid_m=0.8
        )
        self.assertIs(outcome.result, GateResult.CROSSED)

    def test_the_guard_can_be_disabled_for_tightly_spaced_courses(self):
        # A course with two gates genuinely within a metre needs this off.
        outcome, _ = self._run(
            gate_fix(0.9, 0.0), [COMMITTABLE], crossed_gate_avoid_m=0.0
        )
        self.assertIs(outcome.result, GateResult.CROSSED)

    def test_phase_1a_passes_nothing_and_nothing_is_checked(self):
        log = RecordingLog()
        with patch.object(gate_leg, "time", self.clock):
            outcome = gate_leg.approach_and_cross_one_gate(
                self.nav, ScriptedMission([COMMITTABLE]), leg_config(),
                gate_index=0, log=log,
            )
        self.assertIs(outcome.result, GateResult.CROSSED)
        self.assertFalse(log.contains("SAME gate"))


class AbortAndDeadlineTests(unittest.TestCase):
    """Every way a leg can end without the drone deciding to."""

    def setUp(self):
        self.clock = FakeClock()
        self.nav = FakeNav(self.clock)

    def test_a_stopped_controller_aborts_immediately(self):
        self.nav.running = False
        outcome, _ = run_leg(
            self.nav, ScriptedMission([COMMITTABLE]), leg_config(), self.clock
        )

        self.assertIs(outcome.result, GateResult.ABORTED)
        self.assertIn("controller stopped", outcome.reason)

    def test_stale_telemetry_aborts_and_names_the_estimator(self):
        self.nav.telemetry_healthy = False
        self.nav.estimator_text = " (ESTIMATOR DEGRADED: velocity_horiz)"
        outcome, _ = run_leg(
            self.nav, ScriptedMission([COMMITTABLE]), leg_config(), self.clock
        )

        self.assertIs(outcome.result, GateResult.ABORTED)
        self.assertIn("telemetry older than", outcome.reason)
        self.assertIn("ESTIMATOR DEGRADED", outcome.reason)

    def test_losing_offboard_reports_the_estimator_rather_than_blaming_the_link(self):
        """REGRESSION -- PROJECT_STATE gap 4: ESTIMATOR_STATUS was not read.

        PX4 sets `offboard_control_signal_lost` when the estimator fails its
        velocity innovation check past COM_VEL_FS_EVH. The obvious reading of
        that -- "the companion link dropped" -- is wrong roughly as often as it
        is right on a flow-only airframe, and the plan predicted it would be
        misdiagnosed for days. The abort message now carries the estimator's
        own verdict on the same line.
        """
        self.nav.set_state(custom_mode=0)  # PX4 is no longer in OFFBOARD
        self.nav.estimator_text = (
            " (ESTIMATOR DEGRADED: velocity innovation ratio 1.42"
            " -- suspect optical flow, not the offboard link)"
        )
        outcome, _ = run_leg(
            self.nav, ScriptedMission([COMMITTABLE]), leg_config(), self.clock
        )

        self.assertIs(outcome.result, GateResult.ABORTED)
        self.assertIn("no longer in OFFBOARD", outcome.reason)
        self.assertIn("suspect optical flow", outcome.reason)

    def test_the_gate_deadline_is_enforced(self):
        self.nav.move_cost_s = 20.0
        outcome, _ = run_leg(
            self.nav,
            ScriptedMission([FAR]),
            leg_config(approach_timeout_s=30.0),
            self.clock,
        )

        self.assertIs(outcome.result, GateResult.TIMED_OUT)
        self.assertIn("30s budget", outcome.reason)

    def test_every_outcome_carries_diagnosis_fields(self):
        # A named result is only useful with the numbers that produced it.
        self.nav.integrate_velocity = True
        outcome, _ = run_leg(
            self.nav, ScriptedMission([FAR, COMMITTABLE]), leg_config(), self.clock
        )

        self.assertEqual(outcome.gate_index, 0)
        self.assertGreater(outcome.attempts, 0)
        self.assertGreaterEqual(outcome.elapsed_s, 0.0)
        self.assertIsNotNone(outcome.fix)
        self.assertIsNotNone(outcome.entry_state)
        self.assertIsNotNone(outcome.exit_state)


class ObservationHygieneTests(unittest.TestCase):
    """What the leg refuses to act on."""

    def setUp(self):
        self.clock = FakeClock()
        self.nav = FakeNav(self.clock)

    def test_an_edge_on_gate_is_low_confidence_and_is_not_committed(self):
        # A gate seen at 80 degrees of cone has a worthless plane fit. It is in
        # range and roughly centred, so range alone would have committed to it.
        edge_on = detection(forward=0.5, right=0.1, yaw_deg=80.0, dist=0.51)
        outcome, log = run_leg(
            self.nav,
            ScriptedMission([edge_on]),
            leg_config(max_approach_attempts=2, commit_max_cone_deg=60.0),
            self.clock,
        )

        self.assertIsNot(outcome.result, GateResult.CROSSED)
        self.assertTrue(log.contains("LOW CONFIDENCE") or log.contains("cone"))

    def test_a_gate_outside_the_altitude_envelope_is_clamped_and_announced(self):
        # 3 m below the drone would put the gate underground; the clamp must
        # happen AND be visible, because a silent correction hides the bad
        # detection that caused it.
        too_low = detection(forward=1.0, down=3.0, dist=3.16)
        _outcome, log = run_leg(
            self.nav,
            ScriptedMission([too_low]),
            leg_config(max_approach_attempts=1),
            self.clock,
        )

        self.assertTrue(log.contains("clamped") or log.contains("altitude outside"))


if __name__ == "__main__":
    unittest.main()
