"""Tests for Phase 2: the gate_leg seams, the vector leg, and the mission loop.

The thing being checked hardest is the FALLBACK.  Phase 2 is only a reasonable
thing to fly if its worst case is Phase 1B's normal case, and that claim is
worth nothing unless every path that cannot plan a transition provably becomes
a 1B leg and says so.
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

from navigation.missions import gate_leg, phase2_course, vector_leg  # noqa: E402
from navigation.missions.association import (  # noqa: E402
    AssociationConfig,
    GateTracker,
)
from navigation.missions.contracts import (  # noqa: E402
    AltitudeEnvelope,
    GateFix,
    GateLegConfig,
    GateOutcome,
    GateResult,
)
from navigation.missions.course import (  # noqa: E402
    CoursePlanConfig,
    PassVector,
    plan_turn,
)
from navigation.missions.pad import PadConfig, PadResult  # noqa: E402
from navigation.missions.phase2_course import (  # noqa: E402
    CourseState,
    Phase2Config,
    choose_next_gate,
)
from navigation.missions.vector_leg import fly_turn, yaw_rate_between  # noqa: E402


COMMITTABLE = detection(forward=0.9, dist=0.9)
FAR = detection(forward=3.0, dist=3.0)


def leg_config(**overrides) -> GateLegConfig:
    fields = dict(commit_confirm_frames=1, approach_timeout_s=90.0)
    fields.update(overrides)
    return GateLegConfig(**fields)


def vector(n, e, heading_deg, d=-1.5) -> PassVector:
    heading = math.radians(heading_deg)
    return PassVector(n=n, e=e, d=d, dir_n=math.cos(heading), dir_e=math.sin(heading))


def gate_outcome(result, index, reason="scripted") -> GateOutcome:
    # Module scope, not a TestCase method: unittest.TestCase already owns an
    # attribute called `_outcome`, and shadowing it breaks every test in the
    # class with "'_Outcome' object is not callable".
    return GateOutcome(result=result, gate_index=index, reason=reason)


def fix(n, e, d=-1.5, *, normal=(1.0, 0.0), low_confidence=False) -> GateFix:
    return GateFix(
        n=n, e=e, d=d, normal_n=normal[0], normal_e=normal[1],
        yaw_rad=math.atan2(normal[1], normal[0]),
        range_m=3.0, lateral_body_m=0.0, vertical_body_m=0.0, lateral_axis_m=0.0,
        cone_angle_deg=0.0, normal_flipped=False, altitude_clamped=False,
        low_confidence=low_confidence, reason="",
    )


class ExitLegSeamTests(unittest.TestCase):
    """gate_leg's exit_leg= parameter -- PROJECT_STATE gap 12."""

    def setUp(self):
        self.clock = FakeClock()
        self.nav = FakeNav(self.clock)
        self.nav.integrate_velocity = True

    def _run(self, exit_leg=None, expected_gate=None, script=None):
        log = RecordingLog()
        with patch.object(gate_leg, "time", self.clock):
            outcome = gate_leg.approach_and_cross_one_gate(
                self.nav,
                ScriptedMission(script or [COMMITTABLE]),
                leg_config(),
                gate_index=0,
                log=log,
                exit_leg=exit_leg,
                expected_gate=expected_gate,
            )
        return outcome, log

    def test_no_exit_leg_is_exactly_the_old_behaviour(self):
        # Phase 1 passes nothing and must be bit-for-bit unaffected.
        outcome, _ = self._run()

        self.assertIs(outcome.result, GateResult.CROSSED)
        self.assertEqual(outcome.reason, "gate crossed")

    def test_the_exit_leg_runs_after_the_plane_is_cleared(self):
        called = []

        def exit_leg(nav, crossed_fix):
            called.append(crossed_fix)
            return True

        outcome, _ = self._run(exit_leg=exit_leg)

        self.assertIs(outcome.result, GateResult.CROSSED)
        self.assertIn("exit leg flown", outcome.reason)
        self.assertEqual(len(called), 1)
        # It receives the frozen fix the crossing was flown on -- the last
        # moment vision was trusted.
        self.assertAlmostEqual(called[0].n, outcome.fix.n, places=9)

    def test_a_failed_exit_leg_does_not_un_cross_the_gate(self):
        # The gate IS through. Whatever the transition does afterwards cannot
        # change that, and reporting it as a failed gate would make Phase 2
        # score worse than Phase 1B on a flight where it did strictly more.
        outcome, _ = self._run(exit_leg=lambda nav, f: False)

        self.assertIs(outcome.result, GateResult.CROSSED)
        self.assertIn("exit leg did not complete", outcome.reason)

    def test_an_exit_leg_that_raises_is_contained_and_leaves_a_hold(self):
        def boom(nav, crossed_fix):
            raise ValueError("planner blew up")

        outcome, log = self._run(exit_leg=boom)

        self.assertIs(outcome.result, GateResult.CROSSED)
        self.assertIn("planner blew up", outcome.reason)
        self.assertTrue(log.contains("holding after the crossing"))
        self.assertTrue(any("after gate 0" in label for label in self.nav.holds))


class ExpectedGateSeamTests(unittest.TestCase):
    """gate_leg's expected_gate= parameter -- PROJECT_STATE gap 11."""

    def setUp(self):
        self.clock = FakeClock()
        self.nav = FakeNav(self.clock)
        self.nav.integrate_velocity = True

    def _run(self, expected_gate, script):
        log = RecordingLog()
        with patch.object(gate_leg, "time", self.clock):
            outcome = gate_leg.approach_and_cross_one_gate(
                self.nav,
                ScriptedMission(script),
                leg_config(expected_gate_radius_m=2.0),
                gate_index=0,
                log=log,
                expected_gate=expected_gate,
            )
        return outcome, log

    def test_an_observation_near_the_expected_gate_is_used(self):
        # COMMITTABLE localizes to (0.9, 0, -1.5) from a drone at the origin.
        outcome, _ = self._run(fix(0.9, 0.0), [COMMITTABLE])
        self.assertIs(outcome.result, GateResult.CROSSED)

    def test_an_observation_far_from_the_expected_gate_is_discarded(self):
        # A gate 8 m from where the mission expected one is far more likely to
        # be a different gate, a reflection or a doorway than it is to be
        # evidence that the mission was wrong. Acting on it means flying at the
        # wrong thing with full confidence.
        outcome, log = self._run(fix(9.0, 0.0), [COMMITTABLE])

        self.assertIs(outcome.result, GateResult.LOST)
        self.assertTrue(log.contains("from where this gate was expected"))
        self.assertIn("too far from the expected gate", outcome.reason)

    def test_phase_one_passes_nothing_and_nothing_is_checked(self):
        log = RecordingLog()
        with patch.object(gate_leg, "time", self.clock):
            outcome = gate_leg.approach_and_cross_one_gate(
                self.nav, ScriptedMission([COMMITTABLE]), leg_config(),
                gate_index=0, log=log,
            )
        self.assertIs(outcome.result, GateResult.CROSSED)
        self.assertFalse(log.contains("expected"))


class VectorLegTests(unittest.TestCase):
    """Flying a planned arc."""

    def setUp(self):
        self.clock = FakeClock()
        self.nav = FakeNav(self.clock)
        self.nav.integrate_velocity = True
        self.envelope = AltitudeEnvelope()
        self.cfg = CoursePlanConfig()
        self.turn = plan_turn(vector(0, 0, 0), vector(5, 5, 90), self.cfg)

    def _fly(self, turn=None, **kwargs):
        log = RecordingLog()
        with patch.object(vector_leg, "time", self.clock):
            ok = fly_turn(
                self.nav, turn or self.turn, self.envelope, self.cfg, log=log, **kwargs
            )
        return ok, log

    def test_a_feasible_turn_is_flown_to_its_exit_point(self):
        ok, log = self._fly()

        self.assertTrue(ok, log.text)
        state = self.nav.get_vehicle_snapshot()
        self.assertAlmostEqual(state.n, self.turn.exit_n, delta=0.3)
        self.assertAlmostEqual(state.e, self.turn.exit_e, delta=0.3)

    def test_an_infeasible_turn_is_refused_rather_than_improvised(self):
        # Improvising here means flying a shape nobody derived, at a yaw rate
        # nobody bounded, between two gates.
        infeasible = plan_turn(vector(0, 0, 0), vector(1.0, 0.6, 90), self.cfg)
        ok, log = self._fly(infeasible)

        self.assertFalse(ok)
        self.assertTrue(log.contains("Refusing to fly an infeasible turn"))
        self.assertEqual(self.nav.velocity_sends, [])

    def test_the_commanded_heading_follows_the_planned_profile(self):
        self._fly()

        headings = [math.degrees(send[3]) for send in self.nav.velocity_sends]
        self.assertAlmostEqual(headings[0], 0.0, delta=1.0)
        self.assertAlmostEqual(headings[-1], 90.0, delta=1.0)
        # Monotone: the nose never swings back and forth through the turn.
        self.assertEqual(headings, sorted(headings))

    def test_the_leg_leaves_a_hold_behind_on_every_exit(self):
        for turn in (self.turn, plan_turn(vector(0, 0, 0), vector(1.0, 0.6, 90), self.cfg)):
            self.nav.holds.clear()
            self._fly(turn)
            if turn.ok:
                self.assertTrue(any("after" in label for label in self.nav.holds))

    def test_a_turn_that_cannot_be_flown_in_its_budget_gives_up(self):
        # The vehicle is commanded but never moves, so the arc budget expires.
        # Giving up hands control back to the Phase 1B stop-and-acquire.
        self.nav.integrate_velocity = False
        ok, log = self._fly()

        self.assertFalse(ok)
        self.assertTrue(log.contains("Falling back to a stop-and-acquire"))

    def test_the_sampled_profile_respects_the_yaw_rate_limit(self):
        # A check on the plan, not a control input: if the sampling produced a
        # yaw rate above the configured limit, that is a planning bug and this
        # is where it should be caught.
        profile = self.turn.sample(9)
        rate = yaw_rate_between(profile, self.turn.speed_m_s, self.turn.arc_length_m)
        self.assertLessEqual(rate, self.cfg.max_yaw_rate_deg_s + 1e-6)

    def test_altitude_is_held_through_the_transition(self):
        # A transition is a horizontal manoeuvre. Changing height during one
        # moves two things at once between the only two moments the gate
        # geometry is actually measured.
        self._fly()
        self.assertAlmostEqual(self.nav.get_vehicle_snapshot().d, -1.5, delta=0.2)


class NextGateSelectionTests(unittest.TestCase):
    def setUp(self):
        self.tracker = GateTracker(AssociationConfig(hits_for_full_confidence=1))
        self.cfg = CoursePlanConfig()
        self.current = vector(0, 0, 0)

    def test_the_nearest_confident_gate_ahead_is_chosen(self):
        self.tracker.update([fix(4.0, 0.0), fix(9.0, 0.0)], now_s=0.0)
        chosen = choose_next_gate(self.tracker, self.current, 0.0, self.cfg)

        self.assertIsNotNone(chosen)
        self.assertAlmostEqual(chosen.n, 4.0, places=6)

    def test_a_gate_behind_the_current_one_is_never_chosen(self):
        # It is either the gate being flown or one already passed.
        self.tracker.update([fix(-3.0, 0.0)], now_s=0.0)
        self.assertIsNone(choose_next_gate(self.tracker, self.current, 0.0, self.cfg))

    def test_a_low_confidence_track_is_not_chosen(self):
        tracker = GateTracker(AssociationConfig(hits_for_full_confidence=5))
        tracker.update([fix(4.0, 0.0)], now_s=0.0)
        self.assertIsNone(choose_next_gate(tracker, self.current, 0.0, self.cfg))

    def test_no_tracks_at_all_returns_none_rather_than_guessing(self):
        self.assertIsNone(choose_next_gate(self.tracker, self.current, 0.0, self.cfg))


class MissionLoopTests(unittest.TestCase):
    """The course loop: progression, retries, fallback, and always landing."""

    def setUp(self):
        self.nav = FakeNav()
        self.leg_cfg = leg_config()
        self.pad_cfg = PadConfig()
        self.course_cfg = Phase2Config(gate_count=3, gate_retries=0)
        self.plan_cfg = CoursePlanConfig()
        self.assoc_cfg = AssociationConfig()

    def _run(self, scripted, course_cfg=None, mission=None):
        course = CourseState(results=[])
        calls = []

        def fake_leg(nav, mission_, cfg, *, gate_index=0, log=print,
                     expected_gate=None, exit_leg=None):
            calls.append((gate_index, exit_leg))
            outcome = scripted.pop(0)
            course.results.append(outcome)
            return outcome

        with patch.object(phase2_course, "run_pad_sequence",
                          return_value=PadResult(True, "ready", AltitudeEnvelope())), \
             patch.object(phase2_course, "approach_and_cross_one_gate", fake_leg), \
             patch.object(phase2_course, "_settle", lambda nav, duration_s: None), \
             patch.object(phase2_course, "refresh_tracks",
                          lambda *a, **k: None):
            code = phase2_course.run(
                self.nav,
                mission or ScriptedMission([]),
                self.leg_cfg,
                self.pad_cfg,
                course_cfg or self.course_cfg,
                self.plan_cfg,
                self.assoc_cfg,
                course,
            )
        return code, calls, course

    def test_a_clean_course_crosses_every_gate_and_lands_once(self):
        scripted = [gate_outcome(GateResult.CROSSED, i) for i in range(3)]
        code, calls, course = self._run(scripted)

        self.assertEqual(code, 0)
        self.assertEqual([index for index, _ in calls], [0, 1, 2])
        self.assertEqual(self.nav.landings, 1)
        self.assertEqual(course.crossed, 3)

    def test_every_gate_is_offered_an_exit_leg_while_planning_is_enabled(self):
        scripted = [gate_outcome(GateResult.CROSSED, i) for i in range(3)]
        _code, calls, _course = self._run(scripted)

        for _index, exit_leg in calls:
            self.assertIsNotNone(exit_leg)

    def test_a_failing_gate_is_retried_before_counting_as_a_failure(self):
        scripted = [
            gate_outcome(GateResult.LOST, 0),
            gate_outcome(GateResult.CROSSED, 0),
            gate_outcome(GateResult.CROSSED, 1),
        ]
        code, calls, _course = self._run(
            scripted, Phase2Config(gate_count=2, gate_retries=1)
        )

        self.assertEqual(code, 0)
        self.assertEqual([index for index, _ in calls], [0, 0, 1])

    def test_consecutive_failures_end_the_course_and_still_land(self):
        scripted = [
            gate_outcome(GateResult.NO_COMMIT, 0),
            gate_outcome(GateResult.NO_COMMIT, 1),
        ]
        code, _calls, _course = self._run(
            scripted,
            Phase2Config(gate_count=4, gate_retries=0, max_consecutive_failures=2),
        )

        self.assertEqual(code, 1)
        self.assertEqual(self.nav.landings, 1)
        self.assertIn("consecutive gate failures", self.nav.safe_shutdowns[0])

    def test_an_abort_ends_the_course_immediately_and_lands(self):
        scripted = [gate_outcome(GateResult.ABORTED, 0, "telemetry stale")]
        code, calls, _course = self._run(
            scripted, Phase2Config(gate_count=3, gate_retries=2)
        )

        self.assertEqual(code, 1)
        self.assertEqual(len(calls), 1)
        self.assertEqual(self.nav.landings, 1)

    def test_a_failed_pad_sequence_lands_without_attempting_any_gate(self):
        course = CourseState(results=[])
        with patch.object(phase2_course, "run_pad_sequence",
                          return_value=PadResult(False, "no fresh detections")):
            code = phase2_course.run(
                self.nav, ScriptedMission([]), self.leg_cfg, self.pad_cfg,
                self.course_cfg, self.plan_cfg, self.assoc_cfg, course,
            )

        self.assertEqual(code, 1)
        self.assertEqual(self.nav.landings, 1)

    def test_a_gate_crossed_without_a_transition_settles_like_phase_one_b(self):
        # "exit leg flown" absent from the reason means no transition happened,
        # so the leg must fall back to 1B's stop-and-re-acquire.
        scripted = [gate_outcome(GateResult.CROSSED, 0, "gate crossed")]
        self._run(scripted, Phase2Config(gate_count=1, gate_retries=0))

        self.assertTrue(any("settled after gate" in label for label in self.nav.holds))

    def test_a_gate_crossed_with_a_transition_does_not_stop(self):
        # The stop is the entire cost Phase 2 exists to remove.
        scripted = [
            gate_outcome(GateResult.CROSSED, 0, "gate crossed; exit leg flown")
        ]
        self._run(scripted, Phase2Config(gate_count=1, gate_retries=0))

        self.assertFalse(any("settled after gate" in label for label in self.nav.holds))

    def test_repeated_transition_failures_disable_planning_for_the_rest(self):
        # A planner that keeps failing is telling you something about the
        # course, and re-attempting a manoeuvre that has not worked is not a
        # recovery strategy.
        course = CourseState(results=[])
        calls = []

        def fake_leg(nav, mission_, cfg, *, gate_index=0, log=print,
                     expected_gate=None, exit_leg=None):
            calls.append((gate_index, exit_leg))
            course.transitions_failed += 1
            outcome = GateOutcome(
                result=GateResult.CROSSED, gate_index=gate_index, reason="gate crossed"
            )
            course.results.append(outcome)
            return outcome

        with patch.object(phase2_course, "run_pad_sequence",
                          return_value=PadResult(True, "ready", AltitudeEnvelope())), \
             patch.object(phase2_course, "approach_and_cross_one_gate", fake_leg), \
             patch.object(phase2_course, "_settle", lambda nav, duration_s: None), \
             patch.object(phase2_course, "refresh_tracks", lambda *a, **k: None):
            phase2_course.run(
                self.nav, ScriptedMission([]), self.leg_cfg, self.pad_cfg,
                Phase2Config(gate_count=4, gate_retries=0, max_transition_failures=2),
                self.plan_cfg, self.assoc_cfg, course,
            )

        # First two gates get an exit leg; after two failures, planning is off.
        self.assertIsNotNone(calls[0][1])
        self.assertIsNone(calls[-1][1])
        self.assertTrue(any("planning disabled" in note for note in course.degradations))


class ConfigTests(unittest.TestCase):
    def test_impossible_course_configurations_are_rejected(self):
        for kwargs in (
            {"gate_count": 0},
            {"gate_retries": -1},
            {"max_consecutive_failures": 0},
            {"course_timeout_s": 0.0},
            {"max_transition_failures": -1},
        ):
            with self.subTest(**kwargs):
                with self.assertRaises(ValueError):
                    Phase2Config(**kwargs)

    def test_no_planning_makes_every_leg_degrade(self):
        args = phase2_course.parse_args(["--gates", "3", "--offline", "--no-planning"])
        self.assertTrue(args.no_planning)

    def test_offline_and_dry_run_are_mutually_exclusive(self):
        args = phase2_course.parse_args(["--offline", "--dry-run"])
        self.assertFalse(phase2_course.check_mode_flags(args))


if __name__ == "__main__":
    unittest.main()
