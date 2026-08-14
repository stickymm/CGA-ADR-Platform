import math
import sys
import types
import unittest

fake_mavutil = types.ModuleType("pymavlink.mavutil")
fake_mavutil.mavfile = object
fake_pymavlink = types.ModuleType("pymavlink")
fake_pymavlink.mavutil = fake_mavutil
sys.modules.setdefault("pymavlink", fake_pymavlink)
sys.modules.setdefault("pymavlink.mavutil", fake_mavutil)

from navigation.navigation import (  # noqa: E402
    KP_POS,
    MOVE_TIMEOUT,
    POSITION_TOLERANCE_M,
    GateDetection,
    Setpoint,
    SetpointKind,
    estimate_move_timeout,
    is_valid_detection,
    type_mask_for,
)


def detection(dist):
    return GateDetection(
        timestamp=0.0,
        dist=dist,
        forward=dist,
        right=0.0,
        down=0.0,
        roll=0.0,
        pitch=0.0,
        yaw_deg=0.0,
    )


class MoveTimeoutTests(unittest.TestCase):
    def test_the_hand_derived_budget_for_the_pass_through_leg(self):
        # The leg that always failed in the old course mission: 1.0 m standoff
        # plus 1.5 m beyond the gate = 2.5 m, flown at 0.15 m/s.
        #
        # Pinned at the ORIGINAL 0.10 m tolerance so the arithmetic below stays
        # the hand-derived arithmetic it was checked against, independently of
        # what the default tolerance is today.
        #
        # The speed clamp binds while error > max_speed / KP_POS = 0.15 / 1.2
        #                                                        = 0.125 m
        #   constant-speed run: (2.5 - 0.125) / 0.15 = 2.375 / 0.15 = 15.833333 s
        #   exponential tail:   ln(0.125 / 0.10) / 1.2
        #                     = ln(1.25) / 1.2 = 0.22314355 / 1.2 = 0.18595296 s
        #   ideal total                                            = 16.019286 s
        #   plus a 3.0 s margin at unity scale                     = 19.019286 s
        self.assertAlmostEqual(KP_POS, 1.2, places=9)

        budget = estimate_move_timeout(
            2.5, 0.15, tolerance_m=0.10, margin_s=3.0, scale=1.0
        )
        self.assertAlmostEqual(budget, 19.019286, places=5)

    def test_the_hand_derived_budget_at_the_current_default_tolerance(self):
        # Same leg at the tolerance actually in force (0.15 m -- see the comment
        # on POSITION_TOLERANCE_M for why 0.10 m was inside the flow noise
        # floor). Only the exponential tail changes:
        #   exponential tail: ln(0.125 / 0.15) / 1.2
        #                   = -0.18232156 / 1.2 = -0.15193463 s
        # i.e. the tolerance is already wider than the knee, so the decay phase
        # is over before it starts and the run phase alone covers the move.
        #   ideal total  = 15.833333 - 0.151935 = 15.681399 s
        #   plus margin                          = 18.681399 s
        self.assertAlmostEqual(POSITION_TOLERANCE_M, 0.15, places=9)

        budget = estimate_move_timeout(2.5, 0.15, margin_s=3.0, scale=1.0)
        self.assertAlmostEqual(budget, 18.681399, places=5)

    def test_the_old_fixed_timeout_could_not_cover_that_leg(self):
        # A permanent regression guard on the defect: 15.83 s of constant-speed
        # travel alone already exceeds the old hard-coded 15 s budget, so the
        # move reported failure on every single run while flying correctly.
        old_fixed_timeout_s = 15.0
        run_phase_s = (2.5 - 0.15 / KP_POS) / 0.15
        self.assertGreater(run_phase_s, old_fixed_timeout_s)
        self.assertGreater(estimate_move_timeout(2.5, 0.15), old_fixed_timeout_s)

    def test_short_hops_still_get_the_floor(self):
        # A 0.25 m step must not be given a 3-second budget just because it is
        # short; the floor keeps some slack for settling.
        self.assertAlmostEqual(estimate_move_timeout(0.25, 0.35), MOVE_TIMEOUT, places=9)

    def test_the_move_floor_no_longer_swallows_the_gate_deadline(self):
        # REGRESSION GUARD -- MED bug, 2026-08-10.
        #
        # The bug was not in any single constant, it was in the relationship
        # between three of them: a 0.10 m arrival tolerance inside the optical
        # flow noise floor, a 15 s move floor, and a 75 s per-gate deadline. A
        # 0.25 m approach step was issued with a 15 s budget it routinely spent
        # in full, so the deadline delivered ~5 attempts against a config whose
        # banner advertised 14.
        step_budget_s = estimate_move_timeout(0.25, 0.35, tolerance_m=0.15)
        observation_s = 0.5
        attempts = 14

        old_reachable = 75.0 / (observation_s + 15.0)
        self.assertLess(old_reachable, 6.0)  # the defect, preserved as arithmetic

        needed_s = attempts * (observation_s + step_budget_s)
        self.assertLessEqual(needed_s, 90.0)  # the current approach_timeout_s

    def test_the_budget_grows_with_distance_and_shrinks_with_speed(self):
        self.assertGreater(
            estimate_move_timeout(6.0, 0.15), estimate_move_timeout(2.5, 0.15)
        )
        self.assertGreater(
            estimate_move_timeout(6.0, 0.15), estimate_move_timeout(6.0, 0.50)
        )

    def test_a_move_shorter_than_the_tolerance_needs_no_travel_time(self):
        self.assertAlmostEqual(
            estimate_move_timeout(0.01, 0.5, margin_s=3.0, scale=1.0, floor_s=0.1),
            3.0,
            places=9,
        )

    def test_invalid_inputs_are_rejected(self):
        for distance, speed in ((-1.0, 0.5), (math.inf, 0.5), (1.0, 0.0), (1.0, -0.5)):
            with self.subTest(distance=distance, speed=speed):
                with self.assertRaises(ValueError):
                    estimate_move_timeout(distance, speed)


class ApproachBudgetAfterFirstFlightTests(unittest.TestCase):
    """The retuned approach defaults, checked against the flight that failed.

    First live gate approach, 2026-08-13. The aircraft flew well and tracked the
    gate cleanly; it simply ran out of attempts, reaching 1.41 m of a gate it
    needed to be within 1.00 m of, on attempt 14 of 14, still closing.
    """

    OBSERVED_START_RANGE_M = 3.23
    OBSERVED_CLOSURE_PER_STEP_M = 0.14   # 0.25 m steps, 56% efficient

    def setUp(self):
        from navigation.missions.contracts import GateLegConfig

        self.cfg = GateLegConfig()

    def test_the_old_defaults_could_not_have_reached_the_gate(self):
        # The defect, preserved as arithmetic so nobody quietly reverts it.
        needed_m = self.OBSERVED_START_RANGE_M - 1.00
        attempts_required = needed_m / self.OBSERVED_CLOSURE_PER_STEP_M

        self.assertGreater(attempts_required, 14)

    def test_the_new_attempt_cap_covers_that_approach_even_at_the_old_efficiency(self):
        # Worst case: assume the separate vertical budget buys nothing at all
        # and every step still yields only the observed 0.14 m.
        needed_m = self.OBSERVED_START_RANGE_M - self.cfg.commit_distance_m
        attempts_required = needed_m / self.OBSERVED_CLOSURE_PER_STEP_M

        self.assertLessEqual(attempts_required, self.cfg.max_approach_attempts)

    def test_the_deadline_can_still_deliver_the_attempts_it_advertises(self):
        from navigation.missions.contracts import attempt_budget_s, deadline_consistency

        self.assertLessEqual(attempt_budget_s(self.cfg), self.cfg.approach_timeout_s)
        self.assertEqual(deadline_consistency(self.cfg), ())

    def test_the_arrival_tolerance_is_still_smaller_than_the_bigger_step(self):
        # Raising step_size_m must not let a step "arrive" without moving.
        self.assertLess(self.cfg.arrival_tolerance_m, self.cfg.step_size_m)

    def test_the_vertical_budget_stays_under_the_horizontal_one(self):
        # The noisy axis must not dominate the step...
        self.assertLess(self.cfg.vertical_step_m, self.cfg.step_size_m)

    def test_the_vertical_step_must_exceed_the_arrival_tolerance(self):
        """REGRESSION -- flight of 2026-08-14, and it grounded the approach.

        move_to_target's arrival test is a 3-D distance. A vertical step smaller
        than that tolerance is already inside it, so the move reports "Reached"
        the instant the HORIZONTAL error closes -- with the whole descent still
        outstanding. Guaranteed descent per attempt: zero.

        It shipped at 0.12 against a 0.15 m tolerance. Five consecutive moves
        each commanded a 0.12 m descent; net altitude change was -0.01 m. The
        aircraft sat 0.50 m above the gate for the entire approach and lost
        sight of it over the top.
        """
        self.assertGreater(self.cfg.vertical_step_m, self.cfg.arrival_tolerance_m)
        guaranteed = self.cfg.vertical_step_m - self.cfg.arrival_tolerance_m
        self.assertGreaterEqual(guaranteed, 0.05)

    def test_a_config_that_could_never_descend_is_rejected_outright(self):
        from navigation.missions.contracts import GateLegConfig

        with self.assertRaises(ValueError) as caught:
            GateLegConfig(vertical_step_m=0.12, arrival_tolerance_m=0.15)
        self.assertIn("never change altitude", str(caught.exception))

    def test_commit_tolerances_must_fit_the_airframe_through_the_gate(self):
        from navigation.missions.contracts import GateLegConfig

        # A 0.97 m gate with a 0.115 m airframe radius leaves 0.371 m usable.
        GateLegConfig(commit_vertical_tol_m=0.35)          # fits
        with self.assertRaises(ValueError) as caught:
            GateLegConfig(commit_vertical_tol_m=0.45)      # does not
        self.assertIn("usable half-opening", str(caught.exception))

    def test_the_commit_tolerances_were_NOT_loosened(self):
        # The gate is a 0.97155 m square. With a ~0.4 m airframe there is only
        # ~0.29 m of true clearance per side, so 0.20 / 0.25 m is already at the
        # limit. Making the approach easier must never come from making the
        # decision to fly at a gate easier.
        self.assertAlmostEqual(self.cfg.commit_lateral_tol_m, 0.20, places=9)
        self.assertAlmostEqual(self.cfg.commit_vertical_tol_m, 0.25, places=9)
        self.assertAlmostEqual(self.cfg.commit_distance_m, 1.00, places=9)
        self.assertEqual(self.cfg.commit_confirm_frames, 3)


class TypeMaskTests(unittest.TestCase):
    def test_velocity_and_yaw_mask_is_the_documented_value(self):
        # ignore x(1) y(2) z(4) + ax(64) ay(128) az(256) + yaw_rate(2048) = 2503
        self.assertEqual(type_mask_for(SetpointKind.VELOCITY_YAW), 2503)
        self.assertEqual(1 + 2 + 4 + 64 + 128 + 256 + 2048, 2503)

    def test_position_and_yaw_mask_is_the_documented_value(self):
        # ignore vx(8) vy(16) vz(32) + ax(64) ay(128) az(256) + yaw_rate(2048) = 2552
        self.assertEqual(type_mask_for(SetpointKind.POSITION_YAW), 2552)
        self.assertEqual(8 + 16 + 32 + 64 + 128 + 256 + 2048, 2552)

    def test_brake_and_hold_altitude_mask_is_the_documented_value(self):
        # Active: vx, vy, z, yaw. This is the takeoff and hold setpoint: a full
        # position mask would carry an x/y target, and any x/y other than the
        # exact position at the mode-switch instant makes the vehicle translate
        # toward it at up to MPC_XY_VEL_MAX while barely off the ground.
        # ignore x(1) y(2) + vz(32) + ax(64) ay(128) az(256) + yaw_rate(2048) = 2531
        self.assertEqual(type_mask_for(SetpointKind.BRAKE_HOLD_ALT), 2531)
        self.assertEqual(1 + 2 + 32 + 64 + 128 + 256 + 2048, 2531)

    def test_idle_has_no_mask(self):
        with self.assertRaises(ValueError):
            type_mask_for(SetpointKind.IDLE)


class SetpointSemanticsTests(unittest.TestCase):
    def test_only_velocity_setpoints_are_transient(self):
        # The deadman must age out a velocity command, because an unrefreshed one
        # keeps the vehicle flying. It must NOT age out a hold or an altitude
        # target, because those stay true indefinitely -- ageing them out would
        # override a climb-to-altitude, which is exactly what it did before this
        # distinction existed.
        self.assertTrue(Setpoint(kind=SetpointKind.VELOCITY_YAW).is_transient)
        self.assertFalse(Setpoint(kind=SetpointKind.POSITION_YAW).is_transient)
        self.assertFalse(Setpoint(kind=SetpointKind.BRAKE_HOLD_ALT).is_transient)
        self.assertFalse(Setpoint(kind=SetpointKind.IDLE).is_transient)

    def test_a_climbing_setpoint_is_recognised(self):
        # PX4 latches want_takeoff as soon as the vehicle is armed and sees a
        # fresh setpoint with negative vz. The pad sequence refuses to wait for
        # the operator while streaming one, or arming and lift-off would be the
        # same event.
        self.assertTrue(Setpoint(kind=SetpointKind.VELOCITY_YAW, vd=-0.5).climbs())
        self.assertFalse(Setpoint(kind=SetpointKind.VELOCITY_YAW, vd=0.0).climbs())
        self.assertFalse(Setpoint(kind=SetpointKind.VELOCITY_YAW, vd=+0.5).climbs())

    def test_an_altitude_target_above_the_vehicle_is_not_a_climbing_velocity(self):
        # A BRAKE_HOLD_ALT setpoint is a position target, so PX4's want_takeoff
        # check on vz does not apply to it.
        self.assertFalse(Setpoint(kind=SetpointKind.BRAKE_HOLD_ALT, d=-5.0).climbs())


class DetectionValidityTests(unittest.TestCase):
    def test_the_no_detection_sentinel_is_rejected(self):
        self.assertFalse(is_valid_detection(detection(999.0)))

    def test_none_is_rejected(self):
        self.assertFalse(is_valid_detection(None))

    def test_a_real_gate_is_accepted(self):
        self.assertTrue(is_valid_detection(detection(3.0)))

    def test_implausible_but_non_sentinel_values_are_rejected(self):
        # The old exact-equality check accepted a "gate" at 998.7 m, and would
        # have accepted any sentinel that was not bit-identical to 999.0.
        self.assertFalse(is_valid_detection(detection(998.7)))
        self.assertFalse(is_valid_detection(detection(0.0)))
        self.assertFalse(is_valid_detection(detection(float("nan"))))
        self.assertFalse(is_valid_detection(detection(float("inf"))))


if __name__ == "__main__":
    unittest.main()
