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
        # The speed clamp binds while error > max_speed / KP_POS = 0.15 / 1.2
        #                                                        = 0.125 m
        #   constant-speed run: (2.5 - 0.125) / 0.15 = 2.375 / 0.15 = 15.833333 s
        #   exponential tail:   ln(0.125 / 0.10) / 1.2
        #                     = ln(1.25) / 1.2 = 0.22314355 / 1.2 = 0.18595296 s
        #   ideal total                                            = 16.019286 s
        #   plus a 3.0 s margin at unity scale                     = 19.019286 s
        self.assertAlmostEqual(KP_POS, 1.2, places=9)
        self.assertAlmostEqual(POSITION_TOLERANCE_M, 0.10, places=9)

        budget = estimate_move_timeout(2.5, 0.15, margin_s=3.0, scale=1.0)
        self.assertAlmostEqual(budget, 19.019286, places=5)

    def test_the_old_fixed_timeout_could_not_cover_that_leg(self):
        # A permanent regression guard on the defect: 15.83 s of constant-speed
        # travel alone already exceeds the old hard-coded 15 s budget, so the
        # move reported failure on every single run while flying correctly.
        run_phase_s = (2.5 - 0.15 / KP_POS) / 0.15
        self.assertGreater(run_phase_s, MOVE_TIMEOUT)
        self.assertGreater(estimate_move_timeout(2.5, 0.15), MOVE_TIMEOUT)

    def test_short_hops_still_get_the_floor(self):
        # A 0.25 m step must not be given a 3-second budget just because it is
        # short; the floor keeps some slack for settling.
        self.assertAlmostEqual(estimate_move_timeout(0.25, 0.35), MOVE_TIMEOUT, places=9)

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
