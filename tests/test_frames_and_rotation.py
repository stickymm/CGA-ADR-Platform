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
    GateDetection,
    VehicleState,
    body_to_local,
    deg_to_rad,
)
from navigation.missions.contracts import AltitudeEnvelope, GateLegConfig  # noqa: E402
from navigation.missions.frames import (  # noqa: E402
    average_detections,
    circular_mean_deg,
    clamp_altitude,
    crossed_gate_plane,
    evaluate_commit,
    gate_cone_angle_deg,
    lateral_offset_from_axis,
    localize_gate,
    pass_through_target,
    resolve_gate_normal,
    rotate_body_to_ned,
    standoff_target,
)


def detection(**overrides):
    fields = dict(
        timestamp=0.0,
        dist=3.0,
        forward=3.0,
        right=0.0,
        down=0.0,
        roll=0.0,
        pitch=0.0,
        yaw_deg=0.0,
    )
    fields.update(overrides)
    return GateDetection(**fields)


class RotationTests(unittest.TestCase):
    def test_three_two_one_rotation_reduces_to_yaw_only_when_level(self):
        # Hand-derived: at yaw=90deg the body x axis points East and body y points
        # South, so forward 2.0 -> East +2.0 and right 1.0 -> North -1.0, with the
        # down component passing through unrotated.
        result = rotate_body_to_ned(2.0, 1.0, -0.5, yaw_rad=math.pi / 2)

        self.assertAlmostEqual(result[0], -1.0, places=9)
        self.assertAlmostEqual(result[1], 2.0, places=9)
        self.assertAlmostEqual(result[2], -0.5, places=9)

    def test_three_two_one_rotation_matches_legacy_body_to_local_when_level(self):
        # Adopting full attitude must not change any existing behaviour while the
        # vehicle is level, or the two existing missions would silently change.
        for yaw_rad in (0.0, math.pi / 4, math.pi / 2, -2.0, 3.0):
            with self.subTest(yaw_rad=yaw_rad):
                legacy = body_to_local(2.0, 1.0, -0.5, yaw_rad)
                full = rotate_body_to_ned(2.0, 1.0, -0.5, yaw_rad=yaw_rad)
                for legacy_axis, full_axis in zip(legacy, full):
                    self.assertAlmostEqual(legacy_axis, full_axis, places=12)

    def test_pitch_moves_a_straight_ahead_gate_vertically(self):
        # Hand-derived: with yaw=roll=0 the matrix reduces to Ry(pitch), whose
        # first and third rows are (cos p, 0, sin p) and (-sin p, 0, cos p).
        # For a body vector (3, 0, 0) at 30 degrees of pitch:
        #     n = 3 * cos(30 deg) = 3 * 0.8660254 = 2.5980762
        #     d = -3 * sin(30 deg) = -3 * 0.5     = -1.5
        result = rotate_body_to_ned(3.0, 0.0, 0.0, yaw_rad=0.0, pitch_rad=deg_to_rad(30.0))

        self.assertAlmostEqual(result[0], 2.5980762, places=6)
        self.assertAlmostEqual(result[1], 0.0, places=9)
        self.assertAlmostEqual(result[2], -1.5, places=9)

        # The yaw-only form reports zero vertical displacement -- a 1.5 m error
        # fed straight into the commanded altitude.
        self.assertAlmostEqual(body_to_local(3.0, 0.0, 0.0, 0.0)[2], 0.0, places=9)

    def test_ten_degrees_of_pitch_at_three_metres_is_half_a_metre_of_altitude(self):
        # 3 * sin(10 deg) = 3 * 0.17364818 = 0.52094453
        result = rotate_body_to_ned(3.0, 0.0, 0.0, yaw_rad=0.0, pitch_rad=deg_to_rad(10.0))
        self.assertAlmostEqual(result[2], -0.52094453, places=7)


class CircularMeanTests(unittest.TestCase):
    def test_circular_mean_does_not_collapse_across_the_wrap(self):
        # The arithmetic mean of +179 and -179 is 0, which is 180 degrees wrong.
        # sin sums to zero and cos sums to -0.9998477, so atan2 gives +pi.
        self.assertAlmostEqual(circular_mean_deg([179.0, -179.0]), 180.0, places=6)
        self.assertNotAlmostEqual(circular_mean_deg([179.0, -179.0]), 0.0, places=3)

    def test_circular_mean_matches_arithmetic_mean_away_from_the_wrap(self):
        self.assertAlmostEqual(circular_mean_deg([10.0, 20.0, 30.0]), 20.0, places=6)

    def test_circular_mean_rejects_an_empty_sequence(self):
        with self.assertRaises(ValueError):
            circular_mean_deg([])

    def test_averaging_detections_uses_a_circular_mean_for_angles(self):
        averaged = average_detections(
            [detection(yaw_deg=179.0, forward=3.0), detection(yaw_deg=-179.0, forward=1.0)]
        )
        self.assertAlmostEqual(averaged.yaw_deg, 180.0, places=6)
        self.assertAlmostEqual(averaged.forward, 2.0, places=9)


class GateNormalTests(unittest.TestCase):
    # Gate 5 m due north of the drone.  The two cases are the same physical gate
    # with the detector reporting its plane normal in opposite directions.
    GATE_N, GATE_E = 5.0, 0.0
    DRONE_N, DRONE_E = 0.0, 0.0

    def test_normal_pointing_back_at_the_drone_is_flipped(self):
        # Reported 170 deg -> candidate normal (-0.98480775, 0.17364818).
        # Dot with drone->gate (5, 0) = -4.924 < 0, so it points back at us.
        normal_n, normal_e, flipped = resolve_gate_normal(
            self.GATE_N, self.GATE_E, self.DRONE_N, self.DRONE_E, deg_to_rad(170.0)
        )

        self.assertTrue(flipped)
        self.assertAlmostEqual(normal_n, 0.98480775, places=7)
        self.assertAlmostEqual(normal_e, -0.17364818, places=7)

    def test_normal_already_pointing_away_is_left_alone(self):
        normal_n, normal_e, flipped = resolve_gate_normal(
            self.GATE_N, self.GATE_E, self.DRONE_N, self.DRONE_E, deg_to_rad(-10.0)
        )

        self.assertFalse(flipped)
        self.assertAlmostEqual(normal_n, 0.98480775, places=7)
        self.assertAlmostEqual(normal_e, -0.17364818, places=7)

    def test_both_detector_conventions_yield_the_identical_heading(self):
        # This equality IS the point of resolving the ambiguity geometrically:
        # the flown heading must not depend on which way the detector reports the
        # plane.  Only the logged `flipped` flag differs.
        flipped_case = resolve_gate_normal(
            self.GATE_N, self.GATE_E, self.DRONE_N, self.DRONE_E, deg_to_rad(170.0)
        )
        plain_case = resolve_gate_normal(
            self.GATE_N, self.GATE_E, self.DRONE_N, self.DRONE_E, deg_to_rad(-10.0)
        )

        flipped_heading = math.atan2(flipped_case[1], flipped_case[0])
        plain_heading = math.atan2(plain_case[1], plain_case[0])

        self.assertAlmostEqual(flipped_heading, plain_heading, places=9)
        self.assertAlmostEqual(math.degrees(flipped_heading), -10.0, places=6)
        self.assertNotEqual(flipped_case[2], plain_case[2])

    def test_a_standoff_target_lands_between_the_drone_and_the_gate(self):
        # The practical consequence: whichever convention is reported, the
        # standoff must never land behind the gate.
        for reported_deg in (0.0, 180.0):
            with self.subTest(reported_deg=reported_deg):
                fix = localize_gate(
                    detection(forward=5.0, yaw_deg=reported_deg),
                    VehicleState(n=0.0, e=0.0, d=-1.5, yaw_rad=0.0),
                    AltitudeEnvelope(),
                )
                target = standoff_target(fix, 1.0, AltitudeEnvelope())
                self.assertAlmostEqual(target.n, 4.0, places=6)
                self.assertGreater(target.n, 0.0)
                self.assertLess(target.n, fix.n)


class ConeAndOffsetTests(unittest.TestCase):
    def test_cone_angle_is_the_arctangent_of_the_offset_over_the_range(self):
        # Gate at (5, 0) with normal +North; drone 1.5 m east of the axis.
        # The along-axis leg is 5.0 and the perpendicular leg is 1.5, so the
        # angle is atan(1.5 / 5.0) = atan(0.3) = 16.699244 degrees.
        angle = gate_cone_angle_deg(5.0, 0.0, 1.0, 0.0, 0.0, 1.5)
        self.assertAlmostEqual(angle, math.degrees(math.atan(0.3)), places=9)
        self.assertAlmostEqual(angle, 16.699244, places=5)

    def test_cone_angle_is_zero_straight_down_the_axis(self):
        self.assertAlmostEqual(gate_cone_angle_deg(5.0, 0.0, 1.0, 0.0, 0.0, 0.0), 0.0, places=9)

    def test_cone_angle_reports_edge_on_for_a_degenerate_zero_length_vector(self):
        self.assertAlmostEqual(gate_cone_angle_deg(1.0, 2.0, 1.0, 0.0, 1.0, 2.0), 90.0, places=9)

    def test_lateral_offset_is_the_perpendicular_distance_from_the_axis(self):
        self.assertAlmostEqual(
            lateral_offset_from_axis(5.0, 0.0, 1.0, 0.0, 0.0, 1.5), 1.5, places=9
        )


class AltitudeEnvelopeTests(unittest.TestCase):
    ENVELOPE = AltitudeEnvelope(takeoff_alt_m=1.5, min_alt_m=0.8, max_alt_m=2.5, d_takeoff=0.0)

    def test_a_target_inside_the_band_is_untouched(self):
        clamped, was_clamped = clamp_altitude(-1.5, self.ENVELOPE)
        self.assertAlmostEqual(clamped, -1.5, places=9)
        self.assertFalse(was_clamped)

    def test_a_target_below_the_floor_is_raised(self):
        # d = -0.2 is only 0.2 m above the pad; the floor is 0.8 m, i.e. d = -0.8.
        clamped, was_clamped = clamp_altitude(-0.2, self.ENVELOPE)
        self.assertAlmostEqual(clamped, -0.8, places=9)
        self.assertTrue(was_clamped)

    def test_a_target_above_the_ceiling_is_lowered(self):
        clamped, was_clamped = clamp_altitude(-3.0, self.ENVELOPE)
        self.assertAlmostEqual(clamped, -2.5, places=9)
        self.assertTrue(was_clamped)

    def test_the_envelope_follows_a_non_zero_pad_reference(self):
        envelope = AltitudeEnvelope(d_takeoff=-0.05)
        clamped, was_clamped = clamp_altitude(-0.5, envelope)
        self.assertAlmostEqual(clamped, -0.85, places=9)
        self.assertTrue(was_clamped)

    def test_an_inverted_envelope_is_rejected_at_construction(self):
        with self.assertRaises(ValueError):
            AltitudeEnvelope(min_alt_m=2.5, max_alt_m=0.8)

    def test_a_takeoff_altitude_outside_the_envelope_is_rejected(self):
        with self.assertRaises(ValueError):
            AltitudeEnvelope(takeoff_alt_m=3.0, min_alt_m=0.8, max_alt_m=2.5)


class CrossingTests(unittest.TestCase):
    def _fix(self):
        return localize_gate(
            detection(forward=5.0),
            VehicleState(n=0.0, e=0.0, d=-1.5, yaw_rad=0.0),
            AltitudeEnvelope(),
        )

    def test_the_drone_has_not_crossed_until_it_is_past_the_plane_by_the_clearance(self):
        # Gate is at n = 5.0 with its normal pointing north, so the along-normal
        # progress is simply (drone_n - 5.0).  Values are kept off the exact
        # threshold: 5.8 - 5.0 is 0.7999999999999998 in binary floating point, so
        # an equality-boundary assertion would be testing float representation
        # rather than the geometry.
        fix = self._fix()
        self.assertFalse(crossed_gate_plane(5.5, 0.0, fix, clearance_m=0.8))
        self.assertTrue(crossed_gate_plane(5.9, 0.0, fix, clearance_m=0.8))

    def test_crossing_is_judged_along_the_normal_not_by_euclidean_distance(self):
        # A drone well past the plane but 2 m off to the side HAS crossed: the
        # old position-tolerance test would have called this a failure forever.
        fix = self._fix()
        self.assertTrue(crossed_gate_plane(6.0, 2.0, fix, clearance_m=0.8))

    def test_a_pass_through_target_sits_beyond_the_gate(self):
        fix = self._fix()
        target = pass_through_target(fix, 1.5, AltitudeEnvelope())
        self.assertAlmostEqual(target.n, 6.5, places=6)


class CommitGateTests(unittest.TestCase):
    CONFIG = GateLegConfig()

    def _fix_at(self, **overrides):
        return localize_gate(
            detection(**overrides),
            VehicleState(n=0.0, e=0.0, d=-1.5, yaw_rad=0.0),
            AltitudeEnvelope(),
        )

    def test_a_close_and_centred_gate_commits(self):
        verdict = evaluate_commit(self._fix_at(forward=0.9), self.CONFIG)
        self.assertTrue(verdict.ok, verdict.reason)

    def test_a_close_but_laterally_offset_gate_does_not_commit(self):
        # The case slant range alone cannot see, and the one that clips a gate
        # leg on an open-loop pass.  Hand-derived: forward 0.75 m with a 0.50 m
        # lateral offset gives a slant range of hypot(0.75, 0.50) = 0.901 m,
        # comfortably inside the 1.00 m commit distance -- so range says "go"
        # while the drone is half a metre off the gate's centre line.
        fix = self._fix_at(forward=0.75, right=0.5)
        self.assertAlmostEqual(fix.range_m, 0.9013878, places=6)

        verdict = evaluate_commit(fix, self.CONFIG)
        self.assertFalse(verdict.ok)
        self.assertTrue(verdict.in_range)
        self.assertFalse(verdict.aligned_lateral)
        self.assertIn("lateral", verdict.reason)

    def test_a_centred_but_distant_gate_does_not_commit(self):
        verdict = evaluate_commit(self._fix_at(forward=3.0), self.CONFIG)
        self.assertFalse(verdict.ok)
        self.assertFalse(verdict.in_range)

    def test_an_edge_on_gate_does_not_commit(self):
        verdict = evaluate_commit(self._fix_at(forward=0.9, yaw_deg=75.0), self.CONFIG)
        self.assertFalse(verdict.ok)
        self.assertFalse(verdict.within_cone)

    def test_the_verdict_reports_every_term_for_the_log(self):
        described = evaluate_commit(self._fix_at(forward=0.9, right=0.6), self.CONFIG).describe()
        for token in ("range=", "lat=", "vert=", "cone="):
            self.assertIn(token, described)


class LocalizationTests(unittest.TestCase):
    def test_range_is_recomputed_and_not_taken_from_the_dist_field(self):
        # The vision process derives `dist` from the raw pose but publishes
        # forward/right/down from the smoothed pose, so the two disagree.  We
        # must trust the components, which are what the geometry uses.
        fix = localize_gate(
            detection(dist=99.0, forward=3.0, right=4.0, down=0.0),
            VehicleState(n=0.0, e=0.0, d=-1.5, yaw_rad=0.0),
            AltitudeEnvelope(),
        )
        self.assertAlmostEqual(fix.range_m, 5.0, places=9)

    def test_camera_offsets_shift_the_localized_gate(self):
        fix = localize_gate(
            detection(forward=3.0),
            VehicleState(n=0.0, e=0.0, d=-1.5, yaw_rad=0.0),
            AltitudeEnvelope(),
            cam_offset_right_m=0.25,
        )
        self.assertAlmostEqual(fix.e, 0.25, places=9)
        self.assertAlmostEqual(fix.lateral_body_m, 0.25, places=9)

    def test_a_gate_below_the_floor_is_clamped_and_flagged_low_confidence(self):
        # A gate detected 3 m below the drone would command flight into the floor.
        fix = localize_gate(
            detection(forward=3.0, down=3.0),
            VehicleState(n=0.0, e=0.0, d=-1.5, yaw_rad=0.0),
            AltitudeEnvelope(),
        )
        self.assertTrue(fix.altitude_clamped)
        self.assertTrue(fix.low_confidence)
        self.assertAlmostEqual(fix.d, -0.8, places=9)

    def test_vehicle_pitch_is_applied_to_the_localized_gate(self):
        level = localize_gate(
            detection(forward=3.0),
            VehicleState(n=0.0, e=0.0, d=-1.5, yaw_rad=0.0),
            AltitudeEnvelope(),
        )
        pitched = localize_gate(
            detection(forward=3.0),
            VehicleState(n=0.0, e=0.0, d=-1.5, yaw_rad=0.0, pitch_rad=deg_to_rad(10.0)),
            AltitudeEnvelope(),
        )
        # Nose up by 10 degrees puts the same detection 0.52 m higher.
        self.assertAlmostEqual(level.d - pitched.d, 0.52094453, places=6)


if __name__ == "__main__":
    unittest.main()
