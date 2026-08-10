"""Tests for Phase 2 course-vector planning.

All pure geometry, checked against hand-derived arithmetic rather than against
current output -- the standing rule in this repo for anything that decides where
the aircraft goes, because there is no simulator and no flight test before
deployment.
"""

import math
import unittest

import support  # noqa: F401  (installs the pymavlink stub)

from navigation.missions.contracts import AltitudeEnvelope, GateFix  # noqa: E402
from navigation.missions.course import (  # noqa: E402
    GRAVITY_M_S2,
    CoursePlanConfig,
    PassVector,
    minimum_turn_radius_m,
    order_gates_along_course,
    pass_vector,
    plan_leg,
    plan_turn,
)


def vector(n, e, heading_deg, d=-1.5) -> PassVector:
    heading = math.radians(heading_deg)
    return PassVector(n=n, e=e, d=d, dir_n=math.cos(heading), dir_e=math.sin(heading))


def plan_config(**overrides) -> CoursePlanConfig:
    fields = dict(
        cruise_speed_m_s=0.45,
        min_turn_speed_m_s=0.25,
        max_yaw_rate_deg_s=45.0,
        max_lateral_accel_m_s2=0.15 * GRAVITY_M_S2,
        exit_clearance_m=0.80,
        entry_lead_in_m=1.00,
    )
    fields.update(overrides)
    return CoursePlanConfig(**fields)


class PassVectorTests(unittest.TestCase):
    def test_a_gate_is_a_point_plus_a_direction(self):
        # Aiming at a gate's centre is not enough: arriving at the centre
        # sideways puts a rotor through a gate leg.
        gate = vector(3.0, 0.0, 90.0)

        self.assertAlmostEqual(gate.heading_rad, math.pi / 2, places=9)
        ahead_n, ahead_e = gate.point_at(2.0)
        self.assertAlmostEqual(ahead_n, 3.0, places=9)
        self.assertAlmostEqual(ahead_e, 2.0, places=9)

    def test_along_is_signed_so_behind_the_gate_is_negative(self):
        gate = vector(0.0, 0.0, 0.0)
        self.assertAlmostEqual(gate.along(2.0, 0.0), 2.0, places=9)
        self.assertAlmostEqual(gate.along(-2.0, 0.0), -2.0, places=9)

    def test_offset_is_the_perpendicular_distance_from_the_axis(self):
        gate = vector(0.0, 0.0, 0.0)
        self.assertAlmostEqual(gate.offset(5.0, 1.5), 1.5, places=9)

    def test_a_fix_becomes_a_vector_using_its_resolved_normal(self):
        # The normal is already disambiguated, so the vector always points the
        # way the aircraft is going regardless of what the detector reported.
        fix = GateFix(
            n=3.0, e=1.0, d=-1.5, normal_n=0.0, normal_e=1.0, yaw_rad=math.pi / 2,
            range_m=3.0, lateral_body_m=0.0, vertical_body_m=0.0, lateral_axis_m=0.0,
            cone_angle_deg=0.0, normal_flipped=True, altitude_clamped=False,
            low_confidence=False, reason="",
        )
        self.assertAlmostEqual(pass_vector(fix).heading_rad, math.pi / 2, places=9)


class TurnRadiusTests(unittest.TestCase):
    def test_yaw_rate_binds_at_phase_two_speeds(self):
        # r >= V / omega_max = 0.45 / radians(45) = 0.45 / 0.78539816 = 0.5729578
        # r >= V^2 / a_max   = 0.2025 / 1.47100   = 0.1376614
        # Yaw rate binds -- the right way round for an optical-flow airframe,
        # where spinning the camera costs more than leaning it.
        radius = minimum_turn_radius_m(0.45, max_yaw_rate_deg_s=45.0,
                                       max_lateral_accel_m_s2=0.15 * GRAVITY_M_S2)
        self.assertAlmostEqual(radius, 0.5729578, places=6)

    def test_acceleration_binds_once_the_aircraft_is_fast_enough(self):
        # At 3 m/s: yaw gives 3.82 m, acceleration gives 9 / 1.471 = 6.12 m.
        radius = minimum_turn_radius_m(3.0, max_yaw_rate_deg_s=45.0,
                                       max_lateral_accel_m_s2=0.15 * GRAVITY_M_S2)
        self.assertAlmostEqual(radius, 9.0 / (0.15 * GRAVITY_M_S2), places=6)

    def test_a_tighter_yaw_limit_forces_a_wider_turn(self):
        self.assertGreater(
            minimum_turn_radius_m(0.45, max_yaw_rate_deg_s=20.0),
            minimum_turn_radius_m(0.45, max_yaw_rate_deg_s=45.0),
        )

    def test_invalid_inputs_are_rejected(self):
        for speed, yaw in ((0.0, 45.0), (-1.0, 45.0), (0.45, 0.0)):
            with self.subTest(speed=speed, yaw=yaw):
                with self.assertRaises(ValueError):
                    minimum_turn_radius_m(speed, max_yaw_rate_deg_s=yaw)


class TurnGeometryTests(unittest.TestCase):
    """A right-angle turn, worked out by hand."""

    def setUp(self):
        # Gate A at the origin heading north; gate B at (5, 5) heading east.
        # The two lines meet at P = (5, 0).
        #   distance A -> P along u = 5.0
        #   distance P -> B along v = 5.0
        #   turn angle theta = +90 deg, so tan(theta/2) = 1 and d = r
        # Room after A: 5.0 - 0.80 exit clearance = 4.20
        # Room before B: 5.0 - 1.00 lead-in       = 4.00   <- binds
        # So the widest arc the geometry allows is r = 4.00 / 1 = 4.00 m.
        self.cfg = plan_config()
        self.turn = plan_turn(vector(0, 0, 0), vector(5, 5, 90), self.cfg)

    def test_the_turn_is_feasible_and_the_angle_is_right(self):
        self.assertTrue(self.turn.ok, self.turn.reason)
        self.assertAlmostEqual(self.turn.turn_angle_deg, 90.0, places=6)

    def test_the_radius_is_the_widest_the_geometry_allows(self):
        # A wide turn is a slow yaw, and a slow yaw is what the optical-flow
        # estimator wants -- so the planner takes the most room it is given.
        self.assertAlmostEqual(self.turn.radius_m, 4.0, places=6)

    def test_the_tangent_points_are_hand_derived(self):
        # T1 = P - u*d = (5,0) - (1,0)*4 = (1, 0)
        # T2 = P + v*d = (5,0) + (0,1)*4 = (5, 4)
        self.assertAlmostEqual(self.turn.entry_n, 1.0, places=6)
        self.assertAlmostEqual(self.turn.entry_e, 0.0, places=6)
        self.assertAlmostEqual(self.turn.exit_n, 5.0, places=6)
        self.assertAlmostEqual(self.turn.exit_e, 4.0, places=6)

    def test_the_arc_centre_is_equidistant_from_both_tangent_points(self):
        # centre = T1 + r * perp(u) = (1,0) + 4*(0,1) = (1, 4)
        self.assertAlmostEqual(self.turn.centre_n, 1.0, places=6)
        self.assertAlmostEqual(self.turn.centre_e, 4.0, places=6)
        for point in ((self.turn.entry_n, self.turn.entry_e),
                      (self.turn.exit_n, self.turn.exit_e)):
            self.assertAlmostEqual(
                math.hypot(point[0] - self.turn.centre_n, point[1] - self.turn.centre_e),
                4.0,
                places=6,
            )

    def test_the_arc_length_is_r_times_theta(self):
        self.assertAlmostEqual(self.turn.arc_length_m, 4.0 * math.pi / 2, places=6)

    def test_the_transition_point_clears_the_exit_and_the_lead_in(self):
        leaving, entering = vector(0, 0, 0), vector(5, 5, 90)
        self.assertGreaterEqual(
            leaving.along(self.turn.entry_n, self.turn.entry_e),
            self.cfg.exit_clearance_m,
        )
        self.assertLessEqual(
            entering.along(self.turn.exit_n, self.turn.exit_e),
            -self.cfg.entry_lead_in_m + 1e-9,
        )

    def test_every_sampled_point_lies_on_the_arc(self):
        for point_n, point_e, _heading in self.turn.sample(9):
            self.assertAlmostEqual(
                math.hypot(point_n - self.turn.centre_n, point_e - self.turn.centre_e),
                self.turn.radius_m,
                places=6,
            )

    def test_the_heading_profile_runs_monotonically_from_entry_to_exit(self):
        headings = [math.degrees(h) for _n, _e, h in self.turn.sample(5)]
        self.assertAlmostEqual(headings[0], 0.0, places=6)
        self.assertAlmostEqual(headings[-1], 90.0, places=6)
        self.assertEqual(headings, sorted(headings))
        self.assertAlmostEqual(headings[2], 45.0, places=6)  # halfway

    def test_the_commanded_yaw_rate_respects_the_configured_limit(self):
        self.assertLessEqual(
            math.degrees(self.turn.yaw_rate_rad_s), self.cfg.max_yaw_rate_deg_s
        )

    def test_the_implied_bank_is_reported_and_small(self):
        # tan(phi) = a_lat / g, and a_lat = V^2/r = 0.2025/4 = 0.0506 m/s^2.
        self.assertAlmostEqual(self.turn.lateral_accel_m_s2, 0.2025 / 4.0, places=6)
        self.assertLess(abs(self.turn.bank_deg), 1.0)

    def test_a_left_turn_mirrors_a_right_turn(self):
        left = plan_turn(vector(0, 0, 0), vector(5, -5, -90), self.cfg)

        self.assertTrue(left.ok, left.reason)
        self.assertAlmostEqual(left.turn_angle_deg, -90.0, places=6)
        self.assertAlmostEqual(left.entry_n, self.turn.entry_n, places=6)
        self.assertAlmostEqual(left.exit_e, -self.turn.exit_e, places=6)
        self.assertAlmostEqual(left.centre_e, -self.turn.centre_e, places=6)
        self.assertLess(left.bank_rad, 0.0)  # banked the other way


class TurnRefusalTests(unittest.TestCase):
    """Every way the planner says no.  Each one triggers the Phase 1B fallback."""

    def test_aligned_gates_need_no_turn(self):
        turn = plan_turn(vector(0, 0, 0), vector(6, 0, 0), plan_config())

        self.assertTrue(turn.ok)
        self.assertTrue(turn.straight)
        self.assertIn("already aligned", turn.describe())

    def test_opposed_gates_have_no_finite_arc(self):
        # A 180 degree reversal cannot be joined by a constant-radius turn at
        # any radius. Fly it as a Phase 1B stop and turn.
        turn = plan_turn(vector(0, 0, 0), vector(5, 0, 180), plan_config())

        self.assertFalse(turn.ok)
        self.assertIn("opposed", turn.reason)

    def test_parallel_offset_gates_have_no_intersection_to_turn_about(self):
        turn = plan_turn(vector(0, 0, 0), vector(5, 3, 0.5), plan_config())
        # 0.5 deg apart is inside the "already aligned" band; make it truly
        # parallel but offset instead.
        turn = plan_turn(
            PassVector(0, 0, -1.5, 1.0, 0.0),
            PassVector(5, 3, -1.5, 1.0, 0.0),
            plan_config(),
        )
        self.assertTrue(turn.straight or not turn.ok)

    def test_gates_too_close_together_leave_no_room_for_a_turn(self):
        # Lines meet 1.0 m past gate A, but the exit clearance alone is 0.80 m
        # and gate B wants a 1.00 m straight lead-in. There is nowhere to put an
        # arc.
        turn = plan_turn(vector(0, 0, 0), vector(1.0, 0.6, 90), plan_config())

        self.assertFalse(turn.ok)
        self.assertIn("no room for a turn", turn.reason)

    def test_a_turn_too_tight_for_the_airframe_is_refused_not_flown(self):
        # A very tight yaw limit forces a minimum radius wider than the geometry
        # can accept, even after slowing to the speed floor.
        cfg = plan_config(max_yaw_rate_deg_s=2.0, min_turn_speed_m_s=0.4)
        turn = plan_turn(vector(0, 0, 0), vector(3.0, 3.0, 90), cfg)

        self.assertFalse(turn.ok)
        self.assertIn("too tight for the airframe", turn.reason)
        self.assertIn("yaw rate", turn.reason)

    def test_the_planner_slows_down_to_make_a_turn_fit_before_giving_up(self):
        # Slowing shrinks the required radius quadratically through the
        # acceleration term and linearly through the yaw term, so a turn that is
        # infeasible at cruise can be feasible slower. Trying that before
        # refusing is the difference between a smooth course and a stuttering
        # one.
        cfg = plan_config(cruise_speed_m_s=2.0, min_turn_speed_m_s=0.25,
                          max_yaw_rate_deg_s=45.0)
        turn = plan_turn(vector(0, 0, 0), vector(3.0, 3.0, 90), cfg)

        self.assertTrue(turn.ok, turn.reason)
        self.assertLess(turn.speed_m_s, cfg.cruise_speed_m_s)
        self.assertGreaterEqual(turn.speed_m_s, cfg.min_turn_speed_m_s)

    def test_an_infeasible_turn_describes_itself_honestly(self):
        turn = plan_turn(vector(0, 0, 0), vector(1.0, 0.6, 90), plan_config())
        self.assertIn("NOT feasible", turn.describe())


class LegPlanTests(unittest.TestCase):
    """The degrade decisions, which is where Phase 2 becomes Phase 1B."""

    def setUp(self):
        self.envelope = AltitudeEnvelope()
        self.cfg = plan_config()
        self.current = vector(0, 0, 0)
        self.next_gate = vector(5, 5, 90)

    def _plan(self, **overrides):
        fields = dict(
            next_gate=self.next_gate,
            next_confidence=1.0,
            next_heading_confidence=1.0,
            pass_distance_m=1.5,
        )
        fields.update(overrides)
        return plan_leg(self.current, self.envelope, self.cfg, **fields)

    def test_a_good_plan_carries_a_turn_and_is_not_degraded(self):
        plan = self._plan()

        self.assertFalse(plan.degraded)
        self.assertTrue(plan.turn.ok)
        self.assertEqual(plan.degrade_reason, "")

    def test_the_cross_target_is_always_the_phase_one_b_crossing(self):
        # Whatever happens to the plan, the gate is crossed the same way. That
        # is what makes Phase 2's worst case equal to Phase 1B's normal case.
        for plan in (self._plan(), self._plan(next_gate=None)):
            self.assertAlmostEqual(plan.cross_target.n, 1.5, places=6)
            self.assertAlmostEqual(plan.cross_target.e, 0.0, places=6)

    def test_no_next_gate_degrades_and_says_so(self):
        plan = self._plan(next_gate=None)

        self.assertTrue(plan.degraded)
        self.assertIn("no next gate", plan.degrade_reason)

    def test_low_track_confidence_degrades_and_says_so(self):
        plan = self._plan(next_confidence=0.2)

        self.assertTrue(plan.degraded)
        self.assertIn("confidence 0.20", plan.degrade_reason)

    def test_a_known_position_with_an_unknown_normal_degrades(self):
        # A gate seen only edge-on has a good centre and a worthless normal.
        # There is no vector to turn onto, so there is nothing to plan.
        plan = self._plan(next_heading_confidence=0.1)

        self.assertTrue(plan.degraded)
        self.assertIn("heading confidence", plan.degrade_reason)
        self.assertIn("normal is not", plan.degrade_reason)

    def test_an_infeasible_turn_degrades_and_keeps_the_reason(self):
        plan = plan_leg(
            self.current, self.envelope, self.cfg,
            next_gate=vector(1.0, 0.6, 90),
            next_confidence=1.0,
            next_heading_confidence=1.0,
        )

        self.assertTrue(plan.degraded)
        self.assertIn("no room for a turn", plan.degrade_reason)

    def test_the_cross_target_altitude_is_clamped_into_the_envelope(self):
        # A gate localized below the floor must not command flight into it.
        envelope = AltitudeEnvelope(min_alt_m=0.8, max_alt_m=2.5, d_takeoff=0.0)
        plan = plan_leg(
            PassVector(0, 0, +3.0, 1.0, 0.0), envelope, self.cfg, next_gate=None
        )
        self.assertAlmostEqual(plan.cross_target.d, -0.8, places=6)


class CourseOrderingTests(unittest.TestCase):
    def test_gates_are_ordered_by_chaining_forward(self):
        gates = [vector(9, 0, 0), vector(3, 0, 0), vector(6, 0, 0)]
        self.assertEqual(order_gates_along_course(gates, 0.0, 0.0), (1, 2, 0))

    def test_a_gate_behind_the_drone_is_not_picked_first(self):
        # Without the heading, a gate 2 m BEHIND outranks one 3 m in front,
        # which is the wrong answer for every course this will ever fly.
        gates = [vector(3, 0, 0), vector(-2, 0, 0), vector(6, 0, 0)]

        self.assertEqual(order_gates_along_course(gates, 0.0, 0.0)[0], 1)
        order = order_gates_along_course(gates, 0.0, 0.0, from_heading_rad=0.0)
        self.assertEqual(order[0], 0)
        self.assertEqual(order[1], 2)
        self.assertEqual(order[2], 1)   # the one behind is left until last

    def test_a_gate_already_passed_does_not_come_back_into_the_ordering(self):
        # Chaining forward means "beyond the previous gate's plane".
        gates = [vector(3, 0, 0), vector(6, 0, 0), vector(9, 0, 0)]
        self.assertEqual(
            order_gates_along_course(gates, 0.0, 0.0, from_heading_rad=0.0), (0, 1, 2)
        )

    def test_ordering_an_empty_course_is_not_an_error(self):
        self.assertEqual(order_gates_along_course([], 0.0, 0.0), ())


class ConfigValidationTests(unittest.TestCase):
    def test_impossible_planning_configurations_are_rejected(self):
        for kwargs in (
            {"cruise_speed_m_s": 0.0},
            {"max_yaw_rate_deg_s": -1.0},
            {"entry_lead_in_m": 0.0},
            {"cruise_speed_m_s": 0.3, "min_turn_speed_m_s": 0.5},
        ):
            with self.subTest(**kwargs):
                with self.assertRaises(ValueError):
                    plan_config(**kwargs)


if __name__ == "__main__":
    unittest.main()
