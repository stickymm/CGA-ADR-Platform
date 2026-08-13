import math
import sys
import types
import unittest
from unittest.mock import patch


fake_mavutil = types.ModuleType("pymavlink.mavutil")
fake_mavutil.mavfile = object
fake_pymavlink = types.ModuleType("pymavlink")
fake_pymavlink.mavutil = fake_mavutil
sys.modules.setdefault("pymavlink", fake_pymavlink)
sys.modules.setdefault("pymavlink.mavutil", fake_mavutil)

from navigation.navigation import (  # noqa: E402
    GateDetection,
    GateMission,
    LocalTarget,
    NavigationController,
    NO_DETECTION_DIST,
    VehicleState,
    body_to_local,
    deg_to_rad,
    is_valid_detection,
    rad_to_deg,
    wrap_pi,
)


class NavigationGeometryTests(unittest.TestCase):
    def test_degree_and_radian_conversion_round_trip(self):
        self.assertAlmostEqual(rad_to_deg(deg_to_rad(123.4)), 123.4)

    def test_wrap_pi_normalizes_angles(self):
        self.assertAlmostEqual(wrap_pi(3 * math.pi), -math.pi)
        self.assertAlmostEqual(wrap_pi(-3 * math.pi), -math.pi)
        self.assertAlmostEqual(wrap_pi(math.pi / 2), math.pi / 2)

    def test_body_to_local_rotates_horizontal_axes(self):
        north, east, down = body_to_local(2.0, 1.0, -0.5, math.pi / 2)

        self.assertAlmostEqual(north, -1.0)
        self.assertAlmostEqual(east, 2.0)
        self.assertAlmostEqual(down, -0.5)

    def test_missing_detection_sentinel_is_invalid(self):
        missing = GateDetection(
            timestamp=0.0,
            dist=NO_DETECTION_DIST,
            forward=0.0,
            right=0.0,
            down=0.0,
            roll=0.0,
            pitch=0.0,
            yaw_deg=0.0,
        )
        detected = GateDetection(
            timestamp=0.0,
            dist=3.0,
            forward=3.0,
            right=0.0,
            down=0.0,
            roll=0.0,
            pitch=0.0,
            yaw_deg=0.0,
        )

        self.assertFalse(is_valid_detection(None))
        self.assertFalse(is_valid_detection(missing))
        self.assertTrue(is_valid_detection(detected))

    def test_gate_mission_camera_offsets_are_configurable(self):
        mission = GateMission(
            cam_offset_right_m=0.0,
            cam_offset_down_m=0.0,
            cam_yaw_offset_deg=0.0,
            aim_bias_down_m=0.0,   # this test is about the MOUNT offsets
        )
        state = VehicleState(yaw_rad=0.0)
        gate = GateDetection(
            timestamp=0.0,
            dist=5.0,
            forward=5.0,
            right=0.0,
            down=0.0,
            roll=0.0,
            pitch=0.0,
            yaw_deg=0.0,
        )

        gate_n, gate_e, gate_d, gate_yaw = mission.detection_to_gate_local(
            gate,
            state,
        )

        self.assertAlmostEqual(gate_n, 5.0)
        self.assertAlmostEqual(gate_e, 0.0)
        self.assertAlmostEqual(gate_d, 0.0)
        self.assertAlmostEqual(gate_yaw, 0.0)

    def test_move_to_target_respects_per_move_speed_limit(self):
        class ScriptedNavigationController(NavigationController):
            def __init__(self):
                super().__init__("fake")
                self._running.set()
                self.snapshots = iter(
                    (
                        VehicleState(n=0.0, e=0.0, d=0.0, yaw_rad=0.0),
                        VehicleState(n=3.0, e=4.0, d=0.0, yaw_rad=0.0),
                    )
                )
                self.commands = []

            def get_vehicle_snapshot(self):
                return next(self.snapshots)

            def send_velocity_and_yaw_target(self, vn, ve, vd, yaw_rad):
                self.commands.append((vn, ve, vd, yaw_rad))

        nav = ScriptedNavigationController()
        target = LocalTarget(n=3.0, e=4.0, d=0.0, yaw_rad=0.0)

        with patch("navigation.navigation.time.sleep", return_value=None):
            reached = nav.move_to_target(
                target,
                "slow test move",
                max_speed_m_s=0.2,
                timeout_s=30.0,
            )

        self.assertTrue(reached)
        vn, ve, vd, _ = nav.commands[0]
        self.assertAlmostEqual(math.sqrt(vn**2 + ve**2 + vd**2), 0.2)


if __name__ == "__main__":
    unittest.main()
