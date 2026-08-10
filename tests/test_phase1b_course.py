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

from navigation.navigation import VehicleState  # noqa: E402
from navigation.missions import phase1b_course  # noqa: E402
from navigation.missions.contracts import (  # noqa: E402
    AltitudeEnvelope,
    GateLegConfig,
    GateOutcome,
    GateResult,
)
from navigation.missions.pad import PadConfig, PadResult  # noqa: E402
from navigation.missions.phase1b_course import CourseConfig  # noqa: E402


class FakeNavigation:
    """Duck-typed stand-in; records what the course loop asked it to do."""

    def __init__(self):
        self.running = True
        self.altitude_envelope = AltitudeEnvelope()
        self.landings = 0
        self.safe_shutdowns = []
        self.holds = []

    def get_vehicle_snapshot(self):
        return VehicleState()

    def hold_position(self, *, yaw_rad=None, label="hold"):
        self.holds.append(label)

    def land(self):
        self.landings += 1
        return True

    def safe_shutdown(self, reason, *, hold_s=3.0):
        self.safe_shutdowns.append(reason)
        self.landings += 1
        return True


def outcome(result, gate_index, reason="scripted"):
    return GateOutcome(result=result, gate_index=gate_index, reason=reason)


class CourseConfigTests(unittest.TestCase):
    def test_a_course_needs_at_least_one_gate(self):
        with self.assertRaises(ValueError):
            CourseConfig(gate_count=0)

    def test_negative_retries_are_rejected(self):
        with self.assertRaises(ValueError):
            CourseConfig(gate_retries=-1)

    def test_the_give_up_threshold_must_allow_at_least_one_failure(self):
        with self.assertRaises(ValueError):
            CourseConfig(max_consecutive_failures=0)

    def test_the_course_timeout_must_be_positive(self):
        with self.assertRaises(ValueError):
            CourseConfig(course_timeout_s=0.0)

    def test_the_gate_count_is_configurable_and_not_hardcoded_to_eight(self):
        self.assertEqual(CourseConfig(gate_count=5).gate_count, 5)


class CourseLoopTests(unittest.TestCase):
    """Exercises the loop policy with the per-gate work scripted out."""

    def setUp(self):
        self.nav = FakeNavigation()
        self.leg_cfg = GateLegConfig()
        self.pad_cfg = PadConfig()

    def _run(self, scripted, course_cfg):
        calls = []

        def fake_attempt(nav, mission, cfg, *, gate_index=0, log=print):
            calls.append(gate_index)
            return scripted.pop(0)

        with patch.object(phase1b_course, "run_pad_sequence",
                          return_value=PadResult(True, "ready", AltitudeEnvelope())), \
             patch.object(phase1b_course, "approach_and_cross_one_gate", fake_attempt), \
             patch.object(phase1b_course, "_settle", lambda nav, duration_s: None):
            code = phase1b_course.run(
                self.nav, object(), self.leg_cfg, self.pad_cfg, course_cfg
            )
        return code, calls

    def test_a_clean_course_crosses_every_gate_and_lands_once(self):
        scripted = [outcome(GateResult.CROSSED, i) for i in range(3)]
        code, calls = self._run(scripted, CourseConfig(gate_count=3, gate_retries=0))

        self.assertEqual(code, 0)
        self.assertEqual(calls, [0, 1, 2])
        self.assertEqual(self.nav.landings, 1)
        self.assertEqual(self.nav.safe_shutdowns, [])

    def test_the_course_stops_and_settles_between_gates(self):
        # Phase 1B is deliberately unoptimized: a full stop between every gate.
        # Phase 2 replaces exactly this with a planned transition.
        scripted = [outcome(GateResult.CROSSED, i) for i in range(2)]
        self._run(scripted, CourseConfig(gate_count=2, gate_retries=0))
        settles = [label for label in self.nav.holds if "settled after gate" in label]
        self.assertEqual(len(settles), 2)

    def test_a_failing_gate_is_retried_before_counting_as_a_failure(self):
        scripted = [
            outcome(GateResult.LOST, 0),
            outcome(GateResult.CROSSED, 0),
            outcome(GateResult.CROSSED, 1),
        ]
        code, calls = self._run(scripted, CourseConfig(gate_count=2, gate_retries=1))

        self.assertEqual(code, 0)
        self.assertEqual(calls, [0, 0, 1])
        self.assertEqual(self.nav.landings, 1)

    def test_consecutive_failures_end_the_course_and_still_land(self):
        # multi_stage_gate.py landed only on completing all eight gates; every
        # other exit left the vehicle in offboard. Every exit here lands.
        scripted = [outcome(GateResult.NO_COMMIT, 0), outcome(GateResult.NO_COMMIT, 1)]
        code, calls = self._run(
            scripted,
            CourseConfig(gate_count=4, gate_retries=0, max_consecutive_failures=2),
        )

        self.assertEqual(code, 1)
        self.assertEqual(calls, [0, 1])
        self.assertEqual(self.nav.landings, 1)
        self.assertEqual(len(self.nav.safe_shutdowns), 1)
        self.assertIn("consecutive gate failures", self.nav.safe_shutdowns[0])

    def test_the_failure_counter_resets_after_a_successful_gate(self):
        scripted = [
            outcome(GateResult.NO_COMMIT, 0),
            outcome(GateResult.CROSSED, 1),
            outcome(GateResult.NO_COMMIT, 2),
            outcome(GateResult.CROSSED, 3),
        ]
        code, calls = self._run(
            scripted,
            CourseConfig(gate_count=4, gate_retries=0, max_consecutive_failures=2),
        )

        # Two failures happened, but never two in a row, so the course finished.
        self.assertEqual(calls, [0, 1, 2, 3])
        self.assertEqual(self.nav.safe_shutdowns, [])
        # Not every gate was crossed, so this is still not a clean run.
        self.assertEqual(code, 1)
        self.assertEqual(self.nav.landings, 1)

    def test_an_abort_ends_the_course_immediately_without_retrying(self):
        # ABORTED means the vehicle or link is unhealthy; retrying would be
        # commanding motion in a state we already decided we do not trust.
        scripted = [outcome(GateResult.ABORTED, 0, "telemetry stale")]
        code, calls = self._run(
            scripted, CourseConfig(gate_count=3, gate_retries=2)
        )

        self.assertEqual(code, 1)
        self.assertEqual(calls, [0])
        self.assertEqual(self.nav.landings, 1)
        self.assertIn("telemetry stale", self.nav.safe_shutdowns[0])

    def test_a_failed_pad_sequence_lands_without_attempting_any_gate(self):
        with patch.object(phase1b_course, "run_pad_sequence",
                          return_value=PadResult(False, "no fresh detections")):
            code = phase1b_course.run(
                self.nav, object(), self.leg_cfg, self.pad_cfg, CourseConfig()
            )

        self.assertEqual(code, 1)
        self.assertEqual(self.nav.landings, 1)
        self.assertIn("no fresh detections", self.nav.safe_shutdowns[0])


class CourseArgumentTests(unittest.TestCase):
    def test_the_gate_count_comes_from_the_command_line(self):
        args = phase1b_course.parse_args(["--gates", "6", "--offline"])
        self.assertEqual(args.gates, 6)
        self.assertTrue(args.offline)

    def test_offline_and_dry_run_are_mutually_exclusive(self):
        args = phase1b_course.parse_args(["--offline", "--dry-run"])
        self.assertFalse(phase1b_course.check_mode_flags(args))


if __name__ == "__main__":
    unittest.main()
