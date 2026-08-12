"""Tests for ``pad`` -- everything between "python -m ..." and "hovering".

The pad sequence is the part of the mission where a mistake is still cheap, and
the part where the ordering constraints are least obvious from reading any one
line.  Three of them are load-bearing and are asserted directly:

* a ``udpin:`` pymavlink connection cannot transmit until it has received a
  packet, so the link comes first;
* PX4 refuses OFFBOARD unless setpoints are already streaming, so the stream
  starts before the mode request;
* PX4 auto-disarms a vehicle that is armed but has not taken off, so arming is
  the operator's last action, not a countdown -- and lift-off must not be the
  same event as arming.
"""

import unittest
from unittest.mock import patch

from support import FakeClock, FakeNav, RecordingLog, detection, state  # noqa: E402

from navigation.navigation import Setpoint, SetpointKind, type_mask_for  # noqa: E402
from navigation.missions import pad  # noqa: E402
from navigation.missions.contracts import AltitudeEnvelope  # noqa: E402
from navigation.missions.pad import PadConfig, assert_not_climbing  # noqa: E402


class PadNav(FakeNav):
    """FakeNav plus the pad-sequence surface, recording the order of events."""

    def __init__(self, clock=None, **overrides):
        super().__init__(clock)
        self.events = []
        self.started = False
        self.streaming = False
        self.offboard_ok = True
        self.arm_ok = True
        self.start_raises = None
        self.position_hz = 30.0
        self.attitude_hz = 50.0
        self.intervals_requested = 0
        self.climb_rate_m_per_read = 0.25
        self.climbing = True
        self.set_state(d=0.0, landed_state=1, armed=False, custom_mode=0)
        for name, value in overrides.items():
            setattr(self, name, value)

    def start(self, *, require_offboard=True, heartbeat_timeout_s=30.0):
        if self.start_raises is not None:
            raise self.start_raises
        self.events.append("start")
        self.started = True
        # The real pad sequence starts the stream itself via nav.start(); the
        # priming setpoint is finite zeros, never a climb.
        self.streaming = True
        self.set_setpoint(
            Setpoint(kind=SetpointKind.VELOCITY_YAW, label="prime")
        )

    def request_message_intervals(self, intervals_hz=None):
        self.events.append("intervals")
        self.intervals_requested += 1

    def measure_telemetry_rate(self, duration_s=3.0):
        self.events.append("measure")
        return self.position_hz, self.attitude_hz

    def wait_for_armed(self, *, timeout_s=300.0, poll_s=0.2):
        self.events.append("wait_for_armed")
        if self.arm_ok:
            self.set_state(d=self._state.d, armed=True, landed_state=1)
        return self.arm_ok

    def request_offboard(self, *, timeout_s=10.0, attempts=6):
        self.events.append("request_offboard")
        if not self.streaming:
            raise RuntimeError("start_streaming() must run before requesting OFFBOARD")
        return self.offboard_ok

    def get_vehicle_snapshot(self):
        # Climb toward the latched altitude target so `takeoff` can complete
        # against telemetry rather than against a fixed sleep.
        latest = self.current_setpoint()
        if (
            self.climbing
            and latest.kind is SetpointKind.BRAKE_HOLD_ALT
            and latest.label == "takeoff"
        ):
            error = latest.d - self._state.d
            step = max(-self.climb_rate_m_per_read,
                       min(self.climb_rate_m_per_read, error))
            self.set_state(
                d=self._state.d + step,
                armed=self._state.armed,
                landed_state=2 if self._state.d + step < -0.25 else 1,
            )
        return self._state


class FrozenFeedMission:
    """A vision feed that is alive, fresh, and saying the same thing forever.

    ``blind=True`` publishes the all-999 no-detection sentinel: a perfectly
    healthy publisher that simply cannot see a gate.  ``gate_above=True``
    publishes the geometry that actually occurs on the pad -- the drone is on
    the floor and the gate hangs in the air, so ``down`` is strongly negative.
    """

    def __init__(self, clock, *, frozen=False, packets=True, fresh=True,
                 blind=False, gate_above=False):
        self.clock = clock
        self.feed_frozen = frozen
        self.identical_frame_count = 15 if frozen else 0
        self.detection_max_age_s = 0.4
        self.udp_ip = "127.0.0.1"
        self.udp_port = 5050
        self.packets = packets
        self.fresh = fresh
        self.blind = blind
        self.gate_above = gate_above

    def get_latest_detection_snapshot(self):
        if not self.packets:
            return None
        age = 0.0 if self.fresh else 10.0
        stamp = self.clock.time() - age
        if self.blind:
            # Exactly what vision/opencv_processing.py pads absent gates with.
            return detection(
                timestamp=stamp, dist=999.0, forward=999.0, right=999.0,
                down=999.0, roll=999.0, pitch=999.0, yaw_deg=999.0,
            )
        if self.gate_above:
            # Field values, 2026-08-12: gate 4.91 m out and 1.58 m ABOVE the
            # camera while the aircraft sits on the floor.
            return detection(
                timestamp=stamp, dist=5.14, forward=4.91, right=0.30, down=-1.58,
            )
        return detection(timestamp=stamp)

    def start(self):
        pass

    def stop(self):
        pass


def run_pad(nav, mission, cfg, clock, confirm_result=True):
    log = RecordingLog()
    with patch.object(pad, "time", clock), \
         patch.object(pad, "confirm", lambda *a, **k: confirm_result):
        result = pad.run_pad_sequence(
            nav, mission, cfg, banner_rows=(), title="TEST", log=log
        )
    return result, log


def pad_config(**overrides) -> PadConfig:
    fields = dict(
        altitude=AltitudeEnvelope(),
        detection_wait_s=6.0,
        detection_observe_s=1.0,
        takeoff_timeout_s=20.0,
    )
    fields.update(overrides)
    return PadConfig(**fields)


class ClimbInvariantTests(unittest.TestCase):
    """The invariant that keeps the vehicle on the pad until it is asked to fly."""

    def test_a_climbing_setpoint_before_offboard_is_refused(self):
        # PX4 latches want_takeoff the instant the vehicle is armed and sees a
        # fresh setpoint with negative vz. If the stream already carried the
        # climb while the operator was being asked to arm, arming and lift-off
        # would be the same event.
        nav = FakeNav()
        nav.set_setpoint(Setpoint(kind=SetpointKind.VELOCITY_YAW, vd=-0.5))

        with self.assertRaises(RuntimeError) as caught:
            assert_not_climbing(nav)
        self.assertIn("before OFFBOARD was verified", str(caught.exception))

    def test_finite_zeros_and_altitude_holds_are_allowed(self):
        nav = FakeNav()
        for setpoint in (
            Setpoint(kind=SetpointKind.VELOCITY_YAW, vd=0.0),
            Setpoint(kind=SetpointKind.BRAKE_HOLD_ALT, d=-5.0),
        ):
            nav.set_setpoint(setpoint)
            assert_not_climbing(nav)  # must not raise


class PadSequenceOrderTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.nav = PadNav(self.clock)
        self.mission = FrozenFeedMission(self.clock)

    def test_a_clean_pad_run_reaches_altitude(self):
        result, log = run_pad(self.nav, self.mission, pad_config(), self.clock)

        self.assertTrue(result.ok, result.reason)
        self.assertIsNotNone(result.envelope)
        self.assertTrue(log.contains("Reached"))

    def test_arming_happens_before_offboard_is_requested(self):
        # Arm is the operator's commit action and their abort switch, so it is
        # the last gate before any motion.
        run_pad(self.nav, self.mission, pad_config(), self.clock)

        self.assertLess(
            self.nav.events.index("wait_for_armed"),
            self.nav.events.index("request_offboard"),
        )

    def test_the_stream_is_running_before_offboard_is_requested(self):
        # request_offboard raises if it is not; this asserts the sequence never
        # gets there with the stream down.
        result, _ = run_pad(self.nav, self.mission, pad_config(), self.clock)
        self.assertTrue(result.ok, result.reason)
        self.assertTrue(self.nav.streaming)

    def test_the_altitude_envelope_is_latched_to_the_pad_not_to_zero(self):
        # The EKF origin is wherever PX4 decided it was; assuming d_takeoff = 0
        # puts the whole envelope in the wrong place.
        self.nav.set_state(d=-0.42, armed=False, landed_state=1)
        result, _ = run_pad(self.nav, self.mission, pad_config(), self.clock)

        self.assertAlmostEqual(result.envelope.d_takeoff, -0.42, places=6)
        self.assertAlmostEqual(self.nav.altitude_envelope.d_takeoff, -0.42, places=6)


class PadRefusalTests(unittest.TestCase):
    """Every threshold here is a refusal to fly, not a warning."""

    def setUp(self):
        self.clock = FakeClock()
        self.nav = PadNav(self.clock)
        self.mission = FrozenFeedMission(self.clock)

    def test_a_failed_controller_start_is_reported_not_raised(self):
        self.nav.start_raises = RuntimeError("no heartbeat")
        result, _ = run_pad(self.nav, self.mission, pad_config(), self.clock)

        self.assertFalse(result.ok)
        self.assertIn("could not start controller", result.reason)

    def test_slow_telemetry_refuses_takeoff(self):
        # A 10 Hz position stream cannot serve a 10 Hz control loop.
        self.nav.position_hz = 4.0
        result, _ = run_pad(self.nav, self.mission, pad_config(), self.clock)

        self.assertFalse(result.ok)
        self.assertIn("telemetry too slow", result.reason)
        self.assertNotIn("wait_for_armed", self.nav.events)

    def test_the_rate_check_can_be_skipped_deliberately(self):
        self.nav.position_hz = 0.0
        self.nav.attitude_hz = 0.0
        result, log = run_pad(
            self.nav, self.mission, pad_config(skip_telemetry_rate_check=True), self.clock
        )

        self.assertTrue(result.ok, result.reason)
        self.assertTrue(log.contains("SKIPPED"))

    def test_no_vision_packets_refuses_takeoff(self):
        result, _ = run_pad(
            self.nav,
            FrozenFeedMission(self.clock, packets=False),
            pad_config(detection_wait_s=0.5),
            self.clock,
        )

        self.assertFalse(result.ok)
        self.assertIn("no fresh gate detections", result.reason)

    def test_a_frozen_vision_feed_refuses_takeoff(self):
        """REGRESSION -- PROJECT_STATE gap 1: frozen-feed detection was absent.

        Signals 1 (no datagram) and 2 (the all-999 row) both worked. This is
        signal 3: the publisher is alive, the packets are fresh, and the pose
        never changes -- a stalled camera or a wedged pipeline. It is the only
        one of the three that looks like health, and without it the drone takes
        off and flies a frozen gate position. `udp_injector --case frozen`
        exists specifically to produce this.
        """
        result, log = run_pad(
            self.nav,
            FrozenFeedMission(self.clock, frozen=True),
            pad_config(detection_wait_s=1.0),
            self.clock,
        )

        self.assertFalse(result.ok)
        self.assertTrue(log.contains("FROZEN"))
        self.assertNotIn("wait_for_armed", self.nav.events)

    def test_no_detections_can_be_overridden_explicitly(self):
        result, log = run_pad(
            self.nav,
            FrozenFeedMission(self.clock, packets=False),
            pad_config(detection_wait_s=0.5, require_detections=False),
            self.clock,
        )

        self.assertTrue(result.ok, result.reason)
        self.assertTrue(log.contains("--allow-no-detections"))

    def test_an_operator_abort_at_the_prompt_never_arms(self):
        result, _ = run_pad(
            self.nav, self.mission, pad_config(), self.clock, confirm_result=False
        )

        self.assertFalse(result.ok)
        self.assertIn("operator aborted", result.reason)
        self.assertNotIn("wait_for_armed", self.nav.events)

    def test_never_arming_never_requests_offboard(self):
        self.nav.arm_ok = False
        result, _ = run_pad(self.nav, self.mission, pad_config(), self.clock)

        self.assertFalse(result.ok)
        self.assertIn("never armed", result.reason)
        self.assertNotIn("request_offboard", self.nav.events)

    def test_unconfirmed_offboard_never_takes_off(self):
        self.nav.offboard_ok = False
        result, _ = run_pad(self.nav, self.mission, pad_config(), self.clock)

        self.assertFalse(result.ok)
        self.assertIn("OFFBOARD was not confirmed", result.reason)
        self.assertFalse(
            any(sp.label == "takeoff" for sp in self.nav.setpoints)
        )


class TakeoffTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.nav = PadNav(self.clock)
        self.envelope = AltitudeEnvelope(takeoff_alt_m=1.5, d_takeoff=0.0)

    def _takeoff(self, cfg=None):
        log = RecordingLog()
        with patch.object(pad, "time", self.clock):
            ok = pad.takeoff(self.nav, self.envelope, cfg or pad_config(), log=log)
        return ok, log

    def test_takeoff_climbs_to_the_envelope_altitude(self):
        ok, log = self._takeoff()

        self.assertTrue(ok)
        self.assertTrue(log.contains("Reached"))
        self.assertLessEqual(self.nav.get_vehicle_snapshot().d, -1.35)

    def test_takeoff_uses_a_brake_and_hold_setpoint_never_a_position_mask(self):
        """REGRESSION -- a full position mask can translate at MPC_XY_VEL_MAX.

        A POSITION_YAW setpoint carries an x/y target. If that is anything but
        the position captured at the mode-switch instant, PX4 flies toward it at
        up to MPC_XY_VEL_MAX -- 12 m/s by default -- while the vehicle is 30 cm
        off the ground. Commanding zero horizontal *velocity* cannot run away,
        while still letting PX4 close the altitude loop.
        """
        self._takeoff()

        takeoff_setpoints = [sp for sp in self.nav.setpoints if sp.label == "takeoff"]
        self.assertTrue(takeoff_setpoints)
        for setpoint in takeoff_setpoints:
            self.assertIs(setpoint.kind, SetpointKind.BRAKE_HOLD_ALT)
            self.assertEqual(type_mask_for(setpoint.kind), 2531)
            self.assertEqual(setpoint.vn, 0.0)
            self.assertEqual(setpoint.ve, 0.0)

    def test_the_takeoff_setpoint_is_not_a_climbing_velocity(self):
        # BRAKE_HOLD_ALT is a position target, so PX4's want_takeoff check on vz
        # does not apply to it -- which is why the pad can stream it safely.
        self._takeoff()
        takeoff_setpoint = next(sp for sp in self.nav.setpoints if sp.label == "takeoff")
        self.assertFalse(takeoff_setpoint.climbs())

    def test_takeoff_announces_leaving_the_ground_from_telemetry(self):
        _ok, log = self._takeoff()
        self.assertTrue(log.contains("Airborne"))

    def test_a_takeoff_that_never_climbs_times_out_and_says_where_it_got_to(self):
        self.nav.climbing = False
        ok, log = self._takeoff(pad_config(takeoff_timeout_s=2.0))

        self.assertFalse(ok)
        self.assertTrue(log.contains("Takeoff timed out"))

    def test_arrival_is_judged_against_telemetry_not_a_fixed_sleep(self):
        # PX4 spends ~1 s spooling up and ~3 s ramping thrust, so any hard-coded
        # climb duration is guesswork. A slow climb must still succeed.
        self.nav.climb_rate_m_per_read = 0.02
        ok, _ = self._takeoff(pad_config(takeoff_timeout_s=60.0))
        self.assertTrue(ok)


class DetectionWaitTests(unittest.TestCase):
    """`wait_for_detections` distinguishes four states, not two."""

    def setUp(self):
        self.clock = FakeClock()
        self.nav = FakeNav(self.clock)

    def _wait(self, mission, wait_s=2.0, min_observe_s=0.5):
        log = RecordingLog()
        with patch.object(pad, "time", self.clock):
            ok = pad.wait_for_detections(
                mission, self.nav, wait_s, log=log, min_observe_s=min_observe_s
            )
        return ok, log

    def test_a_live_changing_feed_with_a_real_gate_is_accepted(self):
        ok, log = self._wait(FrozenFeedMission(self.clock))
        self.assertTrue(ok)
        self.assertTrue(log.contains("REAL gate"))

    def test_silence_is_named_as_silence(self):
        ok, log = self._wait(FrozenFeedMission(self.clock, packets=False))
        self.assertFalse(ok)
        self.assertTrue(log.contains("No vision packets"))

    def test_stale_packets_are_named_as_stale(self):
        ok, log = self._wait(FrozenFeedMission(self.clock, fresh=False))
        self.assertFalse(ok)
        self.assertTrue(log.contains("stale or a no-detection row"))

    def test_a_frozen_feed_is_named_as_frozen_not_as_health(self):
        ok, log = self._wait(FrozenFeedMission(self.clock, frozen=True))
        self.assertFalse(ok)
        self.assertTrue(log.contains("FROZEN"))
        self.assertFalse(log.contains("REAL gate"))

    def test_a_blind_feed_of_999_sentinels_is_refused(self):
        """REGRESSION -- HIGH, field log 2026-08-12.

        The pad printed "Vision feed alive and CHANGING over 1.0s:
        fwd=+999.00m right=+999.00m down=+999.00m dist=999.00m" and would have
        taken off. The publisher was healthy and fresh; it simply could not see
        a gate. wait_for_detections checked `det is not None` and the timestamp
        age but never `is_valid_detection`, and the frozen-feed detector
        deliberately ignores all-999 payloads -- so both guards missed the same
        case at the same time and `require_detections` did nothing.
        """
        ok, log = self._wait(FrozenFeedMission(self.clock, blind=True))

        self.assertFalse(ok)
        self.assertFalse(log.contains("REAL gate"))
        self.assertTrue(log.contains("NONE carries a gate"))
        # Named as "blind", not as "stale" -- they send you to check different
        # things (aim/lighting/range vs. a dead or lagging publisher).
        self.assertFalse(log.contains("stale or a no-detection row"))

    def test_a_gate_above_the_drone_on_the_pad_is_accepted(self):
        """The drone starts on the floor and the gate hangs in the air.

        A pad-time detection therefore has a large NEGATIVE `down`. Validity is
        judged on `dist` alone; nothing here may reject a gate for being high,
        or the aircraft could never be cleared to fly a real course.
        """
        ok, log = self._wait(FrozenFeedMission(self.clock, gate_above=True))

        self.assertTrue(ok, log.text)
        self.assertTrue(log.contains("REAL gate"))
        self.assertTrue(log.contains("down=-1.58m"))
        self.assertTrue(log.contains("gate ABOVE the camera"))

    def test_a_blind_feed_refuses_takeoff_at_the_pad(self):
        # End to end, not just the helper: require_detections must actually
        # stop the sequence before the operator is asked to arm.
        nav = PadNav(self.clock)
        result, log = run_pad(
            nav,
            FrozenFeedMission(self.clock, blind=True),
            pad_config(detection_wait_s=1.0),
            self.clock,
        )

        self.assertFalse(result.ok)
        self.assertIn("no fresh gate detections", result.reason)
        self.assertNotIn("wait_for_armed", nav.events)


class BannerTests(unittest.TestCase):
    def test_the_banner_prints_every_row_and_every_precondition(self):
        log = RecordingLog()
        pad.print_banner(
            "TITLE",
            (("commit distance", "1.00 m"), ("approach step", "0.25 m")),
            ("props fitted", "area clear"),
            log=log,
        )

        self.assertTrue(log.contains("TITLE"))
        self.assertTrue(log.contains("commit distance"))
        self.assertTrue(log.contains("[ ] props fitted"))
        self.assertTrue(log.contains("[ ] area clear"))


if __name__ == "__main__":
    unittest.main()
