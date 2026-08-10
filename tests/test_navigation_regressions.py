"""Regression tests for the 2026-08-10 review bugs, and for the seams added with them.

Each test names the severity and what the defect actually cost, because the
value of these is not the assertion -- it is stopping someone re-introducing the
behaviour while believing they are simplifying something.
"""

import inspect
import threading
import unittest
from unittest.mock import patch

from support import FakeClock, FakeNav, detection  # noqa: E402

from navigation import navigation as nav_module  # noqa: E402
from navigation.navigation import (  # noqa: E402
    DryRunLink,
    GateMission,
    MissionOutcome,
    NavigationController,
    NullLink,
    Setpoint,
    SetpointKind,
    VehicleState,
    ESTIMATOR_ATTITUDE,
    ESTIMATOR_POS_HORIZ_REL,
    ESTIMATOR_POS_VERT_AGL,
    ESTIMATOR_VELOCITY_HORIZ,
    body_to_local,
)


class RecordingLink:
    """A link that records instead of transmitting, and can stop a loop."""

    def __init__(self, *, suppresses=False, stop_after=None, stop_event=None):
        self.setpoints = []
        self.commands = []
        self.heartbeats = 0
        self.closed = False
        self._suppresses = suppresses
        self._stop_after = stop_after
        self._stop_event = stop_event

    @property
    def suppresses_commands(self):
        return self._suppresses

    target_system = 1
    target_component = 1

    def set_position_target(self, type_mask, n, e, d, vn, ve, vd, yaw_rad, *, label=""):
        self.setpoints.append((type_mask, n, e, d, vn, ve, vd, yaw_rad, label))
        if self._stop_after is not None and len(self.setpoints) >= self._stop_after:
            self._stop_event.clear()

    def command_long(self, command, params=(), *, description=""):
        self.commands.append((command, tuple(params), description))

    def heartbeat(self):
        self.heartbeats += 1

    def close(self):
        self.closed = True


def controller(clock=None, *, link=None, dry_run=False) -> NavigationController:
    """A controller with no MAVLink connection and a recording link installed."""
    nav = NavigationController("udpin:127.0.0.1:0", dry_run=dry_run)
    nav.link = link if link is not None else RecordingLink()
    nav._running.set()
    return nav


class LandingBudgetTests(unittest.TestCase):
    """HIGH -- land() spent ~15 s of a 40 s budget and lied about the result."""

    def setUp(self):
        self.clock = FakeClock()
        self.link = RecordingLink()
        self.nav = controller(link=self.link)

    def _land_with(self, disarm_at_s=None, **kwargs):
        """Run land() against a vehicle that disarms at a chosen virtual time."""
        start = self.clock.time()

        def snapshot():
            armed = True
            if disarm_at_s is not None and self.clock.time() - start >= disarm_at_s:
                armed = False
            return VehicleState(
                armed=armed,
                landed_state=2 if armed else 1,
                heartbeat_received_s=self.clock.time(),
                have_local_position=True,
                have_attitude=True,
            )

        with patch.object(nav_module, "time", self.clock), \
             patch.object(self.nav, "get_vehicle_snapshot", snapshot):
            return self.nav.land(**kwargs)

    def test_a_landing_that_completes_after_the_old_ceiling_is_confirmed(self):
        # THE BUG: attempts=3 x a 5 s inner wait gave up at ~15 s, printed
        # "Landing NOT confirmed" while the aircraft was landing perfectly, and
        # made Phase 1A exit 1 on a good landing. 25 s is past the old ceiling
        # and well inside the 40 s budget.
        self.assertTrue(self._land_with(disarm_at_s=25.0, timeout_s=40.0))
        self.assertTrue(self.nav._landing_confirmed)

    def test_the_full_budget_is_used_before_reporting_failure(self):
        self.assertFalse(self._land_with(disarm_at_s=None, timeout_s=40.0))
        # It waited out the whole budget rather than stopping at 3 attempts.
        self.assertGreaterEqual(self.clock.slept, 39.0)

    def test_land_is_reissued_more_than_the_old_three_times(self):
        # Re-issuing still happens -- PX4 can transiently reject a LAND -- it
        # just no longer bounds the wait. 40 s / 5 s = 8 issues.
        self._land_with(disarm_at_s=None, timeout_s=40.0, reissue_after_s=5.0)
        self.assertGreater(len(self.link.commands), 3)

    def test_a_pre_arm_abort_does_not_claim_the_aircraft_landed(self):
        # LOW, same review: `not state.armed` is true on the pad, so an abort
        # before arming printed "Landed and disarmed." -- a completed flight
        # that never happened.
        with patch.object(nav_module, "time", self.clock), \
             patch.object(
                 self.nav, "get_vehicle_snapshot",
                 lambda: VehicleState(armed=False, landed_state=1,
                                      heartbeat_received_s=self.clock.time()),
             ):
            self.assertTrue(self.nav.land())
        self.assertEqual(self.link.commands, [])  # nothing was commanded

    def test_landing_is_idempotent(self):
        self._land_with(disarm_at_s=1.0)
        issued = len(self.link.commands)
        self.assertTrue(self._land_with(disarm_at_s=1.0))
        self.assertEqual(len(self.link.commands), issued)

    def test_a_dry_run_land_does_not_wait_out_a_budget_it_cannot_meet(self):
        nav = controller(link=RecordingLink(suppresses=True), dry_run=True)
        with patch.object(nav_module, "time", self.clock), \
             patch.object(
                 nav, "get_vehicle_snapshot",
                 lambda: VehicleState(armed=True, landed_state=2,
                                      heartbeat_received_s=self.clock.time()),
             ):
            self.assertTrue(nav.land(timeout_s=40.0))
        self.assertLess(self.clock.slept, 1.0)


class DeadmanTests(unittest.TestCase):
    """HIGH -- the deadman cried wolf on the pad before every single flight."""

    def _run_streamer(self, nav, *, sends=3):
        link = RecordingLink(
            stop_after=sends, stop_event=nav._stream_enabled
        )
        nav.link = link
        nav._stream_enabled.set()
        with patch.object(nav_module, "time", FakeClock()):
            nav._setpoint_streamer()
        return link

    def _stale_prime(self, nav):
        # A priming setpoint issued long ago: exactly what STEP 5 leaves on the
        # wire while input() blocks waiting for the operator to arm.
        with nav._setpoint_lock:
            nav._setpoint = Setpoint(
                kind=SetpointKind.VELOCITY_YAW, label="prime", issued_s=-10_000.0
            )

    def test_arm_wait_quiescence_does_not_fire_the_in_flight_alarm(self):
        # THE BUG: confirmed live, every run. The action taken was safe; the
        # message was not, because a warning that fires before every flight is
        # one the operator has already learned to scroll past by the time it
        # means something.
        nav = controller()
        self._stale_prime(nav)
        nav._vehicle = VehicleState(armed=False, custom_mode=0)

        with patch("builtins.print") as printed:
            self._run_streamer(nav)

        output = " ".join(str(call) for call in printed.call_args_list)
        self.assertNotIn("DEADMAN", output)
        self.assertIn("idling", output)

    def test_the_priming_setpoint_is_kept_fresh_rather_than_replaced(self):
        # PX4 needs a setpoint newer than COM_OF_LOSS_T to accept OFFBOARD at
        # all, so the stream has to stay fresh through an arbitrarily long arm
        # wait -- without the deadman having to fire to achieve it.
        nav = controller()
        self._stale_prime(nav)
        nav._vehicle = VehicleState(armed=False, custom_mode=0)

        self._run_streamer(nav)

        self.assertIs(nav.current_setpoint().kind, SetpointKind.VELOCITY_YAW)
        self.assertEqual(nav.current_setpoint().label, "prime")

    def test_a_stale_velocity_in_flight_still_trips_the_deadman(self):
        # The safety property is unchanged: armed and in OFFBOARD, an
        # unrefreshed velocity means the vehicle is flying on a dead
        # instruction, and it must brake to a hold and say so loudly.
        nav = controller()
        with nav._setpoint_lock:
            nav._setpoint = Setpoint(
                kind=SetpointKind.VELOCITY_YAW, vn=0.4, label="approach",
                issued_s=-10_000.0,
            )
        nav._vehicle = VehicleState(
            armed=True, custom_mode=nav_module.PX4_CUSTOM_MAIN_MODE_OFFBOARD << 16
        )

        with patch("builtins.print") as printed:
            self._run_streamer(nav)

        output = " ".join(str(call) for call in printed.call_args_list)
        self.assertIn("DEADMAN", output)
        self.assertIs(nav.current_setpoint().kind, SetpointKind.BRAKE_HOLD_ALT)

    def test_being_acted_on_requires_both_armed_and_offboard(self):
        nav = controller()
        offboard = nav_module.PX4_CUSTOM_MAIN_MODE_OFFBOARD << 16
        for armed, mode, expected in (
            (False, 0, False),
            (True, 0, False),          # armed but the pilot has the aircraft
            (False, offboard, False),  # offboard requested, still disarmed
            (True, offboard, True),
        ):
            with self.subTest(armed=armed, mode=mode):
                nav._vehicle = VehicleState(armed=armed, custom_mode=mode)
                self.assertEqual(nav._setpoint_is_being_acted_on(), expected)


class ObserveGateSetpointTests(unittest.TestCase):
    """MED -- observe_gate interleaved a position mask and a velocity mask."""

    def test_the_observation_window_transmits_no_velocity_setpoints(self):
        # THE BUG: it latched BRAKE_HOLD_ALT (z as a position) and then also
        # sent VELOCITY_YAW zeros (z as a velocity) at 25 Hz. PX4 got both.
        # vz = 0 holds a RATE, not an altitude, so the vehicle was free to drift
        # vertically through every observation window -- which is exactly when
        # it is being asked to measure how high a gate is.
        mission = GateMission()
        nav = FakeNav()
        clock = FakeClock()

        with patch.object(nav_module, "time", clock), \
             patch.object(mission, "get_latest_detection_snapshot",
                          lambda: detection(timestamp=clock.time())):
            result = mission.observe_gate(nav, duration=0.5)

        self.assertIsNotNone(result)
        self.assertEqual(nav.velocity_sends, [])
        self.assertIn("observing", nav.holds)

    def test_a_frozen_feed_is_rejected_even_though_every_sample_looks_fresh(self):
        mission = GateMission()
        nav = FakeNav()
        clock = FakeClock()
        # 15 identical payloads: the publisher is alive and the packets are new.
        for _ in range(15):
            mission._note_payload([[3.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
        self.assertTrue(mission.feed_frozen)

        with patch.object(nav_module, "time", clock), \
             patch.object(mission, "get_latest_detection_snapshot",
                          lambda: detection(timestamp=clock.time())):
            self.assertIsNone(mission.observe_gate(nav, duration=0.5))


class FrozenFeedTests(unittest.TestCase):
    """PROJECT_STATE gap 1 -- the third staleness signal."""

    def setUp(self):
        self.mission = GateMission(frozen_feed_frames=15)

    def _push(self, rows, times=1):
        for _ in range(times):
            self.mission._note_payload(rows)

    def test_identical_payloads_eventually_declare_the_feed_frozen(self):
        rows = [[3.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
        self._push(rows, 14)
        self.assertFalse(self.mission.feed_frozen)
        self._push(rows, 1)
        self.assertTrue(self.mission.feed_frozen)

    def test_a_changing_payload_resets_the_counter(self):
        self._push([[3.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0]], 14)
        self._push([[2.9, 2.9, 0.0, 0.0, 0.0, 0.0, 0.0]], 1)
        self.assertFalse(self.mission.feed_frozen)
        self.assertEqual(self.mission.identical_frame_count, 1)

    def test_an_empty_sky_is_not_a_frozen_feed(self):
        # When no gate is in view every packet is a bit-identical wall of 999s.
        # That is correct behaviour, and flagging it would make the whole signal
        # useless within half a second of looking at empty space.
        self._push([[999.0] * 7] * 3, 60)
        self.assertFalse(self.mission.feed_frozen)

    def test_the_sentinel_rows_around_a_real_gate_are_ignored(self):
        # The wire format always carries three rows. Only the valid ones count,
        # so a single tracked gate still trips the detector.
        rows = [[3.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0], [999.0] * 7, [999.0] * 7]
        self._push(rows, 15)
        self.assertTrue(self.mission.feed_frozen)


class MultiGateCaptureTests(unittest.TestCase):
    """PHASE 2A -- the listener consumed gates[0] and discarded the other rows."""

    def setUp(self):
        self.mission = GateMission()

    def _ingest(self, rows, fps=30.0):
        self.mission._ingest_payload({"fps": fps, "gates": rows})

    def test_every_valid_row_is_kept_not_just_the_nearest(self):
        self._ingest([
            [3.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [7.1, 7.0, 1.5, 0.0, 0.0, 0.0, 25.0],
            [12.1, 12.0, -2.0, 0.0, 0.0, 0.0, -20.0],
        ])

        detections = self.mission.get_latest_detections_snapshot()
        self.assertEqual(len(detections), 3)
        self.assertEqual([det.row_index for det in detections], [0, 1, 2])
        self.assertAlmostEqual(detections[2].right, -2.0)

    def test_the_sentinel_rows_are_dropped_from_the_multi_gate_view(self):
        self._ingest([
            [3.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [999.0] * 7,
            [999.0] * 7,
        ])
        self.assertEqual(len(self.mission.get_latest_detections_snapshot()), 1)

    def test_the_phase_one_view_keeps_the_sentinel_so_staleness_signal_two_survives(self):
        # observe_gate relies on the all-999 row OVERWRITING the last good pose.
        # Filtering it out of the single-gate view would silently delete the
        # second of the three staleness signals.
        self._ingest([[3.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
        self._ingest([[999.0] * 7] * 3)

        latest = self.mission.get_latest_detection_snapshot()
        self.assertIsNotNone(latest)
        self.assertAlmostEqual(latest.dist, 999.0)
        self.assertEqual(self.mission.get_latest_detections_snapshot(), [])

    def test_the_publisher_frame_rate_is_carried_through(self):
        self._ingest([[3.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0]], fps=28.5)
        self.assertAlmostEqual(
            self.mission.get_latest_detections_snapshot()[0].source_fps, 28.5
        )

    def test_a_short_or_malformed_row_does_not_kill_the_listener(self):
        # A units change or a publisher revision must degrade, not crash the
        # thread that carries every detection.
        self._ingest([[3.0, 3.0, 0.0], ["x", 1, 2, 3, 4, 5, 6]])
        self.assertEqual(self.mission.get_latest_detections_snapshot(), [])


class SourceFilterTests(unittest.TestCase):
    """MED -- only HEARTBEAT was filtered by source system."""

    class Message:
        def __init__(self, kind, system=1, component=1, **fields):
            self._kind = kind
            self._system = system
            self._component = component
            self.__dict__.update(fields)

        def get_type(self):
            return self._kind

        def get_srcSystem(self):
            return self._system

        def get_srcComponent(self):
            return self._component

    class Master:
        target_system = 1
        target_component = 1

        def __init__(self, messages, nav):
            self._messages = list(messages)
            self._nav = nav

        def recv_match(self, blocking=True, timeout=0.2):
            if not self._messages:
                self._nav._running.clear()
                return None
            return self._messages.pop(0)

    def _read(self, messages):
        nav = controller()
        nav.master = self.Master(messages, nav)
        nav._mavlink_reader()
        return nav.get_vehicle_snapshot()

    def test_a_foreign_position_is_not_folded_into_this_vehicles_state(self):
        # THE BUG: a GCS or a second vehicle sharing the link publishes its own
        # LOCAL_POSITION_NED and ATTITUDE. Folding those in means flying one
        # aircraft using another one's pose, with a log that looks perfect.
        foreign = self.Message(
            "LOCAL_POSITION_NED", system=42,
            x=99.0, y=99.0, z=-99.0, vx=0.0, vy=0.0, vz=0.0,
        )
        state = self._read([foreign])

        self.assertFalse(state.have_local_position)
        self.assertEqual(state.n, 0.0)

    def test_the_autopilots_own_position_is_accepted(self):
        own = self.Message(
            "LOCAL_POSITION_NED", system=1,
            x=1.0, y=2.0, z=-3.0, vx=0.0, vy=0.0, vz=0.0,
        )
        state = self._read([own])

        self.assertTrue(state.have_local_position)
        self.assertAlmostEqual(state.n, 1.0)
        self.assertAlmostEqual(state.d, -3.0)

    def test_a_foreign_attitude_and_landed_state_are_also_rejected(self):
        messages = [
            self.Message("ATTITUDE", system=42, roll=1.0, pitch=1.0, yaw=1.0),
            self.Message("EXTENDED_SYS_STATE", system=42, landed_state=2),
        ]
        state = self._read(messages)

        self.assertFalse(state.have_attitude)
        self.assertEqual(state.landed_state, 0)

    def test_a_different_component_on_the_right_system_is_rejected(self):
        # A camera or gimbal component on the same sysid is not the autopilot.
        message = self.Message(
            "LOCAL_POSITION_NED", system=1, component=100,
            x=5.0, y=0.0, z=0.0, vx=0.0, vy=0.0, vz=0.0,
        )
        self.assertFalse(self._read([message]).have_local_position)

    def test_position_updates_feed_the_deskew_history(self):
        nav = controller()
        nav.master = self.Master(
            [self.Message("LOCAL_POSITION_NED", x=float(i), y=0.0, z=0.0,
                          vx=0.0, vy=0.0, vz=0.0)
             for i in range(5)],
            nav,
        )
        nav._mavlink_reader()

        self.assertEqual(len(nav.state_history()), 5)


class DryRunSeamTests(unittest.TestCase):
    """PROJECT_STATE gap 6 -- dry-run was a convention, not a mechanism."""

    def test_no_sender_consults_the_dry_run_flag(self):
        # THE POINT OF THE SEAM. dry_run may only be read where the Link is
        # CHOSEN. If a sender tests it again, the next command someone adds
        # without remembering to add its own branch transmits during a bench
        # test with props fitted.
        source = inspect.getsource(NavigationController)
        allowed = {"__init__", "start"}
        for name, member in inspect.getmembers(
            NavigationController, predicate=inspect.isfunction
        ):
            if name in allowed:
                continue
            with self.subTest(method=name):
                self.assertNotIn(
                    "self.dry_run", inspect.getsource(member),
                    msg=f"{name} tests self.dry_run; route it through self.link instead",
                )
        self.assertIn("self.dry_run", source)  # __init__/start still choose

    def test_a_dry_run_link_never_touches_its_inner_link(self):
        inner = RecordingLink()
        link = DryRunLink(inner, log=lambda message: None)

        link.set_position_target(2503, 0, 0, 0, 1.0, 0.0, 0.0, 0.0, label="x")
        link.command_long(21, (1.0, 6.0), description="DO_SET_MODE")
        link.heartbeat()

        self.assertEqual(inner.setpoints, [])
        self.assertEqual(inner.commands, [])
        self.assertEqual(inner.heartbeats, 0)
        self.assertTrue(link.suppresses_commands)

    def test_a_dry_run_controller_transmits_nothing_through_any_sender(self):
        inner = RecordingLink()
        nav = NavigationController("udpin:127.0.0.1:0", dry_run=True)
        nav.link = DryRunLink(inner, log=lambda message: None)
        nav._running.set()

        nav.send_velocity_and_yaw_target(1.0, 0.0, -0.5, 0.0)
        nav._send_raw_setpoint(Setpoint(kind=SetpointKind.BRAKE_HOLD_ALT, d=-1.5))
        nav._send_set_mode_offboard()
        nav._send_land_command()
        with patch.object(nav_module.mavutil, "mavlink", create=True):
            pass

        self.assertEqual(inner.setpoints, [])
        self.assertEqual(inner.commands, [])

    def test_the_offline_link_is_not_a_suppressing_link(self):
        # The offline controller intercepts its senders upstream and its model
        # acts on them. Treating it as "suppressed" would make land() short
        # circuit and the offline mission would never observe a landing.
        self.assertFalse(NullLink().suppresses_commands)

    def test_a_dry_run_controller_prints_its_setpoints_at_a_readable_rate(self):
        lines = []
        link = DryRunLink(None, log=lines.append)
        for _ in range(40):
            link.set_position_target(2503, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, label="hold")

        # Identical setpoints collapse to one line, not forty.
        self.assertEqual(len(lines), 1)
        link.set_position_target(2503, 0, 0, 0, 0.5, 0.0, 0.0, 0.0, label="move")
        self.assertEqual(len(lines), 2)


class EstimatorStatusTests(unittest.TestCase):
    """PROJECT_STATE gap 4 -- flow degradation read as offboard signal loss."""

    HEALTHY = (
        ESTIMATOR_ATTITUDE | ESTIMATOR_VELOCITY_HORIZ
        | ESTIMATOR_POS_HORIZ_REL | ESTIMATOR_POS_VERT_AGL
    )

    def test_a_healthy_estimator_reports_no_faults(self):
        state = VehicleState(estimator_flags=self.HEALTHY, estimator_received_s=1.0)
        self.assertEqual(state.estimator_faults(), ())

    def test_a_cleared_velocity_flag_is_a_named_fault(self):
        state = VehicleState(
            estimator_flags=self.HEALTHY & ~ESTIMATOR_VELOCITY_HORIZ,
            estimator_received_s=1.0,
        )
        self.assertIn("velocity_horiz", state.estimator_faults())

    def test_a_rejected_innovation_is_a_named_fault(self):
        # PX4 uses 1.0 as the pass/fail line for an innovation test ratio.
        state = VehicleState(
            estimator_flags=self.HEALTHY,
            estimator_vel_ratio=1.42,
            estimator_received_s=1.0,
        )
        self.assertTrue(
            any("velocity innovation" in fault for fault in state.estimator_faults())
        )

    def test_never_having_seen_the_message_is_not_a_fault(self):
        # A PX4 build that does not stream message 230 is not an unhealthy
        # estimator, and reporting it as one would be its own misdiagnosis.
        self.assertEqual(VehicleState().estimator_faults(), ())
        self.assertFalse(VehicleState().estimator_seen)

    def test_the_note_distinguishes_a_link_problem_from_a_flow_problem(self):
        nav = controller()

        nav._vehicle = VehicleState(
            estimator_flags=self.HEALTHY, estimator_received_s=1.0
        )
        self.assertIn("really is an offboard/link problem", nav.estimator_note())

        nav._vehicle = VehicleState(
            estimator_flags=self.HEALTHY & ~ESTIMATOR_VELOCITY_HORIZ,
            estimator_received_s=1.0,
        )
        note = nav.estimator_note()
        self.assertIn("ESTIMATOR DEGRADED", note)
        self.assertIn("suspect optical flow", note)

        nav._vehicle = VehicleState()
        self.assertIn("no ESTIMATOR_STATUS seen", nav.estimator_note())


class StateHistoryTests(unittest.TestCase):
    """PROJECT_STATE gap 7 -- detection deskew."""

    def test_the_history_is_bounded(self):
        nav = controller()
        nav.history_depth = 5
        for i in range(20):
            nav._record_history(VehicleState(n=float(i), position_received_s=float(i)))

        history = nav.state_history()
        self.assertEqual(len(history), 5)
        self.assertAlmostEqual(history[-1].n, 19.0)

    def test_state_at_returns_the_closest_sample_in_time(self):
        nav = controller()
        for i in range(10):
            nav._record_history(VehicleState(n=float(i), position_received_s=float(i)))

        self.assertAlmostEqual(nav.state_at(3.4).n, 3.0)
        self.assertAlmostEqual(nav.state_at(3.6).n, 4.0)

    def test_state_at_falls_back_to_the_current_snapshot(self):
        nav = controller()
        nav._vehicle = VehicleState(n=7.0)
        self.assertAlmostEqual(nav.state_at(123.0).n, 7.0)


class RotationTests(unittest.TestCase):
    """PROJECT_STATE gap 5 -- the 3-2-1 rotation lived only in frames.py."""

    def test_the_legacy_signature_is_bit_identical_when_level(self):
        # Adopting full attitude must not change the two legacy missions.
        for yaw in (0.0, 0.7, 1.5708, -2.0, 3.0):
            with self.subTest(yaw=yaw):
                self.assertEqual(
                    body_to_local(2.0, 1.0, -0.5, yaw),
                    body_to_local(2.0, 1.0, -0.5, yaw, 0.0, 0.0),
                )

    def test_pitch_is_no_longer_ignored(self):
        # 3 m ahead at 10 degrees of pitch is 0.52 m of vertical error when
        # pitch is dropped -- straight into the commanded altitude.
        level = body_to_local(3.0, 0.0, 0.0, 0.0)
        pitched = body_to_local(3.0, 0.0, 0.0, 0.0, 0.17453293, 0.0)

        self.assertAlmostEqual(level[2], 0.0, places=9)
        self.assertAlmostEqual(pitched[2], -0.52094453, places=6)


class MissionOutcomeTests(unittest.TestCase):
    """PROJECT_STATE gap 9 -- run_mission returned None."""

    def test_the_exit_code_requires_both_success_and_a_confirmed_landing(self):
        self.assertEqual(MissionOutcome(True, "ok", landed=True).exit_code, 0)
        self.assertEqual(MissionOutcome(True, "ok", landed=False).exit_code, 1)
        self.assertEqual(MissionOutcome(False, "bad", landed=True).exit_code, 1)

    def test_run_mission_reports_a_failing_mission_rather_than_raising(self):
        nav = controller()

        class Boom:
            def start(self):
                pass

            def run(self, nav):
                raise ValueError("gate fell over")

            def stop(self):
                pass

        with patch.object(nav, "start", lambda: None), \
             patch.object(nav, "stop", lambda: None):
            outcome = nav.run_mission(Boom())

        self.assertFalse(outcome.ok)
        self.assertIn("gate fell over", outcome.reason)
        self.assertEqual(outcome.exit_code, 1)


if __name__ == "__main__":
    unittest.main()
