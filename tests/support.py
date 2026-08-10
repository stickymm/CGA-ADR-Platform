"""Test doubles shared by the mission tests.

Not named ``test_*`` on purpose, so ``unittest discover`` does not collect it.

WHY DOUBLES AND NOT THE OFFLINE BACKEND
    ``offline_backend`` is a real kinematic model driven by wall-clock time in a
    background thread.  It is the right tool for an end-to-end run and the wrong
    one for asking "does the recovery ladder escalate in the documented order
    after exactly three misses" -- that question needs the clock and the
    detections under the test's control, not the scheduler's.  The offline model
    is exercised separately, by its own tests and by the offline mission run.

The clock here is virtual: ``sleep`` advances it instead of blocking, so a test
covering a 90 second gate deadline runs in microseconds and always reaches the
same branch.
"""

import sys
import types

from typing import List, Optional


def _install_pymavlink_stub():
    """Make ``pymavlink`` importable, and make ``mavutil.mavlink`` subscriptable.

    Idempotent and order-independent, which matters because the older test files
    install a barer stub of their own and ``unittest discover`` imports modules
    in alphabetical order.  If the real pymavlink is already loaded this leaves
    it completely alone.
    """
    mavutil = sys.modules.get("pymavlink.mavutil")
    if mavutil is None:
        mavutil = types.ModuleType("pymavlink.mavutil")
        mavutil.mavfile = object
        package = types.ModuleType("pymavlink")
        package.mavutil = mavutil
        sys.modules["pymavlink"] = package
        sys.modules["pymavlink.mavutil"] = mavutil

    if not hasattr(mavutil, "mavlink"):
        mavutil.mavlink = _NamedConstants()
    return mavutil


class _NamedConstants:
    """Any attribute is a distinct, stable integer.

    Enough for code that only passes MAVLink enum values through to a link that
    the test is watching.  Any test that cares about a *specific* wire value
    asserts on the documented number instead (see the type_mask tests).
    """

    def __init__(self):
        self._values = {}

    def __getattr__(self, name):
        if name.startswith("_"):
            raise AttributeError(name)
        return self._values.setdefault(name, len(self._values) + 1)


_install_pymavlink_stub()

from navigation.navigation import (  # noqa: E402
    GateDetection,
    LocalTarget,
    Setpoint,
    SetpointKind,
    VehicleState,
    PX4_CUSTOM_MAIN_MODE_OFFBOARD,
)
from navigation.missions.contracts import AltitudeEnvelope  # noqa: E402


OFFBOARD_CUSTOM_MODE = PX4_CUSTOM_MAIN_MODE_OFFBOARD << 16


class FakeClock:
    """A virtual clock.  ``sleep`` advances it rather than blocking.

    Substituted for the ``time`` module inside the module under test, so a
    branch guarded by a 90 s deadline is reachable in a unit test without the
    test taking 90 seconds or depending on scheduler luck.
    """

    def __init__(self, start: float = 1_000.0):
        self.now = start
        self.slept = 0.0

    def time(self) -> float:
        return self.now

    def monotonic(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.slept += seconds
        self.now += seconds

    def advance(self, seconds: float) -> None:
        self.now += seconds


def detection(**overrides) -> GateDetection:
    """A plausible dead-ahead detection, 3 m out, unless overridden."""
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


def state(**overrides) -> VehicleState:
    """A healthy airborne vehicle state, 1.5 m up and in OFFBOARD."""
    fields = dict(
        n=0.0,
        e=0.0,
        d=-1.5,
        yaw_rad=0.0,
        have_local_position=True,
        have_attitude=True,
        armed=True,
        custom_mode=OFFBOARD_CUSTOM_MODE,
        landed_state=2,
        position_received_s=1_000.0,
        attitude_received_s=1_000.0,
        heartbeat_received_s=1_000.0,
        estimator_flags=0xFFFF,
        estimator_received_s=1_000.0,
    )
    fields.update(overrides)
    return VehicleState(**fields)


class FakeNav:
    """Duck-typed ``NavigationController``.  Records everything, flies nothing.

    Moves teleport by default: this class exists to test *sequencing and branch
    selection*, and a controller that needs several ticks to arrive turns every
    such test into a test of the controller instead.  Set ``move_results`` to
    script failures, or ``move_travel_fraction`` for partial moves.
    """

    def __init__(self, clock: Optional[FakeClock] = None, **overrides):
        self.clock = clock or FakeClock()
        self.running = True
        self.dry_run = False
        self.altitude_envelope = AltitudeEnvelope()
        self._state = state()

        # Recorded calls, in order.
        self.moves: List[tuple] = []
        self.slews: List[tuple] = []
        self.holds: List[str] = []
        self.setpoints: List[Setpoint] = []
        self.velocity_sends: List[tuple] = []
        self.landings = 0
        self.safe_shutdowns: List[str] = []

        # Scripting hooks.
        self.move_results: List[bool] = []
        self.move_travel_fraction = 1.0
        self.move_cost_s = 0.0
        self.telemetry_healthy = True
        self.estimator_text = " (estimator healthy)"
        # When True, every velocity command integrates into the pose, which is
        # what lets a test drive the open-loop crossing leg to a real plane
        # crossing instead of stubbing its result.
        self.integrate_velocity = False
        self.integration_dt = 0.1

        for name, value in overrides.items():
            setattr(self, name, value)

    # --- state ----------------------------------------------------------
    def set_state(self, **overrides) -> None:
        self._state = state(**overrides)

    def get_vehicle_snapshot(self) -> VehicleState:
        return self._state

    def telemetry_ok(self, max_age_s: float) -> bool:
        return self.telemetry_healthy

    def estimator_note(self) -> str:
        return self.estimator_text

    # --- commands -------------------------------------------------------
    def move_to_target(self, target: LocalTarget, label: str, *,
                       max_speed_m_s=0.5, tolerance_m=0.15, timeout_s=None) -> bool:
        self.moves.append((label, target, max_speed_m_s, tolerance_m))
        self.clock.advance(self.move_cost_s)

        ok = self.move_results.pop(0) if self.move_results else True
        fraction = self.move_travel_fraction if ok else 0.0
        current = self._state
        self._state = state(
            n=current.n + (target.n - current.n) * fraction,
            e=current.e + (target.e - current.e) * fraction,
            d=current.d + (target.d - current.d) * fraction,
            yaw_rad=target.yaw_rad,
        )
        return ok

    def slew_yaw(self, target_yaw_rad: float, *, label="yaw slew",
                 max_rate_deg_s=45.0, tolerance_deg=5.0, timeout_s=None) -> bool:
        self.slews.append((label, target_yaw_rad, max_rate_deg_s))
        self._state = state(
            n=self._state.n, e=self._state.e, d=self._state.d, yaw_rad=target_yaw_rad
        )
        return True

    def hold_position(self, *, yaw_rad=None, label="hold") -> None:
        self.holds.append(label)

    def set_setpoint(self, setpoint: Setpoint) -> None:
        self.setpoints.append(setpoint)

    def current_setpoint(self) -> Setpoint:
        return self.setpoints[-1] if self.setpoints else Setpoint()

    def send_velocity_and_yaw_target(self, vn, ve, vd, yaw_rad) -> None:
        self.velocity_sends.append((vn, ve, vd, yaw_rad))
        if not self.integrate_velocity:
            return
        dt = self.integration_dt
        current = self._state
        self._state = state(
            n=current.n + vn * dt,
            e=current.e + ve * dt,
            d=current.d + vd * dt,
            yaw_rad=yaw_rad,
        )

    def _hold_from_state(self, state_in, yaw_rad=None, *, label="hold") -> None:
        self.holds.append(label)

    def land(self, **kwargs) -> bool:
        self.landings += 1
        return True

    def safe_shutdown(self, reason: str, *, hold_s=3.0) -> bool:
        self.safe_shutdowns.append(reason)
        self.landings += 1
        return True


class ScriptedMission:
    """A mission whose ``observe_gate`` returns a scripted sequence.

    ``None`` entries are "the gate was not seen", which is what drives the
    recovery ladder.  When the script runs out the last entry repeats, so a test
    only has to write down the part it cares about.
    """

    def __init__(self, script, *, cam_offsets=(0.0, 0.0, 0.0)):
        self.script = list(script)
        self.observations = 0
        self.cam_offset_right_m, self.cam_offset_down_m, self.cam_yaw_offset_deg = cam_offsets
        self.detection_max_age_s = 0.4
        self.udp_ip = "127.0.0.1"
        self.udp_port = 5050
        self.feed_frozen = False
        self.identical_frame_count = 0

    def observe_gate(self, nav, duration: float = 1.0):
        self.observations += 1
        if not self.script:
            return None
        if len(self.script) == 1:
            return self.script[0]
        return self.script.pop(0)

    def get_latest_detection_snapshot(self):
        return None


class RecordingLog:
    """Collects log lines so a test can assert on what the operator was told."""

    def __init__(self):
        self.lines: List[str] = []

    def __call__(self, message: str) -> None:
        self.lines.append(str(message))

    @property
    def text(self) -> str:
        return "\n".join(self.lines)

    def contains(self, needle: str) -> bool:
        return needle.lower() in self.text.lower()
