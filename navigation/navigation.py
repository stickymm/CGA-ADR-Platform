import json
import math
import socket
import threading
import time
from dataclasses import dataclass, replace
from enum import Enum
from typing import Optional

from pymavlink import mavutil


# =========================
# CONFIG
# =========================

UDP_IP = "127.0.0.1"
UDP_PORT = 5050

MAVLINK_CONN = "udpin:127.0.0.1:14551"

POSITION_RATE_HZ = 10
POSITION_TOLERANCE_M = 0.10
YAW_TOLERANCE_DEG = 5.0
MOVE_TIMEOUT = 15.0

# --- VELOCITY CONTROLLER TUNING ---
MAX_FLIGHT_SPEED_M_S = 0.5
KP_POS = 1.2

# --- HARDWARE OFFSETS ---
# Sign conventions, recovered from the abandoned navigation_closed_loop.py
# prototype, which was the only surviving record of what these mean.  Each
# offset is added to the raw detection before the body->NED rotation, so a
# positive value makes the gate appear further right/down/clockwise and
# therefore flies the drone right/down/clockwise to match.
CAM_OFFSET_RIGHT_M = -0.25   # Negative = shift drone Left | Positive = shift drone Right
CAM_OFFSET_DOWN_M = -0.10    # Negative = shift drone Up   | Positive = shift drone Down
CAM_YAW_OFFSET_DEG = -10.0   # Negative = Yaw Left | Positive = Yaw Right

NO_DETECTION_DIST = 999.0
# The vision process pads absent gates with an all-999.0 row.  Comparing floats
# with != happens to work for a value that round-trips exactly through JSON, but
# it is brittle: a units change or a different publisher turns "no gate" into
# "gate at 999 m".  Treat anything implausible as no-detection instead.
MIN_PLAUSIBLE_DIST_M = 0.05
MAX_PLAUSIBLE_DIST_M = 100.0

# --- PX4 / MAVLINK CONSTANTS ---
# PX4 packs its mode into HEARTBEAT.custom_mode as
#   main_mode = (custom_mode >> 16) & 0xFF,  sub_mode = (custom_mode >> 24) & 0xFF
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
MAV_MODE_FLAG_SAFETY_ARMED = 128

# Setpoint stream. PX4 requires a setpoint newer than COM_OF_LOSS_T (default
# 1.0 s) both to ENTER offboard and to stay in it, and MulticopterPositionControl
# applies a second, independent 1 s freshness check. 20 Hz gives 20x margin on
# both, so a GC pause or a slow frame cannot drop us out mid-gate.
SETPOINT_RATE_HZ = 20
SETPOINT_STALE_S = 0.5

# A move that cannot physically finish inside its timeout reports failure even
# when it flew perfectly. Derive the budget from distance and speed instead of
# using one fixed number, and keep MOVE_TIMEOUT as the floor for short hops.
MOVE_TIMEOUT_MARGIN_S = 3.0
MOVE_TIMEOUT_SCALE = 1.5


# =========================
# DATA TYPES
# =========================


@dataclass
class VehicleState:
    """Latest MAVLink-derived local pose snapshot for the drone.

    New fields are appended after the original six so that positional
    construction, which the existing tests rely on, keeps working unchanged.

    ``roll_rad``/``pitch_rad`` are the vehicle's own attitude, not to be confused
    with ``GateDetection.roll``/``pitch``, which describe the *gate's* plane.
    ``have_local_position``/``have_attitude`` latch true on first receipt and
    mean "seen at least once"; use the ``*_received_s`` stamps with
    :meth:`is_fresh` to ask whether telemetry is healthy *now*.
    """
    n: float = 0.0
    e: float = 0.0
    d: float = 0.0
    yaw_rad: float = 0.0
    have_local_position: bool = False
    have_attitude: bool = False
    roll_rad: float = 0.0
    pitch_rad: float = 0.0
    vn: float = 0.0
    ve: float = 0.0
    vd: float = 0.0
    armed: bool = False
    custom_mode: int = 0
    base_mode: int = 0
    landed_state: int = 0          # MAV_LANDED_STATE: 1=on ground, 2=in air
    position_received_s: float = 0.0
    attitude_received_s: float = 0.0
    heartbeat_received_s: float = 0.0

    def is_fresh(self, now_s: float, max_age_s: float) -> bool:
        """True when position and attitude have both arrived within ``max_age_s``."""
        if not (self.have_local_position and self.have_attitude):
            return False
        return (
            now_s - self.position_received_s <= max_age_s
            and now_s - self.attitude_received_s <= max_age_s
        )

    @property
    def in_offboard(self) -> bool:
        """True when PX4's HEARTBEAT reports OFFBOARD.

        PX4 packs its flight mode into ``custom_mode`` as
        ``main_mode = (custom_mode >> 16) & 0xFF``.  OFFBOARD is main mode 6, so
        the whole field reads 0x00060000 == 393216.

        Do NOT substitute pymavlink's ``master.flightmode`` here: its
        ``interpret_px4_mode`` requires ``base_mode & 28 == 28`` while PX4 sends
        145 when armed in OFFBOARD, so it reports "UNKNOWN" in exactly the state
        we need to detect.  That is fixed only on pymavlink main, not in any
        release.
        """
        return ((self.custom_mode >> 16) & 0xFF) == PX4_CUSTOM_MAIN_MODE_OFFBOARD


@dataclass
class GateDetection:
    """One gate observation coming from the vision pipeline over UDP."""
    timestamp: float
    dist: float
    forward: float
    right: float
    down: float
    roll: float
    pitch: float
    yaw_deg: float


@dataclass
class LocalTarget:
    """Position and yaw target expressed in local NED coordinates."""
    n: float
    e: float
    d: float
    yaw_rad: float


class SetpointKind(Enum):
    """Which fields of a SET_POSITION_TARGET_LOCAL_NED are active."""

    IDLE = "idle"
    VELOCITY_YAW = "velocity_yaw"            # vx, vy, vz + absolute yaw
    POSITION_YAW = "position_yaw"            # x, y, z + absolute yaw
    BRAKE_HOLD_ALT = "brake_hold_alt"        # vx, vy + z position + absolute yaw


def type_mask_for(kind: SetpointKind) -> int:
    """The SET_POSITION_TARGET_LOCAL_NED type_mask for a setpoint kind.

    A bit that is SET means "ignore this field".  Bits, low to high:
    x=1, y=2, z=4, vx=8, vy=16, vz=32, ax=64, ay=128, az=256,
    force=512, yaw=1024, yaw_rate=2048.

    PX4 requires the x and y axes to be controlled by the *same* kind of
    setpoint -- position or velocity, not one of each -- so only these
    combinations are offered.
    """
    if kind is SetpointKind.VELOCITY_YAW:
        # ignore x,y,z + accel + yaw_rate  ->  1+2+4 + 64+128+256 + 2048 = 2503
        return 2503
    if kind is SetpointKind.POSITION_YAW:
        # ignore vx,vy,vz + accel + yaw_rate  ->  8+16+32 + 64+128+256 + 2048 = 2552
        return 2552
    if kind is SetpointKind.BRAKE_HOLD_ALT:
        # active: vx, vy, z, yaw.  ignore x,y + vz + accel + yaw_rate
        #   -> 1+2 + 32 + 64+128+256 + 2048 = 2531
        return 2531
    raise ValueError(f"no type_mask for {kind!r}")


@dataclass(frozen=True)
class Setpoint:
    """One immutable offboard setpoint, swapped atomically by the streamer.

    Held frozen so the publisher thread can copy a reference under the lock and
    transmit outside it without the value changing underneath it.
    """

    kind: SetpointKind = SetpointKind.IDLE
    vn: float = 0.0
    ve: float = 0.0
    vd: float = 0.0
    n: float = 0.0
    e: float = 0.0
    d: float = 0.0
    yaw_rad: float = 0.0
    issued_s: float = 0.0
    label: str = ""

    @property
    def is_transient(self) -> bool:
        """True if this setpoint must be refreshed to stay meaningful.

        A velocity command is a transient: it says "keep moving this way", so if
        the mission stops refreshing it the vehicle keeps flying on a stale
        instruction.  That is the case the deadman must catch.

        A position or brake-and-hold setpoint is steady-state: it says "be here",
        which stays true indefinitely.  Ageing one out would be a false positive
        -- and an expensive one, because it is exactly what a climb-to-altitude
        or a deliberate hold looks like.
        """
        return self.kind is SetpointKind.VELOCITY_YAW

    def climbs(self) -> bool:
        """True if this setpoint would command upward motion.

        PX4 latches ``want_takeoff`` the instant the vehicle is armed and a fresh
        setpoint has a negative vz.  If the streamer were already publishing the
        climb setpoint when the operator arms, the mode change and lift-off would
        be the same event.  The pad sequence asserts this is False until it has
        verified OFFBOARD.
        """
        if self.kind is SetpointKind.VELOCITY_YAW:
            return self.vd < 0.0
        return False


# =========================
# GEOMETRY / SMALL HELPERS
# =========================

def wrap_pi(angle: float) -> float:
    """Normalize any angle into the [-pi, pi) range."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi

def deg_to_rad(deg: float) -> float:
    return deg * math.pi / 180.0

def rad_to_deg(rad: float) -> float:
    return rad * 180.0 / math.pi

def body_to_local(forward: float, right: float, down: float, yaw_rad: float):
    dn = math.cos(yaw_rad) * forward - math.sin(yaw_rad) * right
    de = math.sin(yaw_rad) * forward + math.cos(yaw_rad) * right
    dd = down
    return dn, de, dd

def local_forward_vector(yaw_rad: float):
    return math.cos(yaw_rad), math.sin(yaw_rad)

def is_valid_detection(det: Optional[GateDetection]) -> bool:
    """True when a detection is present and plausibly a real gate.

    A range check rather than ``!= NO_DETECTION_DIST``: exact float equality
    against a sentinel is fragile, and it also accepts absurd-but-not-999 values
    such as a 998.7 m "gate".
    """
    if det is None:
        return False
    if not math.isfinite(det.dist):
        return False
    return MIN_PLAUSIBLE_DIST_M <= det.dist <= MAX_PLAUSIBLE_DIST_M


def estimate_move_timeout(
    distance_m: float,
    max_speed_m_s: float,
    *,
    kp: float = KP_POS,
    tolerance_m: float = POSITION_TOLERANCE_M,
    margin_s: float = MOVE_TIMEOUT_MARGIN_S,
    scale: float = MOVE_TIMEOUT_SCALE,
    floor_s: float = MOVE_TIMEOUT,
) -> float:
    """Time budget for a move, derived from how long it can physically take.

    The controller commands ``v = kp * error`` clamped to ``max_speed_m_s``, so a
    move has two phases: a constant-speed run while ``error > max_speed / kp``,
    then an exponential decay into the arrival tolerance.

        t_run   = (distance - knee) / max_speed        where knee = max_speed / kp
        t_decay = ln(knee / tolerance) / kp

    A fixed 15 s budget cannot cover a 2.5 m leg at 0.15 m/s -- that needs
    15.83 s of run alone -- so the move reports failure every single time while
    having flown correctly.  Scaling the ideal time leaves room for the real
    accel/decel lag that this open-loop estimate ignores.
    """
    if not math.isfinite(distance_m) or distance_m < 0.0:
        raise ValueError("distance_m must be a non-negative finite number")
    if not math.isfinite(max_speed_m_s) or max_speed_m_s <= 0.0:
        raise ValueError("max_speed_m_s must be a positive finite number")
    if kp <= 0.0 or tolerance_m <= 0.0:
        raise ValueError("kp and tolerance_m must be positive")

    knee_m = max_speed_m_s / kp
    if distance_m <= tolerance_m:
        ideal_s = 0.0
    elif distance_m <= knee_m:
        ideal_s = math.log(distance_m / tolerance_m) / kp
    else:
        ideal_s = (distance_m - knee_m) / max_speed_m_s + math.log(knee_m / tolerance_m) / kp

    return max(floor_s, margin_s + scale * ideal_s)


class NavigationController:
    """Owns MAVLink I/O, vehicle state, and reusable flight primitives."""
    def __init__(
        self,
        mavlink_conn: str,
        *,
        dry_run: bool = False,
    ):
        self.mavlink_conn = mavlink_conn
        # When True every outbound command is printed instead of transmitted.
        # Inbound telemetry still flows, so the geometry can be checked on a
        # bench against a real vehicle that cannot move.
        self.dry_run = dry_run

        self.master: Optional[mavutil.mavfile] = None

        self._running = threading.Event()
        self._vehicle = VehicleState()
        self._vehicle_lock = threading.Lock()
        self._mavlink_thread: Optional[threading.Thread] = None

        # Lock order is always _vehicle_lock -> _setpoint_lock. Never hold
        # _setpoint_lock across a transmit.
        self._setpoint_lock = threading.Lock()
        self._setpoint = Setpoint()
        self._stream_enabled = threading.Event()
        self._setpoint_thread: Optional[threading.Thread] = None
        self._deadman_tripped = False

        self._heartbeat_thread: Optional[threading.Thread] = None

        self._dry_run_last_signature = None
        self._dry_run_last_log_s = 0.0

        self._landing_confirmed = False
        self._land_commanded_s = 0.0

        self._master_lock = threading.RLock()
        self._closed = False

        # Set by the pad sequence once the takeoff reference is latched. While
        # None, move_to_target does not clamp, which preserves the exact prior
        # behaviour for the two existing missions when run without a pad step.
        self.altitude_envelope = None

    @property
    def running(self) -> bool:
        return self._running.is_set()

    # =========================
    # LIFECYCLE
    # =========================

    def start(self, *, require_offboard: bool = True, heartbeat_timeout_s: float = 30.0):
        """Connect to MAVLink, start the background threads, and enter offboard.

        ``require_offboard=False`` stops after the stream is running, which is
        what the pad sequence wants: it needs setpoints flowing while the vehicle
        is still disarmed, then requests OFFBOARD only once the operator has
        armed.  The default preserves the original behaviour for existing
        missions.
        """
        if self.running:
            return

        # Clear any stale state so repeated missions do not reuse old pose flags.
        with self._vehicle_lock:
            self._vehicle = VehicleState()
        self._landing_confirmed = False
        self._closed = False

        print(f"[*] Connecting to MAVLink at {self.mavlink_conn}...")
        self.master = mavutil.mavlink_connection(self.mavlink_conn, source_system=254)

        # This is NOT politeness. With a `udpin:` connection pymavlink cannot
        # transmit at all until it has received a packet: mavudp.write() iterates
        # a client set that is only populated inside recv(), and send failures
        # are silently swallowed. Every byte written before the first inbound
        # datagram is discarded without error.
        if self.master.wait_heartbeat(timeout=heartbeat_timeout_s) is None:
            self.master = None
            raise RuntimeError(
                f"No MAVLink HEARTBEAT within {heartbeat_timeout_s:.0f}s on "
                f"{self.mavlink_conn}. Nothing can be transmitted until one arrives."
            )
        print(
            f"[*] MAVLink heartbeat received "
            f"(system {self.master.target_system}, component {self.master.target_component})"
        )

        self._running.set()
        self._mavlink_thread = threading.Thread(
            target=self._mavlink_reader,
            daemon=True,
            name="nav-mavlink-reader",
        )
        self._mavlink_thread.start()

        self._start_heartbeat()

        if not self.wait_for_vehicle_state():
            raise RuntimeError("Vehicle telemetry never arrived; refusing to continue")

        self.start_streaming()

        if require_offboard and not self.request_offboard():
            raise RuntimeError("Could not confirm OFFBOARD mode; refusing to continue")

    def _start_heartbeat(self):
        """Emit our own 1 Hz HEARTBEAT so PX4 will route COMMAND_ACKs back.

        PX4 gates acknowledgements on having seen the requesting component, so a
        companion that never announces itself can issue commands and never learn
        whether they were accepted.
        """
        self._heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop,
            daemon=True,
            name="nav-heartbeat",
        )
        self._heartbeat_thread.start()

    def _heartbeat_loop(self):
        while self.running:
            if not self.dry_run and self.master is not None:
                try:
                    self.master.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                        0, 0, 0,
                    )
                except Exception:
                    pass
            time.sleep(1.0)

    def request_message_intervals(self, intervals_hz=None) -> None:
        """Ask PX4 for the streams this mission depends on.

        PX4's default rates on a companion link depend on MAV_x_MODE and can be
        as low as 1 Hz, which is useless for a 10 Hz position loop.  An ACK only
        means the interval was stored, never that the rate will be achieved --
        so callers must follow this with :meth:`measure_telemetry_rate`.
        """
        if intervals_hz is None:
            intervals_hz = {
                mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED: 30.0,
                mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE: 50.0,
                mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE: 5.0,
            }

        for message_id, rate_hz in intervals_hz.items():
            interval_us = 1e6 / rate_hz
            if self.dry_run:
                print(f"[DRY-RUN] would request message {message_id} at {rate_hz:.0f} Hz")
                continue
            if self.master is None:
                return
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                float(message_id),
                float(interval_us),
                0, 0, 0, 0, 0,
            )
            time.sleep(0.05)

    def measure_telemetry_rate(self, duration_s: float = 3.0):
        """Measure achieved position and attitude rates over a window.

        Requesting an interval is not the same as receiving it -- the link may be
        bandwidth-starved, a router may filter, or another GCS may have
        overwritten the interval.  Measure, then decide.

        Returns:
            ``(position_hz, attitude_hz)``.
        """
        start = self.get_vehicle_snapshot()
        first_position, first_attitude = start.position_received_s, start.attitude_received_s
        position_count = attitude_count = 0

        deadline = time.time() + duration_s
        while self.running and time.time() < deadline:
            state = self.get_vehicle_snapshot()
            if state.position_received_s != first_position:
                position_count += 1
                first_position = state.position_received_s
            if state.attitude_received_s != first_attitude:
                attitude_count += 1
                first_attitude = state.attitude_received_s
            time.sleep(0.002)

        return position_count / duration_s, attitude_count / duration_s

    def stop(self):
        """Stop background work, leaving a stationary setpoint behind.

        ``self.master`` is no longer set to ``None`` here.  Doing so raced every
        in-flight sender: a thread that had already passed its ``is None`` guard
        would dereference ``None`` and raise ``AttributeError`` instead of the
        intended ``RuntimeError``.  A ``_closed`` flag conveys the same thing
        without the window.
        """
        if self._stream_enabled.is_set():
            # Last word on the wire is "stay put", not whatever velocity the
            # mission happened to be commanding when it was interrupted.
            try:
                self.hold_position(label="shutdown")
                self._transmit(self.current_setpoint())
            except Exception:
                pass

        self.stop_streaming()
        self._running.clear()

        if self._heartbeat_thread is not None and self._heartbeat_thread.is_alive():
            self._heartbeat_thread.join(timeout=1.0)

        if self._mavlink_thread is not None and self._mavlink_thread.is_alive():
            self._mavlink_thread.join(timeout=1.0)

        with self._master_lock:
            self._closed = True
            if self.master is not None:
                close = getattr(self.master, "close", None)
                if callable(close):
                    close()

    def run_mission(self, mission):
        """Run one mission object against this controller with managed lifecycle.

        ``start()`` is inside the ``try`` so that a failure part-way through it --
        after the reader thread has already been spawned -- still runs the
        cleanup rather than leaking the thread.
        """
        try:
            self.start()
            mission.start()
            mission.run(self)
        finally:
            mission.stop()
            self.stop()

    # =========================
    # VEHICLE STATE ACCESS
    # =========================

    def get_vehicle_snapshot(self) -> VehicleState:
        with self._vehicle_lock:
            # Copy under the lock so callers can read a stable snapshot
            # without holding the shared-state lock themselves.
            return replace(self._vehicle)

    def _is_autopilot(self, msg) -> bool:
        """True when a message came from the flight controller itself.

        ``recv_match`` sees every component on the link.  Once ``wait_heartbeat``
        has run, ``target_system`` holds the autopilot's system id; before that
        it is 0 and we accept anything so the first heartbeat can be observed.
        """
        if self.master is None:
            return False
        target = getattr(self.master, "target_system", 0)
        return target == 0 or msg.get_srcSystem() == target

    def _mavlink_reader(self):
        """Continuously fold incoming MAVLink messages into VehicleState."""
        if self.master is None:
            return

        while self.running:
            msg = self.master.recv_match(blocking=True, timeout=0.2)
            if msg is None:
                continue

            msg_type = msg.get_type()
            now = time.time()
            with self._vehicle_lock:
                if msg_type == "LOCAL_POSITION_NED":
                    self._vehicle.n = float(msg.x)
                    self._vehicle.e = float(msg.y)
                    self._vehicle.d = float(msg.z)
                    self._vehicle.vn = float(msg.vx)
                    self._vehicle.ve = float(msg.vy)
                    self._vehicle.vd = float(msg.vz)
                    self._vehicle.have_local_position = True
                    self._vehicle.position_received_s = now
                elif msg_type == "ATTITUDE":
                    self._vehicle.yaw_rad = float(msg.yaw)
                    self._vehicle.roll_rad = float(msg.roll)
                    self._vehicle.pitch_rad = float(msg.pitch)
                    self._vehicle.have_attitude = True
                    self._vehicle.attitude_received_s = now
                elif msg_type == "HEARTBEAT" and self._is_autopilot(msg):
                    # The only source of truth for arm state and flight mode.
                    # Both are needed before we command any motion.  Filtered by
                    # source, because a GCS sharing this link also emits
                    # HEARTBEATs and its base_mode would otherwise be read as the
                    # vehicle's.
                    self._vehicle.base_mode = int(msg.base_mode)
                    self._vehicle.custom_mode = int(msg.custom_mode)
                    self._vehicle.armed = bool(
                        int(msg.base_mode) & MAV_MODE_FLAG_SAFETY_ARMED
                    )
                    self._vehicle.heartbeat_received_s = now
                elif msg_type == "EXTENDED_SYS_STATE":
                    # The only reliable "are we airborne / have we touched down"
                    # signal. Landing is confirmed on this plus the disarm flag,
                    # never on the flight mode.
                    self._vehicle.landed_state = int(msg.landed_state)

    # =========================
    # FLIGHT PRIMITIVES
    # =========================

    def wait_for_vehicle_state(self, *, timeout_s: float = 30.0) -> bool:
        """Block until local position and attitude arrive, or the timeout expires.

        Bounded: an unbounded wait here blocks the main thread forever if
        telemetry never comes, with no diagnostic and no way out.
        """
        print(f"[*] Waiting up to {timeout_s:.0f}s for LOCAL_POSITION_NED and ATTITUDE...")
        deadline = time.time() + timeout_s

        while self.running and time.time() < deadline:
            state = self.get_vehicle_snapshot()
            if state.have_local_position and state.have_attitude:
                print("[*] Vehicle state ready")
                return True
            time.sleep(0.1)

        state = self.get_vehicle_snapshot()
        print(
            f"[!] Timed out waiting for telemetry "
            f"(position={state.have_local_position}, attitude={state.have_attitude}). "
            f"Check that PX4 is streaming LOCAL_POSITION_NED and ATTITUDE on this link."
        )
        return False

    def telemetry_ok(self, max_age_s: float) -> bool:
        """True when position and attitude have both arrived recently.

        The ``have_*`` flags latch true on first receipt and never clear, so they
        answer "did we ever see this", not "is it healthy now".  This asks the
        second question, which is the one that matters in flight.
        """
        return self.get_vehicle_snapshot().is_fresh(time.time(), max_age_s)

    # =========================
    # SETPOINT STREAM
    # =========================

    def set_setpoint(self, setpoint: Setpoint):
        """Atomically replace the setpoint the streamer publishes."""
        with self._setpoint_lock:
            self._setpoint = replace(setpoint, issued_s=time.time())

    def current_setpoint(self) -> Setpoint:
        with self._setpoint_lock:
            return self._setpoint

    def hold_position(self, *, yaw_rad: Optional[float] = None, label: str = "hold"):
        """Latch a brake-and-hold-altitude setpoint at the current position.

        Uses zero horizontal *velocity* with the current altitude as a *position*
        target.  Not a full position hold: on an optical-flow airframe the
        horizontal estimate drifts, and a position hold would chase that drift at
        up to MPC_XY_VEL_MAX.  Braking laterally cannot run away, while letting
        PX4 close the altitude loop keeps height honest.
        """
        self._hold_from_state(self.get_vehicle_snapshot(), yaw_rad, label=label)

    def _hold_from_state(
        self,
        state: VehicleState,
        yaw_rad: Optional[float] = None,
        *,
        label: str = "hold",
    ):
        """Latch a brake-and-hold-altitude setpoint from an already-read state."""
        self.set_setpoint(
            Setpoint(
                kind=SetpointKind.BRAKE_HOLD_ALT,
                vn=0.0,
                ve=0.0,
                d=state.d,
                yaw_rad=state.yaw_rad if yaw_rad is None else yaw_rad,
                label=label,
            )
        )

    def start_streaming(self, *, label: str = "prime"):
        """Begin publishing setpoints, starting from a stationary hold.

        Must be running *before* OFFBOARD is requested: PX4 will not accept the
        mode unless a setpoint newer than COM_OF_LOSS_T already exists.  Safe to
        run while disarmed and in any mode -- offboard_control_mode is published
        regardless of arm state, which is what lets the operator arm last.
        """
        if self._stream_enabled.is_set():
            return

        state = self.get_vehicle_snapshot()
        # Finite zeros, never NaN: PX4 decides a message counts as proof of life
        # with `velocity = !isAllNan(...)`, so an all-NaN setpoint is ignored.
        self.set_setpoint(
            Setpoint(kind=SetpointKind.VELOCITY_YAW, yaw_rad=state.yaw_rad, label=label)
        )

        self._stream_enabled.set()
        self._setpoint_thread = threading.Thread(
            target=self._setpoint_streamer,
            daemon=True,
            name="nav-setpoint-streamer",
        )
        self._setpoint_thread.start()
        print(f"[*] Setpoint stream started at {SETPOINT_RATE_HZ} Hz (zero velocity)")

    def stop_streaming(self):
        self._stream_enabled.clear()
        thread = self._setpoint_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        self._setpoint_thread = None

    def _setpoint_streamer(self):
        """Publish the current setpoint at a fixed rate until told to stop.

        Absolute-time scheduling, because ``sleep(period)`` accumulates drift.

        The deadman is the reason this thread exists.  If mission code stops
        updating the setpoint -- blocked, deadlocked, or unwinding through an
        exception -- the stream stays alive so PX4 does not drop offboard, but
        the value degrades to a stationary brake.  Without it, a Ctrl-C during a
        move leaves the vehicle in OFFBOARD coasting on the last non-zero
        velocity with nothing transmitting at all.
        """
        period = 1.0 / SETPOINT_RATE_HZ
        next_tick = time.monotonic()

        while self.running and self._stream_enabled.is_set():
            setpoint = self.current_setpoint()

            stale = (
                setpoint.is_transient
                and time.time() - setpoint.issued_s > SETPOINT_STALE_S
            )
            if stale:
                if not self._deadman_tripped:
                    self._deadman_tripped = True
                    print(
                        f"[!] DEADMAN: velocity setpoint '{setpoint.label}' not "
                        f"refreshed for {SETPOINT_STALE_S:.2f}s. Braking to a hold."
                    )
                self.hold_position(label="deadman")
                setpoint = self.current_setpoint()
            elif self._deadman_tripped:
                self._deadman_tripped = False

            try:
                self._transmit(setpoint)
            except Exception as exc:  # never let the stream die on one bad send
                if self.running:
                    print(f"[!] Setpoint transmit failed: {exc}")

            next_tick += period
            time.sleep(max(0.0, next_tick - time.monotonic()))

    def _transmit(self, setpoint: Setpoint):
        """Single chokepoint for every setpoint that leaves this process.

        VELOCITY_YAW is dispatched through the long-standing public
        ``send_velocity_and_yaw_target`` so that existing subclasses which
        override it keep intercepting everything, including what the streamer
        emits.
        """
        if setpoint.kind is SetpointKind.IDLE:
            return
        if setpoint.kind is SetpointKind.VELOCITY_YAW:
            self.send_velocity_and_yaw_target(
                setpoint.vn, setpoint.ve, setpoint.vd, setpoint.yaw_rad
            )
            return
        self._send_raw_setpoint(setpoint)

    def _send_raw_setpoint(self, setpoint: Setpoint):
        if self.dry_run:
            self._log_dry_run_setpoint(setpoint)
            return
        if self.master is None:
            raise RuntimeError("MAVLink connection not started")

        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask_for(setpoint.kind),
            setpoint.n,
            setpoint.e,
            setpoint.d,
            setpoint.vn,
            setpoint.ve,
            setpoint.vd,
            0,
            0,
            0,
            setpoint.yaw_rad,
            0,
        )

    def _log_dry_run_setpoint(self, setpoint: Setpoint):
        """Print setpoints on change plus a 1 Hz heartbeat, never every tick.

        At 20 Hz a per-transmit log is unreadable, which would defeat the point
        of having a dry-run mode at all.
        """
        now = time.monotonic()
        signature = (
            setpoint.kind,
            round(setpoint.vn, 3), round(setpoint.ve, 3), round(setpoint.vd, 3),
            round(setpoint.n, 3), round(setpoint.e, 3), round(setpoint.d, 3),
            round(setpoint.yaw_rad, 3),
        )
        changed = signature != self._dry_run_last_signature
        if changed or now - self._dry_run_last_log_s >= 1.0:
            self._dry_run_last_signature = signature
            self._dry_run_last_log_s = now
            marker = "CHANGE" if changed else "  ..  "
            print(
                f"[DRY-RUN {marker}] {setpoint.kind.value:15s} "
                f"mask={type_mask_for(setpoint.kind):5d} "
                f"v=({setpoint.vn:+.2f},{setpoint.ve:+.2f},{setpoint.vd:+.2f}) "
                f"p=({setpoint.n:+.2f},{setpoint.e:+.2f},{setpoint.d:+.2f}) "
                f"yaw={rad_to_deg(setpoint.yaw_rad):+7.1f}deg  [{setpoint.label}]"
            )

    # =========================
    # MODE CONTROL
    # =========================

    def enable_offboard_control(self):
        """Backwards-compatible wrapper around :meth:`request_offboard`."""
        return self.request_offboard()

    def request_offboard(self, *, timeout_s: float = 10.0, attempts: int = 6) -> bool:
        """Command OFFBOARD and confirm PX4 actually entered it.

        Deliberately does NOT use ``master.set_mode("OFFBOARD")``.  With no
        HEARTBEAT seen yet, pymavlink cannot tell the autopilot is PX4, falls
        through to the ArduPilot mode table, prints "Unknown mode", returns None,
        and sends **zero bytes** -- while the caller happily reports success.

        Ground truth is the HEARTBEAT's ``custom_mode``, not the COMMAND_ACK and
        certainly not ``master.flightmode`` (which misreports OFFBOARD as
        "UNKNOWN" in every released pymavlink).

        Returns True only when OFFBOARD is confirmed observed.
        """
        if not self._stream_enabled.is_set():
            raise RuntimeError(
                "start_streaming() must run before requesting OFFBOARD; "
                "PX4 rejects the mode unless setpoints are already flowing"
            )

        deadline = time.time() + timeout_s
        for attempt in range(1, attempts + 1):
            if self.get_vehicle_snapshot().in_offboard:
                print("[*] OFFBOARD confirmed via HEARTBEAT.custom_mode")
                return True

            print(f"[*] Requesting OFFBOARD (attempt {attempt}/{attempts})...")
            self._send_set_mode_offboard()

            settle = time.time() + min(1.0, max(0.0, deadline - time.time()))
            while time.time() < settle:
                if self.get_vehicle_snapshot().in_offboard:
                    print("[*] OFFBOARD confirmed via HEARTBEAT.custom_mode")
                    return True
                time.sleep(0.05)

            if time.time() >= deadline:
                break

        state = self.get_vehicle_snapshot()
        print(
            f"[!] OFFBOARD NOT confirmed after {attempts} attempts "
            f"(custom_mode={state.custom_mode}, armed={state.armed})"
        )
        return False

    def _send_set_mode_offboard(self):
        """MAV_CMD_DO_SET_MODE with PX4's custom main mode for OFFBOARD."""
        if self.dry_run:
            print("[DRY-RUN] would send MAV_CMD_DO_SET_MODE base=1 custom_main=6 (OFFBOARD)")
            return
        if self.master is None:
            raise RuntimeError("MAVLink connection not started")

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            PX4_CUSTOM_MAIN_MODE_OFFBOARD,
            0, 0, 0, 0, 0,
        )

    def wait_for_armed(self, *, timeout_s: float = 300.0, poll_s: float = 0.2) -> bool:
        """Block until the operator arms, or the timeout expires.

        Arming is the operator's commit action and their abort switch, so this is
        the last gate before any motion.  Waiting for the armed flag rather than
        counting down avoids racing PX4's COM_DISARM_PRFLT pre-takeoff
        auto-disarm, which would otherwise disarm the vehicle mid-countdown.
        """
        deadline = time.time() + timeout_s
        announced = False

        while self.running and time.time() < deadline:
            if self.get_vehicle_snapshot().armed:
                print("[*] Vehicle ARMED")
                return True
            if not announced:
                print("[*] Waiting for operator to ARM (RC or QGC)...")
                announced = True
            time.sleep(poll_s)

        print("[!] Timed out waiting for arm")
        return False

    def send_velocity_and_yaw_target(self, vn: float, ve: float, vd: float, yaw_rad: float):
        """Send one MAVLink local-NED velocity command with an absolute yaw target."""
        if self.dry_run:
            self._log_dry_run_setpoint(
                Setpoint(
                    kind=SetpointKind.VELOCITY_YAW,
                    vn=vn, ve=ve, vd=vd, yaw_rad=yaw_rad,
                    label=self.current_setpoint().label,
                )
            )
            return
        if self.master is None:
            raise RuntimeError("MAVLink connection not started")

        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )

        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            0,
            0,
            0,
            vn,
            ve,
            vd,
            0,
            0,
            0,
            yaw_rad,
            0,
        )

    def move_to_target(
        self,
        final_target: LocalTarget,
        label: str,
        *,
        max_speed_m_s: float = MAX_FLIGHT_SPEED_M_S,
        timeout_s: Optional[float] = None,
    ) -> bool:
        """Drive to a local target with per-move speed and timeout limits.

        ``timeout_s=None`` derives the budget from the distance and speed via
        :func:`estimate_move_timeout`.  A single fixed value cannot serve both a
        0.25 m nudge and a 2.5 m pass-through at 0.15 m/s -- the latter needs
        more than 15 s of travel alone, so it used to report failure on every
        single run while flying correctly.  Passing an explicit value still
        overrides, which keeps existing callers unchanged.

        When :attr:`altitude_envelope` is set, the commanded ``d`` is clamped
        into it and the clamp is logged loudly: a bad detection would otherwise
        command flight into the floor, and a silent correction is worse than none
        because it hides the bad detection.
        """
        if not math.isfinite(max_speed_m_s) or max_speed_m_s <= 0.0:
            raise ValueError("max_speed_m_s must be a positive finite number")

        final_target = self._clamp_target_altitude(final_target, label)

        if timeout_s is None:
            # Only read vehicle state when we actually need it, so an explicit
            # timeout_s costs exactly the same number of snapshots as before.
            state = self.get_vehicle_snapshot()
            span = math.sqrt(
                (final_target.n - state.n) ** 2
                + (final_target.e - state.e) ** 2
                + (final_target.d - state.d) ** 2
            )
            timeout_s = estimate_move_timeout(span, max_speed_m_s)
            budget_note = f", Dist={span:.2f}m"
        elif not math.isfinite(timeout_s) or timeout_s <= 0.0:
            raise ValueError("timeout_s must be a positive finite number")
        else:
            budget_note = ""

        print(
            f"[*] Moving to {label}: "
            f"N={final_target.n:.2f}, E={final_target.e:.2f}, "
            f"D={final_target.d:.2f}, Yaw={rad_to_deg(final_target.yaw_rad):.1f} deg, "
            f"MaxSpeed={max_speed_m_s:.2f}m/s{budget_note}, "
            f"Budget={timeout_s:.1f}s"
        )

        start_time = time.time()
        period = 1.0 / POSITION_RATE_HZ
        last_state: Optional[VehicleState] = None

        try:
            while self.running:
                state = self.get_vehicle_snapshot()
                last_state = state
                err_n = final_target.n - state.n
                err_e = final_target.e - state.e
                err_d = final_target.d - state.d
                yaw_err = wrap_pi(final_target.yaw_rad - state.yaw_rad)

                dist_xyz = math.sqrt(err_n**2 + err_e**2 + err_d**2)
                if dist_xyz < POSITION_TOLERANCE_M and abs(rad_to_deg(yaw_err)) < YAW_TOLERANCE_DEG:
                    print(f"[*] Reached {label}")
                    self.send_velocity_and_yaw_target(0.0, 0.0, 0.0, final_target.yaw_rad)
                    return True

                if time.time() - start_time > timeout_s:
                    print(
                        f"[!] Timeout moving to {label} after {timeout_s:.1f}s "
                        f"({dist_xyz:.2f}m short). Treating as FAILED."
                    )
                    self.send_velocity_and_yaw_target(0.0, 0.0, 0.0, final_target.yaw_rad)
                    return False

                vn = KP_POS * err_n
                ve = KP_POS * err_e
                vd = KP_POS * err_d

                cmd_speed = math.sqrt(vn**2 + ve**2 + vd**2)
                if cmd_speed > max_speed_m_s:
                    vn = (vn / cmd_speed) * max_speed_m_s
                    ve = (ve / cmd_speed) * max_speed_m_s
                    vd = (vd / cmd_speed) * max_speed_m_s

                self.set_setpoint(
                    Setpoint(
                        kind=SetpointKind.VELOCITY_YAW,
                        vn=vn, ve=ve, vd=vd,
                        yaw_rad=final_target.yaw_rad,
                        label=label,
                    )
                )
                self.send_velocity_and_yaw_target(vn, ve, vd, final_target.yaw_rad)
                time.sleep(period)

            return False
        finally:
            # Every exit -- arrival, timeout, stop(), or an exception unwinding
            # through this frame -- leaves a stationary setpoint behind. Without
            # this, aborting mid-move left the last non-zero velocity latched.
            # Reuses the last state already read by the loop rather than taking a
            # fresh snapshot, so this costs no extra telemetry read.
            if last_state is not None:
                self._hold_from_state(
                    last_state, final_target.yaw_rad, label=f"after {label}"
                )

    def _clamp_target_altitude(self, target: LocalTarget, label: str) -> LocalTarget:
        """Apply the altitude envelope, announcing any correction."""
        envelope = self.altitude_envelope
        if envelope is None:
            return target

        highest = envelope.d_takeoff - envelope.max_alt_m
        lowest = envelope.d_takeoff - envelope.min_alt_m
        clamped = min(max(target.d, highest), lowest)
        if clamped == target.d:
            return target

        print(
            f"[!] ALTITUDE CLAMP on '{label}': commanded d={target.d:+.2f} "
            f"({envelope.d_takeoff - target.d:+.2f}m AGL) -> d={clamped:+.2f} "
            f"({envelope.d_takeoff - clamped:+.2f}m AGL); "
            f"envelope {envelope.min_alt_m:.2f}-{envelope.max_alt_m:.2f}m AGL"
        )
        return replace(target, d=clamped)

    def turn_around_180(self):
        """Hold position and command a 180-degree yaw change."""
        print("[*] 0 Gate Detections! Turning 180 degrees...")
        state = self.get_vehicle_snapshot()
        target = LocalTarget(
            n=state.n,
            e=state.e,
            d=state.d,
            yaw_rad=wrap_pi(state.yaw_rad + math.pi),
        )
        self.move_to_target(target, "180 Degree Turnaround")

    def land(self, *, timeout_s: float = 40.0, attempts: int = 3) -> bool:
        """Command AUTO.LAND and confirm the vehicle actually landed and disarmed.

        Three things this gets right that the previous version did not:

        * **The setpoint stream keeps running throughout.**  The old code sent one
          setpoint and then slept 0.5 s in silence before transmitting anything
          else.  Streaming after a LAND command is harmless -- PX4 has left
          offboard -- whereas stopping early leaves the vehicle in OFFBOARD with a
          dead stream if the command was not actually accepted.
        * **Success is judged on ``landed_state`` and the disarm flag, not the
          flight mode.**  PX4's failsafe framework may legitimately redirect
          AUTO.LAND to Descend, so a retry loop keyed on the mode would spin
          forever while the aircraft was landing perfectly well.
        * **It is idempotent**, so the mission-exit path can call it without
          having to track whether it already did.
        """
        if self._landing_confirmed:
            return True

        self.hold_position(label="pre-land hold")

        deadline = time.time() + timeout_s
        announced_touchdown = False
        for attempt in range(1, attempts + 1):
            print(f"[*] Commanding LAND (attempt {attempt}/{attempts})...")
            if not self._send_land_command():
                return False

            while time.time() < deadline:
                state = self.get_vehicle_snapshot()
                if not state.armed and state.heartbeat_received_s > 0.0:
                    print("[*] Landed and disarmed.")
                    self._landing_confirmed = True
                    return True
                if state.landed_state == 1 and not announced_touchdown:
                    announced_touchdown = True
                    print("[*] Touchdown detected; waiting for auto-disarm...")
                time.sleep(0.2)
                if time.time() - self._land_commanded_s > 5.0:
                    break  # re-issue: PX4 may have temporarily rejected it

            if time.time() >= deadline:
                break

        print("[!] Landing NOT confirmed within budget. Vehicle may still be airborne.")
        return False

    def _send_land_command(self) -> bool:
        self._land_commanded_s = time.time()
        if self.dry_run:
            print("[DRY-RUN] would send MAV_CMD_NAV_LAND")
            return True
        if self.master is None:
            print("[!] Cannot land: MAVLink connection not started")
            return False

        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                0, 0, 0, 0, 0, 0, 0, 0,
            )
            return True
        except Exception as exc:
            print(f"[!] Failed to send land command: {exc}")
            return False

    def safe_shutdown(self, reason: str, *, hold_s: float = 3.0) -> bool:
        """The one safe default: brake, give the operator a beat, then land.

        Called from every abort path.  The hold is deliberately short and
        bounded -- an unbounded hover is not "safe", it is a hover until the
        battery runs out, and streaming zeros the whole time actively defeats
        PX4's own offboard-loss failsafe.
        """
        print("\n" + "=" * 62)
        print(f"[!] SAFE SHUTDOWN: {reason}")
        print(f"[!] Holding {hold_s:.1f}s -- TAKE MANUAL CONTROL NOW IF REQUIRED")
        print("=" * 62)

        self.hold_position(label=f"safe shutdown: {reason}")
        deadline = time.time() + hold_s
        while self.running and time.time() < deadline:
            time.sleep(0.1)

        return self.land()

class Mission:
    """Minimal mission interface used by NavigationController.run_mission()."""
    def start(self):
        pass

    def run(self, nav: NavigationController):
        raise NotImplementedError

    def stop(self):
        pass


class GateMission(Mission):
    """Base class for missions that consume gate detections over UDP."""
    def __init__(
        self,
        udp_ip: str = UDP_IP,
        udp_port: int = UDP_PORT,
        *,
        cam_offset_right_m: float = CAM_OFFSET_RIGHT_M,
        cam_offset_down_m: float = CAM_OFFSET_DOWN_M,
        cam_yaw_offset_deg: float = CAM_YAW_OFFSET_DEG,
        detection_max_age_s: float = 0.5,
    ):
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.cam_offset_right_m = cam_offset_right_m
        self.cam_offset_down_m = cam_offset_down_m
        self.cam_yaw_offset_deg = cam_yaw_offset_deg
        # Detections older than this are treated as no-detection. Guards against
        # the vision process dying while the last good pose stays latched.
        self.detection_max_age_s = detection_max_age_s

        self._running = threading.Event()
        self._latest_detection: Optional[GateDetection] = None
        self._detection_lock = threading.Lock()
        self._udp_thread: Optional[threading.Thread] = None

    @property
    def running(self) -> bool:
        return self._running.is_set()

    def start(self):
        """Start the background UDP listener that receives gate detections."""
        if self.running:
            return

        with self._detection_lock:
            self._latest_detection = None

        self._running.set()
        self._udp_thread = threading.Thread(
            target=self._udp_gate_listener,
            daemon=True,
            name="gate-mission-udp-listener",
        )
        self._udp_thread.start()

    def stop(self):
        """Stop the UDP listener loop."""
        self._running.clear()

        if self._udp_thread is not None and self._udp_thread.is_alive():
            self._udp_thread.join(timeout=1.0)

    def get_latest_detection_snapshot(self) -> Optional[GateDetection]:
        with self._detection_lock:
            if self._latest_detection is None:
                return None
            # Like vehicle snapshots, callers get a copy instead of the shared object.
            return replace(self._latest_detection)

    def _udp_gate_listener(self):
        """Listen for the latest vision packet and keep only the newest gate."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((self.udp_ip, self.udp_port))
        sock.settimeout(0.2)

        print(f"[*] Listening for gate telemetry on UDP {self.udp_ip}:{self.udp_port}")

        try:
            while self.running:
                try:
                    data, _ = sock.recvfrom(4096)
                    payload = json.loads(data.decode("utf-8"))
                    gates = payload.get("gates", [])
                    if not gates:
                        continue

                    gate = gates[0]
                    det = GateDetection(
                        timestamp=time.time(),
                        dist=float(gate[0]),
                        forward=float(gate[1]),
                        right=float(gate[2]),
                        down=float(gate[3]),
                        roll=float(gate[4]),
                        pitch=float(gate[5]),
                        yaw_deg=float(gate[6]),
                    )

                    with self._detection_lock:
                        self._latest_detection = det

                except socket.timeout:
                    continue
                except OSError:
                    if self.running:
                        print("[!] UDP gate listener socket error.")
                    return
                except Exception as exc:
                    print(f"[!] UDP gate listener error: {exc}")
        finally:
            sock.close()

    def observe_gate(self, nav: NavigationController, duration: float = 1.0) -> Optional[GateDetection]:
        """Hold position, sample detections for a short window, then average them.

        Two corrections over the original:

        * **Stale detections are rejected.**  When no gate is visible the vision
          process still sends a real all-999 row every frame, which correctly
          overwrites the last detection -- so that case was always handled.  The
          hole is the vision *process dying*: nothing overwrites, and this loop
          would average ~25 copies of a frozen pose per second and announce a
          lock.  A dead pipeline was indistinguishable from a live one.
        * **Angles use a circular mean.**  Averaging +179 and -179 arithmetically
          gives 0, which is 180 degrees wrong, and gate yaws sit near the wrap
          routinely.

        Position is held with a real hold setpoint rather than zero velocity,
        which matters now that observation windows can be seconds long.
        """
        from .missions.frames import average_detections  # local: avoids an import cycle

        print(f"[*] Observing gate for {duration}s...")

        samples = []
        stale_seen = 0
        hold_state = nav.get_vehicle_snapshot()
        nav._hold_from_state(hold_state, label="observing")

        t0 = time.time()
        while nav.running and time.time() - t0 < duration:
            nav.send_velocity_and_yaw_target(0.0, 0.0, 0.0, hold_state.yaw_rad)

            det = self.get_latest_detection_snapshot()
            if is_valid_detection(det):
                if time.time() - det.timestamp <= self.detection_max_age_s:
                    samples.append(det)
                else:
                    stale_seen += 1

            time.sleep(0.04)

        if not samples:
            if stale_seen:
                print(
                    f"[!] Detections are STALE (older than "
                    f"{self.detection_max_age_s:.2f}s). The vision process may have died."
                )
            else:
                print("[!] No valid gate detections seen.")
            return None

        avg = average_detections(samples)
        print(
            f"[*] Lock acquired from {len(samples)} samples: "
            f"fwd={avg.forward:+.2f}m right={avg.right:+.2f}m down={avg.down:+.2f}m "
            f"yaw={avg.yaw_deg:+.1f}deg"
        )
        return avg

    def detection_to_gate_local(self, det: GateDetection, state: VehicleState):
        """Convert a camera-relative gate detection into local NED gate pose."""
        corrected_right = det.right + self.cam_offset_right_m
        corrected_down = det.down + self.cam_offset_down_m

        dn, de, dd = body_to_local(det.forward, corrected_right, corrected_down, state.yaw_rad)
        gate_n = state.n + dn
        gate_e = state.e + de
        gate_d = state.d + dd

        corrected_yaw_deg = det.yaw_deg + self.cam_yaw_offset_deg
        gate_yaw = wrap_pi(state.yaw_rad + deg_to_rad(corrected_yaw_deg))

        return gate_n, gate_e, gate_d, gate_yaw

    def build_standoff_target(
        self,
        nav: NavigationController,
        det: GateDetection,
        standoff_m: float,
    ) -> LocalTarget:
        """Build a target that stops in front of the gate by the given standoff."""
        state = nav.get_vehicle_snapshot()
        gate_n, gate_e, gate_d, gate_yaw = self.detection_to_gate_local(det, state)
        f_n, f_e = local_forward_vector(gate_yaw)

        return LocalTarget(
            n=gate_n - standoff_m * f_n,
            e=gate_e - standoff_m * f_e,
            d=gate_d,
            yaw_rad=gate_yaw,
        )

    def build_pass_through_target(
        self,
        nav: NavigationController,
        det: GateDetection,
        pass_dist_m: float,
    ) -> LocalTarget:
        """Build a target that carries the drone through and past the gate."""
        state = nav.get_vehicle_snapshot()
        gate_n, gate_e, gate_d, gate_yaw = self.detection_to_gate_local(det, state)
        f_n, f_e = local_forward_vector(gate_yaw)

        return LocalTarget(
            n=gate_n + pass_dist_m * f_n,
            e=gate_e + pass_dist_m * f_e,
            d=gate_d,
            yaw_rad=gate_yaw,
        )

    def run(self, nav: NavigationController):
        raise NotImplementedError
