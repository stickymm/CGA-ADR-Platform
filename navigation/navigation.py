import json
import math
import socket
import threading
import time
from dataclasses import dataclass, replace
from enum import Enum
from typing import Callable, List, Optional, Sequence, Tuple

from pymavlink import mavutil


# =========================
# CONFIG
# =========================

UDP_IP = "127.0.0.1"
UDP_PORT = 5050

MAVLINK_CONN = "udpin:127.0.0.1:14551"

POSITION_RATE_HZ = 10

# Arrival tolerance for move_to_target.
#
# WHY 0.15 m AND NOT 0.10 m
#   Position here comes from EKF2 fused optical flow + rangefinder, with no GPS
#   and no globally-consistent map.  PX4's flow-aided horizontal position
#   estimate carries roughly 0.1-0.2 m of 1-sigma noise indoors even when the
#   flow quality is good (EKF2_OF_N_MIN defaults to 0.15 rad/s of flow noise,
#   and the position solution is the integral of that).  A 0.10 m arrival test
#   therefore sits *inside* the noise floor of the very signal it is testing:
#   the vehicle can be physically stationary and centred and still fail the
#   test, burning its whole move budget every attempt.  Confirmed live -- a
#   0.25 m approach step consumed the full 15 s budget.
#
#   0.15 m is achievable by an airframe that is genuinely settled, and it is
#   still only 60% of the 0.25 m default approach step, so a step remains a real
#   move rather than an instant no-op.  The commit decision does NOT depend on
#   this number: it uses directly-observed body-frame lateral/vertical offsets
#   (0.20 / 0.25 m), so loosening the arrival tolerance does not loosen the one
#   gate that decides whether the drone flies at a gate.
POSITION_TOLERANCE_M = 0.15
YAW_TOLERANCE_DEG = 5.0

# Floor on a move's time budget, for hops too short for the distance-derived
# estimate to matter.
#
# WHY 5.0 s AND NOT 15.0 s
#   The floor used to dominate every approach step: a 0.25 m step at 0.35 m/s
#   ideally takes ~0.8 s, and was given 15 s.  With a 75 s per-gate deadline
#   that bought ~5 real attempts against a config advertising 14.  The three
#   numbers have to agree, so they are now derived from each other:
#
#       per-attempt cost  =  observation window + move budget
#       gate deadline     >=  max_approach_attempts * per-attempt cost
#
#   At the defaults (0.5 s observation, 5.0 s move floor, 14 attempts) that is
#   14 * 5.5 = 77 s, so GateLegConfig.approach_timeout_s defaults to 90 s and
#   the advertised attempt count is actually reachable.
#   ``contracts.attempt_budget_s`` computes this and the pre-flight banner
#   prints it, so a mis-tuned set of flags is visible before the props spin.
MOVE_TIMEOUT = 5.0

# --- VELOCITY CONTROLLER TUNING ---
MAX_FLIGHT_SPEED_M_S = 0.5
KP_POS = 1.2

# Maximum commanded yaw rate.  PX4's MPC_YAWRAUTO_MAX (default 45 deg/s) is a
# backstop, not a plan: this code has to control the rate itself, because a fast
# yaw injects rotation-induced optical flow that the estimator must cancel from
# gyro data alone, and degrading the position estimate is the last thing wanted
# during a search or a turn.  Every yaw change goes through
# NavigationController.slew_yaw rather than being stepped in one command.
MAX_YAW_RATE_DEG_S = 45.0

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

# --- ESTIMATOR_STATUS (MAVLink msg 230) ---
# PX4 drops OFFBOARD and reports `offboard_control_signal_lost` when the
# estimator's velocity innovation blows past COM_VEL_FS_EVH -- i.e. the real
# cause is flow degradation, not the offboard link.  Subscribing to
# ESTIMATOR_STATUS is what makes those two distinguishable in a console log
# instead of a multi-day misdiagnosis.
ESTIMATOR_ATTITUDE = 1
ESTIMATOR_VELOCITY_HORIZ = 2
ESTIMATOR_VELOCITY_VERT = 4
ESTIMATOR_POS_HORIZ_REL = 8
ESTIMATOR_POS_HORIZ_ABS = 16
ESTIMATOR_POS_VERT_ABS = 32
ESTIMATOR_POS_VERT_AGL = 64
ESTIMATOR_PRED_POS_HORIZ_REL = 256
ESTIMATOR_PRED_POS_HORIZ_ABS = 512

# The flags this airframe actually depends on: attitude, horizontal velocity
# (that is the flow), and a *relative* horizontal position (there is no GPS, so
# the absolute flags are expected to be clear and are not required).
ESTIMATOR_REQUIRED_FLAGS = (
    ("attitude", ESTIMATOR_ATTITUDE),
    ("velocity_horiz", ESTIMATOR_VELOCITY_HORIZ),
    ("pos_horiz_rel", ESTIMATOR_POS_HORIZ_REL),
    ("pos_vert_agl", ESTIMATOR_POS_VERT_AGL),
)

# An innovation test ratio at or above 1.0 means the EKF is rejecting that
# sensor.  PX4 uses 1.0 as the pass/fail line; warn a little before it.
ESTIMATOR_RATIO_WARN = 0.8

# --- VISION FEED STALENESS ---
# The third of three independent staleness signals.  Signal 1 is "no datagram
# arrived"; signal 2 is "the datagram carries the all-999 no-detection row".
# Signal 3 is this one: datagrams keep arriving, they carry a plausible gate,
# and the payload never changes -- a stalled camera or a wedged pipeline.
# Without it the drone flies a frozen gate position with a perfectly healthy
# looking feed.  15 frames is 0.5 s at the 30 fps the vision process publishes;
# real detections carry float noise in the third decimal, so 15 bit-identical
# frames from a live camera is not a thing that happens.
FROZEN_FEED_FRAMES = 15

# --- STATE HISTORY (detection deskew) ---
# A detection is stamped when it arrives, but it describes where the gate was
# when the *frame* was captured -- one vision pipeline latency earlier.  At
# Phase 1 speeds (0.35 m/s x 100 ms = 3.5 cm) that is ignorable; at Phase 2
# speeds through a turn it is not.  30 samples at the 30 Hz LOCAL_POSITION_NED
# rate is one second of history, which covers any plausible pipeline latency.
STATE_HISTORY_DEPTH = 30


# =========================
# BACKGROUND-THREAD LOGGING
# =========================
#
# Diagnostic chatter from the reader thread lands in the middle of the one
# blocking prompt the operator ever sees:
#
#     Type GO to continue (anything else aborts): [*] PX4 setpoint echo: mask=63
#
# Confirmed in a field log, 2026-08-12.  Garbling the single prompt where a
# mistyped answer aborts the mission is not an acceptable price for a
# diagnostic line, so purely-informational background logging can be muted for
# the duration of the prompt.
#
# SAFETY MESSAGES ARE NEVER ROUTED THROUGH THIS.  The deadman, the frozen-feed
# alarm, transmit failures and every abort reason all use bare `print`, so they
# are never suppressed.  This mutes diagnostics only.

_BACKGROUND_LOG_MUTED = threading.Event()


def mute_background_logs() -> None:
    """Silence purely-diagnostic background-thread logging."""
    _BACKGROUND_LOG_MUTED.set()


def unmute_background_logs() -> None:
    _BACKGROUND_LOG_MUTED.clear()


def background_logs_muted() -> bool:
    return _BACKGROUND_LOG_MUTED.is_set()


def _diagnostic_log(message: str) -> None:
    """Print unless diagnostics are muted.  Never used for a safety message."""
    if not _BACKGROUND_LOG_MUTED.is_set():
        print(message)


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
    # --- ESTIMATOR_STATUS, so flow degradation is not read as link loss ---
    estimator_flags: int = 0
    estimator_vel_ratio: float = 0.0
    estimator_pos_horiz_ratio: float = 0.0
    estimator_hagl_ratio: float = 0.0
    estimator_received_s: float = 0.0

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

    def estimator_faults(self) -> Tuple[str, ...]:
        """Which estimator signals this airframe needs are currently unhealthy.

        Empty when the estimator is fine, or when no ``ESTIMATOR_STATUS`` has
        been seen yet -- "no data" is reported separately by
        :attr:`estimator_seen` rather than being faked into a fault, because a
        PX4 build that does not stream message 230 is not an unhealthy
        estimator.
        """
        if self.estimator_received_s <= 0.0:
            return ()

        faults = [
            name
            for name, bit in ESTIMATOR_REQUIRED_FLAGS
            if not self.estimator_flags & bit
        ]
        for name, ratio in (
            ("velocity innovation", self.estimator_vel_ratio),
            ("horizontal position innovation", self.estimator_pos_horiz_ratio),
            ("height-above-ground innovation", self.estimator_hagl_ratio),
        ):
            if ratio >= ESTIMATOR_RATIO_WARN:
                faults.append(f"{name} ratio {ratio:.2f}")
        return tuple(faults)

    @property
    def estimator_seen(self) -> bool:
        return self.estimator_received_s > 0.0


@dataclass
class GateDetection:
    """One gate observation coming from the vision pipeline over UDP.

    ``row_index`` and ``source_fps`` are Phase 2 association inputs and default
    to the Phase 1 values, so every existing positional construction keeps
    working.  **Row index is not a stable gate identity** -- the vision process
    sorts rows nearest-first every frame, so row 0 becomes row 1 the moment two
    gates swap order.  It is carried only so a log line can say which row a fix
    came from; association is done geometrically in ``missions/association.py``.
    """
    timestamp: float
    dist: float
    forward: float
    right: float
    down: float
    roll: float
    pitch: float
    yaw_deg: float
    row_index: int = 0
    source_fps: float = 0.0


@dataclass
class LocalTarget:
    """Position and yaw target expressed in local NED coordinates."""
    n: float
    e: float
    d: float
    yaw_rad: float


@dataclass(frozen=True)
class MissionOutcome:
    """What ``run_mission`` did.  Returned instead of ``None``.

    ``exit_code`` is the thing a ``__main__`` actually wants, and deriving it
    here means every entry point agrees on what "success" means: the mission
    completed *and* the aircraft is confirmed down.
    """

    ok: bool
    reason: str
    elapsed_s: float = 0.0
    landed: bool = False

    @property
    def exit_code(self) -> int:
        return 0 if (self.ok and self.landed) else 1


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
# LINKS -- the one seam every outbound byte passes through
# =========================
#
# Before this existed, ``--dry-run`` was an ``if self.dry_run:`` branch inlined
# into five separate senders.  It worked, but it was a convention rather than a
# mechanism: the *next* command someone adds is transmitted during a bench test
# unless they remember to add a sixth branch.  On an aircraft with props fitted
# that is not a defect class worth keeping.
#
# So there is now exactly one way out of this process -- a Link -- and
# ``DryRunLink`` is a Link that cannot transmit.  A command added later without
# its author having heard of dry-run mode still cannot reach the wire, because
# there is no other path to the wire.
#
# Reads deliberately do NOT go through the Link.  Inbound telemetry must keep
# flowing in dry-run mode (that is the entire point of the bench test), and it
# is read from ``master.recv_match`` by the reader thread.


class PymavlinkLink:
    """The real link: MAVLink out over a pymavlink connection."""

    def __init__(self, master):
        self._master = master

    @property
    def suppresses_commands(self) -> bool:
        return False

    @property
    def target_system(self) -> int:
        return getattr(self._master, "target_system", 0)

    @property
    def target_component(self) -> int:
        return getattr(self._master, "target_component", 0)

    def set_position_target(
        self,
        type_mask: int,
        n: float, e: float, d: float,
        vn: float, ve: float, vd: float,
        yaw_rad: float,
        *,
        label: str = "",
    ) -> None:
        self._master.mav.set_position_target_local_ned_send(
            0,
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            n, e, d,
            vn, ve, vd,
            0, 0, 0,
            yaw_rad,
            0,
        )

    def command_long(
        self,
        command: int,
        params: Sequence[float] = (),
        *,
        description: str = "",
    ) -> None:
        values = list(params) + [0.0] * (7 - len(params))
        self._master.mav.command_long_send(
            self.target_system,
            self.target_component,
            command,
            0,
            *values[:7],
        )

    def heartbeat(self) -> None:
        self._master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0,
        )

    def close(self) -> None:
        close = getattr(self._master, "close", None)
        if callable(close):
            close()


class NullLink:
    """A link with nowhere to go.  Used by the offline model."""

    @property
    def suppresses_commands(self) -> bool:
        # The offline controller intercepts its senders upstream and its
        # kinematic model acts on them, so commands ARE acted on -- they just do
        # not travel over a wire. This is not a bench-only no-op link.
        return False

    @property
    def target_system(self) -> int:
        return 0

    @property
    def target_component(self) -> int:
        return 0

    def set_position_target(self, *args, **kwargs) -> None:
        return

    def command_long(self, *args, **kwargs) -> None:
        return

    def heartbeat(self) -> None:
        return

    def close(self) -> None:
        return


class DryRunLink:
    """A link that prints what it would have sent and transmits nothing.

    Wraps an optional inner link purely so the log can show the real target
    system/component when a vehicle is actually connected.  It never calls the
    inner link's senders -- that is the invariant this class exists to hold.

    Setpoints are logged on *change* plus a 1 Hz keep-alive rather than on every
    transmit: at 20 Hz a per-tick log is unreadable, which would defeat the point
    of having a dry-run mode at all.
    """

    def __init__(self, inner=None, *, log: Callable[[str], None] = print):
        self._inner = inner
        self._log = log
        self._last_signature = None
        self._last_log_s = 0.0
        self.sent_setpoints = 0
        self.sent_commands = 0

    @property
    def suppresses_commands(self) -> bool:
        return True

    @property
    def target_system(self) -> int:
        return self._inner.target_system if self._inner is not None else 0

    @property
    def target_component(self) -> int:
        return self._inner.target_component if self._inner is not None else 0

    def set_position_target(
        self,
        type_mask: int,
        n: float, e: float, d: float,
        vn: float, ve: float, vd: float,
        yaw_rad: float,
        *,
        label: str = "",
    ) -> None:
        self.sent_setpoints += 1
        now = time.monotonic()
        signature = (
            type_mask,
            round(vn, 3), round(ve, 3), round(vd, 3),
            round(n, 3), round(e, 3), round(d, 3),
            round(yaw_rad, 3),
        )
        changed = signature != self._last_signature
        if not changed and now - self._last_log_s < 1.0:
            return

        self._last_signature = signature
        self._last_log_s = now
        marker = "CHANGE" if changed else "  ..  "
        self._log(
            f"[DRY-RUN {marker}] setpoint mask={type_mask:5d} "
            f"v=({vn:+.2f},{ve:+.2f},{vd:+.2f}) "
            f"p=({n:+.2f},{e:+.2f},{d:+.2f}) "
            f"yaw={rad_to_deg(yaw_rad):+7.1f}deg  [{label}]"
        )

    def command_long(
        self,
        command: int,
        params: Sequence[float] = (),
        *,
        description: str = "",
    ) -> None:
        self.sent_commands += 1
        shown = description or f"command {command}"
        rendered = ", ".join(f"{value:g}" for value in params)
        self._log(f"[DRY-RUN] would send {shown}" + (f" ({rendered})" if rendered else ""))

    def heartbeat(self) -> None:
        return  # our own heartbeat is outbound traffic like anything else

    def close(self) -> None:
        if self._inner is not None:
            self._inner.close()


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

def body_to_local(
    forward: float,
    right: float,
    down: float,
    yaw_rad: float,
    pitch_rad: float = 0.0,
    roll_rad: float = 0.0,
):
    """Rotate a body-frame (FRD) vector into local NED using full attitude.

    Standard aerospace 3-2-1 sequence -- yaw about z, then pitch about y, then
    roll about x -- matching PX4's ``ATTITUDE`` message convention::

        R = Rz(yaw) @ Ry(pitch) @ Rx(roll)

          [ cy*cp   cy*sp*sr - sy*cr   cy*sp*cr + sy*sr ]
          [ sy*cp   sy*sp*sr + cy*cr   sy*sp*cr - cy*sr ]
          [ -sp     cp*sr              cp*cr            ]

    ``pitch_rad`` and ``roll_rad`` default to zero, so every existing caller
    that passes only a yaw gets **bit-identical** results to the old yaw-only
    implementation -- the two legacy missions cannot silently change behaviour.
    Callers that have the vehicle's real attitude should pass it: a gate 3 m
    ahead observed at 10 degrees of body pitch is mis-placed vertically by
    ``3 * sin(10 deg) = 0.52 m`` when pitch is ignored, and that error feeds
    straight into the commanded altitude.

    This is the single implementation; ``missions.frames.rotate_body_to_ned``
    is a keyword-only alias of it rather than a second copy of the matrix.
    """
    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
    cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
    cr, sr = math.cos(roll_rad), math.sin(roll_rad)

    dn = (cy * cp) * forward + (cy * sp * sr - sy * cr) * right + (cy * sp * cr + sy * sr) * down
    de = (sy * cp) * forward + (sy * sp * sr + cy * cr) * right + (sy * sp * cr - cy * sr) * down
    dd = (-sp) * forward + (cp * sr) * right + (cp * cr) * down
    return dn, de, dd

def local_forward_vector(yaw_rad: float):
    return math.cos(yaw_rad), math.sin(yaw_rad)

def _safe_float(value, default: float = float("nan")) -> float:
    """Parse a wire value without letting one malformed row kill the listener."""
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


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
        #
        # This flag now selects a *Link*; it is not consulted anywhere else.
        # Nothing in this class may test `self.dry_run` before transmitting --
        # if it did, the seam would be back to being a convention.
        self.dry_run = dry_run

        self.master: Optional[mavutil.mavfile] = None
        # A dry run must be able to print its commands before (and without) a
        # connection, exactly as the old inline branches did.
        self.link = DryRunLink() if dry_run else None

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
        self._prime_quiescence_noted = False

        self._landing_confirmed = False
        self._land_commanded_s = 0.0

        # Latest POSITION_TARGET_LOCAL_NED (85): PX4's echo of the setpoint it
        # believes it is tracking, which is the only external confirmation that
        # our type_mask was interpreted the way we intended.
        self._echo_lock = threading.Lock()
        self._echo = None
        self._echo_received_s = 0.0
        self._echo_last_log_s = 0.0
        self._echo_last_mask = None

        # Short history of vehicle poses, so a detection stamped in the past can
        # be localized against where the vehicle actually was at that instant.
        self._history: List[VehicleState] = []
        self._history_lock = threading.Lock()
        self.history_depth = STATE_HISTORY_DEPTH

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

        # One seam, chosen once.  Everything outbound goes through it from here.
        wire = PymavlinkLink(self.master)
        self.link = DryRunLink(wire) if self.dry_run else wire
        if self.dry_run:
            print("[*] DRY RUN: a DryRunLink is installed; nothing will be transmitted")

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
            link = self.link
            if link is not None:
                try:
                    link.heartbeat()
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
                # PX4's echo of the setpoint it is actually tracking. The only
                # external confirmation that a type_mask was interpreted the way
                # this code intended, which is what makes a --dry-run bench
                # session conclusive rather than merely quiet.
                mavutil.mavlink.MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED: 5.0,
                # Flow health. Without this, PX4 dropping offboard because the
                # estimator rejected the flow reads in the log as "offboard
                # signal lost" and gets debugged as a link problem for days.
                mavutil.mavlink.MAVLINK_MSG_ID_ESTIMATOR_STATUS: 2.0,
            }

        link = self._require_link()
        for message_id, rate_hz in intervals_hz.items():
            interval_us = 1e6 / rate_hz
            link.command_long(
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                (float(message_id), float(interval_us)),
                description=f"MAV_CMD_SET_MESSAGE_INTERVAL msg {message_id} at {rate_hz:.0f} Hz",
            )
            time.sleep(0.05)

    def _require_link(self):
        """The link, or a clear error.  Every sender goes through this."""
        link = self.link
        if link is None:
            raise RuntimeError("MAVLink connection not started")
        return link

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
            if self.link is not None:
                try:
                    self.link.close()
                except Exception:
                    pass

    def run_mission(self, mission) -> "MissionOutcome":
        """Run one mission object against this controller with managed lifecycle.

        ``start()`` is inside the ``try`` so that a failure part-way through it --
        after the reader thread has already been spawned -- still runs the
        cleanup rather than leaking the thread.

        Returns a :class:`MissionOutcome` rather than ``None``: a caller that
        wants a process exit code should not have to infer one from stdout.
        """
        started_s = time.time()
        try:
            self.start()
            mission.start()
            mission.run(self)
        except KeyboardInterrupt:
            return MissionOutcome(
                False, "operator interrupt", elapsed_s=time.time() - started_s
            )
        except Exception as exc:
            return MissionOutcome(
                False, f"unhandled error: {exc!r}", elapsed_s=time.time() - started_s
            )
        finally:
            mission.stop()
            self.stop()

        return MissionOutcome(
            True,
            "mission returned normally",
            elapsed_s=time.time() - started_s,
            landed=self._landing_confirmed,
        )

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
        has run, ``target_system``/``target_component`` hold the autopilot's ids;
        before that they are 0 and we accept anything so the first heartbeat can
        be observed.

        This is applied to **every** state-bearing message, not just HEARTBEAT.
        A ground station or a second vehicle sharing this link publishes its own
        ``LOCAL_POSITION_NED`` and ``ATTITUDE``; folding those in means flying
        one aircraft using another one's pose, and the log would look perfect
        while it happened.
        """
        if self.master is None:
            return False
        system = getattr(self.master, "target_system", 0)
        component = getattr(self.master, "target_component", 0)
        if system and msg.get_srcSystem() != system:
            return False
        if component and msg.get_srcComponent() != component:
            return False
        return True

    def _mavlink_reader(self):
        """Continuously fold incoming MAVLink messages into VehicleState."""
        if self.master is None:
            return

        while self.running:
            msg = self.master.recv_match(blocking=True, timeout=0.2)
            if msg is None:
                continue
            if not self._is_autopilot(msg):
                continue

            msg_type = msg.get_type()
            now = time.time()

            if msg_type == "POSITION_TARGET_LOCAL_NED":
                self._record_setpoint_echo(msg, now)
                continue

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
                elif msg_type == "HEARTBEAT":
                    # The only source of truth for arm state and flight mode.
                    # Both are needed before we command any motion.
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
                elif msg_type == "ESTIMATOR_STATUS":
                    self._vehicle.estimator_flags = int(msg.flags)
                    self._vehicle.estimator_vel_ratio = float(msg.vel_ratio)
                    self._vehicle.estimator_pos_horiz_ratio = float(msg.pos_horiz_ratio)
                    self._vehicle.estimator_hagl_ratio = float(
                        getattr(msg, "hagl_ratio", 0.0)
                    )
                    self._vehicle.estimator_received_s = now
                else:
                    continue
                snapshot = replace(self._vehicle)

            if msg_type == "LOCAL_POSITION_NED":
                self._record_history(snapshot)

    # =========================
    # STATE HISTORY  (detection deskew)
    # =========================

    def _record_history(self, state: VehicleState) -> None:
        with self._history_lock:
            self._history.append(state)
            if len(self._history) > self.history_depth:
                del self._history[: len(self._history) - self.history_depth]

    def state_history(self) -> List[VehicleState]:
        with self._history_lock:
            return list(self._history)

    def state_at(self, timestamp_s: float) -> VehicleState:
        """The recorded pose closest in time to ``timestamp_s``.

        A detection describes where the gate was when its *frame* was captured,
        one vision-pipeline latency before the datagram arrived.  Localizing it
        against the pose the vehicle holds *now* smears that latency into the
        gate position.  At 0.35 m/s that is 3.5 cm and ignorable; through a
        Phase 2 turn at 1 m/s it is not.

        Falls back to the current snapshot when there is no history, so this is
        always safe to call.
        """
        with self._history_lock:
            history = list(self._history)
        if not history:
            return self.get_vehicle_snapshot()
        return min(history, key=lambda s: abs(s.position_received_s - timestamp_s))

    # =========================
    # SETPOINT ECHO  (POSITION_TARGET_LOCAL_NED, msg 85)
    # =========================

    def _record_setpoint_echo(self, msg, now: float) -> None:
        """Store PX4's echo of the setpoint it believes it is tracking.

        Logged only when the interpreted ``type_mask`` changes, plus a 10 s
        keep-alive.  A mask that comes back different from the one sent is the
        single clearest sign that PX4 did not read the setpoint the way this
        code meant it, and it is otherwise invisible until the vehicle moves.
        """
        mask = int(getattr(msg, "type_mask", 0))
        echo = (
            mask,
            float(msg.x), float(msg.y), float(msg.z),
            float(msg.vx), float(msg.vy), float(msg.vz),
            float(msg.yaw),
        )
        with self._echo_lock:
            self._echo = echo
            self._echo_received_s = now
            changed = mask != self._echo_last_mask
            due = now - self._echo_last_log_s >= 10.0
            if changed or due:
                self._echo_last_mask = mask
                self._echo_last_log_s = now
            else:
                return

        # Diagnostic, not a safety message: muted while the operator prompt is
        # blocking so it cannot land in the middle of "Type GO to continue".
        _diagnostic_log(
            f"[*] PX4 setpoint echo: mask={mask} "
            f"p=({echo[1]:+.2f},{echo[2]:+.2f},{echo[3]:+.2f}) "
            f"v=({echo[4]:+.2f},{echo[5]:+.2f},{echo[6]:+.2f}) "
            f"yaw={rad_to_deg(echo[7]):+.1f}deg"
        )

    def setpoint_echo(self):
        """``(type_mask, x, y, z, vx, vy, vz, yaw)`` last echoed by PX4, or None."""
        with self._echo_lock:
            return self._echo

    def estimator_note(self) -> str:
        """A log suffix naming the estimator's state, or "" when it is healthy.

        Appended to every "we lost offboard" style message.  PX4 sets
        ``offboard_control_signal_lost`` when the *estimator* fails its velocity
        innovation check past ``COM_VEL_FS_EVH``, so the message an operator sees
        blames the link for what is actually flow degradation.  This makes the
        distinction visible in the same line.
        """
        state = self.get_vehicle_snapshot()
        if not state.estimator_seen:
            return " (no ESTIMATOR_STATUS seen; cannot tell link loss from flow loss)"
        faults = state.estimator_faults()
        if not faults:
            return " (estimator healthy, so this really is an offboard/link problem)"
        return (
            " (ESTIMATOR DEGRADED: " + ", ".join(faults) +
            " -- suspect optical flow, not the offboard link)"
        )

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
            if stale and not self._setpoint_is_being_acted_on():
                # EXPECTED QUIESCENCE, not a deadman condition.
                #
                # PX4 only acts on offboard setpoints while armed AND in
                # OFFBOARD.  Until both are true -- which covers the whole of
                # STEP 5, where the priming setpoint sits unrefreshed for as long
                # as the operator takes to arm -- an unrefreshed zero-velocity
                # command cannot make the vehicle do anything at all.
                #
                # The old code fired the full in-flight alarm here on every
                # single pad run.  The action it took was safe; the message was
                # not, because a warning that cries wolf before every flight is a
                # warning the operator has already learned to scroll past by the
                # time it means something.  Re-stamp and stay quiet.
                self._restamp_setpoint()
                if not self._prime_quiescence_noted:
                    self._prime_quiescence_noted = True
                    print(
                        f"[*] Setpoint '{setpoint.label}' is idling while the vehicle "
                        f"is not yet armed in OFFBOARD -- expected before takeoff, "
                        f"holding zeros."
                    )
                setpoint = self.current_setpoint()
            elif stale:
                if not self._deadman_tripped:
                    self._deadman_tripped = True
                    print(
                        f"[!] DEADMAN: velocity setpoint '{setpoint.label}' not "
                        f"refreshed for {SETPOINT_STALE_S:.2f}s while armed in "
                        f"OFFBOARD. Braking to a hold."
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

    def _setpoint_is_being_acted_on(self) -> bool:
        """True when PX4 would actually fly what this process is streaming.

        Armed *and* in OFFBOARD.  Either one alone means the setpoint stream is
        being published into a vehicle that is not following it -- on the pad
        before arming, or in a pilot-controlled mode -- so a stale setpoint is
        inert rather than dangerous.  This is the whole difference between the
        arm-wait quiescence and a genuine in-flight stall.
        """
        state = self.get_vehicle_snapshot()
        return state.armed and state.in_offboard

    def _restamp_setpoint(self) -> None:
        """Refresh the current setpoint's issue time without changing its value.

        Keeps the priming zeros fresh through an arbitrarily long arm wait, so
        the stream still satisfies PX4's COM_OF_LOSS_T freshness requirement
        without the deadman having to fire to achieve it.
        """
        with self._setpoint_lock:
            self._setpoint = replace(self._setpoint, issued_s=time.time())

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
        self._require_link().set_position_target(
            type_mask_for(setpoint.kind),
            setpoint.n,
            setpoint.e,
            setpoint.d,
            setpoint.vn,
            setpoint.ve,
            setpoint.vd,
            setpoint.yaw_rad,
            label=f"{setpoint.kind.value}: {setpoint.label}",
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
        self._require_link().command_long(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            (MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PX4_CUSTOM_MAIN_MODE_OFFBOARD),
            description="MAV_CMD_DO_SET_MODE base=1 custom_main=6 (OFFBOARD)",
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
        self._require_link().set_position_target(
            type_mask_for(SetpointKind.VELOCITY_YAW),
            0.0, 0.0, 0.0,
            vn, ve, vd,
            yaw_rad,
            label=f"velocity_yaw: {self.current_setpoint().label}",
        )

    def move_to_target(
        self,
        final_target: LocalTarget,
        label: str,
        *,
        max_speed_m_s: float = MAX_FLIGHT_SPEED_M_S,
        timeout_s: Optional[float] = None,
        tolerance_m: float = POSITION_TOLERANCE_M,
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
        if not math.isfinite(tolerance_m) or tolerance_m <= 0.0:
            raise ValueError("tolerance_m must be a positive finite number")

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
            # The budget must be derived with the SAME tolerance the arrival test
            # uses, or the move is given time to reach a precision it is not
            # being judged on (or, worse, judged on a precision it was never
            # given time to reach).
            timeout_s = estimate_move_timeout(
                span, max_speed_m_s, tolerance_m=tolerance_m
            )
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
            f"Tol={tolerance_m:.2f}m, Budget={timeout_s:.1f}s"
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
                if dist_xyz < tolerance_m and abs(rad_to_deg(yaw_err)) < YAW_TOLERANCE_DEG:
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

    def slew_yaw(
        self,
        target_yaw_rad: float,
        *,
        label: str = "yaw slew",
        max_rate_deg_s: float = MAX_YAW_RATE_DEG_S,
        tolerance_deg: float = YAW_TOLERANCE_DEG,
        timeout_s: Optional[float] = None,
    ) -> bool:
        """Rotate to an absolute yaw at a bounded rate, holding position.

        WHY THIS EXISTS RATHER THAN ONE BIG YAW COMMAND
            Commanding a 180-degree yaw in a single setpoint asks PX4 to turn as
            fast as ``MPC_YAWRAUTO_MAX`` allows.  That parameter defaults to
            45 deg/s, but it is a *firmware default*: an airframe startup script
            can raise it, nobody on the flight line reads it, and this code then
            has no control at all over how fast its own aircraft spins.

            The rate matters on this airframe specifically.  Position comes from
            optical flow, and yawing injects rotation-induced flow across the
            whole image that the estimator has to cancel using gyro data alone.
            A fast yaw is therefore a direct attack on the position estimate --
            and every place this is called from is a *recovery*, i.e. exactly
            when the estimate is already the thing being relied on.

            Note the offline model rate-limits yaw internally at 45 deg/s, so an
            offline run looks identical whether or not this exists.  That is why
            this gap survived a green offline mission.

        The intermediate setpoints are BRAKE_HOLD_ALT, so position and altitude
        are held throughout and the deadman never sees a transient.

        Returns True when the measured yaw arrives within ``tolerance_deg``.
        """
        if not math.isfinite(max_rate_deg_s) or max_rate_deg_s <= 0.0:
            raise ValueError("max_rate_deg_s must be a positive finite number")

        start = self.get_vehicle_snapshot()
        total_err_deg = abs(rad_to_deg(wrap_pi(target_yaw_rad - start.yaw_rad)))
        ideal_s = total_err_deg / max_rate_deg_s
        if timeout_s is None:
            timeout_s = MOVE_TIMEOUT_MARGIN_S + MOVE_TIMEOUT_SCALE * ideal_s

        print(
            f"[*] Yaw slew to {rad_to_deg(target_yaw_rad):+.1f} deg for '{label}': "
            f"{total_err_deg:.1f} deg at {max_rate_deg_s:.0f} deg/s "
            f"(~{ideal_s:.1f}s, budget {timeout_s:.1f}s)"
        )

        max_rate_rad_s = deg_to_rad(max_rate_deg_s)
        commanded = start.yaw_rad
        period = 1.0 / POSITION_RATE_HZ
        started = time.time()
        previous = time.monotonic()

        try:
            while self.running:
                now = time.monotonic()
                dt = min(0.5, now - previous)
                previous = now

                state = self.get_vehicle_snapshot()
                measured_err = wrap_pi(target_yaw_rad - state.yaw_rad)
                if abs(rad_to_deg(measured_err)) <= tolerance_deg:
                    print(f"[*] Yaw slew '{label}' complete")
                    self._hold_from_state(state, target_yaw_rad, label=f"after {label}")
                    return True

                if time.time() - started > timeout_s:
                    print(
                        f"[!] Yaw slew '{label}' timed out after {timeout_s:.1f}s, "
                        f"{rad_to_deg(measured_err):+.1f} deg short"
                    )
                    return False

                # Ramp the *commanded* yaw, never the measured one: chasing the
                # measurement would let a lagging airframe stall the ramp, and
                # let a fast one overshoot the rate limit on the next tick.
                step = wrap_pi(target_yaw_rad - commanded)
                limit = max_rate_rad_s * dt
                commanded = wrap_pi(
                    commanded + math.copysign(min(abs(step), limit), step)
                )

                self.set_setpoint(
                    Setpoint(
                        kind=SetpointKind.BRAKE_HOLD_ALT,
                        vn=0.0,
                        ve=0.0,
                        d=state.d,
                        yaw_rad=commanded,
                        label=label,
                    )
                )
                time.sleep(period)

            return False
        finally:
            if self.running:
                self._hold_from_state(
                    self.get_vehicle_snapshot(), commanded, label=f"after {label}"
                )

    def turn_around_180(self):
        """Hold position and turn 180 degrees at a bounded yaw rate."""
        print("[*] 0 Gate Detections! Turning 180 degrees...")
        state = self.get_vehicle_snapshot()
        return self.slew_yaw(
            wrap_pi(state.yaw_rad + math.pi), label="180 Degree Turnaround"
        )

    def land(self, *, timeout_s: float = 40.0, reissue_after_s: float = 5.0) -> bool:
        """Command AUTO.LAND and confirm the vehicle actually landed and disarmed.

        **The budget is the deadline, not an attempt count.**  The previous
        version wrapped a 5 s inner wait in a 3-attempt outer loop, so it gave up
        at ~15 s of a 40 s budget, printed "Landing NOT confirmed" while the
        aircraft was landing perfectly, and made Phase 1A exit 1 on a good
        landing.  A LAND command is *re-issued* every ``reissue_after_s`` (PX4
        can transiently reject one), but re-issuing is not what bounds the wait:
        the wait is bounded by ``timeout_s`` and nothing else, so the whole
        budget is actually used and the returned verdict is the true one.

        Also true of this implementation, and worth keeping:

        * **The setpoint stream keeps running throughout.**  Streaming after a
          LAND command is harmless -- PX4 has left offboard -- whereas stopping
          early leaves the vehicle in OFFBOARD with a dead stream if the command
          was not actually accepted.
        * **Success is judged on ``landed_state`` and the disarm flag, not the
          flight mode.**  PX4's failsafe framework may legitimately redirect
          AUTO.LAND to Descend, so a retry loop keyed on the mode would spin
          forever while the aircraft was landing perfectly well.
        * **It is idempotent**, so the mission-exit path can call it without
          having to track whether it already did.
        """
        if self._landing_confirmed:
            return True

        entry = self.get_vehicle_snapshot()
        if (
            entry.heartbeat_received_s > 0.0
            and not entry.armed
            and entry.landed_state != 2
        ):
            # Pre-arm abort. There is nothing to land, and saying "Landed and
            # disarmed" here reads in the log as a completed flight that never
            # happened.
            print("[*] Vehicle is disarmed and on the ground; no landing required.")
            self._landing_confirmed = True
            return True

        self.hold_position(label="pre-land hold")

        if self._require_link().suppresses_commands:
            # Nothing was transmitted, so the vehicle was never commanded to
            # land and never will be. Waiting out the full budget to report a
            # failure that is an artefact of the bench setup helps nobody.
            self._send_land_command()
            print("[*] DRY RUN: LAND was not transmitted, so no landing can be observed.")
            self._landing_confirmed = True
            return True

        deadline = time.time() + timeout_s
        announced_touchdown = False
        attempt = 0

        while time.time() < deadline:
            attempt += 1
            print(
                f"[*] Commanding LAND (issue {attempt}, "
                f"{deadline - time.time():.1f}s of the {timeout_s:.0f}s budget left)..."
            )
            if not self._send_land_command():
                return False

            reissue_at = time.time() + reissue_after_s
            while time.time() < deadline:
                state = self.get_vehicle_snapshot()
                if not state.armed and state.heartbeat_received_s > 0.0:
                    print("[*] Landed and disarmed.")
                    self._landing_confirmed = True
                    return True
                if state.landed_state == 1 and not announced_touchdown:
                    announced_touchdown = True
                    print("[*] Touchdown detected; waiting for auto-disarm...")
                if time.time() >= reissue_at:
                    break  # re-issue: PX4 may have temporarily rejected it
                time.sleep(0.2)

        state = self.get_vehicle_snapshot()
        print(
            f"[!] Landing NOT confirmed within the full {timeout_s:.0f}s budget "
            f"({attempt} LAND commands issued, armed={state.armed}, "
            f"landed_state={state.landed_state}). Vehicle may still be airborne."
            + self.estimator_note()
        )
        return False

    def _send_land_command(self) -> bool:
        self._land_commanded_s = time.time()
        try:
            self._require_link().command_long(
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                (),
                description="MAV_CMD_NAV_LAND",
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

        **THE HOLD IS A COURTESY.  THE LANDING IS NOT OPTIONAL.**
        A second Ctrl-C during the hold used to raise ``KeyboardInterrupt`` out
        of ``time.sleep`` and propagate straight past ``return self.land()``, so
        an impatient operator got an aircraft that was never commanded to land.
        Confirmed in a field log, 2026-08-12: two interrupts, zero LAND commands
        sent.  An interrupt now *shortens* the hold rather than cancelling the
        landing -- it asks for something more urgent, not less.
        """
        print("\n" + "=" * 62)
        print(f"[!] SAFE SHUTDOWN: {reason}")
        print(f"[!] Holding {hold_s:.1f}s -- TAKE MANUAL CONTROL NOW IF REQUIRED")
        print("=" * 62)

        try:
            self.hold_position(label=f"safe shutdown: {reason}")
        except Exception as exc:
            # Losing the hold is bad but it must not cost us the landing.
            print(f"[!] Could not latch the safe-shutdown hold: {exc}")

        deadline = time.time() + hold_s
        try:
            while self.running and time.time() < deadline:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("[!] Hold interrupted by the operator -- going straight to LAND")

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
        frozen_feed_frames: int = FROZEN_FEED_FRAMES,
        min_observation_samples: int = 1,
    ):
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.cam_offset_right_m = cam_offset_right_m
        self.cam_offset_down_m = cam_offset_down_m
        self.cam_yaw_offset_deg = cam_yaw_offset_deg
        # Detections older than this are treated as no-detection. Guards against
        # the vision process dying while the last good pose stays latched.
        self.detection_max_age_s = detection_max_age_s
        self.frozen_feed_frames = frozen_feed_frames
        # Fewest averaged samples that may be reported as a lock. Defaults to 1
        # so nothing that constructs a GateMission directly changes behaviour;
        # the missions raise it via the CLI.
        self.min_observation_samples = max(1, int(min_observation_samples))

        self._running = threading.Event()
        self._latest_detection: Optional[GateDetection] = None
        self._latest_detections: List[GateDetection] = []
        self._detection_lock = threading.Lock()
        self._udp_thread: Optional[threading.Thread] = None

        # --- staleness signal 3: payload hash unchanged over N frames ---
        self._payload_signature = None
        self._identical_frames = 0
        self._frames_seen = 0

    @property
    def running(self) -> bool:
        return self._running.is_set()

    def start(self):
        """Start the background UDP listener that receives gate detections."""
        if self.running:
            return

        with self._detection_lock:
            self._latest_detection = None
            self._latest_detections = []
            self._payload_signature = None
            self._identical_frames = 0
            self._frames_seen = 0

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
        """The nearest valid gate in the newest packet, or None.

        Phase 1 consumes only this.  Phase 2 uses
        :meth:`get_latest_detections_snapshot`, which keeps every row.
        """
        with self._detection_lock:
            if self._latest_detection is None:
                return None
            # Like vehicle snapshots, callers get a copy instead of the shared object.
            return replace(self._latest_detection)

    def get_latest_detections_snapshot(self) -> List[GateDetection]:
        """Every valid gate in the newest packet, nearest first.

        The wire format always carries exactly three rows, padded with the
        all-999 sentinel; implausible rows are dropped here so callers never see
        a "gate at 999 m".  Row order is the publisher's nearest-first sort and
        is **not** a stable identity -- see ``missions/association.py``.
        """
        with self._detection_lock:
            return [replace(det) for det in self._latest_detections]

    @property
    def feed_frozen(self) -> bool:
        """True when the vision payload has not changed for N frames.

        The third staleness signal.  The other two -- no datagram at all, and a
        datagram carrying only the no-detection sentinel -- are both cases where
        the feed *tells* you something is wrong.  This is the case where it
        lies: a stalled camera or a wedged pipeline republishing the last good
        pose forever, which reads as a perfectly healthy lock on a gate that the
        drone is no longer anywhere near.
        """
        with self._detection_lock:
            return self._identical_frames >= self.frozen_feed_frames

    @property
    def identical_frame_count(self) -> int:
        with self._detection_lock:
            return self._identical_frames

    @staticmethod
    def _payload_fingerprint(rows) -> Optional[tuple]:
        """A hashable digest of the *valid* rows in one payload, or None.

        Only valid rows count.  When no gate is in view every packet is a
        bit-identical wall of 999s -- correct behaviour that would otherwise be
        flagged as a frozen feed within half a second of looking at empty space.
        """
        valid = [
            tuple(round(float(value), 6) for value in row[:7])
            for row in rows
            if len(row) >= 7
            and MIN_PLAUSIBLE_DIST_M <= _safe_float(row[0]) <= MAX_PLAUSIBLE_DIST_M
        ]
        return tuple(valid) if valid else None

    def _note_payload(self, rows) -> None:
        """Update the frozen-feed counter.  Caller must NOT hold the lock."""
        fingerprint = self._payload_fingerprint(rows)
        with self._detection_lock:
            self._frames_seen += 1
            if fingerprint is None:
                # Nothing in view: not fresh, but not frozen either.
                self._payload_signature = None
                self._identical_frames = 0
                return
            if fingerprint == self._payload_signature:
                # Counts frames carrying this pose, INCLUDING the first one, so
                # the number in the log is the number of frames an operator
                # would have seen go by unchanged.
                self._identical_frames += 1
                if self._identical_frames == self.frozen_feed_frames:
                    print(
                        f"[!] VISION FEED FROZEN: {self._identical_frames} consecutive "
                        f"identical payloads. The publisher is alive but its pose is "
                        f"not changing -- treat every detection as invalid."
                    )
            else:
                self._payload_signature = fingerprint
                self._identical_frames = 1

    @staticmethod
    def _row_to_detection(row, index: int, now: float, fps: float) -> Optional[GateDetection]:
        if len(row) < 7:
            return None
        return GateDetection(
            timestamp=now,
            dist=_safe_float(row[0]),
            forward=_safe_float(row[1]),
            right=_safe_float(row[2]),
            down=_safe_float(row[3]),
            roll=_safe_float(row[4]),
            pitch=_safe_float(row[5]),
            yaw_deg=_safe_float(row[6]),
            row_index=index,
            source_fps=fps,
        )

    def _ingest_payload(self, payload) -> None:
        """Fold one decoded vision packet into the mission's detection state.

        Separated from the socket loop so the wire format can be tested without
        opening a port -- the parsing is where a units change or a short row
        would do its damage, and that has to be checkable on a laptop.
        """
        gates = payload.get("gates", [])
        if not gates:
            return

        self._note_payload(gates)

        now = time.time()
        fps = _safe_float(payload.get("fps", 0.0), 0.0)

        parsed = [
            self._row_to_detection(row, index, now, fps)
            for index, row in enumerate(gates)
        ]
        detections = [det for det in parsed if det is not None and is_valid_detection(det)]

        with self._detection_lock:
            self._latest_detections = detections
            # Phase 1 compatibility: `_latest_detection` is the nearest row and
            # keeps the SENTINEL row when nothing is visible, because
            # observe_gate relies on the sentinel overwriting the last good pose
            # -- that is staleness signal 2, and dropping it here would quietly
            # remove it.
            self._latest_detection = detections[0] if detections else parsed[0]

    def _udp_gate_listener(self):
        """Listen for vision packets and keep every valid gate in the newest one."""
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

                    self._ingest_payload(payload)

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

        THREE INDEPENDENT STALENESS SIGNALS, all of which must pass:

        1. **No datagram at all.**  Nothing arrives; ``_latest_detection`` stays
           whatever it was, so the age check below rejects it.
        2. **The all-999 no-detection row.**  The vision process publishes this
           every frame when no gate is in view, and it correctly overwrites the
           last pose, so ``is_valid_detection`` rejects it.
        3. **A live publisher with a frozen payload.**  Datagrams keep arriving,
           they carry a plausible gate, and nothing in them ever changes -- a
           stalled camera or a wedged pipeline.  Signals 1 and 2 both look
           healthy here.  :attr:`feed_frozen` is the one that catches it, and
           without it the drone flies a frozen gate position with a console log
           that reads like a clean lock.

        **Angles use a circular mean.**  Averaging +179 and -179 arithmetically
        gives 0, which is 180 degrees wrong, and gate yaws sit near the wrap
        routinely.

        SETPOINTS -- one kind, not two.  This used to latch ``BRAKE_HOLD_ALT``
        (z as a *position*) and then also transmit ``VELOCITY_YAW`` zeros (z as a
        *velocity*) directly at 25 Hz.  PX4 received both, interleaved, and
        ``vz = 0`` holds a *rate*, not an altitude -- so the vehicle was free to
        drift vertically through every observation window, which is precisely
        when it is being asked to measure a gate's height.  The direct send was
        vestigial anyway: the streamer already publishes the latched setpoint at
        20 Hz.  It is gone, and the altitude is now a genuine position hold.

        (The horizontal axes stay on a zero-*velocity* brake rather than becoming
        a full position hold.  That is deliberate and unchanged: on an
        optical-flow airframe a position setpoint chases estimator drift at up to
        ``MPC_XY_VEL_MAX`` -- 12 m/s by default -- while braking cannot run away.
        See :meth:`NavigationController.hold_position`.)
        """
        from .missions.frames import average_detections  # local: avoids an import cycle

        print(f"[*] Observing gate for {duration}s...")

        samples = []
        stale_seen = 0
        hold_state = nav.get_vehicle_snapshot()
        # The one and only setpoint for the whole window. The streamer publishes
        # it at 20 Hz; nothing else transmits from inside this loop.
        nav._hold_from_state(hold_state, label="observing")

        t0 = time.time()
        while nav.running and time.time() - t0 < duration:
            if self.feed_frozen:
                print(
                    f"[!] Vision feed FROZEN ({self.identical_frame_count} identical "
                    f"payloads): discarding this observation. The publisher is alive "
                    f"but its pose is not changing."
                )
                return None

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

        if len(samples) < self.min_observation_samples:
            # A thin window is not a lock. In flight one observation averaged a
            # SINGLE sample and reported a range that disagreed with the
            # observations either side of it -- a lone PnP solve carrying the
            # full per-frame error, presented with the same confidence as a
            # 13-sample average. Treat it as a miss and re-observe; the recovery
            # ladder already handles a miss, and a wrong fix is worse than none.
            print(
                f"[!] Only {len(samples)} valid sample(s) in a {duration:.2f}s window "
                f"(need {self.min_observation_samples}). Too thin to average -- "
                f"discarding rather than acting on one raw solve."
            )
            return None

        avg = average_detections(samples)
        print(
            f"[*] Lock acquired from {len(samples)} samples: "
            f"fwd={avg.forward:+.2f}m right={avg.right:+.2f}m down={avg.down:+.2f}m "
            f"yaw={avg.yaw_deg:+.1f}deg"
        )
        return avg

    def detection_to_gate_local(self, det: GateDetection, state: VehicleState):
        """Convert a camera-relative gate detection into local NED gate pose.

        Uses the vehicle's **full** attitude via the 3-2-1 rotation, not yaw
        alone.  A gate 3 m ahead observed at 10 degrees of body pitch is placed
        0.52 m off vertically when pitch is ignored, and that error goes straight
        into the commanded altitude.  ``roll_rad``/``pitch_rad`` default to 0.0
        on a ``VehicleState``, so a caller with no attitude data gets exactly the
        old yaw-only answer.
        """
        corrected_right = det.right + self.cam_offset_right_m
        corrected_down = det.down + self.cam_offset_down_m

        dn, de, dd = body_to_local(
            det.forward,
            corrected_right,
            corrected_down,
            state.yaw_rad,
            getattr(state, "pitch_rad", 0.0),
            getattr(state, "roll_rad", 0.0),
        )
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
