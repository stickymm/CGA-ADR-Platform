"""Types that cross a module seam.

Named ``contracts`` rather than ``types`` on purpose: every test file in this
repo does ``import types`` to build the ``pymavlink`` stub, and a package module
named ``types.py`` is a shadowing footgun that is very hard to debug.

Nothing here does I/O.  These are frozen dataclasses and enums only, so they are
safe to construct in a unit test, log, or compare with ``==``.

UNITS
    Distances are metres.  Angles are radians unless the attribute name ends in
    ``_deg``.  ``d`` is NED down, so ``d = -1.5`` is 1.5 m *above* the origin.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple

from ..navigation import VehicleState


class GateResult(Enum):
    """Why a single-gate attempt ended.

    Deliberately not a ``bool``.  Six of seven ``move_to_target`` call sites in
    this repo ignore its boolean return; a named enum is much harder to drop on
    the floor, and the value is what gets printed in the mission log.
    """

    CROSSED = "crossed"
    NOT_FOUND = "not_found"      # never saw a valid detection within the budget
    LOST = "lost"                # saw it, lost it before commit, retries spent
    NO_COMMIT = "no_commit"      # in range but the commit gate never held
    TIMED_OUT = "timed_out"      # a flight leg exceeded its computed budget
    ABORTED = "aborted"          # operator, envelope, telemetry loss, or stop()


@dataclass(frozen=True)
class GateFix:
    """One gate localized into local NED, with everything the commit gate needs.

    ``normal_n``/``normal_e`` is a unit horizontal vector pointing in the
    *direction of travel* through the gate -- i.e. already disambiguated by
    :func:`~navigation.missions.frames.resolve_gate_normal`, so a caller never
    has to worry about which way the detector happened to report the plane.
    """

    n: float
    e: float
    d: float
    normal_n: float
    normal_e: float
    yaw_rad: float               # atan2(normal_e, normal_n); where to point
    range_m: float               # hypot(forward, right, down), NOT det.dist
    lateral_body_m: float        # gate centre's observed 'right' -- directly measured
    vertical_body_m: float       # gate centre's observed 'down'  -- directly measured
    lateral_axis_m: float        # perpendicular distance from the gate axis (log only)
    cone_angle_deg: float        # angle between the normal and drone->gate
    normal_flipped: bool         # True if the detector's normal was reversed
    altitude_clamped: bool
    low_confidence: bool
    reason: str                  # why low_confidence, or "" when healthy


@dataclass(frozen=True)
class AltitudeEnvelope:
    """Hard altitude limits, referenced to the pad.

    ``d_takeoff`` is the local-NED ``d`` latched on the ground before takeoff, so
    the envelope follows the EKF origin rather than assuming it is zero.
    """

    takeoff_alt_m: float = 1.5
    min_alt_m: float = 0.8
    max_alt_m: float = 2.5
    d_takeoff: float = 0.0

    def __post_init__(self) -> None:
        if not 0.0 < self.min_alt_m < self.max_alt_m:
            raise ValueError("require 0 < min_alt_m < max_alt_m")
        if not self.min_alt_m <= self.takeoff_alt_m <= self.max_alt_m:
            raise ValueError("takeoff_alt_m must lie inside [min_alt_m, max_alt_m]")


@dataclass(frozen=True)
class GateOutcome:
    """The result of one gate attempt.  Returned by ``approach_and_cross_one_gate``."""

    result: GateResult
    gate_index: int
    reason: str
    attempts: int = 0
    reacquires: int = 0
    elapsed_s: float = 0.0
    fix: Optional[GateFix] = None
    entry_state: Optional[VehicleState] = None
    exit_state: Optional[VehicleState] = None

    @property
    def ok(self) -> bool:
        return self.result is GateResult.CROSSED


@dataclass(frozen=True)
class CommitVerdict:
    """Why the drone may or may not stop trusting vision and fly the gate.

    Every term is kept even when the verdict is False so the console log can show
    which one failed and by how much -- this is the single most important
    decision in the mission and it must be diagnosable from a log alone.
    """

    ok: bool
    in_range: bool
    aligned_lateral: bool
    aligned_vertical: bool
    within_cone: bool
    range_m: float
    lateral_body_m: float
    vertical_body_m: float
    cone_angle_deg: float
    reason: str

    def describe(self) -> str:
        """One-line, fixed-width summary for the mission log."""
        def mark(flag: bool) -> str:
            return "OK " if flag else "NO "
        return (
            f"commit={'YES' if self.ok else 'no '} | "
            f"{mark(self.in_range)}range={self.range_m:5.2f}m | "
            f"{mark(self.aligned_lateral)}lat={self.lateral_body_m:+5.2f}m | "
            f"{mark(self.aligned_vertical)}vert={self.vertical_body_m:+5.2f}m | "
            f"{mark(self.within_cone)}cone={self.cone_angle_deg:5.1f}deg"
        )


@dataclass(frozen=True)
class GateLegConfig:
    """Everything tunable about approaching and crossing one gate.

    Defaults are deliberately conservative small-course values.  Every one is
    exposed as a CLI flag and printed in the pre-flight banner.
    """

    # --- approach ---
    commit_distance_m: float = 1.0
    step_size_m: float = 0.25
    approach_speed_m_s: float = 0.35
    observation_duration_s: float = 0.5
    max_approach_attempts: int = 14
    approach_timeout_s: float = 75.0

    # --- commit gate (range AND alignment AND not edge-on) ---
    commit_lateral_tol_m: float = 0.20
    commit_vertical_tol_m: float = 0.25
    commit_max_cone_deg: float = 60.0
    commit_confirm_frames: int = 3
    max_align_attempts: int = 4

    # --- cross ---
    pass_distance_m: float = 1.5
    exit_clearance_m: float = 0.8
    cross_speed_m_s: float = 0.45

    # --- search / recovery ladder ---
    max_observe_retries: int = 3
    scan_half_angle_deg: float = 45.0
    max_scan_sweeps: int = 2
    backoff_distance_m: float = 1.0
    max_backoffs: int = 2

    # --- freshness ---
    max_detection_age_s: float = 0.4
    max_telemetry_age_s: float = 0.5

    altitude: AltitudeEnvelope = AltitudeEnvelope()

    def __post_init__(self) -> None:
        positive = {
            "commit_distance_m": self.commit_distance_m,
            "step_size_m": self.step_size_m,
            "approach_speed_m_s": self.approach_speed_m_s,
            "observation_duration_s": self.observation_duration_s,
            "approach_timeout_s": self.approach_timeout_s,
            "commit_lateral_tol_m": self.commit_lateral_tol_m,
            "commit_vertical_tol_m": self.commit_vertical_tol_m,
            "pass_distance_m": self.pass_distance_m,
            "exit_clearance_m": self.exit_clearance_m,
            "cross_speed_m_s": self.cross_speed_m_s,
            "max_detection_age_s": self.max_detection_age_s,
            "max_telemetry_age_s": self.max_telemetry_age_s,
        }
        for name, value in positive.items():
            if not value > 0.0:
                raise ValueError(f"{name} must be positive, got {value!r}")

        if not 0.0 < self.commit_max_cone_deg < 90.0:
            raise ValueError("commit_max_cone_deg must be in (0, 90)")
        if self.commit_confirm_frames < 1:
            raise ValueError("commit_confirm_frames must be at least 1")
        if self.max_approach_attempts < 1:
            raise ValueError("max_approach_attempts must be at least 1")
        if self.exit_clearance_m >= self.pass_distance_m:
            raise ValueError("exit_clearance_m must be less than pass_distance_m")


def summarize_config(cfg: GateLegConfig) -> Tuple[Tuple[str, str], ...]:
    """Flatten a config into (label, value) rows for the pre-flight banner."""
    env = cfg.altitude
    return (
        ("commit distance", f"{cfg.commit_distance_m:.2f} m"),
        ("commit lateral tol", f"{cfg.commit_lateral_tol_m:.2f} m"),
        ("commit vertical tol", f"{cfg.commit_vertical_tol_m:.2f} m"),
        ("commit max cone", f"{cfg.commit_max_cone_deg:.1f} deg"),
        ("commit confirm frames", f"{cfg.commit_confirm_frames}"),
        ("approach step", f"{cfg.step_size_m:.2f} m"),
        ("approach speed", f"{cfg.approach_speed_m_s:.2f} m/s"),
        ("cross speed", f"{cfg.cross_speed_m_s:.2f} m/s"),
        ("pass distance", f"{cfg.pass_distance_m:.2f} m"),
        ("exit clearance", f"{cfg.exit_clearance_m:.2f} m"),
        ("observation window", f"{cfg.observation_duration_s:.2f} s"),
        ("max detection age", f"{cfg.max_detection_age_s:.2f} s"),
        ("max telemetry age", f"{cfg.max_telemetry_age_s:.2f} s"),
        ("takeoff altitude", f"{env.takeoff_alt_m:.2f} m AGL"),
        ("altitude envelope", f"{env.min_alt_m:.2f} - {env.max_alt_m:.2f} m AGL"),
        ("approach attempts", f"{cfg.max_approach_attempts}"),
        ("approach timeout", f"{cfg.approach_timeout_s:.0f} s"),
        ("recovery ladder", f"observe x{cfg.max_observe_retries}, "
                            f"scan +/-{cfg.scan_half_angle_deg:.0f}deg x{cfg.max_scan_sweeps}, "
                            f"backoff {cfg.backoff_distance_m:.2f}m x{cfg.max_backoffs}"),
    )
