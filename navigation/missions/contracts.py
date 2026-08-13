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
    #
    # RETUNED 2026-08-13 after the first live gate approach. The aircraft flew
    # well, tracked the gate cleanly (lateral down to +/-0.10 m, cone under 3
    # degrees) and simply ran out of attempts: it reached 1.41 m of a gate it
    # needed to be within 1.00 m of, on attempt 14 of 14, still closing.
    #
    # The arithmetic from that flight:
    #     started at   3.23 m, needed  <= 1.00 m  ->  2.23 m to close
    #     delivered    0.14 m of range closure per 0.25 m step  (56% efficient)
    #     therefore    ~16 attempts required, 14 available
    #
    # Three numbers move as a result, and they are coupled: a bigger step and a
    # separate vertical budget raise the per-attempt yield, and a higher attempt
    # cap covers what is left. The deadline moves with them so the advertised
    # attempt count stays reachable -- see attempt_budget_s() below.
    commit_distance_m: float = 1.0
    step_size_m: float = 0.35
    approach_speed_m_s: float = 0.35
    # 0.8 s, not 0.5 s. The 0.5 s window produced as few as ONE usable sample in
    # flight, and a one-sample average is not an average -- see
    # min_observation_samples.
    observation_duration_s: float = 0.8
    max_approach_attempts: int = 24
    approach_timeout_s: float = 150.0

    # Vertical budget for one approach step, spent independently of the
    # horizontal one. Deliberately small: the gate's measured height is the
    # noisiest quantity in the whole pipeline (0.63 m of swing observed in
    # flight), altitude error is not urgent, and it corrects a little on every
    # attempt. Capping it here is what stops that noise consuming the forward
    # progress -- see frames.limit_approach_step.
    vertical_step_m: float = 0.12

    # Fewest averaged detections that may be called a "lock". In flight one
    # observation window produced a single sample and reported a range that
    # disagreed with the observations either side of it. One raw PnP solve is
    # not a measurement; treat a thin window as a miss and re-observe.
    min_observation_samples: int = 3

    # Arrival tolerance for the approach steps of this leg.  Separate from the
    # commit tolerances on purpose: this one asks "did the vehicle finish the
    # move", which is limited by what optical flow can resolve, while the commit
    # tolerances ask "is the gate on the boresight", which is measured directly
    # by the camera and is far better observed.  Loosening this one does not
    # loosen the decision to fly at a gate.
    arrival_tolerance_m: float = 0.15

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
    # Commanded yaw rate for every deliberate heading change. PX4's
    # MPC_YAWRAUTO_MAX is a backstop that an airframe startup script can raise
    # without anyone noticing; this is the number this code actually controls.
    max_yaw_rate_deg_s: float = 45.0
    max_scan_sweeps: int = 2
    backoff_distance_m: float = 1.0
    max_backoffs: int = 2

    # --- Phase 2 seam ---
    # How far an observation may be from where the caller expected this gate
    # before it is discarded. Only used when `expected_gate=` is passed; Phase 1
    # never sets it. Sized larger than plausible flow drift over one leg and
    # smaller than any sane gate spacing.
    expected_gate_radius_m: float = 2.0

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
        positive["arrival_tolerance_m"] = self.arrival_tolerance_m
        positive["vertical_step_m"] = self.vertical_step_m
        for name, value in positive.items():
            if not value > 0.0:
                raise ValueError(f"{name} must be positive, got {value!r}")

        if self.min_observation_samples < 1:
            raise ValueError("min_observation_samples must be at least 1")

        # An arrival tolerance at or above the step size makes every approach
        # step arrive before it moves, so the drone would "approach" the gate by
        # standing still and announcing success.
        if self.arrival_tolerance_m >= self.step_size_m:
            raise ValueError(
                f"arrival_tolerance_m ({self.arrival_tolerance_m:.2f} m) must be "
                f"smaller than step_size_m ({self.step_size_m:.2f} m), or every "
                f"approach step arrives without moving"
            )

        if not 0.0 < self.commit_max_cone_deg < 90.0:
            raise ValueError("commit_max_cone_deg must be in (0, 90)")
        if self.commit_confirm_frames < 1:
            raise ValueError("commit_confirm_frames must be at least 1")
        if self.max_approach_attempts < 1:
            raise ValueError("max_approach_attempts must be at least 1")
        if self.exit_clearance_m >= self.pass_distance_m:
            raise ValueError("exit_clearance_m must be less than pass_distance_m")


def attempt_move_budget_s(cfg: GateLegConfig) -> float:
    """Worst-case time budget one approach step can consume, in seconds.

    Mirrors what ``move_to_target`` will actually compute for a full-size step:
    the distance-derived estimate, floored by ``MOVE_TIMEOUT``.  Imported lazily
    so this module keeps its "no I/O, no imports that pull in pymavlink" property
    for the tests that stub the driver out.
    """
    from ..navigation import estimate_move_timeout

    return estimate_move_timeout(
        cfg.step_size_m,
        cfg.approach_speed_m_s,
        tolerance_m=cfg.arrival_tolerance_m,
    )


def attempt_budget_s(cfg: GateLegConfig) -> float:
    """Wall clock the config's advertised approach attempts actually need.

    One attempt is an observation window plus one bounded move, so::

        attempts * (observation_duration_s + worst-case move budget)

    THIS IS THE ARITHMETIC THAT WAS INCONSISTENT.  With the old numbers -- a
    0.10 m arrival tolerance inside the flow noise floor, a 15 s move floor, and
    a 75 s gate deadline -- a 0.25 m step was issued with a 15 s budget it
    routinely spent in full, so the deadline bought about 5 attempts against a
    config whose banner promised 14.  Nothing in the code noticed, because no
    single number was wrong on its own; only the relationship between them was.

    It is a *lower bound* on what the deadline needs: the recovery ladder (yaw
    sweeps, back-offs) draws on the same deadline, so a healthy margin is
    expected on top.
    """
    return cfg.max_approach_attempts * (
        cfg.observation_duration_s + attempt_move_budget_s(cfg)
    )


def deadline_consistency(cfg: GateLegConfig) -> Tuple[str, ...]:
    """Human-readable complaints about tolerance/timeout/deadline disagreement.

    Returned rather than raised: a short ``--approach-timeout-s`` is a legitimate
    thing to want on a bench, and refusing to run would be worse than saying so.
    The pre-flight banner prints whatever comes back, so a mis-tuned set of flags
    is visible before the props spin rather than inferred from a log afterwards.
    """
    complaints = []
    needed = attempt_budget_s(cfg)
    if needed > cfg.approach_timeout_s:
        reachable = max(
            1,
            int(
                cfg.approach_timeout_s
                / (cfg.observation_duration_s + attempt_move_budget_s(cfg))
            ),
        )
        complaints.append(
            f"approach_timeout_s={cfg.approach_timeout_s:.0f}s cannot deliver the "
            f"{cfg.max_approach_attempts} attempts advertised (needs {needed:.0f}s); "
            f"about {reachable} are actually reachable"
        )
    if cfg.arrival_tolerance_m > cfg.commit_lateral_tol_m:
        complaints.append(
            f"arrival_tolerance_m={cfg.arrival_tolerance_m:.2f}m is looser than "
            f"commit_lateral_tol_m={cfg.commit_lateral_tol_m:.2f}m, so a step can "
            f"'arrive' outside the commit gate and never converge"
        )
    return tuple(complaints)


def summarize_config(cfg: GateLegConfig) -> Tuple[Tuple[str, str], ...]:
    """Flatten a config into (label, value) rows for the pre-flight banner."""
    env = cfg.altitude
    return (
        ("commit distance", f"{cfg.commit_distance_m:.2f} m"),
        ("commit lateral tol", f"{cfg.commit_lateral_tol_m:.2f} m"),
        ("commit vertical tol", f"{cfg.commit_vertical_tol_m:.2f} m"),
        ("commit max cone", f"{cfg.commit_max_cone_deg:.1f} deg"),
        ("commit confirm frames", f"{cfg.commit_confirm_frames}"),
        ("approach step", f"{cfg.step_size_m:.2f} m horizontal, "
                          f"{cfg.vertical_step_m:.2f} m vertical"),
        ("arrival tolerance", f"{cfg.arrival_tolerance_m:.2f} m"),
        ("min samples per lock", f"{cfg.min_observation_samples}"),
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
        ("attempt budget", f"{attempt_budget_s(cfg):.0f} s needed for "
                           f"{cfg.max_approach_attempts} attempts "
                           f"({attempt_move_budget_s(cfg):.1f} s per move)"),
        ("recovery ladder", f"observe x{cfg.max_observe_retries}, "
                            f"scan +/-{cfg.scan_half_angle_deg:.0f}deg x{cfg.max_scan_sweeps}, "
                            f"backoff {cfg.backoff_distance_m:.2f}m x{cfg.max_backoffs}"),
    ) + tuple(
        ("!! INCONSISTENT", complaint) for complaint in deadline_consistency(cfg)
    )
