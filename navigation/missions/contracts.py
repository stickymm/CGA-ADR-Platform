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

    # 1.35 m, not 1.50 m. The gate this stack flies is a 0.97 m square whose
    # centre has measured 1.06 - 1.09 m AGL on every flight, so a 1.50 m takeoff
    # puts the aircraft ~0.42 m above the thing it is looking for before it has
    # taken a single observation. It then has to descend the whole way while
    # simultaneously closing range, and a gate seen from above is the geometry
    # that leaves the top of the frame first. Starting near the gate's own
    # height costs nothing and removes a whole axis of correction from the
    # approach.
    takeoff_alt_m: float = 1.35
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
    #
    # RAISED 1.00 -> 1.80 after the flight of 2026-08-14. 1.00 m was not merely
    # hard to reach on this airframe: it was UNREACHABLE, and the log says so in
    # a single column.
    #
    #     obs 2  drone (18.88, 1.77)  measured range 2.43 m
    #     obs 3  drone (18.84, 1.42)  range 1.93 m   0.35 m flown, 0.50 m closed
    #     obs 4  drone (18.93, 1.05)  range 1.72 m   0.38 m flown, 0.21 m closed
    #     obs 5  drone (19.12, 0.65)  range 1.72 m   0.44 m flown, 0.00 m closed
    #
    # Closure was ~100% efficient from 2.8 m down to 1.9 m and then collapsed to
    # nothing. That is the vision pipeline, not the navigator: a 0.97 m square at
    # 1.7 m nearly fills the frame, the detector refuses any box touching an
    # image edge -- hence the three consecutive misses immediately after obs 4 --
    # and the corner estimates that do survive are clipped, so solvePnP returns a
    # range that stops shrinking no matter how far the aircraft flies.
    #
    # A commit distance inside that wall means the leg can NEVER commit. It just
    # keeps stepping; every step re-derives the target from a fresh fix; the
    # aircraft wanders. That wandering is the "veering right" that ended the
    # flight in an operator abort.
    #
    # So commit where the measurement is still good. At 1.80 m obs 3 and 4
    # reported cone angles of 2.8 and 1.1 degrees from 18 and 19 averaged
    # samples -- the best data of the whole approach.
    #
    # THE COST IS A LONGER BLIND LEG AND IT IS PAID FOR EXPLICITLY.
    # The crossing is no longer a straight line to a point beyond the gate; it is
    # an axis-following controller that drives cross-track error to zero DURING
    # the pass (see gate_leg._fly_through_gate). Lateral error at the gate plane:
    #     old, straight line committed at 1.0 m:  0.15 * 1.5/2.5    = 0.09 m
    #     new, axis-following committed at 1.8 m: 0.20 * e^-(4/0.83) < 0.01 m
    # plus the gate's own localization error in both cases. The two tolerances
    # that actually keep a propeller out of a gate leg -- commit_lateral_tol_m
    # and commit_vertical_tol_m -- are UNCHANGED.
    commit_distance_m: float = 1.8
    step_size_m: float = 0.35
    approach_speed_m_s: float = 0.35
    # 0.8 s, not 0.5 s. The 0.5 s window produced as few as ONE usable sample in
    # flight, and a one-sample average is not an average -- see
    # min_observation_samples.
    observation_duration_s: float = 0.8
    max_approach_attempts: int = 24
    approach_timeout_s: float = 150.0

    # Vertical budget for one approach step, spent independently of the
    # horizontal one, so gate-height noise cannot consume the forward progress
    # (see frames.limit_approach_step).
    #
    # IT MUST EXCEED arrival_tolerance_m, AND THAT IS NOT A STYLE PREFERENCE.
    #   move_to_target's arrival test is a 3-D distance. If the commanded
    #   vertical step is smaller than that tolerance, then hypot(0, 0, step) is
    #   already inside it, and the move reports "Reached" the instant the
    #   HORIZONTAL error closes -- with the entire vertical step still
    #   outstanding. The guaranteed descent per attempt is then exactly zero.
    #
    #   This shipped at 0.12 against a 0.15 m tolerance and did precisely that.
    #   Flight of 2026-08-14: five consecutive moves each commanded a 0.12 m
    #   descent; the net altitude change across all five was -0.01 m. The
    #   aircraft held 0.50 m above the gate for the whole approach and
    #   eventually lost sight of it over the top.
    #
    #   With step > tolerance the achieved descent is at least
    #   (step - tolerance) every attempt. __post_init__ enforces it.
    vertical_step_m: float = 0.25

    # Rolling median window for the gate's estimated POSE -- north, east, down
    # and heading, not just height.
    #
    # It covered height alone until 2026-08-14, which fixed the vertical chase
    # and left the horizontal one untouched. From that flight, six observations
    # of one gate that never moved:
    #
    #     N   19.40  19.36  19.32  19.35  19.53     spread 0.21 m
    #     E   -0.69  -0.60  -0.44  -0.62  -1.02     spread 0.58 m
    #     hdg -75.7  -72.2  -72.7  -74.9  -71.8     spread  3.9 deg
    #
    # Every approach step is aimed at a standoff point derived from all four of
    # those numbers, so unfiltered they inject roughly 0.2 m of target jitter at
    # 1.8 m standoff and the aircraft chases it. Filtering height alone was half
    # a fix.
    #
    # A MEDIAN, not a mean: it discards outliers rather than mixing them in.
    # A ROLLING window, not an early lock: the far-range estimates are the WORST
    # ones (a 0.97 m square subtends little at 3 m and the PnP scale is poorly
    # conditioned), so freezing the first few would lock in the least
    # trustworthy number available. It converges as the aircraft closes, which
    # is exactly when it matters.
    gate_pose_filter_samples: int = 5

    # --- airframe geometry -------------------------------------------------
    # The camera reports where the GATE is. It has no idea how big the aircraft
    # behind it is, and the propellers span far more than the lens.
    #
    # Half the airframe's bounding box, propeller tip to propeller tip, plus
    # whatever margin you want.
    #
    # RAISED 0.115 -> 0.20 after 2026-08-15, when a crossing committed at a
    # measured 0.05 m from the gate axis -- as centred as this stack has ever
    # been -- and still tracked into the gate side. 0.115 m was half of a 9 inch
    # box, which is a small number for a quadcopter carrying props and a
    # battery, and it was doing real work: it set the ceiling the commit
    # tolerances were validated against.
    #
    # THIS IS STILL A GUESS AND IT IS THE WRONG KIND OF NUMBER TO GUESS. Put a
    # tape measure across your own props, tip to opposite tip, halve it, and set
    # it. The gate is 0.97 m; every centimetre here comes straight off the
    # margin the commit gate is allowed to spend.
    airframe_clearance_radius_m: float = 0.20
    # Inner opening of the gate, matching vision/opencv_processing.py's
    # HALF_SIZE * 2 -- the same square solvePnP is scaling every distance from.
    gate_inner_size_m: float = 0.97155

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
    #
    # TIGHTENED 0.20/0.25 -> 0.15/0.18 after 2026-08-15. This is not the fix for
    # that flight -- it committed at +0.07 m lateral and +0.11 m vertical, well
    # inside even the new numbers -- so tightening would not have changed it.
    # It is margin, bought at no cost: the three frames that actually committed
    # measured lateral +0.06 / +0.07 / +0.07 and vertical +0.02 / +0.05 / +0.11,
    # so the tighter gate would have passed identically on the flight it is
    # being tightened because of.
    #
    # 0.15 and not smaller: arrival_tolerance_m is 0.15, and a commit tolerance
    # BELOW the tolerance an approach step can arrive within is a leg that stalls
    # -- deadline_consistency() flags exactly that and the banner prints it.
    commit_lateral_tol_m: float = 0.15
    commit_vertical_tol_m: float = 0.18
    commit_max_cone_deg: float = 60.0
    commit_confirm_frames: int = 3
    # Raised 4 -> 6 alongside commit_distance_m. The ALIGN branch fires whenever
    # the gate is in RANGE but not yet lined up, and moving that range gate from
    # 1.0 m to 1.8 m means the aircraft enters ALIGN several steps earlier in the
    # approach. The same four corrections now have to cover more of the flight.
    max_align_attempts: int = 6

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

    # --- multi-gate seam ---
    # How close a new fix may be to the gate just crossed before it is treated
    # as that same gate seen again rather than the next one. Only used when a
    # caller passes `avoid_gate=`; Phase 1A never does. Sized well under any
    # sane gate spacing and well over the localization noise seen in flight
    # (~0.3 m). Set to 0.0 to disable -- which is the right move on a course
    # where two gates genuinely sit within a metre of each other.
    crossed_gate_avoid_m: float = 0.8

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
        if self.gate_pose_filter_samples < 1:
            raise ValueError("gate_pose_filter_samples must be at least 1")

        # THE ONE THAT COST A FLIGHT. A vertical step inside the arrival
        # tolerance is a vertical step that never has to be flown.
        if self.vertical_step_m <= self.arrival_tolerance_m:
            raise ValueError(
                f"vertical_step_m ({self.vertical_step_m:.2f} m) must be LARGER than "
                f"arrival_tolerance_m ({self.arrival_tolerance_m:.2f} m). The arrival "
                f"test is a 3-D distance, so a smaller vertical step is already "
                f"inside it: every move would report 'Reached' with the whole "
                f"descent still outstanding, and the aircraft would never change "
                f"altitude at all."
            )

        # The camera measures the gate; it does not measure the aircraft. A
        # commit tolerance wider than the real clearance says "close enough"
        # about a position that puts a propeller into a gate leg.
        usable = self.gate_inner_size_m / 2.0 - self.airframe_clearance_radius_m
        if usable <= 0.0:
            raise ValueError(
                f"airframe_clearance_radius_m ({self.airframe_clearance_radius_m:.3f} m) "
                f"leaves no room in a {self.gate_inner_size_m:.3f} m gate"
            )
        for name, tolerance in (
            ("commit_lateral_tol_m", self.commit_lateral_tol_m),
            ("commit_vertical_tol_m", self.commit_vertical_tol_m),
        ):
            if tolerance > usable:
                raise ValueError(
                    f"{name} ({tolerance:.2f} m) exceeds the usable half-opening "
                    f"({usable:.2f} m = {self.gate_inner_size_m/2:.2f} m half-gate "
                    f"- {self.airframe_clearance_radius_m:.3f} m airframe). Committing "
                    f"at that offset can put a propeller into the frame."
                )

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
                          f"{cfg.vertical_step_m:.2f} m vertical "
                          f"(>= {cfg.vertical_step_m - cfg.arrival_tolerance_m:.2f} m "
                          f"descent guaranteed per attempt)"),
        ("arrival tolerance", f"{cfg.arrival_tolerance_m:.2f} m"),
        ("min samples per lock", f"{cfg.min_observation_samples}"),
        ("gate pose filter", f"median of {cfg.gate_pose_filter_samples} "
                             f"observations (N, E, D and heading)"),
        ("airframe clearance", f"{cfg.airframe_clearance_radius_m:.3f} m radius; "
                               f"usable half-opening "
                               f"{cfg.gate_inner_size_m/2 - cfg.airframe_clearance_radius_m:.2f} m "
                               f"in a {cfg.gate_inner_size_m:.2f} m gate"),
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
