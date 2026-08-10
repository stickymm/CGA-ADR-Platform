r"""Course-vector planning.  Pure geometry -- no MAVLink, no controller, no clock.

WHAT PHASE 2 CHANGES, AND WHY IT IS ONLY THIS
    Phase 1B flies gate, full stop, re-acquire, gate.  The stop is the whole
    cost: at 0.35 m/s with a 1 s settle between gates, most of a course is spent
    not moving.  Phase 2 removes the stop by deciding, *before* it finishes gate
    N, where it will start turning toward gate N+1 and what heading it will hold
    through that turn.

    That is all Phase 2 is.  It does not change what a gate is, how one is
    localized, or the commit decision -- those are shared with 1A and 1B and are
    the parts that have been checked against hand-derived arithmetic.

THE MODEL
    A gate is a *vector*, not a point: a position plus the direction you must be
    travelling when you pass through it.  Aiming at a gate's centre is not
    enough -- arriving at the centre sideways puts a rotor through a gate leg.

    Two consecutive gates give two directed lines.  The transition between them
    is a circular arc tangent to both, which is the shortest constant-radius
    path that leaves gate N on its vector and enters gate N+1 on its vector::

                     P (lines intersect)
                    /|
        gate N ----T1 |            T1, T2 are the tangent points.
                  ( \|            The arc runs T1 -> T2, radius r,
                   \  T2          centred normal to both lines.
                    \  \
                     \  v gate N+1

    The tangent points sit ``d = r * |tan(theta/2)|`` either side of the
    intersection, where ``theta`` is the turn angle.  T1 is the **transition
    point**: the place along gate N's exit vector where the turn begins.

WHAT LIMITS THE RADIUS -- both directions matter
    *Lower bound*, from the airframe.  A turn at speed ``V`` and radius ``r``
    demands a yaw rate of ``V/r`` and a lateral acceleration of ``V^2/r``.  This
    aircraft cannot spend either freely:

      - yaw rate is capped by ``MAX_YAW_RATE_DEG_S`` because fast yaw injects
        rotation-induced optical flow that the estimator must cancel from gyro
        data alone, and the position estimate is the thing the whole mission
        rests on;
      - lateral acceleration is capped because a multirotor produces it by
        tilting, ``a = g * tan(tilt)``, and tilt swings the nose-mounted camera
        off the gate exactly when the next gate needs to be in frame.

    *Upper bound*, from the geometry.  The tangent points must both fall in
    usable places: after gate N by at least the exit clearance, and before gate
    N+1 by at least the entry lead-in.  A big radius pushes them outside that
    window.

    When the upper bound is below the lower bound the turn is **infeasible at
    that speed**, and this module says so rather than picking something.  That
    verdict is one of the two triggers for degrading to Phase 1B; the other is
    not knowing where the next gate is.  Both are decided here, in a pure
    function, so both are unit-testable without an aircraft.

ATTITUDE
    ``TurnPlan`` reports the roll the turn implies (``bank_rad``) and the pitch
    implied by any speed change.  These are *predictions*, used for feasibility
    and for logging -- never for localization.  Localization always uses the
    attitude PX4 actually reported, because the whole point of reading
    ``ATTITUDE`` is that the vehicle does not do exactly what it was asked.

FRAMES
    Local NED.  Horizontal directions are unit ``(n, e)`` vectors; ``d`` is down,
    so ``d = -1.5`` is 1.5 m above the origin.  Angles are radians unless the
    name ends ``_deg``.
"""

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

from ..navigation import MAX_YAW_RATE_DEG_S, LocalTarget, wrap_pi
from .contracts import AltitudeEnvelope, GateFix

GRAVITY_M_S2 = 9.80665
_EPS = 1e-9


# =========================
# CONFIG
# =========================


@dataclass(frozen=True)
class CoursePlanConfig:
    """Everything tunable about planning a transition between two gates."""

    cruise_speed_m_s: float = 0.45
    # Slowest the planner may go to make a turn fit before giving up on it.
    min_turn_speed_m_s: float = 0.25
    max_yaw_rate_deg_s: float = MAX_YAW_RATE_DEG_S
    # Lateral acceleration ceiling. 0.15 g ~ 8.5 degrees of tilt: gentle, chosen
    # so the nose-mounted camera keeps the next gate in frame through the turn.
    max_lateral_accel_m_s2: float = 0.15 * GRAVITY_M_S2
    # The turn may not begin until this far past gate N's plane...
    exit_clearance_m: float = 0.80
    # ...and must be finished this far before gate N+1's plane, so the last
    # stretch into a gate is straight and the commit decision is made on a
    # settled, non-turning aircraft.
    entry_lead_in_m: float = 1.00
    # Below this track confidence the next gate is not planned against at all.
    min_next_gate_confidence: float = 0.5
    # Below this heading confidence the gate's NORMAL is not trusted, which
    # means there is no vector to plan toward even if the position is known.
    min_heading_confidence: float = 0.5

    def __post_init__(self) -> None:
        positive = {
            "cruise_speed_m_s": self.cruise_speed_m_s,
            "min_turn_speed_m_s": self.min_turn_speed_m_s,
            "max_yaw_rate_deg_s": self.max_yaw_rate_deg_s,
            "max_lateral_accel_m_s2": self.max_lateral_accel_m_s2,
            "exit_clearance_m": self.exit_clearance_m,
            "entry_lead_in_m": self.entry_lead_in_m,
        }
        for name, value in positive.items():
            if not value > 0.0:
                raise ValueError(f"{name} must be positive, got {value!r}")
        if self.min_turn_speed_m_s > self.cruise_speed_m_s:
            raise ValueError("min_turn_speed_m_s must not exceed cruise_speed_m_s")


# =========================
# VECTORS
# =========================


@dataclass(frozen=True)
class PassVector:
    """A gate as a point plus the direction of travel through it.

    ``dir_n``/``dir_e`` is the resolved gate normal -- already disambiguated by
    :func:`~navigation.missions.frames.resolve_gate_normal`, so it always points
    the way the aircraft is going, whichever way the detector reported the plane.
    """

    n: float
    e: float
    d: float
    dir_n: float
    dir_e: float

    @property
    def heading_rad(self) -> float:
        return math.atan2(self.dir_e, self.dir_n)

    def point_at(self, distance_m: float) -> Tuple[float, float]:
        """A point ``distance_m`` along the vector.  Negative is before the gate."""
        return self.n + distance_m * self.dir_n, self.e + distance_m * self.dir_e

    def along(self, n: float, e: float) -> float:
        """Signed distance of ``(n, e)`` along the vector from the gate centre."""
        return (n - self.n) * self.dir_n + (e - self.e) * self.dir_e

    def offset(self, n: float, e: float) -> float:
        """Perpendicular distance of ``(n, e)`` from the vector's axis."""
        return abs(self.dir_n * (e - self.e) - self.dir_e * (n - self.n))


def pass_vector(fix: GateFix) -> PassVector:
    """The pass-through vector for one localized gate."""
    return PassVector(
        n=fix.n, e=fix.e, d=fix.d, dir_n=fix.normal_n, dir_e=fix.normal_e
    )


def track_pass_vector(track) -> PassVector:
    """The pass-through vector for a :class:`~.association.GateTrack`."""
    return PassVector(
        n=track.n, e=track.e, d=track.d, dir_n=track.normal_n, dir_e=track.normal_e
    )


# =========================
# TURN GEOMETRY
# =========================


@dataclass(frozen=True)
class TurnPlan:
    """A constant-radius transition between two gate vectors.

    ``ok`` False means the turn could not be made to fit; ``reason`` says which
    bound it broke, and the caller degrades to Phase 1B rather than flying a
    compromise nobody chose.
    """

    ok: bool
    reason: str = ""
    radius_m: float = 0.0
    speed_m_s: float = 0.0
    turn_angle_rad: float = 0.0
    # Tangent points: where the turn starts and where it ends.
    entry_n: float = 0.0
    entry_e: float = 0.0
    exit_n: float = 0.0
    exit_e: float = 0.0
    centre_n: float = 0.0
    centre_e: float = 0.0
    arc_length_m: float = 0.0
    yaw_rate_rad_s: float = 0.0
    bank_rad: float = 0.0
    lateral_accel_m_s2: float = 0.0
    entry_heading_rad: float = 0.0
    exit_heading_rad: float = 0.0
    straight: bool = False

    @property
    def turn_angle_deg(self) -> float:
        return math.degrees(self.turn_angle_rad)

    @property
    def bank_deg(self) -> float:
        return math.degrees(self.bank_rad)

    @property
    def duration_s(self) -> float:
        return self.arc_length_m / self.speed_m_s if self.speed_m_s > 0.0 else 0.0

    def heading_at(self, fraction: float) -> float:
        """Heading part-way round the arc.  ``fraction`` runs 0 -> 1.

        Linear in arc length, which for a constant-radius turn at constant speed
        is also linear in time -- so this doubles as the yaw schedule.
        """
        fraction = max(0.0, min(1.0, fraction))
        return wrap_pi(self.entry_heading_rad + fraction * self.turn_angle_rad)

    def position_at(self, fraction: float) -> Tuple[float, float]:
        """Position part-way round the arc, in local NED ``(n, e)``."""
        if self.straight or self.radius_m <= _EPS:
            fraction = max(0.0, min(1.0, fraction))
            return (
                self.entry_n + (self.exit_n - self.entry_n) * fraction,
                self.entry_e + (self.exit_e - self.entry_e) * fraction,
            )
        fraction = max(0.0, min(1.0, fraction))
        # Rotate the entry point about the arc centre by fraction * turn angle.
        angle = fraction * self.turn_angle_rad
        rel_n = self.entry_n - self.centre_n
        rel_e = self.entry_e - self.centre_e
        cos_a, sin_a = math.cos(angle), math.sin(angle)
        return (
            self.centre_n + rel_n * cos_a - rel_e * sin_a,
            self.centre_e + rel_n * sin_a + rel_e * cos_a,
        )

    def sample(self, count: int = 9) -> Tuple[Tuple[float, float, float], ...]:
        """``count`` points of ``(n, e, heading_rad)`` along the turn.

        The heading profile through the transition, for logging, for plotting,
        and for a test to check against hand-derived values.
        """
        if count < 2:
            raise ValueError("count must be at least 2")
        return tuple(
            self.position_at(index / (count - 1)) + (self.heading_at(index / (count - 1)),)
            for index in range(count)
        )

    def describe(self) -> str:
        if not self.ok:
            return f"turn NOT feasible: {self.reason}"
        if self.straight:
            return "no turn needed: the two gate vectors are already aligned"
        return (
            f"turn {self.turn_angle_deg:+.1f}deg r={self.radius_m:.2f}m "
            f"at {self.speed_m_s:.2f}m/s | arc {self.arc_length_m:.2f}m "
            f"({self.duration_s:.1f}s) | yaw {math.degrees(self.yaw_rate_rad_s):.1f}deg/s "
            f"| bank {self.bank_deg:.1f}deg"
        )


def minimum_turn_radius_m(
    speed_m_s: float,
    *,
    max_yaw_rate_deg_s: float = MAX_YAW_RATE_DEG_S,
    max_lateral_accel_m_s2: float = 0.15 * GRAVITY_M_S2,
) -> float:
    """Tightest turn this aircraft may fly at ``speed_m_s``.

    Two independent limits, and the binding one wins::

        yaw rate:      omega = V / r  <=  omega_max   ->  r >= V / omega_max
        lateral accel: a     = V^2/r  <=  a_max       ->  r >= V^2 / a_max

    At Phase 2's default 0.45 m/s with 45 deg/s and 0.15 g, the yaw limit gives
    0.57 m and the acceleration limit 0.14 m, so **yaw rate binds** -- which is
    the right way round for an optical-flow airframe, where spinning the camera
    costs more than leaning it.
    """
    if speed_m_s <= 0.0:
        raise ValueError("speed_m_s must be positive")
    if max_yaw_rate_deg_s <= 0.0 or max_lateral_accel_m_s2 <= 0.0:
        raise ValueError("limits must be positive")

    from_yaw = speed_m_s / math.radians(max_yaw_rate_deg_s)
    from_accel = speed_m_s * speed_m_s / max_lateral_accel_m_s2
    return max(from_yaw, from_accel)


def _cross(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return a[0] * b[1] - a[1] * b[0]


def plan_turn(
    leaving: PassVector,
    entering: PassVector,
    cfg: CoursePlanConfig,
    *,
    speed_m_s: Optional[float] = None,
) -> TurnPlan:
    """Plan the arc from one gate's exit vector onto the next gate's entry vector.

    Returns a ``TurnPlan`` whose ``ok`` is False, with a reason, whenever the
    geometry and the airframe limits cannot be satisfied together.  That is a
    result, not an error: the caller degrades to Phase 1B and logs it.
    """
    speed = cfg.cruise_speed_m_s if speed_m_s is None else speed_m_s
    if speed <= 0.0:
        raise ValueError("speed_m_s must be positive")

    u = (leaving.dir_n, leaving.dir_e)
    v = (entering.dir_n, entering.dir_e)
    dot = u[0] * v[0] + u[1] * v[1]
    cross = _cross(u, v)
    turn_angle = math.atan2(cross, dot)

    # --- already aligned: fly straight through -----------------------------
    if abs(turn_angle) < math.radians(1.0):
        entry = leaving.point_at(cfg.exit_clearance_m)
        exit_point = entering.point_at(-cfg.entry_lead_in_m)
        return TurnPlan(
            ok=True,
            reason="gate vectors already aligned",
            speed_m_s=speed,
            turn_angle_rad=turn_angle,
            entry_n=entry[0], entry_e=entry[1],
            exit_n=exit_point[0], exit_e=exit_point[1],
            arc_length_m=math.dist(entry, exit_point),
            entry_heading_rad=leaving.heading_rad,
            exit_heading_rad=entering.heading_rad,
            straight=True,
        )

    # --- a 180 degree reversal has no finite arc ---------------------------
    if abs(abs(turn_angle) - math.pi) < math.radians(1.0):
        return TurnPlan(
            ok=False,
            reason=(
                "the two gate vectors are opposed (180 deg); no constant-radius "
                "arc joins them. Fly this transition as a Phase 1B stop and turn."
            ),
            turn_angle_rad=turn_angle,
            speed_m_s=speed,
        )

    if abs(cross) < _EPS:
        return TurnPlan(
            ok=False, reason="gate vectors are parallel; no intersection to turn about",
            speed_m_s=speed,
        )

    # --- where the two lines meet ------------------------------------------
    delta = (entering.n - leaving.n, entering.e - leaving.e)
    s_p = _cross(delta, v) / cross           # distance from gate N to P, along u
    p_n = leaving.n + s_p * u[0]
    p_e = leaving.e + s_p * u[1]
    # Distance from P to gate N+1, measured along v (positive = P is before it).
    t_p = entering.along(p_n, p_e)
    distance_p_to_entering = -t_p

    half_tan = abs(math.tan(turn_angle / 2.0))
    if half_tan < _EPS:
        return TurnPlan(ok=False, reason="degenerate turn angle", speed_m_s=speed)

    # --- how much tangent length the geometry can afford -------------------
    room_after_leaving = s_p - cfg.exit_clearance_m
    room_before_entering = distance_p_to_entering - cfg.entry_lead_in_m
    max_tangent_m = min(room_after_leaving, room_before_entering)

    if max_tangent_m <= 0.0:
        return TurnPlan(
            ok=False,
            reason=(
                f"no room for a turn: {room_after_leaving:+.2f} m available after "
                f"gate N's {cfg.exit_clearance_m:.2f} m exit clearance and "
                f"{room_before_entering:+.2f} m before gate N+1's "
                f"{cfg.entry_lead_in_m:.2f} m lead-in"
            ),
            turn_angle_rad=turn_angle,
            speed_m_s=speed,
        )

    geometric_max_radius = max_tangent_m / half_tan

    # --- slow down if that is what makes the turn fit ----------------------
    chosen_speed = speed
    required = minimum_turn_radius_m(
        chosen_speed,
        max_yaw_rate_deg_s=cfg.max_yaw_rate_deg_s,
        max_lateral_accel_m_s2=cfg.max_lateral_accel_m_s2,
    )
    while required > geometric_max_radius and chosen_speed > cfg.min_turn_speed_m_s:
        chosen_speed = max(cfg.min_turn_speed_m_s, chosen_speed * 0.8)
        required = minimum_turn_radius_m(
            chosen_speed,
            max_yaw_rate_deg_s=cfg.max_yaw_rate_deg_s,
            max_lateral_accel_m_s2=cfg.max_lateral_accel_m_s2,
        )

    if required > geometric_max_radius:
        return TurnPlan(
            ok=False,
            reason=(
                f"turn too tight for the airframe: geometry allows r <= "
                f"{geometric_max_radius:.2f} m but {chosen_speed:.2f} m/s needs "
                f"r >= {required:.2f} m (yaw rate <= {cfg.max_yaw_rate_deg_s:.0f} deg/s, "
                f"lateral accel <= {cfg.max_lateral_accel_m_s2:.2f} m/s^2), even after "
                f"slowing to the {cfg.min_turn_speed_m_s:.2f} m/s floor"
            ),
            turn_angle_rad=turn_angle,
            speed_m_s=chosen_speed,
        )

    # Take the widest arc the geometry allows: a wide turn is a slow yaw, and a
    # slow yaw is what the optical flow estimator wants.
    radius = geometric_max_radius
    tangent = radius * half_tan

    entry_n = p_n - u[0] * tangent
    entry_e = p_e - u[1] * tangent
    exit_n = p_n + v[0] * tangent
    exit_e = p_e + v[1] * tangent

    # The arc centre sits 90 degrees from the direction of travel, on the side
    # the turn goes: +90 for a positive (n-toward-e) turn, -90 otherwise.
    sign = 1.0 if turn_angle > 0.0 else -1.0
    centre_n = entry_n + sign * radius * (-u[1])
    centre_e = entry_e + sign * radius * (u[0])

    yaw_rate = chosen_speed / radius
    lateral_accel = chosen_speed * chosen_speed / radius

    return TurnPlan(
        ok=True,
        reason="",
        radius_m=radius,
        speed_m_s=chosen_speed,
        turn_angle_rad=turn_angle,
        entry_n=entry_n, entry_e=entry_e,
        exit_n=exit_n, exit_e=exit_e,
        centre_n=centre_n, centre_e=centre_e,
        arc_length_m=radius * abs(turn_angle),
        yaw_rate_rad_s=yaw_rate,
        # A coordinated turn's bank: tan(phi) = a_lat / g.
        bank_rad=math.atan2(lateral_accel, GRAVITY_M_S2) * sign,
        lateral_accel_m_s2=lateral_accel,
        entry_heading_rad=leaving.heading_rad,
        exit_heading_rad=entering.heading_rad,
    )


# =========================
# LEG PLANNING
# =========================


@dataclass(frozen=True)
class LegPlan:
    """How to fly one gate and set up for the next.

    ``turn`` is None when there is no next gate to plan toward, or when the next
    gate is not known well enough to plan against.  ``degrade_reason`` is then
    non-empty and names which of those it was -- a Phase 2 run that quietly
    behaves like Phase 1B without saying so would be the worst of both.
    """

    cross_target: LocalTarget
    exit_target: Optional[LocalTarget] = None
    turn: Optional[TurnPlan] = None
    degrade_reason: str = ""

    @property
    def degraded(self) -> bool:
        return self.turn is None or not self.turn.ok


def plan_leg(
    current: PassVector,
    envelope: AltitudeEnvelope,
    cfg: CoursePlanConfig,
    *,
    next_gate: Optional[PassVector] = None,
    next_confidence: float = 0.0,
    next_heading_confidence: float = 0.0,
    pass_distance_m: float = 1.5,
) -> LegPlan:
    """Plan the crossing of ``current`` and, if possible, the turn toward the next.

    THE FALLBACK IS THE POINT.  Every path that cannot produce a usable turn
    returns a plan whose ``cross_target`` is exactly what Phase 1B would have
    flown, with ``degrade_reason`` naming the cause.  Phase 2 is then a strict
    improvement on 1B rather than a different mission with its own failure
    modes: the worst it can do is 1B.
    """
    from .frames import clamp_altitude

    cross_d, _ = clamp_altitude(current.d, envelope)
    cross_n, cross_e = current.point_at(pass_distance_m)
    cross_target = LocalTarget(
        n=cross_n, e=cross_e, d=cross_d, yaw_rad=current.heading_rad
    )

    if next_gate is None:
        return LegPlan(
            cross_target=cross_target,
            degrade_reason="no next gate is being tracked; flying the Phase 1B crossing",
        )
    if next_confidence < cfg.min_next_gate_confidence:
        return LegPlan(
            cross_target=cross_target,
            degrade_reason=(
                f"next gate confidence {next_confidence:.2f} is below "
                f"{cfg.min_next_gate_confidence:.2f}; flying the Phase 1B crossing"
            ),
        )
    if next_heading_confidence < cfg.min_heading_confidence:
        return LegPlan(
            cross_target=cross_target,
            degrade_reason=(
                f"next gate heading confidence {next_heading_confidence:.2f} is below "
                f"{cfg.min_heading_confidence:.2f} -- its position is known but its "
                f"normal is not, so there is no vector to turn onto"
            ),
        )

    turn = plan_turn(current, next_gate, cfg)
    if not turn.ok:
        return LegPlan(
            cross_target=cross_target,
            turn=turn,
            degrade_reason=turn.reason,
        )

    exit_d, _ = clamp_altitude(current.d, envelope)
    return LegPlan(
        cross_target=cross_target,
        exit_target=LocalTarget(
            n=turn.entry_n, e=turn.entry_e, d=exit_d, yaw_rad=current.heading_rad
        ),
        turn=turn,
    )


def order_gates_along_course(
    vectors: Sequence[PassVector],
    from_n: float,
    from_e: float,
    from_heading_rad: Optional[float] = None,
) -> Tuple[int, ...]:
    """Indices of ``vectors`` in the order a course would fly them.

    Nearest-first from the given position, then chaining forward: each next gate
    is the nearest one that lies **ahead** of the previous gate's vector, so a
    gate the drone has already passed is not selected again.

    ``from_heading_rad`` applies the same "must be ahead" rule to the *first*
    pick, using the aircraft's own heading.  Without it a gate 2 m behind the
    drone outranks one 3 m in front, which is the wrong answer for every course
    this will ever fly.

    This is a heuristic and it is stated as one.  It is correct for the open,
    forward-progressing courses this aircraft flies; it is wrong for a course
    that doubles back through the same volume, and there is no way to tell the
    difference without gate IDs.  A caller that needs certainty should fly
    Phase 1B, where "next gate" means "the nearest one right now" and is
    re-decided from scratch after every crossing.
    """
    remaining = list(range(len(vectors)))
    ordered = []
    position = (from_n, from_e)

    while remaining:
        if ordered:
            previous = vectors[ordered[-1]]
            ahead = [
                index for index in remaining
                if previous.along(vectors[index].n, vectors[index].e) > 0.0
            ]
        elif from_heading_rad is not None:
            dir_n, dir_e = math.cos(from_heading_rad), math.sin(from_heading_rad)
            ahead = [
                index for index in remaining
                if (vectors[index].n - from_n) * dir_n
                + (vectors[index].e - from_e) * dir_e > 0.0
            ]
        else:
            ahead = list(remaining)
        pool = ahead or remaining
        nearest = min(
            pool,
            key=lambda index: math.hypot(
                vectors[index].n - position[0], vectors[index].e - position[1]
            ),
        )
        ordered.append(nearest)
        remaining.remove(nearest)
        position = (vectors[nearest].n, vectors[nearest].e)

    return tuple(ordered)
