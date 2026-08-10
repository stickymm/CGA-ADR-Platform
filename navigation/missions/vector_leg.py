"""Fly a planned course vector.  The Phase 2 counterpart to ``gate_leg``.

WHAT THIS IS
    ``gate_leg`` flies *to* a gate: observe, step, re-observe, commit, cross.
    This module flies *along a plan*: a transition arc from one gate's exit
    vector onto the next gate's entry vector, at a bounded yaw rate, ending on
    the next gate's centre line so ``gate_leg`` can take over with the gate
    already in frame and the aircraft already pointed through it.

    It is deliberately the same shape as ``gate_leg``'s crossing leg -- a
    receding-horizon, step-limited velocity loop with a derived time budget and
    a named completion test -- because that loop is the one piece of flight code
    in this repo that has been read, argued about, and tested. A second
    controller with different failure modes would double the surface for no
    gain.

WHAT IT REFUSES TO DO
    It does not re-plan.  The plan is computed once, from a fix, before the
    transition starts; if the world turns out not to match, this leg gives up
    and returns False rather than improvising, and the mission falls back to
    Phase 1B's stop-and-acquire.  Improvising here would mean flying a shape
    nobody derived, at a yaw rate nobody bounded, between two gates.

    It also never decides a gate has been crossed.  That decision belongs to
    ``gate_leg``, and there is exactly one implementation of it.
"""

import math
import time
from typing import Callable, Optional

from ..navigation import (
    KP_POS,
    POSITION_RATE_HZ,
    NavigationController,
    Setpoint,
    SetpointKind,
    estimate_move_timeout,
    rad_to_deg,
    wrap_pi,
)
from .contracts import AltitudeEnvelope
from .course import CoursePlanConfig, TurnPlan

LogFn = Callable[[str], None]

# How many points the arc is flown as. Each is a small step, in the same spirit
# as the approach's step limiting: the vehicle is never given a target it could
# run at from far away.
DEFAULT_ARC_STEPS = 8


def fly_turn(
    nav: NavigationController,
    turn: TurnPlan,
    envelope: AltitudeEnvelope,
    cfg: CoursePlanConfig,
    *,
    hold_d: Optional[float] = None,
    arc_steps: int = DEFAULT_ARC_STEPS,
    label: str = "transition",
    log: LogFn = print,
) -> bool:
    """Fly a planned transition arc.  Returns True when its end point is reached.

    The arc is flown as a sequence of waypoints sampled off ``TurnPlan``, each
    carrying the heading the plan says the aircraft should hold there.  Heading
    is commanded as an absolute yaw on every setpoint, and because consecutive
    samples differ by ``turn_angle / arc_steps``, the commanded yaw rate is
    bounded by construction -- the plan already refused any radius that would
    have required more than ``max_yaw_rate_deg_s``.

    Altitude is held at ``hold_d`` (default: wherever the vehicle is when the
    turn starts), clamped into the envelope.  A transition is a horizontal
    manoeuvre; changing height during one would move two things at once between
    the only two moments the gate geometry is actually measured.
    """
    if not turn.ok:
        log(f"[!] Refusing to fly an infeasible turn: {turn.reason}")
        return False
    if arc_steps < 2:
        raise ValueError("arc_steps must be at least 2")

    from .frames import clamp_altitude

    state = nav.get_vehicle_snapshot()
    target_d, clamped = clamp_altitude(
        state.d if hold_d is None else hold_d, envelope
    )
    if clamped:
        log(f"[!] Transition altitude clamped into the envelope: d={target_d:+.2f}")

    samples = turn.sample(arc_steps)
    log(
        f"[*] Flying {label}: {turn.describe()} | "
        f"{arc_steps} arc points from ({turn.entry_n:+.2f},{turn.entry_e:+.2f}) "
        f"to ({turn.exit_n:+.2f},{turn.exit_e:+.2f})"
    )

    # One budget for the whole arc, derived the same way every other leg's is:
    # from the distance actually to be flown and the speed it will be flown at.
    approach_span = math.dist((state.n, state.e), (turn.entry_n, turn.entry_e))
    budget_s = estimate_move_timeout(approach_span + turn.arc_length_m, turn.speed_m_s)
    started = time.time()
    period = 1.0 / POSITION_RATE_HZ

    try:
        for index, (point_n, point_e, heading) in enumerate(samples):
            reached = _fly_to_point(
                nav,
                point_n,
                point_e,
                target_d,
                heading,
                speed_m_s=turn.speed_m_s,
                # The last point is the handover to gate_leg and is worth
                # settling on; the intermediate ones are a path, not a
                # destination, and stopping at each would reinstate the Phase 1B
                # stutter this whole phase exists to remove.
                tolerance_m=(
                    cfg.entry_lead_in_m * 0.5 if index < len(samples) - 1 else 0.25
                ),
                deadline=started + budget_s,
                period=period,
                label=f"{label} {index + 1}/{len(samples)}",
            )
            if not reached:
                elapsed = time.time() - started
                state = nav.get_vehicle_snapshot()
                log(
                    f"[!] {label} did not reach arc point {index + 1}/{len(samples)} "
                    f"within its {budget_s:.1f}s budget ({elapsed:.1f}s elapsed, "
                    f"vehicle at N={state.n:+.2f} E={state.e:+.2f}). "
                    f"Falling back to a stop-and-acquire."
                )
                return False
            if not nav.running:
                return False

        log(f"[*] {label} complete; on the next gate's centre line")
        return True
    finally:
        if nav.running:
            nav.hold_position(
                yaw_rad=turn.exit_heading_rad, label=f"after {label}"
            )


def _fly_to_point(
    nav: NavigationController,
    target_n: float,
    target_e: float,
    target_d: float,
    heading_rad: float,
    *,
    speed_m_s: float,
    tolerance_m: float,
    deadline: float,
    period: float,
    label: str,
) -> bool:
    """Drive toward one arc point, holding the planned heading.

    Same proportional-with-clamp law as ``move_to_target`` and the crossing leg.
    Yaw is commanded absolutely from the plan rather than pointed at the target,
    because through a transition the camera must look where the *next gate* is,
    not where the aircraft is going -- those differ by up to the turn angle, and
    a gate that leaves the frame during the turn has to be re-acquired from
    scratch on arrival.
    """
    while nav.running and time.time() < deadline:
        state = nav.get_vehicle_snapshot()
        err_n = target_n - state.n
        err_e = target_e - state.e
        err_d = target_d - state.d

        if math.hypot(math.hypot(err_n, err_e), err_d) <= tolerance_m:
            return True

        vn, ve, vd = KP_POS * err_n, KP_POS * err_e, KP_POS * err_d
        speed = math.sqrt(vn * vn + ve * ve + vd * vd)
        if speed > speed_m_s:
            scale = speed_m_s / speed
            vn, ve, vd = vn * scale, ve * scale, vd * scale

        nav.set_setpoint(
            Setpoint(
                kind=SetpointKind.VELOCITY_YAW,
                vn=vn, ve=ve, vd=vd,
                yaw_rad=heading_rad,
                label=label,
            )
        )
        nav.send_velocity_and_yaw_target(vn, ve, vd, heading_rad)
        time.sleep(period)

    return False


def describe_heading_profile(turn: TurnPlan, samples: int = 5) -> str:
    """One log line per arc point: where, and what heading.

    Printed before the transition is flown, so the console record shows the
    intended path rather than only the deviations from it.
    """
    if not turn.ok:
        return f"    (no profile: {turn.reason})"
    lines = []
    for index, (point_n, point_e, heading) in enumerate(turn.sample(samples)):
        fraction = index / (samples - 1)
        lines.append(
            f"    t={fraction:4.2f}  N={point_n:+7.2f} E={point_e:+7.2f}  "
            f"hdg={rad_to_deg(heading):+7.1f}deg"
        )
    return "\n".join(lines)


def yaw_rate_between(profile, speed_m_s: float, arc_length_m: float) -> float:
    """Peak commanded yaw rate implied by a sampled heading profile, deg/s.

    A check on the plan rather than a control input: if this exceeds the
    configured limit, the sampling or the plan is wrong, and it is better to
    find that in a unit test than in the air.
    """
    if len(profile) < 2 or arc_length_m <= 0.0 or speed_m_s <= 0.0:
        return 0.0
    segment_time = (arc_length_m / (len(profile) - 1)) / speed_m_s
    if segment_time <= 0.0:
        return 0.0
    return max(
        abs(rad_to_deg(wrap_pi(nxt[2] - cur[2]))) / segment_time
        for cur, nxt in zip(profile, profile[1:])
    )
