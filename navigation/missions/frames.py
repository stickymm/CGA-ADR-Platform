"""Pure geometry for gate navigation.

No MAVLink, no UDP, no NavigationController -- every function here is a pure
function of its arguments, so the whole approach geometry can be unit-tested on
a laptop with no hardware and no simulator.  That property is the point: this
codebase has no flight-test-before-deployment path, so the geometry has to be
provable by arithmetic instead.

FRAMES
    body (FRD)  +x forward, +y right, +z down   -- what the vision process emits
    local NED   +n north,   +e east,  +d down   -- what PX4 reports and accepts

    ``d`` is *down*, so ``d = -1.5`` is 1.5 m above the origin.  Sign errors on
    ``d`` are the classic way to fly a drone into the floor; every function here
    that touches ``d`` says which way is up in its docstring.

UNITS
    Distances metres, angles radians unless the name ends in ``_deg``.
    ``GateDetection.roll``/``pitch``/``yaw_deg`` and the camera offsets are
    degrees; ``VehicleState.yaw_rad`` and ``LocalTarget.yaw_rad`` are radians.
"""

import math
from typing import Iterable, Sequence, Tuple

from ..navigation import (
    GateDetection,
    LocalTarget,
    VehicleState,
    body_to_local,
    wrap_pi,
    deg_to_rad,
    rad_to_deg,
)
from .contracts import AltitudeEnvelope, CommitVerdict, GateFix, GateLegConfig


def rotate_body_to_ned(
    forward: float,
    right: float,
    down: float,
    yaw_rad: float,
    pitch_rad: float = 0.0,
    roll_rad: float = 0.0,
) -> Tuple[float, float, float]:
    """Rotate a body-frame (FRD) vector into local NED.

    Standard aerospace 3-2-1 sequence -- yaw about z, then pitch about y, then
    roll about x -- matching PX4's ``ATTITUDE`` message convention::

        R = Rz(yaw) @ Ry(pitch) @ Rx(roll)

          [ cy*cp   cy*sp*sr - sy*cr   cy*sp*cr + sy*sr ]
          [ sy*cp   sy*sp*sr + cy*cr   sy*sp*cr - cy*sr ]
          [ -sp     cp*sr              cp*cr            ]

    With ``roll = pitch = 0`` this collapses exactly to the planar yaw-only
    rotation the legacy missions perform, which is asserted in the tests -- so
    adopting full attitude cannot silently change any existing behaviour when the
    vehicle is level.

    The correction matters: a gate 3 m ahead observed at 10 degrees of body pitch
    is mis-placed vertically by ``3 * sin(10 deg) = 0.52 m`` if pitch is ignored,
    and that error feeds straight into the commanded altitude.

    IMPLEMENTATION NOTE
        This is a keyword-friendly alias of
        :func:`navigation.navigation.body_to_local`, which now carries the 3-2-1
        matrix itself.  There is deliberately only **one** copy of the rotation
        in the repo: two copies of a rotation matrix is how the legacy missions
        ended up with a different frame convention from the Phase 1 stack in the
        first place.

    Returns:
        ``(dn, de, dd)`` -- the displacement in local NED, metres.
    """
    return body_to_local(forward, right, down, yaw_rad, pitch_rad, roll_rad)


def circular_mean_deg(values: Iterable[float]) -> float:
    """Mean of angles in degrees, correct across the +/-180 wrap.

    The arithmetic mean is wrong for angles: ``mean(179, -179)`` is ``0``, off by
    a full 180 degrees, when the answer is 180.  Gate yaws sit near +/-180
    routinely -- and resolving the gate-normal ambiguity makes that *more*
    common, not less -- so this is a live correctness issue, not a nicety.

    Returns:
        The mean direction in ``[-180, 180)``.  Raises ``ValueError`` if empty.
    """
    sin_sum = 0.0
    cos_sum = 0.0
    count = 0
    for value in values:
        radians = deg_to_rad(value)
        sin_sum += math.sin(radians)
        cos_sum += math.cos(radians)
        count += 1

    if count == 0:
        raise ValueError("circular_mean_deg requires at least one value")

    return rad_to_deg(math.atan2(sin_sum / count, cos_sum / count))


def resolve_gate_normal(
    gate_n: float,
    gate_e: float,
    drone_n: float,
    drone_e: float,
    reported_yaw_rad: float,
) -> Tuple[float, float, bool]:
    """Orient the gate normal along the direction of travel.

    A plane's normal can be reported in either direction, and the detector's
    choice is not guaranteed.  Rather than assuming a convention, decide
    geometrically: dot the candidate normal with the drone->gate vector.  If it
    points back at the drone, flip it 180 degrees.

    After this, the standoff point is *always* on the drone's side of the gate
    and the pass-through *always* continues in the direction of travel -- so the
    approach is correct whichever convention the detector happens to use.

    Degenerate case: if the drone is exactly in the gate plane the dot product is
    zero and there is no travel direction to align with.  We leave the normal
    unflipped and let the caller's cone-angle check reject it, which it will,
    because the cone angle is then exactly 90 degrees.

    Returns:
        ``(normal_n, normal_e, flipped)`` with the normal a unit horizontal
        vector.  ``flipped`` is logged so the field behaviour is auditable.
    """
    normal_n = math.cos(reported_yaw_rad)
    normal_e = math.sin(reported_yaw_rad)

    to_gate_n = gate_n - drone_n
    to_gate_e = gate_e - drone_e

    if normal_n * to_gate_n + normal_e * to_gate_e < 0.0:
        return -normal_n, -normal_e, True
    return normal_n, normal_e, False


def gate_cone_angle_deg(
    gate_n: float,
    gate_e: float,
    normal_n: float,
    normal_e: float,
    drone_n: float,
    drone_e: float,
) -> float:
    """Angle between the gate normal and the drone->gate vector, in degrees.

    Zero means the drone sits on the gate's centre axis looking straight
    through it -- the best case for a planar pose estimate.  Ninety degrees means
    the gate is edge-on, where the plane fit is worthless.  Callers treat
    anything beyond ~60 degrees as low confidence rather than acting on it.

    Returns 90.0 for a zero-length drone->gate vector, which is the correct
    "unusable" answer for a degenerate input.
    """
    to_gate_n = gate_n - drone_n
    to_gate_e = gate_e - drone_e
    span = math.hypot(to_gate_n, to_gate_e)
    if span <= 1e-9:
        return 90.0

    cosine = (normal_n * to_gate_n + normal_e * to_gate_e) / span
    return rad_to_deg(math.acos(max(-1.0, min(1.0, cosine))))


def lateral_offset_from_axis(
    gate_n: float,
    gate_e: float,
    normal_n: float,
    normal_e: float,
    drone_n: float,
    drone_e: float,
) -> float:
    """Perpendicular distance from the gate's centre axis, metres.

    Diagnostic only -- deliberately NOT used by the commit gate.  It depends on
    the gate normal, which comes from the PnP yaw of a near-planar square at
    1-3 m range; that is the classic weakly-observable degree of freedom, and a
    few degrees of yaw error becomes tens of centimetres of apparent offset.
    The commit gate uses the directly-measured body-frame offsets instead.
    """
    return abs(normal_n * (drone_e - gate_e) - normal_e * (drone_n - gate_n))


def clamp_altitude(
    d: float,
    envelope: AltitudeEnvelope,
) -> Tuple[float, bool]:
    """Clamp a commanded NED ``d`` into the altitude envelope.

    ``d`` is down-positive, so *higher* altitude is a *more negative* ``d``.  The
    permitted band is therefore ``[d_takeoff - max_alt, d_takeoff - min_alt]``.

    Referencing the pad's latched ``d_takeoff`` rather than assuming zero means
    the envelope follows the EKF origin wherever it happens to be.

    Returns:
        ``(clamped_d, was_clamped)``.
    """
    highest = envelope.d_takeoff - envelope.max_alt_m   # most negative
    lowest = envelope.d_takeoff - envelope.min_alt_m    # least negative
    clamped = min(max(d, highest), lowest)
    return clamped, clamped != d


def altitude_agl(d: float, envelope: AltitudeEnvelope) -> float:
    """Height above the pad in metres, from a NED ``d``."""
    return envelope.d_takeoff - d


def crossed_gate_plane(
    drone_n: float,
    drone_e: float,
    fix: GateFix,
    clearance_m: float,
) -> bool:
    """True once the drone is ``clearance_m`` past the gate plane along the normal.

    This replaces a position-tolerance arrival test.  Asking whether the drone is
    within 0.10 m of a point *beyond* the gate is a stop-and-settle test applied
    to a fly-through manoeuvre -- it cannot be satisfied while still moving, so
    the pass-through leg reports failure even when it went perfectly.  A signed
    projection onto the gate normal asks the question that actually matters:
    are we through?
    """
    along = (drone_n - fix.n) * fix.normal_n + (drone_e - fix.e) * fix.normal_e
    return along >= clearance_m


def localize_gate(
    det: GateDetection,
    state: VehicleState,
    envelope: AltitudeEnvelope,
    *,
    cam_offset_right_m: float = 0.0,
    cam_offset_down_m: float = 0.0,
    cam_yaw_offset_deg: float = 0.0,
    aim_bias_down_m: float = 0.0,
    max_cone_deg: float = 60.0,
) -> GateFix:
    """Turn one body-relative detection into a local-NED :class:`GateFix`.

    Uses the vehicle's *full* attitude, resolves the normal ambiguity, clamps the
    implied altitude, and flags low-confidence detections rather than acting on
    them.

    ``range_m`` is recomputed as ``hypot(forward, right, down)`` and NOT taken
    from ``det.dist``.  The vision process derives ``dist`` from the *raw* pose
    but publishes ``forward``/``right``/``down`` from the *smoothed* pose, so the
    two disagree -- most of all while moving, which is exactly when the commit
    decision is made.  ``det.dist`` is used only as the no-detection sentinel.

    TWO VERTICAL CORRECTIONS, AND THEY MEAN DIFFERENT THINGS
        ``cam_offset_down_m`` is a physical fact: how far the camera is mounted
        below the vehicle reference point.  Positive means below, which makes
        the gate appear lower and flies the drone down.  See the derivation on
        ``CAM_OFFSET_DOWN_M`` in ``navigation.py`` -- its sign was inverted for
        the first three flights and cost ~0.20 m of altitude on every approach.

        ``aim_bias_down_m`` is a deliberate choice: where inside the gate
        opening to fly.  Positive aims lower, buying clearance for propellers
        and battery, which sit above the reference point and are the parts you
        least want to touch a gate leg.

        Both land on the same axis, so both are folded in here rather than at
        the target-building step.  That keeps the commit decision and the flown
        target measuring the same thing -- if only the target were biased, the
        commit gate would be demanding the vehicle centre on a point the
        approach was deliberately steering away from.

        CONSEQUENCE: ``GateFix.d`` and ``vertical_body_m`` describe the AIM
        POINT, not the geometric gate centre.  With a zero bias they are the
        same thing, which is the default everywhere except a real flight.
    """
    corrected_right = det.right + cam_offset_right_m
    corrected_down = det.down + cam_offset_down_m + aim_bias_down_m

    dn, de, dd = rotate_body_to_ned(
        det.forward,
        corrected_right,
        corrected_down,
        yaw_rad=state.yaw_rad,
        pitch_rad=getattr(state, "pitch_rad", 0.0),
        roll_rad=getattr(state, "roll_rad", 0.0),
    )

    gate_n = state.n + dn
    gate_e = state.e + de
    gate_d_raw = state.d + dd

    reported_yaw = wrap_pi(state.yaw_rad + deg_to_rad(det.yaw_deg + cam_yaw_offset_deg))
    normal_n, normal_e, flipped = resolve_gate_normal(
        gate_n, gate_e, state.n, state.e, reported_yaw
    )

    cone_deg = gate_cone_angle_deg(gate_n, gate_e, normal_n, normal_e, state.n, state.e)
    gate_d, clamped = clamp_altitude(gate_d_raw, envelope)

    reasons = []
    if cone_deg > max_cone_deg:
        reasons.append(f"edge-on ({cone_deg:.1f}deg > {max_cone_deg:.1f}deg)")
    if clamped:
        reasons.append(
            f"altitude outside envelope (raw {altitude_agl(gate_d_raw, envelope):+.2f}m AGL)"
        )

    return GateFix(
        n=gate_n,
        e=gate_e,
        d=gate_d,
        normal_n=normal_n,
        normal_e=normal_e,
        yaw_rad=math.atan2(normal_e, normal_n),
        range_m=math.hypot(math.hypot(det.forward, corrected_right), corrected_down),
        lateral_body_m=corrected_right,
        vertical_body_m=corrected_down,
        lateral_axis_m=lateral_offset_from_axis(
            gate_n, gate_e, normal_n, normal_e, state.n, state.e
        ),
        cone_angle_deg=cone_deg,
        normal_flipped=flipped,
        altitude_clamped=clamped,
        low_confidence=bool(reasons),
        reason="; ".join(reasons),
    )


def localize_gates(
    detections: Sequence[GateDetection],
    state: VehicleState,
    envelope: AltitudeEnvelope,
    *,
    cam_offset_right_m: float = 0.0,
    cam_offset_down_m: float = 0.0,
    cam_yaw_offset_deg: float = 0.0,
    aim_bias_down_m: float = 0.0,
    max_cone_deg: float = 60.0,
) -> Tuple[GateFix, ...]:
    """Localize **every** gate in one vision packet, not just the nearest.

    Phase 1 consumed ``gates[0]`` and discarded the other two rows.  That is
    correct for 1A, and holds for 1B only because each gate is crossed before
    the next is sought -- "nearest gate" and "next gate" happen to coincide.
    Phase 2 plans a transition toward the *following* gate while still
    approaching the current one, so it needs all of them.

    Every fix is computed against the same vehicle pose, so they are mutually
    consistent even though the vehicle is moving.  Ordering follows the input,
    which the publisher sorts nearest-first -- and which is explicitly **not** a
    stable identity across frames.  See :mod:`.association`.
    """
    return tuple(
        localize_gate(
            det,
            state,
            envelope,
            cam_offset_right_m=cam_offset_right_m,
            cam_offset_down_m=cam_offset_down_m,
            cam_yaw_offset_deg=cam_yaw_offset_deg,
            aim_bias_down_m=aim_bias_down_m,
            max_cone_deg=max_cone_deg,
        )
        for det in detections
    )


def evaluate_commit(fix: GateFix, cfg: GateLegConfig) -> CommitVerdict:
    """Decide whether to stop trusting vision and fly the gate open-loop.

    Three independent conditions, all required:

    1. **Range** -- close enough that the gate is about to leave the camera's
       field of view anyway.
    2. **Alignment** -- the gate centre is near the camera boresight, measured in
       the *body* frame.  ``right``/``down`` are directly observed, unlike an
       axis offset derived from the weakly-observable plane yaw, so the threshold
       is meaningful rather than dominated by PnP noise.
    3. **Not edge-on** -- the plane fit is trustworthy enough to aim with.

    Range alone is not sufficient: slant range stays large while the drone is
    crabbed off to one side, which is precisely the geometry that clips a gate
    leg on an open-loop pass.
    """
    in_range = fix.range_m <= cfg.commit_distance_m
    aligned_lateral = abs(fix.lateral_body_m) <= cfg.commit_lateral_tol_m
    aligned_vertical = abs(fix.vertical_body_m) <= cfg.commit_vertical_tol_m
    within_cone = fix.cone_angle_deg <= cfg.commit_max_cone_deg

    failures = []
    if not in_range:
        failures.append(f"range {fix.range_m:.2f}m > {cfg.commit_distance_m:.2f}m")
    if not aligned_lateral:
        failures.append(
            f"lateral {fix.lateral_body_m:+.2f}m > +/-{cfg.commit_lateral_tol_m:.2f}m"
        )
    if not aligned_vertical:
        failures.append(
            f"vertical {fix.vertical_body_m:+.2f}m > +/-{cfg.commit_vertical_tol_m:.2f}m"
        )
    if not within_cone:
        failures.append(
            f"cone {fix.cone_angle_deg:.1f}deg > {cfg.commit_max_cone_deg:.1f}deg"
        )

    return CommitVerdict(
        ok=not failures and not fix.low_confidence,
        in_range=in_range,
        aligned_lateral=aligned_lateral,
        aligned_vertical=aligned_vertical,
        within_cone=within_cone,
        range_m=fix.range_m,
        lateral_body_m=fix.lateral_body_m,
        vertical_body_m=fix.vertical_body_m,
        cone_angle_deg=fix.cone_angle_deg,
        reason="; ".join(failures) or fix.reason,
    )


def standoff_target(fix: GateFix, standoff_m: float, envelope: AltitudeEnvelope) -> LocalTarget:
    """A point ``standoff_m`` in front of the gate, on the gate's centre axis.

    "In front" means on the drone's side, which the resolved normal guarantees.
    Yaw is set to the gate's normal so the camera stays pointed through the gate.
    """
    if standoff_m < 0.0:
        raise ValueError("standoff_m must be non-negative")
    d, _ = clamp_altitude(fix.d, envelope)
    return LocalTarget(
        n=fix.n - standoff_m * fix.normal_n,
        e=fix.e - standoff_m * fix.normal_e,
        d=d,
        yaw_rad=fix.yaw_rad,
    )


def limit_approach_step(
    current: VehicleState,
    target: LocalTarget,
    max_horizontal_m: float,
    max_vertical_m: float,
) -> LocalTarget:
    """Cap one approach step, budgeting horizontal and vertical SEPARATELY.

    WHY THIS EXISTS RATHER THAN ``limit_target_step``
        ``limit_target_step`` caps the **3-D** displacement.  That is correct for
        a move whose axes are equally trustworthy -- and wrong for an approach,
        because on this airframe they are not remotely equal.

        Measured in flight (2026-08-13, first live gate approach): the
        body-frame ``down`` reading swung across 0.63 m -- from -0.06 m to
        +0.57 m -- while the gate physically never moved.  The commanded
        altitude bobbed between D = -1.16 and -1.52 chasing that noise.  With a
        single 3-D cap, every centimetre of that vertical chase came straight
        out of the forward budget: 0.25 m steps delivered only 0.14 m of range
        closure, 56% efficiency, and the leg ran out of attempts at 1.41 m from
        a gate it needed to reach 1.00 m of.

        Splitting the budget makes the horizontal step immune to vertical noise.
        The vertical cap should be small: altitude error is real but it is not
        urgent, it corrects a little on every attempt, and the commit gate
        measures it directly at the end anyway.

    Both caps are applied independently, so the total 3-D displacement may reach
    ``hypot(max_horizontal_m, max_vertical_m)``.  That is intended -- it is the
    horizontal *progress* that is being protected, not the vector magnitude.
    """
    if max_horizontal_m <= 0.0:
        raise ValueError("max_horizontal_m must be positive")
    if max_vertical_m <= 0.0:
        raise ValueError("max_vertical_m must be positive")

    delta_n = target.n - current.n
    delta_e = target.e - current.e
    delta_d = target.d - current.d

    horizontal = math.hypot(delta_n, delta_e)
    if horizontal > max_horizontal_m:
        scale = max_horizontal_m / horizontal
        delta_n *= scale
        delta_e *= scale

    if abs(delta_d) > max_vertical_m:
        delta_d = math.copysign(max_vertical_m, delta_d)

    return LocalTarget(
        n=current.n + delta_n,
        e=current.e + delta_e,
        d=current.d + delta_d,
        yaw_rad=target.yaw_rad,
    )


def pass_through_target(fix: GateFix, pass_dist_m: float, envelope: AltitudeEnvelope) -> LocalTarget:
    """A point ``pass_dist_m`` beyond the gate, along the direction of travel."""
    if pass_dist_m <= 0.0:
        raise ValueError("pass_dist_m must be positive")
    d, _ = clamp_altitude(fix.d, envelope)
    return LocalTarget(
        n=fix.n + pass_dist_m * fix.normal_n,
        e=fix.e + pass_dist_m * fix.normal_e,
        d=d,
        yaw_rad=fix.yaw_rad,
    )


def backoff_target(
    fix: GateFix,
    state: VehicleState,
    distance_m: float,
    envelope: AltitudeEnvelope,
) -> LocalTarget:
    """Retreat ``distance_m`` straight back along the gate normal.

    Used by the recovery ladder: a gate is most often lost by getting too close
    for it to fit in frame, and backing up widens the field of view again.  Yaw
    is held on the gate so the camera keeps looking where the gate was.
    """
    if distance_m <= 0.0:
        raise ValueError("distance_m must be positive")
    d, _ = clamp_altitude(state.d, envelope)
    return LocalTarget(
        n=state.n - distance_m * fix.normal_n,
        e=state.e - distance_m * fix.normal_e,
        d=d,
        yaw_rad=fix.yaw_rad,
    )


def average_detections(samples: Sequence[GateDetection]) -> GateDetection:
    """Average a burst of detections, using a circular mean for the angles.

    Linear fields are averaged arithmetically; ``roll``/``pitch``/``yaw_deg`` are
    angles and use :func:`circular_mean_deg`, which is what keeps a gate observed
    near +/-180 degrees from averaging to zero.
    """
    if not samples:
        raise ValueError("average_detections requires at least one sample")

    count = len(samples)
    return GateDetection(
        timestamp=max(sample.timestamp for sample in samples),
        dist=sum(s.dist for s in samples) / count,
        forward=sum(s.forward for s in samples) / count,
        right=sum(s.right for s in samples) / count,
        down=sum(s.down for s in samples) / count,
        roll=circular_mean_deg(s.roll for s in samples),
        pitch=circular_mean_deg(s.pitch for s in samples),
        yaw_deg=circular_mean_deg(s.yaw_deg for s in samples),
    )
