"""Approach one gate, commit, and cross it.

THE RULE THIS MODULE EXISTS TO KEEP
    The mission file owns the *sequence*.  This module owns one *step*.

    Someone standing at a test site must be able to open a ``phase1*`` file and
    read every state transition, abort branch and landing decision without
    opening a second file.  They open this one once -- and because Phase 1A,
    Phase 1B and Phase 2 all call the identical function, understanding it once
    covers the whole stack.

    That is why this is a free function returning a named enum rather than a
    base-class template method.  Six of the seven ``move_to_target`` call sites
    in this repo ignore its boolean return; a ``GateResult`` is much harder to
    drop on the floor, and there is no MRO to resolve at 6am.

STATE MACHINE -- every state is bounded, every exit is named

    SEARCH -> APPROACH -> ALIGN -> COMMIT -> EXIT

    SEARCH    hold and re-observe; then a bounded yaw sweep; then back off,
              because a gate is most often lost by getting too close for it to
              fit in frame and retreating widens the field of view again.
    APPROACH  step toward a standoff point on the gate's centre axis.
    ALIGN     in range but not lined up: move onto the axis and re-check.
    COMMIT    freeze the last good fix and fly it open-loop -- the gate leaves
              the camera's field of view as you enter it, so vision is blind
              exactly when the crossing happens.
    EXIT      hold, or whatever the caller passed as the exit behaviour.
"""

import math
import time
from typing import Callable, Optional

from ..navigation import (
    LocalTarget,
    NavigationController,
    Setpoint,
    SetpointKind,
    KP_POS,
    POSITION_RATE_HZ,
    estimate_move_timeout,
    wrap_pi,
)
from .contracts import GateFix, GateLegConfig, GateOutcome, GateResult
from .frames import (
    backoff_target,
    crossed_gate_plane,
    evaluate_commit,
    localize_gate,
    pass_through_target,
    standoff_target,
)
from .single_gate import limit_target_step

LogFn = Callable[[str], None]


def approach_and_cross_one_gate(
    nav: NavigationController,
    mission,
    cfg: GateLegConfig,
    *,
    gate_index: int = 0,
    log: LogFn = print,
    expected_gate: Optional[GateFix] = None,
    exit_leg: Optional[Callable[[NavigationController, GateFix], bool]] = None,
) -> GateOutcome:
    """Fly one gate.  Always returns; never raises for an in-flight condition.

    ``expected_gate``
        Where the caller believes this gate is, in local NED.  Phase 2 knows,
        because it has been tracking the gate across frames; Phase 1 does not
        and passes None, which is the default and changes nothing.

        Used only to **reject** an observation that cannot be the expected gate
        -- it never supplies a position.  A gate 4 m from where the mission
        expected one is far more likely to be a different gate, a reflection, or
        a doorway than it is to be evidence that the mission was wrong.  Acting
        on it means flying at the wrong thing with full confidence; refusing it
        costs one observation.

    ``exit_leg``
        What to do once the plane is crossed, replacing the default "hold".
        This is the seam that lets Phase 2 fly a planned transition out of the
        gate **without gate_leg knowing anything about Phase 2** -- and, just as
        importantly, lets Phase 2 degrade to Phase 1B behaviour by passing None
        rather than by branching.  Returning False marks the exit as
        incomplete; the crossing itself still counts, because it happened.
    """

    started_s = time.time()
    deadline = started_s + cfg.approach_timeout_s
    envelope = nav.altitude_envelope or cfg.altitude
    entry_state = nav.get_vehicle_snapshot()

    attempts = 0
    align_attempts = 0
    observe_failures = 0
    expected_mismatches = 0
    scans_used = 0
    backoffs_used = 0
    reacquires = 0
    confirmed_frames = 0
    last_fix: Optional[GateFix] = None

    def finish(result: GateResult, reason: str) -> GateOutcome:
        log(f"[*] Gate {gate_index}: {result.value.upper()} -- {reason}")
        return GateOutcome(
            result=result,
            gate_index=gate_index,
            reason=reason,
            attempts=attempts,
            reacquires=reacquires,
            elapsed_s=time.time() - started_s,
            fix=last_fix,
            entry_state=entry_state,
            exit_state=nav.get_vehicle_snapshot(),
        )

    log(f"\n--- GATE {gate_index}: acquiring ---")

    while True:
        if not nav.running:
            return finish(GateResult.ABORTED, "controller stopped")
        if time.time() > deadline:
            return finish(
                GateResult.TIMED_OUT,
                f"exceeded {cfg.approach_timeout_s:.0f}s budget for this gate",
            )
        if not nav.telemetry_ok(cfg.max_telemetry_age_s):
            return finish(
                GateResult.ABORTED,
                f"telemetry older than {cfg.max_telemetry_age_s:.2f}s"
                + nav.estimator_note(),
            )
        if not nav.get_vehicle_snapshot().in_offboard:
            # PX4 sets `offboard_control_signal_lost` when the ESTIMATOR fails
            # its velocity innovation check past COM_VEL_FS_EVH, so the obvious
            # reading of this message -- "the companion link dropped" -- is
            # wrong about as often as it is right on a flow-only airframe.
            # estimator_note() says which one it actually was.
            return finish(
                GateResult.ABORTED,
                "PX4 is no longer in OFFBOARD" + nav.estimator_note(),
            )
        if attempts >= cfg.max_approach_attempts:
            return finish(
                GateResult.NO_COMMIT,
                f"used all {cfg.max_approach_attempts} approach attempts",
            )

        detection = mission.observe_gate(nav, duration=cfg.observation_duration_s)

        # ---------------- SEARCH: the recovery ladder ----------------
        if detection is None:
            observe_failures += 1
            log(f"[!] Gate {gate_index} not seen (miss {observe_failures})")

            if observe_failures <= cfg.max_observe_retries:
                continue

            reacquires += 1
            if scans_used < cfg.max_scan_sweeps:
                scans_used += 1
                observe_failures = 0
                log(f"[*] Recovery rung 2: yaw sweep {scans_used}/{cfg.max_scan_sweeps}")
                _yaw_sweep(nav, cfg, log=log)
                continue

            if last_fix is not None and backoffs_used < cfg.max_backoffs:
                backoffs_used += 1
                observe_failures = 0
                log(f"[*] Recovery rung 3: backing off {cfg.backoff_distance_m:.2f} m")
                nav.move_to_target(
                    backoff_target(
                        last_fix, nav.get_vehicle_snapshot(), cfg.backoff_distance_m, envelope
                    ),
                    f"gate {gate_index} back-off",
                    max_speed_m_s=cfg.approach_speed_m_s,
                    tolerance_m=cfg.arrival_tolerance_m,
                )
                continue

            return finish(
                GateResult.NOT_FOUND if last_fix is None else GateResult.LOST,
                "recovery ladder exhausted (observe, sweep, back off)",
            )

        observe_failures = 0
        attempts += 1

        # ---------------- localize and report everything ----------------
        # Deskew: the detection describes where the gate was when its FRAME was
        # captured, not when the datagram arrived. state_at() returns the pose
        # closest to that stamp, falling back to the current snapshot when there
        # is no history, so this is safe for every caller.
        state = nav.state_at(detection.timestamp)
        fix = localize_gate(
            detection,
            state,
            envelope,
            cam_offset_right_m=mission.cam_offset_right_m,
            cam_offset_down_m=mission.cam_offset_down_m,
            cam_yaw_offset_deg=mission.cam_yaw_offset_deg,
            max_cone_deg=cfg.commit_max_cone_deg,
        )

        if expected_gate is not None:
            strayed = math.dist(
                (fix.n, fix.e, fix.d),
                (expected_gate.n, expected_gate.e, expected_gate.d),
            )
            if strayed > cfg.expected_gate_radius_m:
                # Counted separately from observe_failures, which the successful
                # -observation path above resets on every pass. Sharing that
                # counter would mean this one never accumulates, and a mission
                # that rejects every observation would loop until its deadline
                # rather than reporting what happened.
                expected_mismatches += 1
                log(
                    f"[!] Gate {gate_index}: observation is {strayed:.2f} m from where "
                    f"this gate was expected (limit {cfg.expected_gate_radius_m:.2f} m). "
                    f"More likely a different gate or a false positive than a "
                    f"correction. Discarding and re-observing "
                    f"({expected_mismatches}/{cfg.max_observe_retries + 1})."
                )
                if expected_mismatches > cfg.max_observe_retries:
                    return finish(
                        GateResult.LOST,
                        f"every observation was too far from the expected gate "
                        f"position (last {strayed:.2f} m away)",
                    )
                continue

        last_fix = fix
        _log_fix(fix, state, gate_index, attempts, envelope, log)

        verdict = evaluate_commit(fix, cfg)
        log(f"    {verdict.describe()}")
        if verdict.reason:
            log(f"    withheld because: {verdict.reason}")

        # ---------------- COMMIT ----------------
        if verdict.ok:
            confirmed_frames += 1
            log(
                f"    commit gate held {confirmed_frames}/{cfg.commit_confirm_frames} "
                f"consecutive frames"
            )
            if confirmed_frames >= cfg.commit_confirm_frames:
                log(
                    f"\n--- GATE {gate_index}: COMMITTING. Vision is no longer "
                    f"trusted; flying the frozen pose. ---"
                )
                if not _fly_through_gate(nav, fix, cfg, envelope, gate_index, log=log):
                    return finish(
                        GateResult.TIMED_OUT, "pass-through did not clear the gate plane"
                    )

                # The gate IS crossed at this point. Whatever the exit leg does
                # or fails to do cannot un-cross it, so the outcome stays
                # CROSSED and a failed exit is reported in the reason instead.
                if exit_leg is None:
                    return finish(GateResult.CROSSED, "gate crossed")
                try:
                    if exit_leg(nav, fix):
                        return finish(GateResult.CROSSED, "gate crossed; exit leg flown")
                    return finish(
                        GateResult.CROSSED,
                        "gate crossed; exit leg did not complete (holding instead)",
                    )
                except Exception as exc:
                    log(f"[!] Exit leg raised {exc!r}; holding after the crossing")
                    if nav.running:
                        nav.hold_position(yaw_rad=fix.yaw_rad, label=f"after gate {gate_index}")
                    return finish(
                        GateResult.CROSSED, f"gate crossed; exit leg failed: {exc!r}"
                    )
            continue
        confirmed_frames = 0

        # ---------------- ALIGN vs APPROACH ----------------
        if verdict.in_range:
            align_attempts += 1
            if align_attempts > cfg.max_align_attempts:
                return finish(
                    GateResult.NO_COMMIT,
                    f"in range but never aligned after {cfg.max_align_attempts} "
                    f"corrections ({verdict.reason})",
                )
            standoff_m = cfg.commit_distance_m
            label = f"gate {gate_index} align {align_attempts}"
        else:
            standoff_m = max(cfg.commit_distance_m, fix.range_m - cfg.step_size_m)
            label = f"gate {gate_index} standoff {standoff_m:.2f}m"

        desired = standoff_target(fix, standoff_m, envelope)
        stepped = limit_target_step(state, desired, cfg.step_size_m)
        if not nav.move_to_target(
            stepped,
            label,
            max_speed_m_s=cfg.approach_speed_m_s,
            tolerance_m=cfg.arrival_tolerance_m,
        ):
            log(f"[!] Move '{label}' did not complete; re-observing before retrying")


def _log_fix(fix: GateFix, state, gate_index: int, attempt: int, envelope, log: LogFn) -> None:
    """Print everything needed to debug the geometry from a console log alone."""
    log(
        f"  [gate {gate_index} obs {attempt}] drone N={state.n:+.2f} E={state.e:+.2f} "
        f"D={state.d:+.2f} ({envelope.d_takeoff - state.d:+.2f}m AGL) "
        f"yaw={math.degrees(state.yaw_rad):+7.1f} "
        f"roll={math.degrees(state.roll_rad):+5.1f} pitch={math.degrees(state.pitch_rad):+5.1f}"
    )
    log(
        f"    gate NED  N={fix.n:+.2f} E={fix.e:+.2f} D={fix.d:+.2f} "
        f"({envelope.d_takeoff - fix.d:+.2f}m AGL)  heading={math.degrees(fix.yaw_rad):+7.1f}deg"
    )
    log(
        f"    normal    ({fix.normal_n:+.3f}, {fix.normal_e:+.3f})  "
        f"{'FLIPPED (detector reported it reversed)' if fix.normal_flipped else 'as reported'}"
    )
    log(
        f"    measured  range={fix.range_m:.2f}m  body lateral={fix.lateral_body_m:+.2f}m  "
        f"body vertical={fix.vertical_body_m:+.2f}m  axis offset={fix.lateral_axis_m:.2f}m"
    )
    if fix.altitude_clamped:
        log("    NOTE: gate altitude was outside the envelope and has been clamped")
    if fix.low_confidence:
        log(f"    LOW CONFIDENCE: {fix.reason}")


def _yaw_sweep(nav: NavigationController, cfg: GateLegConfig, *, log: LogFn = print) -> None:
    """Sweep the nose either side of its current heading, looking for the gate.

    Rate-limited on purpose, and now actually rate-limited by *this* code rather
    than by hoping PX4's ``MPC_YAWRAUTO_MAX`` is still at its default.  On an
    optical-flow airframe a fast yaw injects rotation-induced flow that the
    estimator has to cancel with gyro data alone, and degrading the position
    estimate during a *recovery* would be exactly the wrong trade.

    The previous version issued each +/-45 degree step as a single
    ``move_to_target`` with a new absolute yaw, i.e. "turn there as fast as you
    like".  ``slew_yaw`` ramps the commanded heading at
    ``cfg.max_yaw_rate_deg_s`` instead.
    """
    state = nav.get_vehicle_snapshot()
    half = math.radians(cfg.scan_half_angle_deg)

    for offset in (+half, -half, 0.0):
        if not nav.running:
            return
        heading = wrap_pi(state.yaw_rad + offset)
        nav.slew_yaw(
            heading,
            label=f"scan to {math.degrees(heading):+.0f}deg",
            max_rate_deg_s=cfg.max_yaw_rate_deg_s,
        )


def _fly_through_gate(
    nav: NavigationController,
    fix: GateFix,
    cfg: GateLegConfig,
    envelope,
    gate_index: int,
    *,
    log: LogFn = print,
) -> bool:
    """Fly the frozen gate pose, completing on a plane crossing.

    The completion test is a signed projection onto the gate normal, NOT arrival
    within a position tolerance.  Asking whether the drone is within 0.10 m of a
    point beyond the gate is a stop-and-settle test applied to a fly-through
    manoeuvre: it cannot be satisfied while still moving, so the leg reports
    failure on every run even when the crossing was perfect.  Combined with a
    fixed 15 s budget that a 2.5 m leg at 0.15 m/s cannot meet, that is why the
    old course mission announced success from an ignored ``False``.
    """
    target = pass_through_target(fix, cfg.pass_distance_m, envelope)
    state = nav.get_vehicle_snapshot()
    span = math.dist((state.n, state.e, state.d), (target.n, target.e, target.d))
    budget_s = estimate_move_timeout(span, cfg.cross_speed_m_s)

    log(
        f"[*] Crossing gate {gate_index}: target N={target.n:+.2f} E={target.e:+.2f} "
        f"D={target.d:+.2f}, {span:.2f}m at {cfg.cross_speed_m_s:.2f}m/s, "
        f"budget {budget_s:.1f}s, clearance {cfg.exit_clearance_m:.2f}m"
    )

    period = 1.0 / POSITION_RATE_HZ
    started = time.time()

    try:
        while nav.running:
            state = nav.get_vehicle_snapshot()

            if crossed_gate_plane(state.n, state.e, fix, cfg.exit_clearance_m):
                log(f"[*] Gate {gate_index} CROSSED (past the plane by "
                    f"{cfg.exit_clearance_m:.2f}m)")
                return True

            if time.time() - started > budget_s:
                along = (state.n - fix.n) * fix.normal_n + (state.e - fix.e) * fix.normal_e
                log(
                    f"[!] Crossing timed out after {budget_s:.1f}s; "
                    f"only {along:+.2f}m along the gate normal"
                )
                return False

            err_n = target.n - state.n
            err_e = target.e - state.e
            err_d = target.d - state.d

            vn, ve, vd = KP_POS * err_n, KP_POS * err_e, KP_POS * err_d
            speed = math.sqrt(vn * vn + ve * ve + vd * vd)
            if speed > cfg.cross_speed_m_s:
                scale = cfg.cross_speed_m_s / speed
                vn, ve, vd = vn * scale, ve * scale, vd * scale

            nav.set_setpoint(
                Setpoint(
                    kind=SetpointKind.VELOCITY_YAW,
                    vn=vn, ve=ve, vd=vd,
                    yaw_rad=fix.yaw_rad,
                    label=f"crossing gate {gate_index}",
                )
            )
            nav.send_velocity_and_yaw_target(vn, ve, vd, fix.yaw_rad)
            time.sleep(period)

        return False
    finally:
        if nav.running:
            nav.hold_position(yaw_rad=fix.yaw_rad, label=f"after gate {gate_index}")
