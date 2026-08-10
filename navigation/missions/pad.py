"""Everything between "python -m ..." and "the drone is hovering".

The ordering here is load-bearing and is not arbitrary:

    connect -> heartbeat -> telemetry -> detections -> START STREAM
    -> banner -> operator confirm -> WAIT FOR ARM -> verify OFFBOARD -> climb

Three constraints force it:

1. A ``udpin:`` pymavlink connection cannot transmit *anything* until it has
   received a packet, so the heartbeat wait comes first or nothing works.
2. PX4 refuses OFFBOARD unless setpoints are already streaming, so the stream
   starts before the mode request.  Priming works while disarmed and in any
   mode, which is what makes the rest possible.
3. PX4 auto-disarms a vehicle that is armed but has not taken off
   (``COM_DISARM_PRFLT``, 10 s by default).  So the operator confirm happens
   *before* arming, and arming is the final commit action -- there is no
   countdown racing that timer, and the RC arm switch stays a physical abort
   throughout.

The invariant that keeps the vehicle on the pad: PX4 latches ``want_takeoff``
the instant it is armed and sees a fresh setpoint with negative vz.  If the
stream carried the climb setpoint while we waited for the operator, arming and
lift-off would be the same event.  The stream therefore carries *finite zeros*
until OFFBOARD has been confirmed, and :func:`assert_not_climbing` checks it.
"""

import time
from dataclasses import dataclass
from typing import Callable, Optional, Sequence, Tuple

from ..navigation import (
    NavigationController,
    Setpoint,
    SetpointKind,
)
from .contracts import AltitudeEnvelope

LogFn = Callable[[str], None]


@dataclass(frozen=True)
class PadConfig:
    """Pre-flight gates.  Every threshold is a refusal-to-fly, not a warning."""

    altitude: AltitudeEnvelope = AltitudeEnvelope()
    min_position_rate_hz: float = 15.0
    min_attitude_rate_hz: float = 20.0
    telemetry_measure_s: float = 3.0
    detection_wait_s: float = 6.0
    # How long the feed must keep *changing* before it counts as alive. Must
    # comfortably exceed frozen_feed_frames / publisher fps (15/30 = 0.5 s).
    detection_observe_s: float = 1.0
    arm_wait_s: float = 300.0
    offboard_timeout_s: float = 10.0
    takeoff_timeout_s: float = 20.0
    takeoff_tolerance_m: float = 0.15
    require_detections: bool = True
    require_confirm: bool = True
    skip_telemetry_rate_check: bool = False


@dataclass(frozen=True)
class PadResult:
    ok: bool
    reason: str
    envelope: Optional[AltitudeEnvelope] = None


def print_banner(
    title: str,
    rows: Sequence[Tuple[str, str]],
    preconditions: Sequence[str],
    log: LogFn = print,
) -> None:
    """The last thing the operator reads before committing the aircraft."""
    width = 68
    log("")
    log("=" * width)
    log(f"  {title}")
    log("=" * width)
    for label, value in rows:
        log(f"  {label:.<34} {value}")
    log("-" * width)
    log("  ASSUMED PRECONDITIONS -- confirm each before arming:")
    for item in preconditions:
        log(f"    [ ] {item}")
    log("=" * width)


def confirm(prompt: str = "Type GO to continue (anything else aborts): ") -> bool:
    """Blocking operator confirmation.

    Deliberately not a countdown: a countdown that runs while the vehicle is
    armed races PX4's pre-takeoff auto-disarm, and a countdown while disarmed
    just delays the operator for no reason.  A keypress waits exactly as long as
    the human needs.
    """
    try:
        return input(prompt).strip().upper() == "GO"
    except (EOFError, KeyboardInterrupt):
        return False


def assert_not_climbing(nav: NavigationController) -> None:
    """Refuse to proceed if the streamed setpoint would command a climb.

    See the module docstring: this is the check that stops the vehicle leaving
    the pad at the instant the operator arms it.
    """
    setpoint = nav.current_setpoint()
    if setpoint.climbs():
        raise RuntimeError(
            "Streamed setpoint commands a climb before OFFBOARD was verified. "
            "PX4 would lift off the moment the operator arms. Refusing."
        )


def wait_for_detections(
    mission,
    nav: NavigationController,
    wait_s: float,
    log: LogFn = print,
    *,
    min_observe_s: float = 1.0,
) -> bool:
    """Confirm the vision process is alive and its detections are fresh.

    Distinguishes the four cases the old code conflated: no packets at all
    (vision is dead or the port is wrong), packets carrying only the 999
    no-detection sentinel (normal, no gate in view), packets carrying a gate
    whose pose never changes (a stalled camera -- the failure that looks exactly
    like health), and fresh real detections.

    Refusing to take off on a frozen feed is the whole point of checking it here:
    airborne, the same condition costs a recovery ladder and a gate.
    """
    deadline = time.time() + wait_s
    saw_packet = False
    first_fresh_s = None
    latest = None

    while nav.running and time.time() < deadline:
        if getattr(mission, "feed_frozen", False):
            log(
                f"[!] Vision feed is FROZEN: {mission.identical_frame_count} identical "
                f"payloads in a row. The publisher is alive but its pose is not "
                f"changing -- a stalled camera or a wedged pipeline. Refusing."
            )
            return False

        det = mission.get_latest_detection_snapshot()
        if det is not None:
            saw_packet = True
            age = time.time() - det.timestamp
            if age <= mission.detection_max_age_s:
                latest = det
                if first_fresh_s is None:
                    first_fresh_s = time.time()
                # Watch for a full frozen-detection window before declaring the
                # feed healthy. Returning on the first fresh packet would pass a
                # stalled camera every time -- it publishes fresh packets, they
                # just all say the same thing.
                elif time.time() - first_fresh_s >= min_observe_s:
                    log(
                        f"[*] Vision feed alive and CHANGING over {min_observe_s:.1f}s: "
                        f"fwd={latest.forward:+.2f}m right={latest.right:+.2f}m "
                        f"down={latest.down:+.2f}m yaw={latest.yaw_deg:+.1f}deg "
                        f"dist={latest.dist:.2f}m (age {age*1000:.0f}ms)"
                    )
                    return True
        time.sleep(0.05)

    if first_fresh_s is not None:
        log(
            f"[!] Vision feed produced fresh detections but not for the full "
            f"{min_observe_s:.1f}s freshness window inside the {wait_s:.1f}s budget."
        )
    elif saw_packet:
        log("[!] Vision packets are arriving but every one is stale or a no-detection row.")
    else:
        log(f"[!] No vision packets on UDP {mission.udp_ip}:{mission.udp_port}.")
    return False


def run_pad_sequence(
    nav: NavigationController,
    mission,
    cfg: PadConfig,
    banner_rows: Sequence[Tuple[str, str]],
    *,
    title: str,
    log: LogFn = print,
) -> PadResult:
    """Take the vehicle from "process started" to "hovering at altitude"."""

    preconditions = [
        "Vision process running and publishing to UDP 127.0.0.1:5050",
        "Drone DISARMED, props fitted, on level ground, clear of the gate",
        "PX4 parameter checklist confirmed (COM_LOW_BAT_ACT, COM_OBL_RC_ACT, "
        "COM_RCL_EXCEPT, EKF2_OF_CTRL, EKF2_HGT_REF, MPC_XY_VEL_MAX)",
        "Safety pilot holding the RC with the mode switch and kill switch ready",
        "Test area clear of people",
    ]

    # --- 1. link, telemetry, and background threads -------------------------
    log("\n[STEP 1] Connecting and waiting for telemetry")
    try:
        nav.start(require_offboard=False)
    except Exception as exc:
        return PadResult(False, f"could not start controller: {exc}")

    # --- 2. telemetry rates -------------------------------------------------
    log("\n[STEP 2] Requesting and measuring telemetry rates")
    nav.request_message_intervals()
    if cfg.skip_telemetry_rate_check:
        log("[*] Telemetry rate check SKIPPED by request")
    else:
        position_hz, attitude_hz = nav.measure_telemetry_rate(cfg.telemetry_measure_s)
        log(
            f"[*] Measured over {cfg.telemetry_measure_s:.0f}s: "
            f"LOCAL_POSITION_NED {position_hz:.1f} Hz, ATTITUDE {attitude_hz:.1f} Hz"
        )
        if position_hz < cfg.min_position_rate_hz or attitude_hz < cfg.min_attitude_rate_hz:
            return PadResult(
                False,
                f"telemetry too slow ({position_hz:.1f}/{attitude_hz:.1f} Hz, need "
                f"{cfg.min_position_rate_hz:.0f}/{cfg.min_attitude_rate_hz:.0f} Hz)",
            )

    # --- 3. vision ----------------------------------------------------------
    log("\n[STEP 3] Checking the vision feed")
    if not wait_for_detections(
        mission,
        nav,
        cfg.detection_wait_s,
        log=log,
        min_observe_s=cfg.detection_observe_s,
    ):
        if cfg.require_detections:
            return PadResult(False, "no fresh gate detections before takeoff")
        log("[!] Continuing without detections because --allow-no-detections was set")

    # --- 4. latch the pad reference and show the operator everything --------
    state = nav.get_vehicle_snapshot()
    envelope = AltitudeEnvelope(
        takeoff_alt_m=cfg.altitude.takeoff_alt_m,
        min_alt_m=cfg.altitude.min_alt_m,
        max_alt_m=cfg.altitude.max_alt_m,
        d_takeoff=state.d,
    )
    nav.altitude_envelope = envelope

    rows = list(banner_rows) + [
        ("pad reference d", f"{state.d:+.2f} m (NED)"),
        ("dry run", "YES -- nothing will be transmitted" if nav.dry_run else "no"),
    ]
    print_banner(title, rows, preconditions, log=log)

    # --- 5. operator confirm, while still DISARMED --------------------------
    log("\n[STEP 4] Operator confirmation (vehicle is still DISARMED)")
    if cfg.require_confirm and not confirm():
        return PadResult(False, "operator aborted at the confirmation prompt")

    # --- 6. arm is the commit action ----------------------------------------
    log("\n[STEP 5] Waiting for ARM -- this is your commit action")
    assert_not_climbing(nav)
    if not nav.wait_for_armed(timeout_s=cfg.arm_wait_s):
        return PadResult(False, "vehicle was never armed")

    # --- 7. offboard, verified ----------------------------------------------
    log("\n[STEP 6] Entering OFFBOARD")
    assert_not_climbing(nav)
    if not nav.request_offboard(timeout_s=cfg.offboard_timeout_s):
        return PadResult(False, "OFFBOARD was not confirmed via HEARTBEAT")

    # --- 8. climb, verified against telemetry rather than a fixed sleep -----
    log(f"\n[STEP 7] Taking off to {envelope.takeoff_alt_m:.2f} m AGL")
    if not takeoff(nav, envelope, cfg, log=log):
        return PadResult(False, "did not reach takeoff altitude", envelope)

    return PadResult(True, "ready", envelope)


def takeoff(
    nav: NavigationController,
    envelope: AltitudeEnvelope,
    cfg: PadConfig,
    *,
    log: LogFn = print,
) -> bool:
    """Climb to the envelope's takeoff altitude and confirm arrival.

    Uses a brake-and-hold-altitude setpoint (xy *velocity* zero, z *position*
    target) rather than a full position setpoint.  A full position setpoint
    carries an x/y target, and if that is anything but the position captured at
    the mode-switch instant the vehicle translates toward it -- at up to
    ``MPC_XY_VEL_MAX``, which defaults to 12 m/s, while 30 cm off the ground.
    Commanding zero horizontal velocity cannot run away like that, while still
    letting PX4 close the altitude loop for us.

    Arrival is confirmed against ``LOCAL_POSITION_NED``, never a fixed sleep:
    PX4 spends ~1 s spooling up and ~3 s ramping thrust, so any hard-coded climb
    duration is guesswork.
    """
    state = nav.get_vehicle_snapshot()
    target_d = envelope.d_takeoff - envelope.takeoff_alt_m

    nav.set_setpoint(
        Setpoint(
            kind=SetpointKind.BRAKE_HOLD_ALT,
            vn=0.0,
            ve=0.0,
            d=target_d,
            yaw_rad=state.yaw_rad,
            label="takeoff",
        )
    )

    deadline = time.time() + cfg.takeoff_timeout_s
    announced_air = False

    while nav.running and time.time() < deadline:
        state = nav.get_vehicle_snapshot()
        altitude = envelope.d_takeoff - state.d

        if state.landed_state == 2 and not announced_air:
            log("[*] Airborne (EXTENDED_SYS_STATE reports IN_AIR)")
            announced_air = True

        if abs(state.d - target_d) <= cfg.takeoff_tolerance_m:
            log(f"[*] Reached {altitude:.2f} m AGL")
            nav.hold_position(label="post-takeoff hold")
            return True

        time.sleep(0.1)

    state = nav.get_vehicle_snapshot()
    log(
        f"[!] Takeoff timed out at {envelope.d_takeoff - state.d:.2f} m AGL "
        f"(wanted {envelope.takeoff_alt_m:.2f} m)"
    )
    return False
