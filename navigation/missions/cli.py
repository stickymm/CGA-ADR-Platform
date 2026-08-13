"""Shared command-line surface and object construction for the gate missions.

Every mission exposes the same flight envelope, gate-approach, recovery and
hardware options, so they live here once rather than being copy-pasted per
mission.  What deliberately does NOT live here is any part of a mission's
*sequence* -- that stays in the mission file, where a reader at a test site can
see the whole flow without opening a second file.
"""

import argparse
from typing import Tuple

from ..navigation import (
    CAM_OFFSET_DOWN_M,
    CAM_OFFSET_RIGHT_M,
    CAM_YAW_OFFSET_DEG,
    MAVLINK_CONN,
    GateMission,
    NavigationController,
)
from .contracts import AltitudeEnvelope, GateLegConfig
from .offline_backend import OfflineNavigationController
from .pad import PadConfig


class DetectionOnlyMission(GateMission):
    """Carries the UDP detection listener; the sequence lives in the mission file.

    ``GateMission`` supplies the listener thread, the detection snapshot, and the
    camera offsets.  The missions drive their own sequence rather than going
    through ``run_mission``, because the pad sequence needs to interleave
    controller startup with operator interaction.
    """

    def run(self, nav):  # pragma: no cover - missions drive their own sequence
        raise NotImplementedError("the mission module drives this sequence directly")


def add_common_arguments(parser: argparse.ArgumentParser) -> None:
    """Add every option shared by the gate missions."""

    modes = parser.add_argument_group("modes")
    modes.add_argument(
        "--offline",
        action="store_true",
        help="no MAVLink at all; run against a kinematic model so the injector "
             "can drive a whole mission on a laptop",
    )
    modes.add_argument(
        "--dry-run",
        action="store_true",
        help="connect to a real vehicle and read its telemetry, print every "
             "command, transmit nothing",
    )

    offline = parser.add_argument_group("offline model")
    offline.add_argument("--offline-lag-s", type=float, default=0.3,
                         help="first-order velocity lag; 0 makes every move succeed "
                              "instantly and hides timeout bugs")
    offline.add_argument("--offline-wind-n", type=float, default=0.0)
    offline.add_argument("--offline-wind-e", type=float, default=0.0)
    offline.add_argument("--offline-latency-s", type=float, default=0.0)
    offline.add_argument("--offline-arm-delay-s", type=float, default=2.0)
    offline.add_argument(
        "--offline-tilt",
        action="store_true",
        help="model roll/pitch from horizontal acceleration, which exercises the "
             "full 3-2-1 rotation. The injector emits vehicle-frame poses and "
             "cannot compensate, so expect the gate estimate to shift while "
             "accelerating -- that is the point, not a bug",
    )

    flight = parser.add_argument_group("flight envelope")
    flight.add_argument("--takeoff-altitude-m", type=float, default=1.5)
    flight.add_argument("--min-altitude-m", type=float, default=0.8)
    flight.add_argument("--max-altitude-m", type=float, default=2.5)
    flight.add_argument("--approach-speed-m-s", type=float, default=0.35)
    flight.add_argument("--cross-speed-m-s", type=float, default=0.45)

    gate = parser.add_argument_group("gate approach")
    gate.add_argument("--commit-distance-m", type=float, default=1.0)
    gate.add_argument("--commit-lateral-tol-m", type=float, default=0.20)
    gate.add_argument("--commit-vertical-tol-m", type=float, default=0.25)
    gate.add_argument("--commit-max-cone-deg", type=float, default=60.0)
    gate.add_argument("--commit-confirm-frames", type=int, default=3)
    gate.add_argument(
        "--step-size-m",
        type=float,
        default=0.35,
        help="HORIZONTAL displacement cap for one approach step. Raised from "
             "0.25 after the first live approach ran out of attempts",
    )
    gate.add_argument(
        "--vertical-step-m",
        type=float,
        default=0.12,
        help="vertical displacement cap for one approach step, budgeted "
             "separately so noisy gate-height estimates cannot eat the forward "
             "progress",
    )
    gate.add_argument(
        "--min-observation-samples",
        type=int,
        default=3,
        help="fewest averaged detections that may be called a lock; a thinner "
             "window is treated as a miss rather than acted on",
    )
    gate.add_argument("--pass-distance-m", type=float, default=1.5)
    gate.add_argument("--exit-clearance-m", type=float, default=0.8)
    gate.add_argument("--observation-duration-s", type=float, default=0.8,
                  help="sampling window per observation. 0.5 s produced as few as ONE usable sample in flight")
    gate.add_argument("--max-detection-age-s", type=float, default=0.4)
    gate.add_argument(
        "--arrival-tolerance-m",
        type=float,
        default=0.15,
        help="how close a move must get before it counts as arrived. Must be "
             "smaller than --step-size-m. 0.10 m is inside the noise floor of "
             "an optical-flow position estimate",
    )
    gate.add_argument(
        "--frozen-feed-frames",
        type=int,
        default=15,
        help="identical vision payloads before the feed is declared frozen",
    )

    limits = parser.add_argument_group("limits and recovery")
    limits.add_argument("--max-approach-attempts", type=int, default=24)
    limits.add_argument("--approach-timeout-s", type=float, default=150.0)
    limits.add_argument(
        "--max-yaw-rate-deg-s",
        type=float,
        default=45.0,
        help="commanded yaw rate for searches and turns. Do not rely on PX4's "
             "MPC_YAWRAUTO_MAX for this -- fast yaw degrades optical flow",
    )
    limits.add_argument("--max-observe-retries", type=int, default=3)
    limits.add_argument("--max-scan-sweeps", type=int, default=2,
                        help="each sweep observes at every heading, and sweep N "
                             "looks N x --scan-half-angle-deg wide")
    limits.add_argument("--scan-half-angle-deg", type=float, default=45.0,
                        help="first-sweep half angle. A zig-zag needs ~45; a box "
                             "course turns ~90, which sweep 2 reaches")
    limits.add_argument(
        "--crossed-gate-avoid-m",
        type=float,
        default=0.8,
        help="multi-gate: reject a fix this close to the gate just crossed, "
             "so one gate cannot be flown and counted twice. 0 disables -- "
             "needed only if two gates genuinely sit within a metre",
    )
    limits.add_argument("--backoff-distance-m", type=float, default=1.0)
    limits.add_argument("--max-backoffs", type=int, default=2)

    hardware = parser.add_argument_group("hardware")
    hardware.add_argument("--mavlink", default=MAVLINK_CONN)
    hardware.add_argument("--camera-right-offset-m", type=float, default=CAM_OFFSET_RIGHT_M)
    hardware.add_argument("--camera-down-offset-m", type=float, default=CAM_OFFSET_DOWN_M)
    hardware.add_argument("--camera-yaw-offset-deg", type=float, default=CAM_YAW_OFFSET_DEG)

    checks = parser.add_argument_group("pre-flight checks")
    checks.add_argument("--allow-no-detections", action="store_true",
                        help="take off even if no fresh gate detection was seen")
    checks.add_argument("--skip-telemetry-rate-check", action="store_true")
    checks.add_argument("--no-confirm", action="store_true",
                        help="skip the operator prompt; implied by --offline")
    checks.add_argument("--min-position-rate-hz", type=float, default=15.0)
    checks.add_argument("--min-attitude-rate-hz", type=float, default=20.0)


def camera_offsets_given(args) -> bool:
    """True if the operator set any camera offset explicitly."""
    return (
        args.camera_right_offset_m != CAM_OFFSET_RIGHT_M
        or args.camera_down_offset_m != CAM_OFFSET_DOWN_M
        or args.camera_yaw_offset_deg != CAM_YAW_OFFSET_DEG
    )


def resolve_offline_camera_offsets(args) -> None:
    """Zero the mount offsets for offline runs, loudly.

    The synthetic injector emits gate poses already in the VEHICLE frame, while
    the real vision process emits raw CAMERA-frame values that the mount offsets
    exist to correct.  Applying the offsets to synthetic data therefore injects a
    bias the drone can never null out: a fixed lateral error that blocks the
    commit gate forever, and a yaw offset that accumulates once per observation.
    Explicit --camera-* flags still override.
    """
    if not args.offline or camera_offsets_given(args):
        return
    print(
        "[*] OFFLINE: camera mount offsets forced to zero "
        "(the injector emits vehicle-frame poses; pass --camera-*-offset-* to override)"
    )
    args.camera_right_offset_m = 0.0
    args.camera_down_offset_m = 0.0
    args.camera_yaw_offset_deg = 0.0


def build_envelope(args) -> AltitudeEnvelope:
    return AltitudeEnvelope(
        takeoff_alt_m=args.takeoff_altitude_m,
        min_alt_m=args.min_altitude_m,
        max_alt_m=args.max_altitude_m,
    )


def build_leg_config(args, envelope: AltitudeEnvelope) -> GateLegConfig:
    return GateLegConfig(
        commit_distance_m=args.commit_distance_m,
        step_size_m=args.step_size_m,
        approach_speed_m_s=args.approach_speed_m_s,
        observation_duration_s=args.observation_duration_s,
        max_approach_attempts=args.max_approach_attempts,
        approach_timeout_s=args.approach_timeout_s,
        commit_lateral_tol_m=args.commit_lateral_tol_m,
        commit_vertical_tol_m=args.commit_vertical_tol_m,
        commit_max_cone_deg=args.commit_max_cone_deg,
        commit_confirm_frames=args.commit_confirm_frames,
        pass_distance_m=args.pass_distance_m,
        exit_clearance_m=args.exit_clearance_m,
        cross_speed_m_s=args.cross_speed_m_s,
        max_observe_retries=args.max_observe_retries,
        scan_half_angle_deg=args.scan_half_angle_deg,
        max_yaw_rate_deg_s=args.max_yaw_rate_deg_s,
        max_scan_sweeps=args.max_scan_sweeps,
        backoff_distance_m=args.backoff_distance_m,
        max_backoffs=args.max_backoffs,
        max_detection_age_s=args.max_detection_age_s,
        arrival_tolerance_m=args.arrival_tolerance_m,
        vertical_step_m=args.vertical_step_m,
        crossed_gate_avoid_m=args.crossed_gate_avoid_m,
        min_observation_samples=args.min_observation_samples,
        altitude=envelope,
    )


def build_pad_config(args, envelope: AltitudeEnvelope) -> PadConfig:
    return PadConfig(
        altitude=envelope,
        min_position_rate_hz=args.min_position_rate_hz,
        min_attitude_rate_hz=args.min_attitude_rate_hz,
        require_detections=not args.allow_no_detections,
        require_confirm=not (args.no_confirm or args.offline),
        skip_telemetry_rate_check=args.skip_telemetry_rate_check or args.offline,
    )


def build_controller(args) -> NavigationController:
    if args.offline:
        return OfflineNavigationController(
            lag_s=args.offline_lag_s,
            wind_ned=(args.offline_wind_n, args.offline_wind_e, 0.0),
            latency_s=args.offline_latency_s,
            arm_delay_s=args.offline_arm_delay_s,
            model_tilt=args.offline_tilt,
        )
    return NavigationController(args.mavlink, dry_run=args.dry_run)


def build_mission(args) -> DetectionOnlyMission:
    return DetectionOnlyMission(
        cam_offset_right_m=args.camera_right_offset_m,
        cam_offset_down_m=args.camera_down_offset_m,
        cam_yaw_offset_deg=args.camera_yaw_offset_deg,
        detection_max_age_s=args.max_detection_age_s,
        frozen_feed_frames=args.frozen_feed_frames,
        min_observation_samples=args.min_observation_samples,
    )


def build(args) -> Tuple[NavigationController, DetectionOnlyMission, GateLegConfig, PadConfig]:
    """Turn parsed arguments into the objects a mission needs."""
    resolve_offline_camera_offsets(args)
    envelope = build_envelope(args)
    return (
        build_controller(args),
        build_mission(args),
        build_leg_config(args, envelope),
        build_pad_config(args, envelope),
    )


def check_mode_flags(args) -> bool:
    """False if the mode flags are contradictory."""
    if args.offline and args.dry_run:
        print("[!] --offline and --dry-run are mutually exclusive")
        return False
    return True
