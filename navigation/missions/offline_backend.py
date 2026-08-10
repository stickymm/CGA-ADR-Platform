"""A no-MAVLink NavigationController for laptop testing.

``--offline`` replaces the MAVLink link with a crude kinematic model so a whole
mission can be driven end to end against the synthetic detection injector, with
no drone, no autopilot and no vision process.

WHAT THIS VALIDATES
    Sequencing, state machines, abort paths, geometry, retry bounds, logging.
    Every failure mode that lives in *this* code.

WHAT IT DOES NOT VALIDATE
    Anything about the aircraft.  There is no rotor dynamics, no PX4 controller,
    no estimator, no failsafe behaviour, no ground effect, no latency of the real
    link.  A mission that flies perfectly offline tells you the logic is sound and
    tells you nothing whatsoever about whether the vehicle will fly.

The model deliberately includes a first-order lag, and optional wind and
telemetry latency, because a stub that teleports instantly to every commanded
setpoint makes every move succeed -- which would hide precisely the timeout and
control-lag bugs this exercise exists to find.
"""

import math
import threading
import time
from typing import Tuple

from ..navigation import (
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    MAV_MODE_FLAG_SAFETY_ARMED,
    PX4_CUSTOM_MAIN_MODE_OFFBOARD,
    NavigationController,
    NullLink,
    Setpoint,
    SetpointKind,
    VehicleState,
    wrap_pi,
)

SIM_RATE_HZ = 50.0
SIM_MAX_YAW_RATE_RAD_S = math.radians(45.0)
SIM_ALTITUDE_KP = 1.2
SIM_LANDING_SPEED_M_S = 0.5
SIM_IN_AIR_ALT_M = 0.25
SIM_GRAVITY_M_S2 = 9.80665
SIM_MAX_TILT_RAD = math.radians(35.0)


class OfflineNavigationController(NavigationController):
    """Kinematic stand-in for a PX4 vehicle.  No socket is ever opened."""

    def __init__(
        self,
        *,
        lag_s: float = 0.3,
        wind_ned: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        latency_s: float = 0.0,
        arm_delay_s: float = 2.0,
        start_d: float = 0.0,
        start_yaw_rad: float = 0.0,
        model_tilt: bool = False,
    ):
        super().__init__("offline://none", dry_run=False)

        if lag_s < 0.0:
            raise ValueError("lag_s must be non-negative")
        if latency_s < 0.0:
            raise ValueError("latency_s must be non-negative")

        self.lag_s = lag_s
        self.wind_ned = wind_ned
        self.latency_s = latency_s
        self.arm_delay_s = arm_delay_s
        # OFF BY DEFAULT, on purpose.  A multirotor tilts to accelerate, so
        # modelling attitude is more realistic -- but the injector publishes
        # gate poses in the VEHICLE frame with no idea that the vehicle is
        # tilted, so switching this on makes the synthetic detections and the
        # synthetic attitude disagree.  That is exactly what you want when
        # exercising the 3-2-1 rotation path (`--offline-tilt`), and exactly what
        # you do not want when checking Phase 1 sequencing.
        self.model_tilt = model_tilt
        # Nothing to transmit to, but the controller's senders still go through
        # a link.  See navigation.NullLink for why this is not a DryRunLink.
        self.link = NullLink()

        self._sim_lock = threading.Lock()
        self._sim_n = 0.0
        self._sim_e = 0.0
        self._sim_d = start_d
        self._sim_yaw = start_yaw_rad
        self._sim_vn = 0.0
        self._sim_ve = 0.0
        self._sim_vd = 0.0
        self._sim_roll = 0.0
        self._sim_pitch = 0.0
        self._ground_d = start_d

        self._sim_armed = False
        self._sim_armed_once = False
        self._sim_offboard = False
        self._sim_landing = False
        self._sim_thread = None
        self._stream_started_s = 0.0

    # ------------------------------------------------------------------
    # lifecycle -- everything MAVLink is replaced
    # ------------------------------------------------------------------

    def start(self, *, require_offboard: bool = True, heartbeat_timeout_s: float = 30.0):
        if self.running:
            return

        print("[*] OFFLINE MODE: no MAVLink connection will be opened")
        print(
            f"[*] Kinematic model: lag={self.lag_s:.2f}s "
            f"wind={self.wind_ned} latency={self.latency_s:.2f}s "
            f"auto-arm after {self.arm_delay_s:.1f}s"
        )

        with self._vehicle_lock:
            self._vehicle = VehicleState()
        self._landing_confirmed = False
        self._closed = False

        self._running.set()
        self._sim_thread = threading.Thread(
            target=self._simulate, daemon=True, name="offline-sim"
        )
        self._sim_thread.start()

        if not self.wait_for_vehicle_state(timeout_s=5.0):
            raise RuntimeError("offline model failed to publish state")

        self.start_streaming()

        if require_offboard and not self.request_offboard():
            raise RuntimeError("offline model refused OFFBOARD")

    def stop(self):
        super().stop()
        thread = self._sim_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        self._sim_thread = None

    def _start_heartbeat(self):
        return  # nothing to announce ourselves to

    def request_message_intervals(self, intervals_hz=None) -> None:
        print("[*] OFFLINE: message intervals not applicable")

    def measure_telemetry_rate(self, duration_s: float = 3.0):
        return SIM_RATE_HZ, SIM_RATE_HZ

    # ------------------------------------------------------------------
    # command interception
    # ------------------------------------------------------------------

    def send_velocity_and_yaw_target(self, vn, ve, vd, yaw_rad):
        return  # the streamer already carries the setpoint the model reads

    def _send_raw_setpoint(self, setpoint: Setpoint):
        return

    def _send_set_mode_offboard(self):
        with self._sim_lock:
            self._sim_offboard = True

    def _send_land_command(self) -> bool:
        self._land_commanded_s = time.time()
        with self._sim_lock:
            self._sim_landing = True
        print("[*] OFFLINE: landing")
        return True

    def force_arm(self):
        """Arm the simulated vehicle immediately, as if the operator flipped the switch."""
        with self._sim_lock:
            self._sim_armed = True

    # ------------------------------------------------------------------
    # the model
    # ------------------------------------------------------------------

    def _simulate(self):
        """Integrate the commanded setpoint into a pose at a fixed rate."""
        period = 1.0 / SIM_RATE_HZ
        previous = time.monotonic()

        while self.running:
            now = time.monotonic()
            dt = min(0.2, now - previous)
            previous = now

            setpoint = self.current_setpoint()
            if self._stream_started_s == 0.0 and setpoint.kind is not SetpointKind.IDLE:
                self._stream_started_s = time.time()

            with self._sim_lock:
                # Arm exactly once, the way a real operator would. Without the
                # latch the model re-arms itself after the post-landing disarm
                # and the mission can never observe a completed landing.
                if (
                    not self._sim_armed
                    and not self._sim_armed_once
                    and self._stream_started_s > 0.0
                    and time.time() - self._stream_started_s >= self.arm_delay_s
                ):
                    self._sim_armed = True
                    self._sim_armed_once = True
                    print("[*] OFFLINE: simulated operator ARMED the vehicle")

                self._step(setpoint, dt)
                pose = (
                    self._sim_n, self._sim_e, self._sim_d, self._sim_yaw,
                    self._sim_vn, self._sim_ve, self._sim_vd,
                    self._sim_armed, self._sim_offboard,
                    self._sim_roll, self._sim_pitch,
                )

            self._publish_state(pose)
            time.sleep(max(0.0, period - (time.monotonic() - now)))

    def _step(self, setpoint: Setpoint, dt: float):
        """Advance the pose by one tick.  Caller holds ``_sim_lock``."""
        if not self._sim_armed or not self._sim_offboard:
            self._sim_vn = self._sim_ve = self._sim_vd = 0.0
            return

        if self._sim_landing:
            target_vn, target_ve = 0.0, 0.0
            target_vd = SIM_LANDING_SPEED_M_S
            target_yaw = self._sim_yaw
        elif setpoint.kind is SetpointKind.VELOCITY_YAW:
            target_vn, target_ve, target_vd = setpoint.vn, setpoint.ve, setpoint.vd
            target_yaw = setpoint.yaw_rad
        elif setpoint.kind is SetpointKind.BRAKE_HOLD_ALT:
            target_vn, target_ve = setpoint.vn, setpoint.ve
            target_vd = SIM_ALTITUDE_KP * (setpoint.d - self._sim_d)
            target_yaw = setpoint.yaw_rad
        elif setpoint.kind is SetpointKind.POSITION_YAW:
            target_vn = SIM_ALTITUDE_KP * (setpoint.n - self._sim_n)
            target_ve = SIM_ALTITUDE_KP * (setpoint.e - self._sim_e)
            target_vd = SIM_ALTITUDE_KP * (setpoint.d - self._sim_d)
            target_yaw = setpoint.yaw_rad
        else:
            target_vn = target_ve = target_vd = 0.0
            target_yaw = self._sim_yaw

        # First-order lag towards the commanded velocity.  Without this every
        # move completes instantly and the timeout logic is never exercised.
        alpha = 1.0 if self.lag_s <= 0.0 else min(1.0, dt / self.lag_s)
        previous_vn, previous_ve = self._sim_vn, self._sim_ve
        self._sim_vn += (target_vn - self._sim_vn) * alpha
        self._sim_ve += (target_ve - self._sim_ve) * alpha
        self._sim_vd += (target_vd - self._sim_vd) * alpha

        self._update_tilt(previous_vn, previous_ve, dt)

        self._sim_n += (self._sim_vn + self.wind_ned[0]) * dt
        self._sim_e += (self._sim_ve + self.wind_ned[1]) * dt
        self._sim_d += (self._sim_vd + self.wind_ned[2]) * dt

        # The floor is solid.
        if self._sim_d > self._ground_d:
            self._sim_d = self._ground_d
            self._sim_vd = 0.0

        yaw_error = wrap_pi(target_yaw - self._sim_yaw)
        max_step = SIM_MAX_YAW_RATE_RAD_S * dt
        self._sim_yaw = wrap_pi(
            self._sim_yaw + math.copysign(min(abs(yaw_error), max_step), yaw_error)
        )

        if self._sim_landing and self._sim_d >= self._ground_d - 0.02:
            self._sim_armed = False
            self._sim_offboard = False

    def _update_tilt(self, previous_vn: float, previous_ve: float, dt: float) -> None:
        """Derive roll and pitch from horizontal acceleration.

        A multirotor has no other way to accelerate: it tilts, and the horizontal
        component of thrust is what moves it.  For small angles

            pitch = -atan2(a_forward, g)      nose down to accelerate forward
            roll  = +atan2(a_right,   g)      right side down to accelerate right

        with the accelerations rotated into the body frame first.  This is the
        only place the offline model produces a non-level attitude, and it is the
        only way an offline run exercises the pitch/roll terms of the 3-2-1
        rotation at all.
        """
        if not self.model_tilt or dt <= 0.0:
            self._sim_roll = 0.0
            self._sim_pitch = 0.0
            return

        accel_n = (self._sim_vn - previous_vn) / dt
        accel_e = (self._sim_ve - previous_ve) / dt

        cos_yaw, sin_yaw = math.cos(self._sim_yaw), math.sin(self._sim_yaw)
        accel_forward = cos_yaw * accel_n + sin_yaw * accel_e
        accel_right = -sin_yaw * accel_n + cos_yaw * accel_e

        pitch = -math.atan2(accel_forward, SIM_GRAVITY_M_S2)
        roll = math.atan2(accel_right, SIM_GRAVITY_M_S2)
        self._sim_pitch = max(-SIM_MAX_TILT_RAD, min(SIM_MAX_TILT_RAD, pitch))
        self._sim_roll = max(-SIM_MAX_TILT_RAD, min(SIM_MAX_TILT_RAD, roll))

    def _publish_state(self, pose):
        n, e, d, yaw, vn, ve, vd, armed, offboard, roll, pitch = pose

        if self.latency_s > 0.0:
            time.sleep(0.0)  # latency is modelled by the stamp below, not a sleep

        stamp = time.time() - self.latency_s
        altitude = self._ground_d - d

        with self._vehicle_lock:
            self._vehicle.n, self._vehicle.e, self._vehicle.d = n, e, d
            self._vehicle.vn, self._vehicle.ve, self._vehicle.vd = vn, ve, vd
            self._vehicle.yaw_rad = yaw
            self._vehicle.roll_rad = roll
            self._vehicle.pitch_rad = pitch
            self._vehicle.have_local_position = True
            self._vehicle.have_attitude = True
            self._vehicle.position_received_s = stamp
            self._vehicle.attitude_received_s = stamp
            self._vehicle.heartbeat_received_s = stamp
            # A healthy simulated estimator, so estimator_note() reads
            # "estimator healthy" offline instead of "never seen".
            self._vehicle.estimator_flags = 0xFFFF
            self._vehicle.estimator_vel_ratio = 0.1
            self._vehicle.estimator_pos_horiz_ratio = 0.1
            self._vehicle.estimator_hagl_ratio = 0.1
            self._vehicle.estimator_received_s = stamp
            self._vehicle.armed = armed
            self._vehicle.base_mode = (
                MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                | (MAV_MODE_FLAG_SAFETY_ARMED if armed else 0)
            )
            self._vehicle.custom_mode = (
                PX4_CUSTOM_MAIN_MODE_OFFBOARD << 16 if offboard else 0
            )
            self._vehicle.landed_state = 2 if altitude > SIM_IN_AIR_ALT_M else 1

        # Feed the deskew ring buffer, which the real controller fills from the
        # LOCAL_POSITION_NED branch of its reader thread.
        self._record_history(self.get_vehicle_snapshot())
