# WALKTHROUGH — what this code actually does, end to end

**Written against commit `11c63b1` on branch `Gabe-Claude`, working tree clean, 269 tests green.**
This describes the code **as it is on disk right now**, not as planned. Where something is
untested or unexercised, it says so.

> **READ THIS FIRST.** The Findings Log at the end contains one **HIGH-severity Phase 2
> bug observed live** (a planned transition can command the aircraft to fly *backwards*
> through the gate it just crossed). It does not affect Phase 1A/1B or a props-off bench
> test. It does affect any Phase 2 flight. See Finding **F-1**.

---

## Ground truth reconciliation (Step 0)

| Check | Result |
|---|---|
| `git status` | **Clean.** Nothing uncommitted, nothing half-finished. |
| `HEAD` | `11c63b1` on `Gabe-Claude`, 5 commits ahead of `c00083c` |
| Test suite | `Ran 269 tests … OK` |
| `compileall` | clean across `navigation scripts vision debug` |

**Discrepancies found between PROJECT_STATE.md and disk — both now corrected in that file:**

1. It claimed last commit `94be44a`; actual `HEAD` was `11c63b1` (the commit that wrote the
   claim). Now lists the whole commit range.
2. It claimed *"The injector binds UDP 5050"*. **It does not.** `udp_injector.py` only ever
   calls `sendto`; the **mission** binds 5050, and so does `debug/telemetry_reader.py`. The
   real constraint is that you may have only one *consumer* **and** one *publisher*. Two
   publishers interleave silently — this actually happened during offline validation and
   produced a run where `--case frozen` was not detected.

Everything else in PROJECT_STATE.md matches disk, including the per-file test counts.

---

# PART 1 — Architecture map

## 1.1 The three processes

This is not one program. On the aircraft it is **three** cooperating processes, and
knowing which is which is the difference between debugging the right thing and not:

```
  ┌──────────────────────────┐   UDP 5050 JSON    ┌───────────────────────────┐
  │  VISION  (Raspberry Pi)  │ ─────────────────► │  NAVIGATION (Raspberry Pi)│
  │  scripts/run_vision.py   │  30 Hz, 3 rows     │  phase1a / 1b / phase2    │
  └──────────────────────────┘                    └───────────────────────────┘
        │ Flask MJPEG :5000                              │ MAVLink udpin :14551
        ▼                                                ▼
   laptop browser                        ┌────────────────────────────┐
                                         │  MAVLINK ROUTER (you must  │
                                         │  provide — see Part 6)     │
                                         └────────────────────────────┘
                                              │serial            │UDP 14550
                                              ▼                  ▼
                                           PX4 FMU          QGroundControl
```

The three never share memory. They share exactly two contracts: the **UDP JSON wire
format** and **MAVLink**. Both are documented below.

## 1.2 File-by-file

### Pure — no I/O, fully unit-testable on a laptop

These are where the correctness lives. Every one is a pure function of its arguments.

| File | Responsibility | Public surface | Depends on | Depended on by |
|---|---|---|---|---|
| `navigation/missions/contracts.py` | Every type crossing a seam + the tolerance/timeout/deadline arithmetic | `GateResult`, `GateFix`, `AltitudeEnvelope`, `GateOutcome`, `CommitVerdict`, `GateLegConfig`, `attempt_move_budget_s`, `attempt_budget_s`, `deadline_consistency`, `summarize_config` | `navigation.VehicleState`; lazily `estimate_move_timeout` | everything |
| `navigation/missions/frames.py` | Gate geometry | `rotate_body_to_ned`, `circular_mean_deg`, `resolve_gate_normal`, `gate_cone_angle_deg`, `lateral_offset_from_axis`, `clamp_altitude`, `altitude_agl`, `crossed_gate_plane`, `localize_gate`, `localize_gates`, `evaluate_commit`, `standoff_target`, `pass_through_target`, `backoff_target`, `average_detections` | `navigation` (types + `body_to_local`), `contracts` | `gate_leg`, `phase2_course`, `course` |
| `navigation/missions/association.py` | Track gates across frames with no gate ID | `AssociationConfig`, `GateTrack`, `AssociationResult`, `GateTracker` | `contracts.GateFix` | `phase2_course` |
| `navigation/missions/course.py` | Course-vector planning | `CoursePlanConfig`, `PassVector`, `TurnPlan`, `LegPlan`, `pass_vector`, `track_pass_vector`, `minimum_turn_radius_m`, `plan_turn`, `plan_leg`, `order_gates_along_course` | `navigation` (`LocalTarget`, `wrap_pi`, `MAX_YAW_RATE_DEG_S`), `contracts`, lazily `frames.clamp_altitude` | `phase2_course`, `vector_leg` |
| `navigation/missions/single_gate.py` | **Legacy, byte-identical.** Two helpers are still live | `plan_standoff_horizon`, `limit_target_step`, `SingleGateMission` | `navigation` | `gate_leg` imports `limit_target_step` |

### Touches MAVLink

| File | Responsibility | Public surface | Notes |
|---|---|---|---|
| `navigation/navigation.py` (2295 lines) | All MAVLink I/O, vehicle state, flight primitives, the UDP gate listener | `NavigationController`, `GateMission`, `Mission`, `VehicleState`, `GateDetection`, `LocalTarget`, `Setpoint`, `SetpointKind`, `MissionOutcome`, `PymavlinkLink`, `DryRunLink`, `NullLink`, `body_to_local`, `wrap_pi`, `estimate_move_timeout`, `is_valid_detection`, `type_mask_for` | **The only file that imports `pymavlink`.** Also the only file that opens the UDP 5050 socket |
| `navigation/missions/offline_backend.py` | `OfflineNavigationController` — kinematic stand-in, no socket ever opened | subclass of `NavigationController` | Overrides every sender; installs a `NullLink` |

### Sequencing / flight logic (calls MAVLink through the controller, opens nothing itself)

| File | Responsibility | Public surface |
|---|---|---|
| `navigation/missions/pad.py` | Everything from process start to hovering | `PadConfig`, `PadResult`, `print_banner`, `confirm`, `assert_not_climbing`, `wait_for_detections`, `run_pad_sequence`, `takeoff` |
| `navigation/missions/gate_leg.py` | **THE shared routine.** One gate: search → approach → align → commit → cross → exit | `approach_and_cross_one_gate` |
| `navigation/missions/vector_leg.py` | Fly a planned Phase 2 transition arc | `fly_turn`, `describe_heading_profile`, `yaw_rate_between` |
| `navigation/missions/cli.py` | Shared argparse surface + object construction | `add_common_arguments`, `build`, `check_mode_flags`, `DetectionOnlyMission` |

### Runnable missions

| File | Phase | Entry |
|---|---|---|
| `navigation/missions/phase1a_single_gate.py` | 1A — one gate | `python -m navigation.missions.phase1a_single_gate` |
| `navigation/missions/phase1b_course.py` | 1B — N gates, full stop between | `… phase1b_course --gates 3` |
| `navigation/missions/phase2_course.py` | 2 — N gates, planned transitions | `… phase2_course --gates 3` |
| `navigation/missions/udp_injector.py` | test tool — synthetic vision, 17 cases | `… udp_injector --list` |

### Vision (Raspberry Pi only — imports `gi`/GStreamer and `hailo_platform`)

| File | Responsibility |
|---|---|
| `vision/gstreamer_class.py` | `CameraStream` — v4l2 MJPEG capture, tee to JPEG sink + BGR sink |
| `vision/opencv_processing.py` | `HailoYOLO` (Hailo-10H inference + CPU DFL decode) and `OpenCVProcessing` (corner refine → fisheye undistort → solvePnP → smooth → UDP publish) |
| `vision/flask_streaming.py` | `FlaskCameraServer` — MJPEG on :5000 |
| `vision/camera_app.py` | `CameraApp`, `create_camera_app` — wires the two together |
| `scripts/run_vision.py` | Entry point. `CAMERA_CONFIGS` lives here |
| `debug/telemetry_reader.py` | Terminal HUD for UDP 5050. **Binds the port — conflicts with a running mission** |
| `debug/debug_hailo.py` | Hailo diagnostics |

### Dead code, kept deliberately

`navigation.py`, `navigation_Vmanual.py`, `navigation_closed_loop.py` at the repo **root**
are prototypes shadowed by the `navigation/` package. `navigation_Vmanual.py` is
byte-identical to the original `navigation.py`; `navigation_closed_loop.py` is truncated
mid-function and was only kept because it was the sole surviving record of the camera
offset sign conventions. **Nothing imports them.**

## 1.3 Call graph — the paths that matter

**Phase 2, top to bottom:**

```
phase2_course.main
 └─ cli.build ──────────────────► NavigationController | OfflineNavigationController
 └─ phase2_course.run
     ├─ pad.run_pad_sequence
     │   ├─ nav.start(require_offboard=False)
     │   │   ├─ mavutil.mavlink_connection(udpin:…, source_system=254)
     │   │   ├─ master.wait_heartbeat()
     │   │   ├─ install PymavlinkLink | DryRunLink
     │   │   ├─ thread: nav._mavlink_reader      ← inbound telemetry
     │   │   ├─ thread: nav._heartbeat_loop      → HEARTBEAT 1 Hz
     │   │   └─ nav.start_streaming
     │   │       └─ thread: nav._setpoint_streamer → SET_POSITION_TARGET_LOCAL_NED 20 Hz
     │   ├─ nav.request_message_intervals        → 5× MAV_CMD_SET_MESSAGE_INTERVAL
     │   ├─ nav.measure_telemetry_rate
     │   ├─ pad.wait_for_detections              ← UDP 5050 (via mission)
     │   ├─ pad.print_banner / pad.confirm
     │   ├─ nav.wait_for_armed
     │   ├─ nav.request_offboard                 → MAV_CMD_DO_SET_MODE
     │   └─ pad.takeoff                          → BRAKE_HOLD_ALT setpoint
     ├─ phase2_course.refresh_tracks
     │   ├─ mission.get_latest_detections_snapshot()
     │   ├─ nav.state_at(timestamp)              ← deskew ring buffer
     │   ├─ frames.localize_gates → GateFix ×N
     │   └─ association.GateTracker.update
     └─ phase2_course._attempt_gate
         └─ gate_leg.approach_and_cross_one_gate(exit_leg=…)
             ├─ mission.observe_gate             ← UDP + BRAKE_HOLD_ALT hold
             ├─ frames.localize_gate / evaluate_commit
             ├─ nav.move_to_target               → VELOCITY_YAW
             ├─ nav.slew_yaw                     → BRAKE_HOLD_ALT ramp
             ├─ gate_leg._fly_through_gate       → VELOCITY_YAW open-loop
             └─ exit_leg  (Phase 2 only)
                 ├─ course.plan_leg → LegPlan{turn | degrade_reason}
                 └─ vector_leg.fly_turn          → VELOCITY_YAW along arc
```

**Phase 1A** is the same minus `refresh_tracks` and `exit_leg`. **Phase 1B** is 1A in a loop
with a settle between gates. All three call the **identical** `approach_and_cross_one_gate`.

## 1.4 Threads (there are five, and they never stop mid-mission)

| Thread | Started by | Rate | What it does |
|---|---|---|---|
| main | `python -m …` | — | Runs the mission sequence |
| `nav-mavlink-reader` | `nav.start()` | as fast as packets arrive | `recv_match` → filter by sysid → fold into `VehicleState`, ring buffer, setpoint echo |
| `nav-setpoint-streamer` | `nav.start_streaming()` | **20 Hz, absolute-time scheduled** | Publishes the latched setpoint; runs the deadman |
| `nav-heartbeat` | `nav.start()` | 1 Hz | Our own HEARTBEAT so PX4 routes `COMMAND_ACK` back |
| `gate-mission-udp-listener` | `mission.start()` | as fast as datagrams arrive | Parses UDP JSON, runs frozen-feed detection |
| `offline-sim` | offline only | 50 Hz | Kinematic model |

**The streamer is the safety-critical one.** It keeps publishing through *every* main-thread
state including an exception unwind and a Ctrl-C, which is what stops the aircraft coasting
on a stale velocity.

---

# PART 2 — Full runtime walkthrough

Setting: **Phase 2, three gates, real aircraft, real vision.** Every step gives you
(1) code, (2) wire, (3) aircraft. Console snippets are the **real** format strings.

---

## Step 1 — Startup and the link

**Code.** `phase2_course.main` → `cli.build` constructs a `NavigationController` with
`mavlink_conn = "udpin:127.0.0.1:14551"`. `mission.start()` spawns the UDP listener
immediately — **before** anything MAVLink — so detections accumulate while the link comes
up. Then `run_pad_sequence` calls `nav.start(require_offboard=False)`.

`nav.start` calls `mavutil.mavlink_connection(conn, source_system=254)` then
**`master.wait_heartbeat(timeout=30)`**. That wait is not politeness. With a `udpin:`
socket, pymavlink **cannot transmit a single byte** until it has received one: `mavudp.write()`
iterates a client set that is only populated inside `recv()`, and send failures are
swallowed. Every byte written before the first inbound datagram is discarded without error.

**Wire.** Nothing outbound yet. Inbound: PX4's 1 Hz `HEARTBEAT` arrives via your router.
pymavlink locks `master.target_system` onto the first *vehicle* heartbeat —
`probably_vehicle_heartbeat()` excludes `MAV_TYPE_GCS`, gimbals, ADSB, onboard controllers
and `MAV_AUTOPILOT_INVALID`, so **QGroundControl on the same link cannot hijack it**, and
neither can our own heartbeat.

**Aircraft.** Sitting on the pad, disarmed, doing nothing.

Once the heartbeat lands, the Link is chosen — **once, here, and nowhere else**:

```python
wire = PymavlinkLink(self.master)
self.link = DryRunLink(wire) if self.dry_run else wire
```

Then three threads start, and `wait_for_vehicle_state()` blocks up to 30 s for
`LOCAL_POSITION_NED` **and** `ATTITUDE`.

```
[*] Listening for gate telemetry on UDP 127.0.0.1:5050

[STEP 1] Connecting and waiting for telemetry
[*] Connecting to MAVLink at udpin:127.0.0.1:14551...
[*] MAVLink heartbeat received (system 1, component 1)
[*] Waiting up to 30s for LOCAL_POSITION_NED and ATTITUDE...
[*] Vehicle state ready
[*] Setpoint stream started at 20 Hz (zero velocity)
```

If telemetry never arrives: `RuntimeError` → `PadResult(False, "could not start controller: …")`
→ `_abort` → `safe_shutdown` → `land()` (which sees disarmed-and-on-ground and returns
"no landing required") → **exit code 1**. Nothing was ever transmitted.

### Step 1b — telemetry rates

**Code.** `nav.request_message_intervals()` sends five `MAV_CMD_SET_MESSAGE_INTERVAL`
commands, 50 ms apart. Then `measure_telemetry_rate(3.0)` **counts what actually arrives**,
because an ACK only means the interval was *stored*.

**Wire — the exact five requests:**

| Message | ID | Requested | Why |
|---|---|---|---|
| `LOCAL_POSITION_NED` | 32 | 30 Hz | The position loop runs at 10 Hz; 30 Hz gives margin |
| `ATTITUDE` | 30 | 50 Hz | Feeds the 3-2-1 rotation |
| `EXTENDED_SYS_STATE` | 245 | 5 Hz | The only reliable airborne/landed signal |
| `POSITION_TARGET_LOCAL_NED` | 85 | 5 Hz | **PX4's echo of the setpoint it thinks it is tracking** |
| `ESTIMATOR_STATUS` | 230 | 2 Hz | Flow health, so flow loss isn't misread as link loss |

**Aircraft.** Still still. Refusal thresholds are 15 Hz position / 20 Hz attitude:

```
[STEP 2] Requesting and measuring telemetry rates
[*] Measured over 3s: LOCAL_POSITION_NED 29.7 Hz, ATTITUDE 48.3 Hz
```
Too slow → `PadResult(False, "telemetry too slow (4.0/12.0 Hz, need 15/20 Hz)")` and the
mission ends **before** the operator is ever asked to arm.

### Step 1c — the vision gate

**Code.** `pad.wait_for_detections(…, min_observe_s=1.0)`. It distinguishes **four** states,
and this is the one place all three staleness signals are checked before flight:

1. **No packets at all** — vision dead or wrong port.
2. **Packets, but only the all-999 sentinel or stale** — normal "no gate in view".
3. **Frozen** — packets fresh, pose never changes. `mission.feed_frozen` after 15
   bit-identical payloads.
4. **Alive and changing** — must stay fresh for a **full 1.0 s**, not one packet.

```
[STEP 3] Checking the vision feed
[*] Vision feed alive and CHANGING over 1.0s: fwd=+3.38m right=-0.00m down=-0.00m yaw=+0.0deg dist=3.38m (age 23ms)
```
or, on a stalled camera — **this is a refusal to fly**:
```
[!] VISION FEED FROZEN: 15 consecutive identical payloads. The publisher is alive but its pose is not changing -- treat every detection as invalid.
[!] Vision feed is FROZEN: 15 identical payloads in a row. The publisher is alive but its pose is not changing -- a stalled camera or a wedged pipeline. Refusing.
[RESULT] MISSION ABORTED: pad sequence failed: no fresh gate detections before takeoff
```

### Step 1d — the banner and the confirm gate

**Code.** The pad reference `d` is latched **now** (`d_takeoff = state.d`) and the whole
altitude envelope is built relative to it — never assumed zero. Then the banner prints
every tunable, then `confirm()` blocks on `input()`.

**This is the "hold-and-confirm gate" you asked about.** It shows you: every commit
threshold, the approach step and arrival tolerance, both speeds, the altitude envelope, the
attempt budget arithmetic, the recovery ladder, all Phase 2 planning limits, the pad
reference `d`, and whether this is a dry run. Then five checkboxes. It waits **as long as
you take** — deliberately not a countdown, because a countdown while armed races PX4's
`COM_DISARM_PRFLT`.

```
  attempt budget.................... 77 s needed for 14 attempts (5.0 s per move)
  ...
  fallback.......................... Phase 1B stop-and-acquire, logged every time
  pad reference d................... +0.00 m (NED)
  dry run........................... no
--------------------------------------------------------------------
  ASSUMED PRECONDITIONS -- confirm each before arming:
    [ ] Vision process running and publishing to UDP 127.0.0.1:5050
    [ ] Drone DISARMED, props fitted, on level ground, clear of the gate
    [ ] PX4 parameter checklist confirmed (COM_LOW_BAT_ACT, COM_OBL_RC_ACT, COM_RCL_EXCEPT, EKF2_OF_CTRL, EKF2_HGT_REF, MPC_XY_VEL_MAX)
    [ ] Safety pilot holding the RC with the mode switch and kill switch ready
    [ ] Test area clear of people
====================================================================

[STEP 4] Operator confirmation (vehicle is still DISARMED)
Type GO to continue (anything else aborts):
```

**Watch for `!! INCONSISTENT` rows.** If the attempt budget cannot be delivered by the
deadline, the banner says so in a row of its own. That is the check that would have caught
the old 0.10 m / 15 s / 75 s mismatch before flight.

Anything other than `GO` → `PadResult(False, "operator aborted at the confirmation prompt")`.
**Nothing has been armed and nothing has moved.**

---

## Step 2 — Arming, and the takeoff sequence

> Your prompt says "given the drone starts ARMED and ON THE GROUND". **The code does not
> support that as a starting state, deliberately.** `wait_for_armed` returns immediately if
> the vehicle is already armed, so the sequence *works*, but the design intends arming to
> be your **last** action, after the banner. If the vehicle is armed before you type `GO`,
> PX4's `COM_DISARM_PRFLT` (10 s default) will disarm it underneath you while you read the
> banner, and STEP 5 will then sit waiting for a re-arm. **Arm after the confirm prompt.**

**Code.** `assert_not_climbing(nav)` runs **twice** — before the arm wait and again before
the OFFBOARD request. It reads the currently-streamed setpoint and raises if `.climbs()`.

```python
def climbs(self):
    if self.kind is SetpointKind.VELOCITY_YAW:
        return self.vd < 0.0
    return False
```

**Why.** PX4 latches `want_takeoff` the instant it is armed and sees a fresh setpoint with
negative `vz`. If the stream carried the climb setpoint while you read the banner, **arming
and lift-off would be the same event.** The stream therefore carries *finite zeros* — never
NaN, because PX4 decides a message counts as proof of life with `velocity = !isAllNan(...)`.

**Wire.** During the whole arm wait, 20 Hz of:
```
SET_POSITION_TARGET_LOCAL_NED  type_mask=2503  (VELOCITY_YAW)
  vx=0 vy=0 vz=0  yaw=<current>
```
**Aircraft.** Disarmed → PX4 ignores offboard setpoints entirely. Nothing moves.

You will see this **once**, and it is expected — not a warning:
```
[*] Setpoint 'prime' is idling while the vehicle is not yet armed in OFFBOARD -- expected before takeoff, holding zeros.

[STEP 5] Waiting for ARM -- this is your commit action
[*] Waiting for operator to ARM (RC or QGC)...
[*] Vehicle ARMED
```

**You arm** (RC switch or QGC). PX4's `HEARTBEAT.base_mode` gets bit 128; the reader thread
folds it into `VehicleState.armed`.

**Aircraft.** Motors spin up to idle. Still on the ground: PX4 is in whatever mode the RC
selected, and is not following our setpoints yet.

Never armed within 300 s → `PadResult(False, "vehicle was never armed")` → abort → land →
exit 1.

---

## Step 3 — Entering OFFBOARD

**Code.** `request_offboard(timeout_s=10, attempts=6)`. It **raises** if the stream is not
already running. It deliberately does **not** call `master.set_mode("OFFBOARD")`: with no
PX4 heartbeat interpreted yet, pymavlink falls through to the ArduPilot mode table, prints
"Unknown mode", returns `None`, and **sends zero bytes** while the caller reports success.

**Wire.**
```
COMMAND_LONG  MAV_CMD_DO_SET_MODE (176)
  param1 = 1  (MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
  param2 = 6  (PX4 custom main mode = OFFBOARD)
```

**Confirmation is `HEARTBEAT.custom_mode`, and nothing else:**
```python
((custom_mode >> 16) & 0xFF) == 6      # i.e. custom_mode == 393216
```
Not the `COMMAND_ACK` (which only says the command was received), and **never**
`master.flightmode` — its `interpret_px4_mode` requires `base_mode & 28 == 28` while PX4
sends 145 when armed in OFFBOARD, so it reports "UNKNOWN" in exactly the state we need.

```
[STEP 6] Entering OFFBOARD
[*] Requesting OFFBOARD (attempt 1/6)...
[*] OFFBOARD confirmed via HEARTBEAT.custom_mode
```

**Aircraft.** PX4 now follows the streamed setpoint — which is still zero velocity, so it
holds. Motors at hover-ish thrust, no translation.

**If PX4 rejects it** (6 attempts / 10 s):
```
[!] OFFBOARD NOT confirmed after 6 attempts (custom_mode=131072, armed=True)
```
→ `PadResult(False, "OFFBOARD was not confirmed via HEARTBEAT")` → `_abort` →
`safe_shutdown` → 3 s hold → `land()`. **The aircraft is armed with props spinning at this
point, so `safe_shutdown` prints the take-manual-control banner and does command LAND.**

**If PX4 drops OFFBOARD later** — mid-approach — `gate_leg`'s per-iteration check catches
it and names the real cause:
```
[*] Gate 1: ABORTED -- PX4 is no longer in OFFBOARD (ESTIMATOR DEGRADED: velocity_horiz, velocity innovation ratio 1.42 -- suspect optical flow, not the offboard link)
```
This matters because PX4 sets `offboard_control_signal_lost` when the **estimator** fails
its velocity innovation check past `COM_VEL_FS_EVH`. The obvious reading — "the companion
link dropped" — is wrong roughly as often as it is right on a flow-only airframe.

> **Known limitation (F-4):** in `--dry-run` this step **always fails**, because nothing was
> transmitted so OFFBOARD can never be observed. A bench run therefore stops here.

---

## Step 4 — Takeoff

**Code.** `pad.takeoff` latches one setpoint and then watches telemetry:

```python
Setpoint(kind=BRAKE_HOLD_ALT, vn=0.0, ve=0.0, d=d_takeoff - 1.5, yaw_rad=<current>, label="takeoff")
```

**Wire.** 20 Hz of:
```
SET_POSITION_TARGET_LOCAL_NED  type_mask=2531
  active: vx=0, vy=0, z=<target_d>, yaw=<current>
  ignored: x, y, vz, all accel, yaw_rate
```

**Why 2531 and not a full position mask (2552).** A full position setpoint carries an x/y
target. If that is anything but the position captured at the mode-switch instant, PX4 flies
toward it at up to **`MPC_XY_VEL_MAX` — 12 m/s by default — while 30 cm off the ground.**
Zero horizontal *velocity* cannot run away, while `z` as a *position* still lets PX4 close
the altitude loop.

**Aircraft.** PX4 spools up (~1 s) then ramps thrust (~3 s) and climbs to 1.5 m AGL,
holding horizontal position by braking to zero ground speed.

**Verification is against `LOCAL_POSITION_NED`, never a fixed sleep:**
```python
if abs(state.d - target_d) <= cfg.takeoff_tolerance_m:   # 0.15 m
```
```
[STEP 7] Taking off to 1.50 m AGL
[*] Airborne (EXTENDED_SYS_STATE reports IN_AIR)
[*] Reached 1.37 m AGL
```

Note **1.37 m for a 1.50 m target** — that is `takeoff_tolerance_m = 0.15` doing its job.
The mission then flies ~0.13 m low for its whole duration, against a 0.80 m floor. In spec,
but know it (Finding **F-8**).

**If it does not climb in 20 s:**
```
[!] Takeoff timed out at 0.42 m AGL (wanted 1.50 m)
```
→ `PadResult(False, "did not reach takeoff altitude", envelope)` → `_abort` → 3 s hold →
LAND. **The aircraft is airborne and armed here**, so this is a real landing.

---

## Step 5 — Searching for the first gate

Phase 2 calls `refresh_tracks` first (harmless if nothing is visible), then
`approach_and_cross_one_gate`, which begins with `mission.observe_gate(nav, duration=0.5)`.

**Code.** `observe_gate` latches **one** setpoint for the whole window —
`BRAKE_HOLD_ALT` at the current pose — and then samples at 25 Hz for 0.5 s. It transmits
**nothing** from inside the loop; the streamer is already publishing at 20 Hz.

> This is the fixed MED bug. It used to latch `BRAKE_HOLD_ALT` *and* also transmit
> `VELOCITY_YAW` zeros at 25 Hz. PX4 received both, interleaved, and `vz = 0` holds a
> **rate**, not an altitude — so the vehicle drifted vertically through every observation
> window, which is exactly when it is measuring how high a gate is.

**Aircraft.** Holds altitude on a position target, brakes horizontally, holds heading.

**If nothing is seen**, the recovery ladder runs (full detail in Part 3). Rung 2 is the yaw
sweep, and this is where **`slew_yaw` matters**:

```python
for offset in (+45°, -45°, 0°):
    nav.slew_yaw(heading, max_rate_deg_s=cfg.max_yaw_rate_deg_s)   # 45 deg/s
```

**What `slew_yaw` actually commands.** Not one big yaw. It ramps the *commanded* heading at
`max_rate * dt` every 100 ms and publishes a **`BRAKE_HOLD_ALT`** setpoint carrying that
intermediate heading. So the wire sees a staircase of position-holding setpoints whose yaw
field advances 4.5° per tick.

```
[!] Gate 0 not seen (miss 1)
[!] Gate 0 not seen (miss 2)
[!] Gate 0 not seen (miss 3)
[!] Gate 0 not seen (miss 4)
[*] Recovery rung 2: yaw sweep 1/2
[*] Yaw slew to +45.0 deg for 'scan to +45deg': 45.0 deg at 45 deg/s (~1.0s, budget 4.5s)
[*] Yaw slew 'scan to +45deg' complete
```

**Why bounded.** `MPC_YAWRAUTO_MAX` defaults to 45 deg/s, but it is a *firmware default* an
airframe startup script can raise. On this airframe a fast yaw injects rotation-induced
optical flow the estimator must cancel from gyro alone — a direct attack on the position
estimate, during a recovery, which is precisely when the estimate is what you are relying on.

**`turn_around_180()`** now does the same thing: `slew_yaw(yaw + π)` — a ~4 s ramp at
45 deg/s, holding position and altitude throughout, instead of one instantaneous 180° step.
It is a public primitive but **no current mission calls it** (Finding **F-9**).

**How long can this go on?** Strictly bounded, and every bound is in the banner:
- 4 observation misses (`max_observe_retries = 3`, so the 4th escalates)
- then 2 yaw sweeps × 3 headings each
- then 2 back-offs of 1.0 m each (only if a gate was ever localized)
- and over the top of all of it, **`approach_timeout_s = 90 s`** per gate.

Worst case ≈ 90 s, then `GateResult.TIMED_OUT`. If it never saw anything: `NOT_FOUND`. If it
saw something and then lost it: `LOST`.

---

## Step 6 — A detection arrives

### 6a — the wire

The vision process sends one UDP datagram per processed frame to `127.0.0.1:5050`:

```json
{"fps": 27.4, "gates": [[2.503, 2.500, -0.087, 0.041, 1.20, -3.40, -8.15],
                        [6.104, 6.090, 1.480, 0.010, 0.80, -1.10, 24.60],
                        [999.0, 999.0, 999.0, 999.0, 999.0, 999.0, 999.0]]}
```

**Always exactly three rows, sorted nearest-first, absent gates padded all-999.**
Row = `[dist, forward, right, down, roll°, pitch°, yaw°]` in **body FRD** metres/degrees.

### 6b — the listener and the three staleness signals

`GateMission._ingest_payload` runs on the listener thread:

1. **`_note_payload(gates)`** — fingerprints only the *valid* rows (all-999 → `None`, which
   resets the counter, because an empty sky is not a frozen feed). 15 identical
   fingerprints → `feed_frozen`.
2. Every row → `GateDetection` with `row_index` and `source_fps`, stamped `time.time()`.
3. `_latest_detections` = valid rows only (Phase 2 uses this).
   `_latest_detection` = nearest valid row, **or the sentinel row if none are valid** —
   because `observe_gate` relies on the sentinel *overwriting* the last good pose. That is
   staleness signal 2, and filtering it here would silently delete it.

`observe_gate` then applies all three:
- signal 1 (nothing arrives) → the age check `time.time() - det.timestamp <= 0.4 s` fails
- signal 2 (all-999) → `is_valid_detection` fails (a *range* check, `0.05 ≤ dist ≤ 100`, not
  float equality against 999.0 — that would accept a "gate at 998.7 m")
- signal 3 (frozen) → `feed_frozen` short-circuits the window and returns `None`

Angles are averaged with a **circular mean** — `mean(+179, −179) = 180`, not 0, and gate
yaws sit near the wrap routinely.

```
[*] Observing gate for 0.5s...
[*] Lock acquired from 11 samples: fwd=+2.50m right=-0.00m down=+0.00m yaw=+0.0deg
```

### 6c — localization in 3D with full attitude

```python
state = nav.state_at(detection.timestamp)     # deskew against the pose at CAPTURE time
fix   = frames.localize_gate(detection, state, envelope, cam_offsets…, max_cone_deg=60)
```

`state_at` searches a 30-sample ring buffer (1 s at 30 Hz `LOCAL_POSITION_NED`) for the pose
closest to the detection's stamp, falling back to the current snapshot if empty.

Inside `localize_gate`:

1. **Camera offsets** added to the raw detection *before* rotation:
   `right += −0.25`, `down += −0.10`, `yaw° += −10.0`.
2. **3-2-1 rotation into NED**, `R = Rz(yaw)·Ry(pitch)·Rx(roll)`, using the vehicle's
   **full** attitude. A gate 3 m ahead at 10° of body pitch is placed **0.52 m** off
   vertically if pitch is ignored, straight into the commanded altitude.
3. **Normal disambiguation** — `resolve_gate_normal` dots the candidate normal with the
   drone→gate vector and flips it if it points back at the drone. After this the standoff
   point is *always* on the drone's side.
4. **Altitude clamp** into `[d_takeoff − 2.5, d_takeoff − 0.8]`, flagged if it fires.
5. **`range_m = hypot(forward, right, down)` — NOT `det.dist`.** Confirmed in the vision
   source: `dist = np.linalg.norm(tvec)` uses the **raw** tvec while `d_x/d_y/d_z` come from
   the **smoothed** tvec, so they disagree most while moving — exactly when the commit
   decision is made. `det.dist` is used only as the no-detection sentinel.
6. **Cone angle** — >60° ⇒ `low_confidence`.

```
  [gate 0 obs 1] drone N=+0.00 E=+0.00 D=-1.44 (+1.44m AGL) yaw=   +0.0 roll= +0.0 pitch= +0.0
    gate NED  N=+2.50 E=-0.00 D=-1.44 (+1.44m AGL)  heading=   +0.0deg
    normal    (+1.000, +0.000)  as reported
    measured  range=2.50m  body lateral=-0.00m  body vertical=+0.00m  axis offset=0.00m
```
Low confidence prints:
```
    LOW CONFIDENCE: edge-on (78.4deg > 60.0deg)
    NOTE: gate altitude was outside the envelope and has been clamped
```
A `low_confidence` fix **can never satisfy the commit gate** (`evaluate_commit` returns
`ok = not failures and not fix.low_confidence`). It is still used for approach stepping —
knowing roughly where a gate is is useful; deciding to *fly through* it is not.

### 6d — association (Phase 2 only)

`refresh_tracks` → `localize_gates` (every row, one shared pose) → `GateTracker.update`.

Nearest-neighbour in local NED, greedy by ascending distance, with **three refusals**:

1. **Gating radius grows with track age** — `0.60 m + 0.25 m/s`, capped at 2.0 m. A track
   seen 40 ms ago should match within centimetres; one last seen 4 s ago has had time for
   the flow-based estimate to drift.
2. **Ambiguity veto** — if best and second-best are within `0.50 m` of each other, **no
   match is made and no new track is created**. Guessing between two closely-spaced gates
   does not give a 50/50 chance of being right; it gives a confident, silent identity swap.
3. **Confidence, not presence** — `min(1, hits/3) × max(0, 1 − age/5 s)`.

```
[*] Gate tracks: 3 tracks | 0 matched | 3 new [0, 1, 2]
[*] Gate tracks: 2 tracks | 1 matched | 1 AMBIGUOUS (refused)
    refused 1 observation(s) as ambiguous: two candidate gates within 0.50 m of each other. Guessing here swaps gate identity silently.
```

Low-confidence fixes update **position** (well observed) but not **heading** — tracked
separately as `heading_confidence`.

---

## Step 7 — The approach

The receding-horizon loop in `gate_leg.approach_and_cross_one_gate`. Per iteration:

**Guards first, in order** — controller running, gate deadline (90 s), telemetry fresh
(0.5 s), **PX4 still in OFFBOARD**, attempts under 14.

Then observe → localize → `evaluate_commit`, which requires **all four** of:

| Term | Threshold | Measured how |
|---|---|---|
| range | ≤ 1.00 m | `hypot(forward, right, down)` |
| lateral | \|·\| ≤ 0.20 m | **body-frame `right`** — directly observed |
| vertical | \|·\| ≤ 0.25 m | **body-frame `down`** — directly observed |
| cone | ≤ 60° | angle between gate normal and drone→gate |

**Alignment is measured in the body frame on purpose.** An axis offset derived from the
plane's PnP yaw is the classic weakly-observable degree of freedom — a few degrees of yaw
error becomes tens of centimetres of apparent offset. `lateral_axis_m` is computed and
logged but **deliberately not used** by the commit gate.

**Range alone is not sufficient**: slant range stays large while the drone is crabbed to one
side, which is precisely the geometry that clips a gate leg on an open-loop pass.

```
    commit=no  | NO range= 2.50m | OK lat=-0.00m | OK vert=+0.00m | OK cone=  0.0deg
    withheld because: range 2.50m > 1.00m
```

**Not committed and out of range → APPROACH.**
```python
standoff_m = max(commit_distance_m, fix.range_m - step_size_m)   # shrink by 0.35 m
desired    = standoff_target(fix, standoff_m, envelope)           # on the gate's centre axis
stepped    = limit_approach_step(state, desired, 0.35, 0.25)      # horizontal and vertical caps, SEPARATELY
nav.move_to_target(stepped, label, max_speed_m_s=0.35, tolerance_m=0.15)
```

The horizontal and vertical budgets are spent independently, so gate-height noise cannot
eat the forward progress. With a single 3-D cap, 0.25 m steps delivered 0.14 m of range
closure in flight — 56% efficient.

Before any of this, the fix has been through the rolling **pose filter**: a 5-observation
median over the gate's N, E, D *and* heading, re-derived into a `GateFix` by
`frames.refix_to_pose` so the commit decision and the flown target read the same numbers.
The gate does not move; its estimate spread 0.21 m / 0.58 m / 0.46 m / 3.9° across five
observations, and unfiltered that is what the aircraft chases.

**Wire.** `move_to_target` runs a 10 Hz proportional loop, `v = 1.2 × error` clamped to
0.35 m/s, and on each tick both latches the setpoint **and** transmits it directly — so the
wire carries velocity setpoints at roughly **30 Hz** during a move (20 Hz streamer + 10 Hz
loop). Same values, so PX4 simply gets a fresher one.

**Aircraft.** Creeps forward ~0.25 m and settles.

```
[*] Moving to gate 0 standoff 2.25m: N=0.25, E=-0.00, D=-1.44, Yaw=0.0 deg, MaxSpeed=0.35m/s, Dist=0.25m, Tol=0.15m, Budget=5.0s
[*] Reached gate 0 standoff 2.25m
```

**Not committed but IN range → ALIGN.** `standoff_m = commit_distance_m` — move onto the
centre axis at 1.8 m and re-check. Bounded at `max_align_attempts = 6`:
```
[*] Gate 0: NO_COMMIT -- in range but never aligned after 6 corrections (lateral -0.42m > +/-0.20m)
```

**Committed** → must hold **3 consecutive frames**. A failing frame resets the count to
zero, and so does a *missed* observation — "consecutive" has to mean consecutive, or a run
can span a dropout and the recovery manoeuvres that follow it. A one-frame PnP glitch at
1.8 m is exactly the input this has to survive.
```
    commit=YES | OK range= 0.94m | OK lat=+0.03m | OK vert=-0.06m | OK cone=  4.1deg
    commit gate held 1/3 consecutive frames
    ...
    commit gate held 3/3 consecutive frames
```

### The exact moment vision stops being trusted

```
--- GATE 0: COMMITTING. Vision is no longer trusted; flying the frozen pose. ---
```

From here `_fly_through_gate` flies the **frozen `GateFix`** open-loop. It never reads a new
detection. **This is not a shortcut — it is forced by physics and by the vision code:** as
you close on a gate it fills the frame, and `opencv_processing.py` explicitly skips PnP for
any box touching the frame edge (`is_partial`). The gate stops producing a pose exactly when
you enter it.

**Completion is a signed projection onto the gate normal, not arrival at a point:**
```python
along = (drone_n - fix.n)*fix.normal_n + (drone_e - fix.e)*fix.normal_e
return along >= 0.80          # exit_clearance_m
```
Asking whether the drone is within 0.15 m of a point *beyond* the gate is a stop-and-settle
test applied to a fly-through manoeuvre — it cannot be satisfied while still moving.

**Open-loop does not mean straight-line.** The velocity is decomposed about the gate's centre
**axis**, with along-track and cross-track clamped *separately*:

```python
along   = (state.n - fix.n)*normal_n + (state.e - fix.e)*normal_e
cross   = (state - fix) - along*normal          # perpendicular offset from the axis
v_along = clamp(KP_POS * (pass_distance_m - along), ±cross_speed_m_s)
v_cross = clamp(-KP_POS * cross,                    ±cross_speed_m_s * 0.6)
```

One shared clamp is what made the aircraft veer. Committing at 1.8 m with a 1.5 m pass
distance gives a 3.3 m along-track error, so `KP_POS × 3.3 = 3.96` m/s is scaled to 0.45 —
a factor of 8.8 — and the cross-track term, being part of the same vector, is scaled by the
same 8.8. A 0.10 m offset commanded 0.014 m/s of correction. The drone flew straight from
wherever it committed and arrived off-centre by however much it was off-centre at commit.
Reserving cross-track its own budget gives the offset a 0.83 s time constant against ~4 s
of flight to the plane: five time constants, so it is gone before it matters.

```
[*] Crossing gate 0: target N=+2.77 E=+0.00 D=-1.44, 2.12m at 0.45m/s, budget 10.0s, clearance 0.80m
    entering 0.18m off the gate axis; centring at up to 0.27m/s while flying through
[*] Gate 0 AT THE PLANE: 0.02m off centre laterally, +0.03m vertically
[*] Gate 0 CROSSED (past the plane by 0.80m)
```
`AT THE PLANE` is the line to read after a flight: it is the only direct measurement of
whether the aircraft actually went through the middle.
Budget blown instead:
```
[!] Crossing timed out after 10.0s; only +0.31m along the gate normal
[*] Gate 0: TIMED_OUT -- pass-through did not clear the gate plane
```

---

## Step 8 — Course-vector behaviour (Phase 2), and the fallback

Immediately after the plane is cleared, `gate_leg` calls the `exit_leg` callback with the
frozen fix. **Phase 1A/1B pass `None` here and get the old "hold" behaviour, unchanged.**

Phase 2's `exit_leg`:

1. `refresh_tracks` — one more look at every gate in the packet.
2. `current = pass_vector(fix)` — the gate as a **point plus a direction**.
3. `choose_next_gate` — confident enough (`≥ 0.50`), and **beyond** the current gate's plane
   by more than the exit clearance. Nearest of what survives. **Returns `None` rather than
   a guess.**
4. `plan_leg` → `LegPlan`.

### When the next gate IS known: the turn

`plan_turn` treats the two gates as two directed lines and fits the circular arc tangent to
both. The first tangent point **is** the transition point:

```
                 P (the two lines intersect)
                /|
    gate N ----T1|          d = r · |tan(θ/2)| either side of P
              ( \|          T1 = P − u·d     ← transition point
               \  T2        T2 = P + v·d     ← rejoin point
                \  \        arc length = r·|θ|
                 \  v gate N+1
```

**Radius is bounded from both ends:**
- *Below*, by the airframe: `r ≥ V/ω_max` (yaw) and `r ≥ V²/a_max` (tilt). At 0.45 m/s with
  45 deg/s and 0.15 g those are **0.573 m** and 0.138 m — **yaw rate binds**, which is the
  right way round when spinning the camera costs more than leaning it.
- *Above*, by geometry: both tangent points must clear gate N's 0.80 m exit clearance and
  gate N+1's 1.00 m straight lead-in.

If the upper bound falls below the lower one, the planner **slows down** (×0.8 steps to a
0.25 m/s floor) before refusing. It takes the **widest** arc the geometry allows, because a
wide turn is a slow yaw and a slow yaw is what the flow estimator wants.

The **heading profile** is linear in arc length — which for constant radius and speed is
also linear in time, so it doubles as the yaw schedule. `vector_leg.fly_turn` samples 8 arc
points and flies each with the **planned heading commanded absolutely**, not pointed at the
target: through a transition the camera must look where the *next gate* is, and those differ
by up to the full turn angle.

```
[*] Transition out of gate 0: turn +90.0deg r=4.00m at 0.45m/s | arc 6.28m (14.0s) | yaw 6.4deg/s | bank 0.3deg
    t=0.00  N=  +1.00 E=  +0.00  hdg=   +0.0deg
    t=0.25  N=  +2.53 E=  +0.30  hdg=  +22.5deg
    t=0.50  N=  +3.83 E=  +1.17  hdg=  +45.0deg
    t=0.75  N=  +4.70 E=  +2.47  hdg=  +67.5deg
    t=1.00  N=  +5.00 E=  +4.00  hdg=  +90.0deg
[*] Flying gate 0 -> 1 transition: turn +90.0deg r=4.00m at 0.45m/s | ... | 8 arc points from (+1.00,+0.00) to (+5.00,+4.00)
[*] gate 0 -> 1 transition complete; on the next gate's centre line
[*] Gate 0: CROSSED -- gate crossed; exit leg flown
```

Altitude is held at whatever it was when the turn started, clamped into the envelope. A
transition is a horizontal manoeuvre — changing height during one moves two things at once
between the only two moments the gate geometry is actually measured.

### When the next gate is NOT known: the Phase 1B fallback

**Five triggers, every one logged with the exact string `PHASE 1B FALLBACK`:**

| Trigger | Console string (verbatim format) |
|---|---|
| No next gate tracked | `no next gate is being tracked; flying the Phase 1B crossing` |
| Track confidence too low | `next gate confidence 0.33 is below 0.50; flying the Phase 1B crossing` |
| Heading not trusted | `next gate heading confidence 0.25 is below 0.50 -- its position is known but its normal is not, so there is no vector to turn onto` |
| Turn infeasible (geometry) | `no room for a turn: -0.90 m available after gate N's 0.80 m exit clearance and +3.73 m before gate N+1's 1.00 m lead-in` |
| Turn infeasible (airframe) | `turn too tight for the airframe: geometry allows r <= 0.31 m but 0.25 m/s needs r >= 0.32 m (yaw rate <= 45 deg/s, lateral accel <= 1.47 m/s^2), even after slowing to the 0.25 m/s floor` |
| Transition failed in flight | `planned transition did not complete in flight` |

**What you will actually see** (this is a real line from the offline Phase 2 run):
```
[*] PHASE 1B FALLBACK -- gate 1: no room for a turn: -0.90 m available after gate N's 0.80 m exit clearance and +3.73 m before gate N+1's 1.00 m lead-in
[*] Gate 1: CROSSED -- gate crossed; exit leg did not complete (holding instead)
```

**The observable difference in the console** between a Phase 2 leg and a fallback leg:
- Phase 2 leg ends `gate crossed; exit leg flown` and there is **no** `settled after gate N`.
- Fallback leg ends `gate crossed; exit leg did not complete (holding instead)` and the next
  thing that happens is a hold plus a 1 s settle.

The end-of-course report lists every degradation:
```
  crossed 3 | transitions flown 1, failed 0
  DEGRADED TO PHASE 1B:
    - gate 1: no room for a turn: -0.90 m available after ...
    - gate 2: no room for a turn: -1.36 m available after ...
```
or, if none:
```
  no degradations: every transition was planned and flown
```

**Whole-course kill switch:** after `max_transition_failures = 2` transitions fail in
flight, planning is disabled for the rest of the course:
```
[!] 2 transitions failed; planning disabled for the rest of the course, flying Phase 1B
```

**`--no-planning`** forces every leg to degrade by setting the confidence thresholds to an
unreachable 2.0. That is your A/B control: same code path, 1B behaviour.

---

## Step 9 — Advancing to the next gate: what carries over, what resets

**Carries over (course scope, `CourseState` + `GateTracker`):**
- `crossed`, `transitions_flown`, `transitions_failed`, `degradations`, `results`
- **the whole gate track set**, including tracks for gates not yet flown
- `consecutive_failures` (reset only by a crossing)
- `planning_enabled` — once false, stays false
- `nav.altitude_envelope` — latched once at the pad, never re-latched
- the setpoint stream, all five threads, and `_landing_confirmed`

**Resets per gate** (all local to `approach_and_cross_one_gate`):
- `attempts`, `align_attempts`, `observe_failures`, `expected_mismatches`, `scans_used`,
  `backoffs_used`, `reacquires`, `confirmed_frames`, `last_fix`
- the 90 s gate deadline
- `entry_state`

**Not reset, and worth knowing:** the vision process's *own* internal smoothing tracker
(`prev_tracked_gates` in `opencv_processing.py`, EMA α = 0.3, 1.5 m jump gate) has no idea a
gate boundary happened. There are **two** association layers in this system and they do not
talk to each other (Finding **F-6**).

---

## Step 10 — Landing: every path that gets here

`land()` is **idempotent** — `_landing_confirmed` short-circuits it, so any exit path may
call it without tracking whether it already did.

| Path | Where | What is different |
|---|---|---|
| **Course complete** | end of `run` | All gates crossed. `land()` from a hover. Exit **0** only if landed **and** all gates crossed |
| **Course ended early** | `_abort` | Prints the SAFE SHUTDOWN banner, holds **3 s** for you to take manual control, *then* lands. Exit 1 |
| **Consecutive gate failures** | `_abort` | Same, reason names the last gate and its outcome |
| **Gate ABORTED** | `_abort` | Telemetry stale or OFFBOARD lost — **the estimator note is in the reason string** |
| **Course timeout (600 s)** | `_abort` | Checked at the top of each gate iteration, so it can only fire *between* gates |
| **Mission deadline (1A, 180 s)** | `_abort` | 1A only, checked after the single gate |
| **Ctrl-C / SIGTERM** | signal handler → `KeyboardInterrupt` → `_abort` | The handler *raises into the main thread* so the `finally` runs. The streamer keeps publishing throughout the unwind and its deadman degrades to a hold |
| **Unhandled exception** | `except Exception` → `_abort` | `unhandled error: ValueError(...)` |
| **Pad failure** | `_abort` | Usually still on the ground → `land()` returns "no landing required" without transmitting |

**What `land()` does on the wire.**
```
COMMAND_LONG  MAV_CMD_NAV_LAND (21)   — re-issued every 5 s
```
**The wait is bounded by the 40 s deadline and nothing else.** Re-issuing exists because PX4
can transiently reject a LAND; it does not terminate the wait. (The old version capped the
outer loop at 3 attempts and gave up at ~15 s of a 40 s budget, printing "NOT confirmed" on
a landing that was going fine.)

**Success is judged on `landed_state` + the disarm flag, never the flight mode** — PX4's
failsafe framework may legitimately redirect AUTO.LAND to Descend.

```
[*] Commanding LAND (issue 1, 40.0s of the 40s budget left)...
[*] Touchdown detected; waiting for auto-disarm...
[*] Landed and disarmed.
```
Failure, after using the **whole** budget:
```
[!] Landing NOT confirmed within the full 40s budget (8 LAND commands issued, armed=True, landed_state=2). Vehicle may still be airborne. (estimator healthy, so this really is an offboard/link problem)
```
Pre-arm abort:
```
[*] Vehicle is disarmed and on the ground; no landing required.
```
Dry run:
```
[DRY-RUN] would send MAV_CMD_NAV_LAND
[*] DRY RUN: LAND was not transmitted, so no landing can be observed.
```

---

# PART 3 — Failure and recovery paths

For each: trigger → ladder → terminal outcome → **validation status**.

## 3.1 Gate lost mid-approach — the three-rung ladder

**Trigger.** `observe_gate` returns `None`.

**Ladder, in order, each bounded:**

| Rung | Condition | Action | Bound |
|---|---|---|---|
| 1 | `observe_failures ≤ 3` | Re-observe from the same pose. **No motion at all.** | `max_observe_retries = 3` |
| 2 | rung 1 spent, `scans_used < 2` | `slew_yaw` to +45°, −45°, back to 0° at 45 deg/s | `max_scan_sweeps = 2` |
| 3 | rungs 1–2 spent, **`last_fix is not None`**, `backoffs_used < 2` | `move_to_target` 1.0 m back along the gate normal | `max_backoffs = 2` |

Rung 3 is guarded on ever having localized a gate — retreating along a normal you never had
is meaningless. A gate is most often lost by getting too close for it to fit in frame, and
backing off widens the field of view again.

**Terminal.** `NOT_FOUND` if `last_fix is None`, else `LOST`, reason
`"recovery ladder exhausted (observe, sweep, back off)"`. In 1B/2 the course loop may retry
the whole gate (`gate_retries = 1`); two consecutive gate failures end the course, and every
exit lands.

**Validation: unit-tested, all branches.** `test_gate_leg.RecoveryLadderTests` — 6 tests
covering each rung, the ordering, both bounds, the `last_fix` guard, and NOT_FOUND vs LOST.
Also exercised end-to-end offline by `--case dropout`.

## 3.2 Detection stream stale or frozen

| Signal | Trigger | Handling | Terminal |
|---|---|---|---|
| 1 — no datagram | `time.time() − det.timestamp > 0.4 s` | counted as `stale_seen` | `observe_gate → None` → rung 1. Prints `Detections are STALE (older than 0.40s). The vision process may have died.` |
| 2 — all-999 | `is_valid_detection` false | sentinel overwrites the last pose | `observe_gate → None` → rung 1. Prints `No valid gate detections seen.` |
| 3 — frozen | 15 identical valid payloads | short-circuits the window | `observe_gate → None`; on the **pad** it is a hard refusal to take off |

**Validation: unit-tested + offline end-to-end.** `FrozenFeedTests` (4), `ObserveGateSetpointTests`
(2), `DetectionWaitTests` (4). Offline: `--case frozen` verified to be refused on the pad.

## 3.3 OFFBOARD rejected or dropped

**Rejected at STEP 6.** 6 attempts over 10 s, confirmation on `HEARTBEAT.custom_mode` only.
Terminal: pad failure → `_abort` → 3 s hold → LAND. **Armed with props spinning.**

**Dropped mid-flight.** Checked at the top of every `gate_leg` iteration:
```
[*] Gate 1: ABORTED -- PX4 is no longer in OFFBOARD (ESTIMATOR DEGRADED: velocity innovation ratio 1.42 -- suspect optical flow, not the offboard link)
```
Terminal: `GateResult.ABORTED` → course `_abort` (never retried — retrying means commanding
motion in a state we already decided not to trust) → land.

> **The detection is only as fast as the loop.** The check runs once per approach
> iteration, i.e. after each ~0.5 s observation and each up-to-5 s move. Worst case you
> notice OFFBOARD is gone **~5 s** after it went. Meanwhile PX4 has already reverted to its
> own failsafe behaviour (`COM_OBL_RC_ACT`), which is what is actually flying the aircraft.

**Validation: unit-tested for the abort branch and the message** (`test_gate_leg`:
`test_losing_offboard_reports_the_estimator_rather_than_blaming_the_link`), and
`request_offboard`'s own retry loop is tested via the pad tests. **The real PX4 rejection
and mid-flight drop behaviour has never been exercised** — no simulator, no flight.

## 3.4 Move timeout

**Trigger.** `time.time() - start > timeout_s`, where the budget is derived from distance
and speed with the **same tolerance** the arrival test uses.

**Handling.** `move_to_target` returns `False`, sends a zero-velocity command, and the
`finally` latches a hold. `gate_leg` treats this as **information, not failure**:
```
[!] Timeout moving to gate 0 standoff 1.55m after 5.0s (0.31m short). Treating as FAILED.
[!] Move 'gate 0 standoff 1.55m' did not complete; re-observing before retrying
```
It re-observes from wherever it actually got to. **Terminal:** none directly — it consumes
an attempt and the 90 s deadline.

**Validation: unit-tested** (`test_a_move_that_does_not_complete_re_observes_rather_than_aborting`)
and the arithmetic is pinned by 6 tests in `test_move_timeout_and_setpoints.py`.

## 3.5 Retry exhaustion per gate

| Level | Bound | Terminal |
|---|---|---|
| Approach attempts | 14 | `NO_COMMIT -- used all 14 approach attempts` |
| Align attempts | 4 | `NO_COMMIT -- in range but never aligned after 4 corrections (…)` |
| Gate deadline | 90 s | `TIMED_OUT -- exceeded 90s budget for this gate` |
| Whole-gate retries | 1 (`--gate-retries`) | next gate, or a consecutive failure |
| Consecutive gate failures | 2 | course `_abort` → land, exit 1 |
| Course timeout | 600 s | course `_abort` → land, exit 1 |

**Validation: unit-tested.** `test_gate_leg` (attempt/align/deadline), `test_phase1b_course`
(14 tests) and `test_phase2_course.MissionLoopTests` (11 tests) for the course-level bounds.

## 3.6 Deadman

**Trigger.** The streamer sees `setpoint.is_transient` (i.e. `VELOCITY_YAW`) **and**
`time.time() - issued_s > 0.5 s`.

**Two branches, and the distinction is the whole point:**

- **Not (armed **and** in OFFBOARD)** — PX4 is not acting on our setpoints at all, so a
  stale one is inert. Re-stamp it, and say so **once**:
  ```
  [*] Setpoint 'prime' is idling while the vehicle is not yet armed in OFFBOARD -- expected before takeoff, holding zeros.
  ```
- **Armed and in OFFBOARD** — this is a genuine stall. Brake to a hold and shout:
  ```
  [!] DEADMAN: velocity setpoint 'gate 0 standoff 1.55m' not refreshed for 0.50s while armed in OFFBOARD. Braking to a hold.
  ```

**Terminal.** No mission-level outcome — it is a *safety net*, not a state. It converts a
stalled main thread into a stationary hold while the stream stays alive so PX4 does not drop
offboard.

**Validation: unit-tested, both branches** (`DeadmanTests`, 4 tests, including that the
in-flight alarm still fires). **The in-flight branch has never fired in a real run** — by
design; nothing offline stalls the main thread for 0.5 s while armed in offboard.

## 3.7 Estimator degradation

**Trigger.** `ESTIMATOR_STATUS` (230) folded into `VehicleState`. `estimator_faults()`
reports a fault when a required flag is clear (`attitude`, `velocity_horiz`, `pos_horiz_rel`,
`pos_vert_agl`) or an innovation ratio ≥ 0.80.

**Handling.** **It never aborts on its own.** It is a *diagnostic suffix* appended by
`estimator_note()` to three messages: telemetry-stale abort, OFFBOARD-lost abort, and
landing-not-confirmed. Three possible suffixes:
```
 (estimator healthy, so this really is an offboard/link problem)
 (ESTIMATOR DEGRADED: velocity_horiz, velocity innovation ratio 1.42 -- suspect optical flow, not the offboard link)
 (no ESTIMATOR_STATUS seen; cannot tell link loss from flow loss)
```

**Terminal.** None of its own. **This is a deliberate scope choice and worth knowing:** a
degrading estimator will not stop the mission; it will only explain the abort that PX4
eventually forces. (Finding **F-7**.)

**Validation: unit-tested** (`EstimatorStatusTests`, 5 tests). **Never seen against real
PX4 telemetry** — the offline model publishes a synthetic healthy estimator.

## 3.8 Telemetry stale in flight

**Trigger.** `nav.telemetry_ok(0.5 s)` false — position *or* attitude older than 0.5 s.
**Terminal.** `ABORTED` immediately, with the estimator note. Course aborts, never retries.
**Validation: unit-tested** (`test_stale_telemetry_aborts_and_names_the_estimator`).

## 3.9 Foreign telemetry on a shared link

**Trigger.** A GCS or second vehicle publishing `LOCAL_POSITION_NED` / `ATTITUDE`.
**Handling.** `_is_autopilot(msg)` gates **every** message in the reader loop.
**Validation: unit-tested** (5 tests). **Caveat — see Finding F-3: the component-id half of
the filter is currently a no-op** with pymavlink 2.4.49. The system-id half works.

## 3.10 Exit leg raises (Phase 2 only)

**Trigger.** Any exception out of the `exit_leg` callback.
**Handling.** Caught inside `gate_leg`, a hold is latched, and the outcome stays **CROSSED**:
```
[!] Exit leg raised ValueError('planner blew up'); holding after the crossing
[*] Gate 0: CROSSED -- gate crossed; exit leg failed: ValueError('planner blew up')
```
The gate *is* through; nothing the transition does afterwards can un-cross it.
**Validation: unit-tested** (`test_an_exit_leg_that_raises_is_contained_and_leaves_a_hold`).

---

# PART 4 — Confidence and testing status

**Legend.** **UT** = unit test against hand-derived arithmetic or scripted branches ·
**OFF** = exercised in an offline end-to-end run · **READ** = logic read through, never
executed against anything real · **NONE** = untested.

## 4.1 Geometry and planning — the strongest part of the codebase

| Component | Status | Validation |
|---|---|---|
| 3-2-1 rotation (incl. roll+pitch cross terms, length preservation) | Implemented | **UT** — 39 tests, hand-derived values |
| Normal-flip resolution, cone angle, altitude clamp | Implemented | **UT** |
| Plane-crossing test | Implemented | **UT** + **OFF** |
| Commit predicate | Implemented | **UT** |
| Circular mean / detection averaging | Implemented | **UT** |
| `estimate_move_timeout` arithmetic | Implemented | **UT** — 6 tests |
| `type_mask` values (2503 / 2531 / 2552) | Implemented | **UT** — asserted against the documented bit sums |
| Turn geometry: tangent points, arc, centre, heading profile | Implemented | **UT** — 38 tests, hand-derived 90° case |
| Turn radius bounds (yaw + tilt), slow-to-fit, every refusal | Implemented | **UT** |
| Association: row-swap immunity, gating radius, ambiguity veto, confidence, lifecycle | Implemented | **UT** — 22 tests |
| `order_gates_along_course` | Implemented | **UT** — but it is a stated heuristic |

## 4.2 Flight sequencing — well covered by tests, never flown

| Component | Status | Validation |
|---|---|---|
| `gate_leg` state machine, all three recovery rungs, every abort | Implemented | **UT** — 25 tests + **OFF** |
| Pad ordering constraints (arm→OFFBOARD, stream→OFFBOARD) | Implemented | **UT** — 26 tests + **OFF** |
| Every pad refusal-to-fly | Implemented | **UT** |
| Takeoff against telemetry, BRAKE_HOLD_ALT mask | Implemented | **UT** + **OFF** |
| Course loop: retries, failure counter, abort, always-lands | Implemented | **UT** — 44 tests across 1B and 2 |
| `land()` deadline behaviour, idempotence, pre-arm, dry-run | Implemented | **UT** — 6 tests |
| Deadman, both branches | Implemented | **UT** — 4 tests |
| Dry-run seam (incl. source-level "no sender reads `self.dry_run`") | Implemented | **UT** — 5 tests |
| Frozen-feed detection | Implemented | **UT** + **OFF** (refused on the pad) |
| Phase 2 `exit_leg` / `expected_gate` seams | Implemented | **UT** — 7 tests |
| Vector leg (arc flight, refusal, heading profile, budget) | Implemented | **UT** — 7 tests + **OFF** |
| Phase 2 mission loop and fallback | Implemented | **UT** — 11 tests + **OFF** |

## 4.3 Logic read-through only — be careful here

| Component | Status | Why it is only READ |
|---|---|---|
| `request_offboard` against a **real PX4 rejection** | Implemented | **READ** — no simulator. The retry loop is tested with a fake; PX4's actual refusal semantics are not |
| Mid-flight OFFBOARD drop | Implemented | **READ** — the *branch* is unit-tested; the real condition has never occurred |
| `ESTIMATOR_STATUS` parsing against real PX4 | Implemented | **READ** — field names (`flags`, `vel_ratio`, `pos_horiz_ratio`, `hagl_ratio`) taken from the MAVLink spec, never seen on the wire |
| `POSITION_TARGET_LOCAL_NED` echo | Implemented | **READ** — parsing never run against real msg 85 |
| `MAV_CMD_SET_MESSAGE_INTERVAL` actually changing PX4 rates | Implemented | **READ** — `measure_telemetry_rate` will tell you, but only on real hardware |
| `_is_autopilot` component filter | Implemented | **READ** — and currently inert, see F-3 |
| `turn_around_180` | Implemented | **READ** — no mission calls it |
| `run_mission` / `MissionOutcome` | Implemented | **UT** for the outcome; the *path* is unused by Phase 1/2 missions |
| Legacy `SingleGateMission` / `multi_stage_gate` | Unchanged | **READ** — 4 pre-existing tests only |

## 4.4 Genuinely untested — read this list before flying

| Item | Status |
|---|---|
| **Camera mount offsets** (`−0.25`, `−0.10`, `−10.0°`) | **NONE.** Never measured against a tape measure. Zeroed in every offline run. **The single largest unvalidated item in the system.** |
| **Optical-flow position drift** | **NONE.** The offline model is drift-free. Real flow drifts and local NED is not globally consistent. Everything Phase 2 does — association, planning, `expected_gate` — assumes a frame that does not drift much over a leg |
| **PX4 itself** | **NONE.** No controller, estimator, failsafe, ground effect, or link latency has ever been in the loop |
| **The Hailo → PnP → UDP pipeline against the navigation stack** | **NONE.** The two have never run together. Only the synthetic injector has ever fed the mission |
| **Phase 2 turn flown on a real aircraft** | **NONE.** And see Finding F-1 |
| **`--dry-run` past STEP 6** | **NONE.** Cannot reach it (F-4) |
| **Vision `FLIP_CAMERA` / calibration YAML against the fitted camera** | **NONE** in this repo's tests |

---

# PART 5 — Configuration reference

Everything tunable, in one place. **CLI flag** column blank means source-edit only.

## 5.1 `navigation/navigation.py` — module constants

| Constant | Value | Controls | Changing it |
|---|---|---|---|
| `UDP_IP` / `UDP_PORT` | `127.0.0.1` / `5050` | Where the vision feed is read | Must match `opencv_processing.py` |
| `MAVLINK_CONN` | `udpin:127.0.0.1:14551` | The link. `udpin` = **we bind and listen** | See Part 6; `--mavlink` overrides |
| `POSITION_RATE_HZ` | `10` | `move_to_target` / crossing / slew loop rate | Higher = smoother + more traffic |
| `POSITION_TOLERANCE_M` | `0.15` | Default arrival tolerance | Below ~0.15 sits inside flow noise; moves burn their budget |
| `YAW_TOLERANCE_DEG` | `5.0` | Arrival yaw tolerance, and `slew_yaw` completion | Tighter = slews may time out |
| `MOVE_TIMEOUT` | `5.0` | **Floor** on a move budget | Raise and the gate deadline stops delivering its attempts |
| `MAX_FLIGHT_SPEED_M_S` | `0.5` | Default `move_to_target` speed cap | Missions pass their own |
| `KP_POS` | `1.2` | Proportional gain, all position loops | Also feeds the budget arithmetic |
| `MAX_YAW_RATE_DEG_S` | `45.0` | Default `slew_yaw` rate | Faster degrades optical flow |
| `CAM_OFFSET_RIGHT_M` | `-0.25` | −ve shifts the drone **left** | **UNVALIDATED** |
| `CAM_OFFSET_DOWN_M` | `-0.10` | −ve shifts the drone **up** | **UNVALIDATED** |
| `CAM_YAW_OFFSET_DEG` | `-10.0` | −ve yaws **left** | **UNVALIDATED** |
| `MIN/MAX_PLAUSIBLE_DIST_M` | `0.05` / `100.0` | Detection validity band | Range check, not `!= 999.0` |
| `SETPOINT_RATE_HZ` | `20` | Streamer rate | PX4 needs < `COM_OF_LOSS_T` (1 s); 20 Hz = 20× margin |
| `SETPOINT_STALE_S` | `0.5` | Deadman trip age | |
| `MOVE_TIMEOUT_MARGIN_S` / `_SCALE` | `3.0` / `1.5` | Budget = `max(floor, margin + scale × ideal)` | |
| `ESTIMATOR_RATIO_WARN` | `0.8` | Innovation-ratio warning line (PX4 fails at 1.0) | |
| `FROZEN_FEED_FRAMES` | `15` | Identical payloads ⇒ frozen (0.5 s at 30 fps) | `--frozen-feed-frames` |
| `STATE_HISTORY_DEPTH` | `30` | Deskew ring buffer (1 s at 30 Hz) | |

## 5.2 `GateLegConfig` — per-gate

| Field | Default | CLI | Controls |
|---|---|---|---|
| `commit_distance_m` | 1.80 m | `--commit-distance-m` | Range term of the commit gate; also the ALIGN standoff. **1.80, not 1.00**: measured range stopped closing at ~1.72 m in flight, so 1.00 m was unreachable |
| `step_size_m` | 0.35 m | `--step-size-m` | **Horizontal** displacement cap per approach step |
| `vertical_step_m` | 0.25 m | `--vertical-step-m` | Vertical cap, budgeted separately. **Must be > `arrival_tolerance_m`** (validated) |
| `arrival_tolerance_m` | 0.15 m | `--arrival-tolerance-m` | Arrival test for approach steps. **Must be < `step_size_m`** (validated) |
| `approach_speed_m_s` | 0.35 m/s | `--approach-speed-m-s` | Speed cap for approach/align/back-off |
| `observation_duration_s` | 0.80 s | `--observation-duration-s` | Sampling window |
| `min_observation_samples` | 3 | `--min-observation-samples` | Thinner window is a miss, not a lock |
| `gate_pose_filter_samples` | 5 | `--gate-pose-filter-samples` | Rolling median window over gate N, E, D **and heading** |
| `max_approach_attempts` | 24 | `--max-approach-attempts` | Observations before NO_COMMIT |
| `approach_timeout_s` | 150 s | `--approach-timeout-s` | Hard per-gate deadline |
| `commit_lateral_tol_m` | 0.20 m | `--commit-lateral-tol-m` | Body-frame lateral term |
| `commit_vertical_tol_m` | 0.25 m | `--commit-vertical-tol-m` | Body-frame vertical term |
| `airframe_clearance_radius_m` | 0.115 m | `--airframe-clearance-radius-m` | Half the airframe box. Caps the two commit tolerances at the usable half-opening (validated) |
| `commit_max_cone_deg` | 60° | `--commit-max-cone-deg` | Edge-on rejection |
| `commit_confirm_frames` | 3 | `--commit-confirm-frames` | Consecutive passes required. A **missed** observation resets the streak |
| `max_align_attempts` | 6 | — | Align corrections before NO_COMMIT |
| `pass_distance_m` | 1.50 m | `--pass-distance-m` | Open-loop target beyond the gate |
| `exit_clearance_m` | 0.80 m | `--exit-clearance-m` | Crossing completion. **Must be < `pass_distance_m`** |
| `cross_speed_m_s` | 0.45 m/s | `--cross-speed-m-s` | Speed cap while crossing |
| `max_observe_retries` | 3 | `--max-observe-retries` | Rung 1 |
| `scan_half_angle_deg` | 45° | `--scan-half-angle-deg` | Rung 2 sweep |
| `max_yaw_rate_deg_s` | 45 °/s | `--max-yaw-rate-deg-s` | Rate for every deliberate heading change |
| `max_scan_sweeps` | 2 | `--max-scan-sweeps` | Rung 2 bound |
| `backoff_distance_m` | 1.00 m | `--backoff-distance-m` | Rung 3 retreat |
| `max_backoffs` | 2 | `--max-backoffs` | Rung 3 bound |
| `expected_gate_radius_m` | 2.00 m | — | Phase 2 seam. **Currently unused, see F-2** |
| `max_detection_age_s` | 0.40 s | `--max-detection-age-s` | Staleness signal 1 |
| `max_telemetry_age_s` | 0.50 s | — | Telemetry-stale abort |

## 5.3 `PadConfig`

| Field | Default | CLI | Controls |
|---|---|---|---|
| `min_position_rate_hz` | 15.0 | `--min-position-rate-hz` | Refusal threshold |
| `min_attitude_rate_hz` | 20.0 | `--min-attitude-rate-hz` | Refusal threshold |
| `telemetry_measure_s` | 3.0 s | — | Measurement window |
| `detection_wait_s` | 6.0 s | — | Total vision-check budget |
| `detection_observe_s` | 1.0 s | — | How long the feed must keep **changing**. Must exceed `frozen_feed_frames / fps` |
| `arm_wait_s` | 300 s | — | Arm-wait timeout |
| `offboard_timeout_s` | 10 s | — | OFFBOARD confirmation |
| `takeoff_timeout_s` | 20 s | — | Climb budget |
| `takeoff_tolerance_m` | 0.15 m | — | Arrival at takeoff altitude (F-8) |
| `require_detections` | True | `--allow-no-detections` | |
| `require_confirm` | True | `--no-confirm`, implied by `--offline` | |
| `skip_telemetry_rate_check` | False | `--skip-telemetry-rate-check` | |

## 5.4 `AltitudeEnvelope`

| Field | Default | CLI | Notes |
|---|---|---|---|
| `takeoff_alt_m` | 1.35 m | `--takeoff-altitude-m` | Must lie inside `[min, max]`. 1.35, not 1.50: the gate centre measures 1.06–1.09 m AGL |
| `min_alt_m` | 0.8 m | `--min-altitude-m` | The floor. Every commanded `d` is clamped |
| `max_alt_m` | 2.5 m | `--max-altitude-m` | The ceiling |
| `d_takeoff` | latched | — | Set at the pad from `LOCAL_POSITION_NED.z` |

## 5.5 Phase 2 — `CoursePlanConfig`

| Field | Default | CLI | Controls |
|---|---|---|---|
| `cruise_speed_m_s` | 0.45 m/s | `--cruise-speed-m-s` | Transition speed |
| `min_turn_speed_m_s` | 0.25 m/s | `--min-turn-speed-m-s` | Slow-to-fit floor before refusing |
| `max_yaw_rate_deg_s` | 45 °/s | `--max-yaw-rate-deg-s` | Lower bound on radius (`r ≥ V/ω`) |
| `max_lateral_accel_m_s2` | 1.47 (0.15 g) | `--max-lateral-accel-m-s2` | Lower bound on radius (`r ≥ V²/a`) |
| `exit_clearance_m` | 0.80 m | `--exit-clearance-m` | **Shared with the gate leg.** Turn may not begin closer than this |
| `entry_lead_in_m` | 1.00 m | `--entry-lead-in-m` | Straight run into the next gate |
| `min_next_gate_confidence` | 0.50 | `--min-next-gate-confidence` | Below ⇒ degrade |
| `min_heading_confidence` | 0.50 | `--min-heading-confidence` | Below ⇒ degrade |

## 5.6 Phase 2 — `AssociationConfig` and `Phase2Config`

| Field | Default | CLI |
|---|---|---|
| `base_gate_radius_m` | 0.60 m | `--assoc-radius-m` |
| `radius_growth_m_per_s` | 0.25 m/s | `--assoc-radius-growth-m-s` |
| `max_gate_radius_m` | 2.00 m | `--assoc-max-radius-m` |
| `ambiguity_margin_m` | 0.50 m | `--assoc-ambiguity-margin-m` |
| `position_smoothing` | 0.60 | — |
| `hits_for_full_confidence` | 3 | — |
| `max_track_age_s` | 5.0 s | `--assoc-track-age-s` |
| `gate_count` | 3 | `--gates` |
| `gate_retries` | 1 | `--gate-retries` |
| `max_consecutive_failures` | 2 | `--max-consecutive-failures` |
| `course_timeout_s` | 600 s | `--course-timeout-s` |
| `settle_between_gates_s` | 1.0 s | `--settle-between-gates-s` |
| `max_transition_failures` | 2 | `--max-transition-failures` |
| — | — | `--no-planning` (forces pure 1B) |

## 5.7 Offline model

| Flag | Default | Controls |
|---|---|---|
| `--offline-lag-s` | 0.3 s | First-order velocity lag. **0 makes every move succeed instantly and hides timeout bugs** |
| `--offline-wind-n/e` | 0.0 | Constant wind |
| `--offline-latency-s` | 0.0 s | Telemetry stamp backdating |
| `--offline-arm-delay-s` | 2.0 s | Simulated operator arm |
| `--offline-tilt` | off | Derive roll/pitch from horizontal acceleration |

## 5.8 Vision — **source-edit only, no CLI**

| Where | Setting | Value |
|---|---|---|
| `scripts/run_vision.py` | `CAMERA_CONFIGS` | `/dev/video0`, 1280×720, 60 fps |
| `vision/opencv_processing.py` | `USE_VIDEO_FILE` | `False` |
| | `FLIP_CAMERA` | **`True`** — 180° rotation for an inverted mount |
| | `MODEL_PATH` | `assets/best.hef` (git-ignored, **you must supply it**) |
| | `CALIB_PATH` | `assets/tevs_CAM2_fisheye_calibration.yaml` (live) / `CAM1` (video) |
| | `HALF_SIZE` | `0.97155 / 2` — the gate model |
| | `LOWER/UPPER_ORANGE` | HSV `[0,30,20]` … `[30,255,255]` |
| | `conf` | `0.5` in `model.predict` |
| | NMS IoU | `0.4` |
| | `ALPHA` | `0.3` pose EMA |
| | jump gate | `1.5 m` |
| | reject | `cv_z < 0` or `distance > 30 m` |
| | `udp_ip/port` | `127.0.0.1:5050` |
| `vision/flask_streaming.py` | port | `5000`, all interfaces, **no auth, no TLS** |

---

# PART 6 — Deploying on the Raspberry Pi 5 + Hailo AI HAT, with QGroundControl

**None of this plumbing is in the repo.** It is what you have to build around it.

## 6.1 The vision process

```bash
# One-time, per README
sudo apt install dkms hailo-h10-all && sudo reboot
hailortcli fw-control identify          # must report the Hailo-10H
sudo apt install python3-gi gir1.2-gstreamer-1.0 gstreamer1.0-tools \
                 gstreamer1.0-plugins-{base,good,bad,ugly}

python3 -m venv --system-site-packages .venv   # --system-site-packages is REQUIRED:
                                               # hailo_platform and gi are OS packages
source .venv/bin/activate && pip install -r requirements.txt
```

**What `python -m scripts.run_vision` actually does:**

1. `CameraStream` builds a GStreamer pipeline:
   `v4l2src → image/jpeg 1280×720@60 → jpegparse → tee`, one branch to a JPEG appsink
   (raw Flask stream), one through `jpegdec → videoconvert → BGR` to a CV appsink. Every
   queue is `max-size-buffers=1 leaky=downstream` and both sinks are `drop=true` — **the
   pipeline always yields the newest frame and discards backlog.** That is what keeps
   latency bounded when the Hailo pass is slower than 60 fps.
2. `HailoYOLO` opens a `VDevice`, loads `assets/best.hef`, sets UINT8 in / FLOAT32 out, and
   pre-computes the YOLOv8 anchor grid (strides 8/16/32 → 8400 anchors).
   **The DFL decode runs on the Pi's CPU in NumPy**, not on the Hailo — it filters by
   confidence *first*, then softmaxes only the surviving boxes. On a Pi 5 that ordering is
   the difference between a usable frame rate and not.
3. Per frame: resize to 640² (`INTER_NEAREST` — measurably faster on CPU), BGR→RGB, infer,
   NMS, **truncate to 3 boxes**.
4. Per box: crop → HSV orange mask → corner refinement (with single-bad-corner
   parallelogram repair) → `cv2.fisheye.undistortPoints` → `solvePnP(SQPNP)` →
   `solvePnPRefineLM` → reject if behind the camera or > 30 m → EMA smooth against the
   previous frame's tracks → Euler angles via `decomposeProjectionMatrix`.
5. **Axis remap into the body frame** — this is the contract:
   ```python
   drone_fwd_x = smooth_tvec[2][0]   # OpenCV +Z (into the scene) → forward
   drone_rgt_y = smooth_tvec[0][0]   # OpenCV +X                  → right
   drone_dwn_z = smooth_tvec[1][0]   # OpenCV +Y (down in image)  → down
   ```
6. Sort by distance, pad to 3 rows with 999s, `sendto` UDP 5050, and push a 960×540 JPEG to
   Flask.

**A box touching the frame edge (`is_partial`) is drawn but never PnP'd.** That is why the
mission commits and flies open-loop — the gate stops producing a pose as you enter it.

**Check it standalone before touching the aircraft:**
```bash
python -m debug.telemetry_reader        # terminal HUD — BINDS 5050, so stop it before flying
# browser → http://<pi>:5000
```

## 6.2 MAVLink plumbing — the piece you must supply

`MAVLINK_CONN = "udpin:127.0.0.1:14551"` means **the navigation process binds and listens**.
Something must *send* MAVLink to that address. PX4 talks UART to the Pi; you need a router
to fan it out to both the mission and QGC.

**Wiring:** PX4 `TELEM2` → Pi UART (`/dev/ttyAMA0`) or a USB-serial adapter. Set on PX4:
`MAV_1_CONFIG = TELEM2`, `MAV_1_MODE = Onboard`, `SER_TEL2_BAUD = 921600`.
**`Onboard` mode matters** — it is what makes PX4 stream `LOCAL_POSITION_NED` and `ATTITUDE`
at companion-useful rates instead of GCS defaults.

**`mavlink-router` config** (`/etc/mavlink-router/main.conf`):
```ini
[General]
TcpServerPort = 0
ReportStats = false
MavlinkDialect = common

[UartEndpoint px4]
Device = /dev/ttyAMA0
Baud = 921600

[UdpEndpoint mission]
Mode = Normal
Address = 127.0.0.1
Port = 14551

[UdpEndpoint qgc]
Mode = Normal
Address = 192.168.1.50     # your laptop
Port = 14550
```
`Mode = Normal` means the router *initiates* to that address — which is exactly what a
`udpin:` binder needs, and why the mission's `wait_heartbeat` unblocks.

**Ordering on the Pi:** router → vision → mission. The mission's first 30 s are a heartbeat
wait, so the router must already be up.

**Identity on the wire:**

| Component | sysid | compid | type |
|---|---|---|---|
| PX4 | 1 | 1 | `MAV_TYPE_QUADROTOR` |
| **This mission** | **254** | 1 | `MAV_TYPE_ONBOARD_CONTROLLER`, `MAV_AUTOPILOT_INVALID` |
| QGroundControl | 255 | 190 | `MAV_TYPE_GCS` |

Both our own type and QGC's are in pymavlink's `probably_vehicle_heartbeat` exclusion list,
so **neither can hijack `target_system`**. That is load-bearing and it works by construction.

**A `systemd` unit for the mission is a bad idea** and the code assumes you will not use
one: STEP 4 blocks on `input()`. Run it from an SSH session you can see and Ctrl-C.

## 6.3 QGroundControl — what to do with it, and what not to

**Connect:** UDP, port 14550, auto-connect off, add a link to the Pi. QGC's heartbeat is
`MAV_TYPE_GCS` so it is invisible to our sysid lock.

**Use QGC for exactly four things:**

1. **The parameter checklist**, before every session. Read the values *off the vehicle* —
   airframe startup scripts override firmware defaults:

   | Parameter | Danger | Want |
   |---|---|---|
   | `EKF2_OF_CTRL` | **`0` = optical flow OFF.** Nothing else works | `1` |
   | `EKF2_HGT_REF` | Height source | Range finder |
   | `EKF2_RNG_A_HMAX` | Max rangefinder altitude | ≥ your ceiling |
   | `MPC_XY_VEL_MAX` | **12 m/s default.** The reason takeoff uses mask 2531 | ≤ 2 m/s indoors |
   | `MPC_YAWRAUTO_MAX` | Backstop only — the code now limits yaw itself | 45 °/s |
   | `COM_LOW_BAT_ACT` | **`0` = no battery action at all** | Land/RTL |
   | `COM_OBL_RC_ACT` | What PX4 does when offboard is lost | Land |
   | `COM_RCL_EXCEPT` | Whether RC loss is excepted in offboard | Deliberate choice |
   | `COM_OF_LOSS_T` | Offboard-loss timeout (1.0 s) | We stream at 20 Hz = 20× margin |
   | `COM_VEL_FS_EVH` | Velocity-innovation failsafe. **The one that fires as "offboard lost"** | Know its value |
   | `COM_DISARM_PRFLT` | Pre-takeoff auto-disarm (10 s) | Why you arm *after* the prompt |

2. **Arming** — as an alternative to the RC switch. Either satisfies `wait_for_armed`.
3. **Watching the mode change** — the Flight Mode indicator must read **Offboard** the
   moment the console prints `OFFBOARD confirmed via HEARTBEAT.custom_mode`. If those two
   disagree, believe the console: it is reading `custom_mode` directly.
4. **Post-flight log download** (`.ulg`) — the only record of what PX4 actually did.

**Do NOT, while the mission is running:**
- Press **Take off**, **Land**, **RTL**, **Pause**, or set a mode. All of these fight the
  mission and PX4 will honour the most recent one. Ctrl-C the mission *first*.
- Leave QGC's own telemetry-rate settings enabled. QGC issues its own
  `SET_MESSAGE_INTERVAL` commands and **will overwrite ours**. If `measure_telemetry_rate`
  reports rates far below what was requested, this is the first thing to check.
- Upload a flight plan. Nothing in this code reads or clears a mission.

**Emergency:** the RC kill switch, then the RC mode switch out of Offboard. Both are
faster and more reliable than anything through QGC or the console.

## 6.4 The bench sequence I would actually run

```bash
# ── Pi, terminal 1 ─────────────────────────────────────────
sudo systemctl start mavlink-router          # or however you run it
# ── Pi, terminal 2 ─────────────────────────────────────────
python -m scripts.run_vision
# ── laptop ─────────────────────────────────────────────────
# browser  → http://<pi>:5000   (confirm boxes, corners, axes on a real gate)
# QGC      → parameter checklist
# ── Pi, terminal 3 ─────────────────────────────────────────
python -m debug.telemetry_reader             # tape-measure the numbers. THEN STOP IT.
python -m navigation.missions.phase1a_single_gate --dry-run
```
**That last command will stop at STEP 6** (Finding F-4). Everything before it — link,
telemetry rates, vision health, the banner, the arm wait — is real, and STEP 3's output
against a tape measure is the camera-offset validation that has never been done.

---

# FINDINGS LOG

Things I noticed while tracing the real execution path. **Nothing here was fixed.**

---

### F-1 — HIGH — A "straight" Phase 2 transition can command the aircraft to fly backwards through the gate it just crossed

**`navigation/missions/course.py`, `plan_turn`, the aligned branch.**

When two consecutive gates are within ~1° of the same heading, `plan_turn` returns
`ok=True` with:
```python
entry      = leaving.point_at(cfg.exit_clearance_m)      # gate N centre + 0.80 m
exit_point = entering.point_at(-cfg.entry_lead_in_m)     # gate N+1 centre − 1.00 m
```
**There is no check that `exit_point` is ahead of `entry`.** The curved branch guards this
(`max_tangent_m <= 0.0` → refuse); the straight branch does not. If the gates are closer
together than `exit_clearance_m + entry_lead_in_m` (1.80 m at defaults), the exit point lies
*behind* the entry point and `vector_leg.fly_turn` obediently flies the drone backwards.

**This was observed live**, in the offline Phase 2 run, and reported as a success:
```
[*] Transition out of gate 0: no turn needed: the two gate vectors are already aligned
    t=0.00  N=  +2.07 ...
    t=1.00  N=  +1.52 ...          ← 0.55 m BACKWARD
[*] gate 0 -> 1 transition complete; on the next gate's centre line
```
Gate 0 sat at N=+1.27 and the next track at N=+2.52 — 1.25 m apart, under the 1.80 m
minimum. The vehicle flew back toward the gate it had just crossed with the nose still
pointed forward.

**Why the tests did not catch it:** `test_aligned_gates_need_no_turn` asserts only
`turn.ok` and `turn.straight`, on gates 6 m apart. There is no test for closely-spaced
aligned gates.

**Impact:** Phase 2 only. Harmless offline and irrelevant to a props-off bench test.
On a real aircraft it is a commanded reversal into a gate at 0.45 m/s.

---

### F-2 — MED — The `expected_gate` seam is implemented and tested, but Phase 2 never uses it

`gate_leg.approach_and_cross_one_gate` accepts `expected_gate=` and rejects observations
further than `expected_gate_radius_m` (2.0 m) from it. `phase2_course._attempt_gate` calls:
```python
approach_and_cross_one_gate(nav, mission, leg_cfg, gate_index=gate_index, exit_leg=exit_leg)
```
— no `expected_gate`. So the gate-confusion guard that Phase 2 exists to be able to use is
dead code in the only mission that has the tracker to feed it. `expected_gate_radius_m` is
also the one `GateLegConfig` field with no CLI flag.

---

### F-3 — MED — The component-id half of the source filter is inert with pymavlink 2.4.49

`_is_autopilot` checks `msg.get_srcComponent() != master.target_component`. But in this
pymavlink version `target_component` is a **property returning `param_sysid[1]`**, which is
initialised to `0` and is never written by heartbeat handling — only by an explicit
assignment nothing in this repo makes. So `component` is always `0` and the guard
`if component and …` never runs.

Consequences: (a) the sysid filter works and is the important half; (b) all outbound
commands go out with `target_component = 0` (broadcast), which PX4 accepts; (c) the
docstring's claim to filter on compid is not currently true. A gimbal or camera component
on **sysid 1** publishing `LOCAL_POSITION_NED` would still be folded in.

---

### F-4 — MED — `--dry-run` cannot get past STEP 6, so the bench path stops before the geometry it was built to validate

`request_offboard` confirms on `HEARTBEAT.custom_mode`. In dry-run nothing is transmitted,
so PX4 never enters OFFBOARD, so confirmation never comes, so the pad fails at STEP 6.
Everything after — the approach loop, the commit gate, the crossing — is unreachable on the
bench. Known and previously reported; repeated here because it directly limits the
camera-offset validation that is the single largest unvalidated item in the system.

---

### F-5 — LOW — Phase 2 decides "did a transition happen" by substring-matching a human-readable reason string

```python
if "exit leg flown" not in outcome.reason:
    nav.hold_position(...); _settle(...)
```
`GateOutcome.reason` is prose intended for the console. Rewording `gate_leg`'s success
message silently changes whether Phase 2 settles between gates. A boolean field on
`GateOutcome` would carry this without coupling behaviour to log text.

---

### F-6 — LOW — There are two independent gate-association layers and they do not know about each other

`vision/opencv_processing.py` keeps `prev_tracked_gates` and does nearest-neighbour matching
(1.5 m jump gate) plus an EMA (α = 0.3) on `tvec`/`rvec`, in the **camera** frame.
`navigation/missions/association.py` does nearest-neighbour matching plus an EMA
(`position_smoothing = 0.6`) in **local NED**. The mission's smoothing therefore runs on
already-smoothed data, and the effective time constant is the product of the two — larger
than either config suggests. Neither layer is wrong; the composition is just not documented
anywhere, and it will matter when tuning association at speed.

---

### F-7 — LOW — Estimator degradation is diagnostic only; it never changes what the aircraft does

`estimator_faults()` is only ever read by `estimator_note()`, which only ever appends text to
messages that were already being printed for another reason. A steadily degrading estimator
(velocity innovation climbing 0.8 → 0.95) produces **no output at all** until something else
aborts. Given the whole point of subscribing to msg 230 was flow degradation, a periodic
"estimator degrading" warning from the streamer thread would be cheap. Deliberate scope
choice, not a defect — but worth knowing it will not warn you.

---

### F-8 — LOW — `takeoff_tolerance_m = 0.15` means the mission systematically flies low, and has no CLI flag

Observed offline: `[*] Reached 1.37 m AGL` for a 1.50 m target. The mission then flies
~0.13 m low for its whole duration against a 0.80 m floor. In spec. Note the *arrival*
tolerance is 0.15 m but nothing re-climbs afterwards — the post-takeoff hold latches
wherever it stopped. Not exposed as a flag, so changing it means editing `PadConfig`.

---

### F-9 — LOW — `turn_around_180()` is live, correct, and called by nothing

It was fixed this pass (it now uses `slew_yaw`), it is a public method, and no mission in
the repo calls it — the recovery ladder uses `_yaw_sweep`. It is reachable only by a caller
that does not exist. Either wire it in as a rung-4 recovery or delete it; leaving it invites
someone to assume the search does a 180 when it does not.

---

### F-10 — LOW — `choose_next_gate` builds a candidate list it may then discard

```python
for track in tracker.tracks:
    ...
    if current is not None: ...
    candidates.append(track)
if not candidates: return None
if current is None:  return None      # ← the loop's work is thrown away
```
Harmless (Phase 2 always passes a real `current`), but the `current is None` path does work
it cannot use, which reads as an unfinished branch.

---

### F-11 — INFO — `vector_leg`'s intermediate waypoint tolerance can exceed the spacing between arc points

Intermediate arc points use `tolerance = cfg.entry_lead_in_m * 0.5` = 0.50 m. With 8 points
on a short arc the spacing can be under 0.50 m, so several points are "reached" without the
vehicle moving — the aircraft cuts the corner and the flown path is chordal rather than the
planned arc. Deliberate in spirit (stopping at every point would reinstate the 1B stutter),
but the tolerance is a fixed fraction of an unrelated parameter rather than a fraction of the
arc-point spacing.

---

### F-12 — INFO — `_fly_through_gate` derives its budget with the default tolerance, not the leg's

```python
budget_s = estimate_move_timeout(span, cfg.cross_speed_m_s)   # tolerance_m defaults to 0.15
```
Every other budget in `gate_leg` now passes `cfg.arrival_tolerance_m` explicitly. Identical
at defaults; diverges the moment someone sets `--arrival-tolerance-m`.

---

### F-13 — INFO — The pad's numbered comments and its printed STEP numbers disagree

`run_pad_sequence` has comments `--- 1.` through `--- 8.` but prints `[STEP 1]` … `[STEP 7]`,
and then `phase1a` prints `[STEP 8]`/`[STEP 9]`. Reading the source and reading the console
give you two different numbering schemes for the same sequence.

---

### F-14 — INFO — `land()`'s pre-arm shortcut depends on `EXTENDED_SYS_STATE` being present

```python
if entry.heartbeat_received_s > 0.0 and not entry.armed and entry.landed_state != 2:
    print("[*] Vehicle is disarmed and on the ground; no landing required.")
```
`landed_state` starts at `0` and is only ever set by msg 245. If that stream is missing (PX4
build, router filtering, or a rate request that did not take), a vehicle **disarmed in
mid-air** — kill switch, battery failsafe — takes this branch and reports "no landing
required". The `heartbeat_received_s > 0.0` guard does not help, because heartbeats are
independent of 245. It is the right shortcut with the stream present, and it is requested
at 5 Hz; it just has no independent check that it arrived.

---

### F-15 — INFO — The injector needs measurement noise to avoid tripping the frozen-feed detector, and this is easy to undo

`--noise-m` defaults to 3 mm 1-sigma. Set it to 0 and any case whose range stops changing
(every case, once it hits `--min-distance-m`) produces bit-identical payloads and correctly
trips the frozen-feed detector. Correct behaviour on genuinely frozen synthetic data, but it
looks like a mission bug the first time you see it.

---

## Direct answer to your closing question

**Would anything I just wrote change my answer to "is this ready for a first bench test with
props off"?**

**No — bench-test it.** Nothing in the findings makes a props-off bench run less safe or
less useful, and the two things that would have (`land()`'s truncated budget, the deadman
crying wolf) are fixed and regression-tested. F-1 is a Phase 2 flight bug; on a bench,
nothing moves.

**But three things change what you should expect from that bench test, and you should know
them before you set up:**

1. **`--dry-run` will stop at STEP 6 and never reach the gate approach** (F-4). If your plan
   was "watch it approach a gate on the bench", that plan does not work today. What you *do*
   get — and it is the highest-value thing available — is STEP 3's live detection line
   against a tape measure. Run `debug/telemetry_reader.py` alongside it for the raw rows,
   then stop it before starting the mission (both bind 5050).

2. **The vision process and the navigation stack have never run together at all.** Every
   offline validation used the synthetic injector. The first time real UDP packets from
   `opencv_processing.py` reach `GateMission._ingest_payload` will be on your bench. Expect
   to find something in that seam — that is the point of the exercise. The two things I
   would watch: whether `dist` and `hypot(fwd,right,down)` diverge as much as expected (the
   raw-vs-smoothed issue is real and confirmed in the vision source), and whether the
   `yaw` column sign matches what `resolve_gate_normal` expects.

3. **The camera offsets are still guesses** (`−0.25`, `−0.10`, `−10.0°`) recovered from a
   dead prototype. They have never been measured. Validating them is the *whole* reason to
   do this bench test, and it is the single largest unvalidated item in the system.

**What I would not do yet:** fly Phase 2. F-1 is a real backwards-flight command with a
plausible trigger (two gates less than 1.80 m apart on the same heading), it fired in the
only end-to-end Phase 2 run that exists, and it reported success while doing it. Phase 1A
first, then 1B, then Phase 2 with `--no-planning`, and only then Phase 2 with planning
enabled — after F-1 is fixed and has a test covering closely-spaced aligned gates.
