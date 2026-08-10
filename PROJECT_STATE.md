# PROJECT STATE — Autonomous Gate-Course Navigation

**Last updated:** 2026-08-10 · **Branch:** `main` · **Last commit:** `c00083c` "Gabe-Claude V1 Code 8/10"
**Status:** Phase 1A and 1B implemented, offline-validated, **never flown**. Phase 2 not started.

This file is the single handoff document. A fresh session should be able to resume from
this alone. Keep it updated as Phase 2 proceeds.

---

## 1. What this project is

An autonomous PX4 multirotor that flies through a course of gates, guided by a
nose-mounted camera. Position comes from **indoor optical flow + rangefinder** — there is
no GPS and no globally-consistent map. There is **no simulator and no flight-test-before-
deployment path**, so the deliverable is not features: it is code whose failure modes are
enumerated, bounded, and diagnosable from a console log alone.

The full design rationale, the PX4/pymavlink verification findings, and the parameter
checklist live in the plan document:
`~/.claude/plans/you-re-building-out-an-frolicking-twilight.md`. **Read it before flying.**

### Three phases

| Phase | What it does | Status |
|---|---|---|
| **1A** | Take off, find one gate, cross it, land | **Implemented, offline-validated** |
| **1B** | Same, looped over N gates with a full stop between each | **Implemented, offline-validated** |
| **2** | Optimized course: association, planning, arc transitions | **Not started** |

Phase 1B is the trustworthy fallback: deliberately unoptimized, and the reference that
Phase 2's behaviour gets checked against.

---

## 2. File inventory

### Modified

| File | Lines | Responsibility |
|---|---|---|
| `navigation/navigation.py` | 1448 | MAVLink I/O, `VehicleState`, setpoint streamer, `move_to_target`, `land`, `GateMission` UDP listener. All changes additive/default-preserving. |

### New — Phase 1 stack (`navigation/missions/`)

| File | Lines | Responsibility |
|---|---|---|
| `contracts.py` | 231 | Every dataclass/enum crossing a seam. No I/O. `GateResult`, `GateFix`, `GateOutcome`, `CommitVerdict`, `GateLegConfig`, `AltitudeEnvelope`. Named `contracts` not `types` because every test file does `import types` for the pymavlink stub. |
| `frames.py` | 437 | **Pure geometry, zero I/O.** 3-2-1 rotation, circular mean, normal-flip resolution, altitude clamp, plane-crossing test, `localize_gate`, `evaluate_commit`. This is where the correctness lives, and it is fully unit-testable. |
| `gate_leg.py` | 344 | **THE shared routine.** `approach_and_cross_one_gate()` — a free function, not a base class. 1A, 1B and (eventually) 2 all call it unchanged. |
| `pad.py` | 303 | Everything between `python -m …` and "the drone is hovering": link, telemetry rates, vision check, banner, operator confirm, arm wait, OFFBOARD verify, takeoff. |
| `cli.py` | 220 | Shared argparse surface + object construction. Also `resolve_offline_camera_offsets()`. |
| `phase1a_single_gate.py` | 148 | Runnable mission. Linear sequence, one `_abort` helper, one call to `gate_leg`. |
| `phase1b_course.py` | 286 | Runnable mission. Same pad sequence + a loop, whole-gate retries, consecutive-failure cap. |
| `offline_backend.py` | 284 | `OfflineNavigationController` — kinematic model for `--offline`. First-order lag, optional wind/latency. Production code, not a test fake. |
| `udp_injector.py` | 251 | Synthetic vision publisher. Emits the real wire format. 12 cases. |

### Unchanged legacy (kept as reference, per plan §3)

| File | Lines | Note |
|---|---|---|
| `navigation/missions/single_gate.py` | 171 | Byte-identical. `plan_standoff_horizon` / `limit_target_step` are **imported** by `gate_leg.py`, never copied. |
| `navigation/missions/multi_stage_gate.py` | 51 | Byte-identical. |
| `navigation.py`, `navigation_Vmanual.py`, `navigation_closed_loop.py` (repo root) | — | **Dead prototypes.** Shadowed by the `navigation/` package. `_Vmanual` is byte-identical to the old `navigation.py`; `_closed_loop` is truncated mid-function. Not in scope. |

### Tests (`tests/`) — 78 tests, all green

| File | Tests | Covers |
|---|---|---|
| `test_frames_and_rotation.py` | 34 | 3-2-1 rotation, circular mean, normal flip, cone/offset, altitude envelope, crossing, commit gate |
| `test_move_timeout_and_setpoints.py` | 17 | `estimate_move_timeout`, `type_mask_for`, `is_transient`, `is_valid_detection` |
| `test_phase1b_course.py` | 14 | Course loop policy with per-gate work scripted out |
| `test_navigation_geometry.py` | 6 | Pre-existing, still green |
| `test_single_gate_mission.py` | 4 | Pre-existing, still green |
| `test_flask_streaming.py` | 3 | Pre-existing, unrelated |

---

## 3. Conventions in force

**Frames.** body FRD (`+x` forward, `+y` right, `+z` down) → local NED. `d` is **down**, so
`d = -1.5` is 1.5 m *above* the origin.

**Units.** Metres, radians — unless the name ends `_deg`. `GateDetection.roll/pitch/yaw_deg`
and the camera offsets are degrees; `VehicleState.yaw_rad` / `LocalTarget.yaw_rad` are radians.

**Vision wire format** (UDP `127.0.0.1:5050`, JSON):
`{"fps": float, "gates": [[dist, forward, right, down, roll, pitch, yaw], ×3]}` —
always exactly 3 rows, sorted nearest-first, absent gates padded all-`999.0`.
**Row index is not a stable gate identity.** Gate model is a 0.97155 m square.

**Ownership rule** (written into `gate_leg.py`'s docstring, and load-bearing):
> The mission file owns the **sequence**. `gate_leg` owns one **step**.

Someone at a test site must read `phase1a_single_gate.py` top to bottom and see every state
transition, abort branch and landing decision without opening a second file.

**Returns are named, not boolean.** `GateResult` enum, not `bool` — because 6 of 7
`move_to_target` call sites in the original code ignore its boolean return.

**Every mission exit lands.** Success, failure, deadline, exception, Ctrl-C.

**Lock order:** `_vehicle_lock → _setpoint_lock → _master_lock`. Never hold
`_setpoint_lock` across a transmit.

---

## 4. Plan decisions — implemented vs. not

### Implemented and verified

- Pad sequence ordering: connect → heartbeat → telemetry → vision → **start stream** →
  banner → confirm → **wait for arm** → verify OFFBOARD → climb.
  (Stream must precede OFFBOARD; arm is the operator's commit action.)
- `assert_not_climbing()` invariant — stream carries finite zeros until OFFBOARD verified,
  so arming and lift-off cannot be the same event.
- Setpoint streamer thread, 20 Hz, absolute-time scheduling, deadman → brake-and-hold.
- `Setpoint.is_transient` — deadman ages **only** velocity setpoints. A steady position/hold
  setpoint stays valid indefinitely. (Without this, takeoff was overridden as "stalled".)
- Takeoff uses `BRAKE_HOLD_ALT` (mask 2531: xy-*velocity* + z-*position*), **not** a full
  position mask — a position mask can translate toward local origin at `MPC_XY_VEL_MAX`
  (default **12 m/s**) while 30 cm off the ground.
- `request_offboard()` — explicit `MAV_CMD_DO_SET_MODE`, verified on
  `HEARTBEAT.custom_mode == 393216`. Never `master.set_mode()` (silent no-op) and never
  `master.flightmode` (misreports OFFBOARD as UNKNOWN in every pymavlink release).
- 1 Hz own-HEARTBEAT from sysid 254 so `COMMAND_ACK` routes back.
- `land()` verified on `landed_state` + disarm flag, never on flight mode. Idempotent.
- `move_to_target` derives its timeout from distance+speed (`estimate_move_timeout`), clamps
  `d` to the altitude envelope with a loud log, and always leaves a hold setpoint in `finally`.
- Crossing test is a **signed projection onto the gate normal**, not a position tolerance.
- Commit gate requires range **AND** body-frame lateral **AND** body-frame vertical **AND**
  cone — held over N consecutive frames. Alignment measured in the **body frame** (directly
  observed), not from the weakly-observable plane yaw.
- Gate-normal 180° ambiguity resolved geometrically; `normal_flipped` logged.
- Circular mean for all angle averaging (`mean(179, -179) = 180`, not 0).
- `range_m = hypot(forward, right, down)`, **not** `det.dist` — the vision process derives
  `dist` from the raw pose and `forward/right/down` from the smoothed pose, so they disagree
  most while moving. `det.dist` is used only as the no-detection sentinel.
- `is_valid_detection` is a plausibility range check, not float equality against 999.0.
- Altitude envelope referenced to the pad's latched `d_takeoff`, not assumed zero.
- `stop()` publishes a hold before teardown; `_closed` flag instead of nulling `self.master`.
- `run_mission`'s `start()` moved inside the `try` so the reader thread cannot leak.
- Camera-offset sign conventions restored as comments (recovered from the dead
  `navigation_closed_loop.py`).

### ⚠️ Planned but NOT implemented

These were in the plan and are **absent from the code**. Each is a real gap, listed worst
first.

| # | Missing | Consequence |
|---|---|---|
| 1 | **Frozen-feed detection** (`_latest_frame`, payload hash over ~15 frames) | The third of the plan's three staleness signals. Signals 1 (no datagram) and 2 (all-999) work. A **stalled camera that keeps transmitting an unchanging pose is not detected** — the drone will fly a frozen gate position. `udp_injector --case frozen` exists to test this and there is nothing to catch it. |
| 2 | **No unit tests for `gate_leg.py` or `pad.py`** | 647 lines containing the actual flight sequence, state machine, recovery ladder, and takeoff — **zero coverage**. Plan promised `test_gate_leg.py`, `test_phase1_missions.py`, `test_offline_backend.py`, `test_gate_localization.py`. |
| 3 | **Yaw-rate limiting** (`MAX_YAW_SPEED_DEG_S = 45.0`, plan item 17) | `turn_around_180()` still commands an instantaneous 180° step; `_yaw_sweep` commands instantaneous ±45°. Mitigated in practice by PX4's own `MPC_YAWRAUTO_MAX` (default 45°/s), but **this code does not control it**, and fast yaw degrades optical flow. Note `offline_backend` *does* rate-limit yaw, so **offline testing masks this**. |
| 4 | **`ESTIMATOR_STATUS` subscription** (plan §7-F.6) | If flow velocity quality degrades past `COM_VEL_FS_EVH`, PX4 sets `offboard_control_signal_lost` and drops offboard — while the log says "offboard signal lost" and the real cause was the estimator. Will be misdiagnosed for days. |
| 5 | **`body_to_local_321` in `navigation.py`** (plan item 4) | The 3-2-1 rotation lives only in `frames.py`. `navigation.py:body_to_local` is still yaw-only, and `GateMission.detection_to_gate_local` still uses it. **Phase 1A/1B get the pitch/roll fix; the legacy `single_gate`/`multi_stage_gate` missions do not.** |
| 6 | **`MavlinkLink`/`PymavlinkLink`/`DryRunLink` protocol** (plan item 18) | `dry_run` is instead an inline `if self.dry_run` branch in five separate methods. It works today, but there is no single interception seam — **a future command added without a `dry_run` branch will transmit during a bench test.** |
| 7 | `VehicleStateHistory` ring buffer / detection deskew (plan item 20) | Phase 2 item. At Phase 1 speeds (0.35 m/s × 100 ms = 3.5 cm) the error is ignorable. |
| 8 | `POSITION_TARGET_LOCAL_NED (85)` at 5 Hz | PX4's echo of the setpoint it is actually tracking — the cleanest confirmation the `type_mask` was interpreted as intended. Would make `--dry-run` bench work far more conclusive. |
| 9 | `MissionOutcome` return from `run_mission` (plan item 15) | Still returns `None`. Low impact; the Phase 1 missions don't use `run_mission`. |
| 10 | `GateDetection.row_index` / `source_fps` | Phase 2 association inputs. |
| 11 | `expected_gate=` parameter on `approach_and_cross_one_gate` | Phase 2 seam. Only `gate_index` and `log` exist. |
| 12 | `exit_leg=` parameter | Phase 2 seam — the mechanism by which Phase 2 degrades to Phase 1B without a special case. |

**Only `gates[0]` is consumed.** `_udp_gate_listener` reads the nearest row and discards the
other two. Correct for 1A. For 1B this assumes "nearest gate == next gate", which holds only
because each gate is crossed before the next is sought. Phase 2 needs all three rows.

---

## 5. Bugs found in review (2026-08-10) — not yet fixed

Found by reading the code and by an instrumented offline run. **None of these are fixed.**

| Sev | Where | Bug |
|---|---|---|
| **HIGH** | `navigation.py` `land()` ~L1158 | **Only ~15 s of the 40 s budget is used.** The inner wait breaks after 5 s and the outer loop is bounded by `attempts=3`, so the function gives up at ~15 s and prints "Landing NOT confirmed" — while 25 s of `timeout_s` remains and the aircraft may be landing perfectly. Phase 1A then returns exit code 1 on a good landing. Fix: make the outer loop deadline-driven, not attempt-driven. |
| **HIGH** | `navigation.py` `_setpoint_streamer` | **Deadman false-positives on the pad, every single run.** Confirmed live: `[!] DEADMAN: velocity setpoint 'prime' not refreshed for 0.50s` fires during STEP 5's arm wait, because the priming setpoint is `VELOCITY_YAW` (transient) and nothing refreshes it while `input()` blocks. The *action* is correct and safe (brake-and-hold on the ground); the *message* is not. This trains the operator to ignore the one warning that matters in flight. Fix: suppress the trip until `armed`. |
| **MED** | `navigation.py` `observe_gate` L1367 | **Conflicting setpoint kinds are interleaved.** It latches `BRAKE_HOLD_ALT` (mask 2531, z-*position*) then in the loop directly sends `VELOCITY_YAW` zeros (mask 2503, z-*velocity*) at 25 Hz. PX4 receives both. `vz = 0` holds *rate*, not altitude, so the vehicle can drift vertically during every observation window. The direct send is vestigial — the streamer already publishes at 20 Hz. Fix: delete the direct send. |
| **MED** | `navigation.py` `_mavlink_reader` | **Only `HEARTBEAT` is filtered by source system.** `LOCAL_POSITION_NED`, `ATTITUDE` and `EXTENDED_SYS_STATE` are folded in from any sysid. On a link shared with QGC or a second vehicle, foreign telemetry becomes this vehicle's state. Fix: apply `_is_autopilot()` to all four branches. |
| **MED** | `navigation.py` constants | **`POSITION_TOLERANCE_M = 0.10` vs `MOVE_TIMEOUT = 15.0` are inconsistent bounds.** Confirmed live: a 0.25 m approach step is issued with `Budget=15.0s` (the floor dominates). With `approach_timeout_s = 75 s`, a drone that cannot settle inside 0.10 m gets **~5 attempts, not the `max_approach_attempts = 14`** the config advertises. 0.10 m is very tight for an optical-flow airframe. Fix: loosen the arrival tolerance for approach steps, or lower the `MOVE_TIMEOUT` floor, and reconcile the two caps. |
| **LOW** | `phase1b_course.py` L277 | **Ctrl-C discards the per-gate report.** `except KeyboardInterrupt: return _abort(nav, "operator interrupt", [], 0)` passes an empty `results`, so after flying two gates the log says "no gates were attempted". `results` is local to `run()`. Fix: hoist it. |
| **LOW** | `navigation.py` `land()` L1165 | Returns success immediately when never armed (`not state.armed` is true on the pad). Outcome is correct, message ("Landed and disarmed.") is misleading on a pre-arm abort. |
| **LOW** | `pad.py` | Confirmed live: takeoff reports success at **1.35 m** for a 1.50 m target (`takeoff_tolerance_m = 0.15`). Within spec, but the whole mission then flies 0.15 m low against a 0.80 m floor. |
| **LOW** | `pad.py` L29 | `field` imported, unused. |

### What `--offline` does **not** exercise

Important when reading a green offline run:

- **Yaw rate** — the model rate-limits it; the real code does not (gap #3 above).
- **Position drift** — the model is drift-free. Real optical flow drifts, and N/E is not
  globally consistent. This is the single biggest untested risk.
- **Camera mount offsets** — `resolve_offline_camera_offsets()` zeroes them, because the
  injector emits vehicle-frame poses while real vision emits camera-frame. The offset path
  is therefore **never exercised offline** and must be verified on the bench.
- **PX4 itself** — no controller, no estimator, no failsafes, no ground effect, no link latency.

---

## 6. How to run

Use the venv: `.venv/bin/python` (pymavlink is installed there).

### Tests
```bash
.venv/bin/python -m unittest discover -s tests -v      # 78 tests, all green
.venv/bin/python -m compileall -q navigation scripts vision debug
```

### Offline — laptop only, no drone, no camera (two terminals)
```bash
# terminal 1
.venv/bin/python -m navigation.missions.udp_injector --case dead-ahead --duration-s 60
# terminal 2
.venv/bin/python -m navigation.missions.phase1a_single_gate --offline
```
```bash
# Phase 1B — the course case sawtooths the range so each cycle reads as a new gate
.venv/bin/python -m navigation.missions.udp_injector --case course --duration-s 200
.venv/bin/python -m navigation.missions.phase1b_course --gates 3 --offline
```

Injector cases: `.venv/bin/python -m navigation.missions.udp_injector --list`
→ `dead-ahead`, `offset-left`, `offset-right`, `above`, `below`, `normal-as-reported`,
`normal-reversed`, `edge-on`, `multiple-gates`, `dropout`, `frozen`, `course`.

> The injector binds UDP 5050 — the same port `debug/telemetry_reader.py` binds. One
> consumer at a time.

### Dry run — real vehicle, real telemetry, **transmits nothing** (props off, bench)
```bash
.venv/bin/python -m navigation.missions.phase1a_single_gate --dry-run
```

### Real flight
```bash
.venv/bin/python -m navigation.missions.phase1a_single_gate
.venv/bin/python -m navigation.missions.phase1b_course --gates 3
```

`--offline` and `--dry-run` are mutually exclusive. Every flag is in `cli.py`
(`add_common_arguments`) and is printed in the pre-flight banner.

---

## 7. Validation status

| Claim | Validated by | Confidence |
|---|---|---|
| Geometry (rotation, circular mean, normal flip, clamp, crossing, commit gate) | 34 unit tests against **hand-derived arithmetic**, not against current output | **High** |
| Move-timeout arithmetic; `type_mask` values; detection validity | 17 unit tests | **High** |
| Course loop policy (retries, failure counter, abort short-circuit, always-lands) | 14 unit tests with per-gate work scripted out | **High** |
| Phase 1A end-to-end sequencing | One instrumented `--offline` run, 2026-08-10: pad → arm → OFFBOARD → 1.35 m climb → 7 observations → commit at 0.50 m → CROSSED → landed, `attempts=7`, 14.2 s | **Medium** — logic only |
| `gate_leg` state machine, recovery ladder, pad sequence | **Nothing.** No unit tests. Only the happy path of the offline run. | **Low** |
| Camera mount offsets / sign conventions | **Nothing.** Zeroed offline; never measured on a bench. | **None** |
| Anything about the aircraft — control, estimator, failsafes, flow drift | **Nothing.** Never flown. | **None** |

---

## 8. Next steps, in order

1. **Fix the review bugs in §5** — start with `land()`'s truncated budget and the deadman
   false-positive; both affect every single flight.
2. **Write `test_gate_leg.py` and `test_pad.py`** — 647 untested lines of flight sequence is
   the largest single risk in the repo.
3. **Implement frozen-feed detection** (gap #1) — `--case frozen` already exists to test it.
4. **Bench test with `--dry-run`** against a tape measure: `forward` vs measured range, body
   lateral vs measured sideways offset, gate `d` vs measured height difference, `cone_angle`
   vs measured skew, `normal_flipped` vs which way the gate faces. The 0.97155 m square is
   an in-frame scale reference. **This is the only way to validate the camera offsets.**
5. **Confirm the PX4 parameter checklist** on the airframe — read every value off the
   vehicle with `param show`; airframe startup scripts override firmware defaults. Table is
   in plan §10. The dangerous defaults: `COM_LOW_BAT_ACT = 0` (no battery action at all),
   `EKF2_OF_CTRL = 0` (optical flow **off**), `MPC_XY_VEL_MAX = 12 m/s`.
6. **First flight: Phase 1A, one gate, netted/tethered, safety pilot on the RC.**
7. Phase 2 — only after 1A and 1B fly repeatably.

### Before any flight — human safety review

- Every number in the config banner, especially the altitude envelope and speeds
- Camera-offset signs, verified on the bench against a tape measure
- The PX4 parameter checklist
- The commit predicate thresholds — the one place the drone stops trusting vision
- That the RC arm switch reliably kills the mission at every phase
