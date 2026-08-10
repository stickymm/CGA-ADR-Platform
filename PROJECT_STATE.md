# PROJECT STATE — Autonomous Gate-Course Navigation

**Last updated:** 2026-08-10 · **Branch:** `Gabe-Claude` · **Last commit:** `94be44a`
**Status:** Phase 1A, 1B and 2 implemented and offline-validated. **Never flown.**

This file is the single handoff document. A fresh session should be able to resume from
this alone.

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
| **1A** | Take off, find one gate, cross it, land | Implemented, offline-validated |
| **1B** | Same, looped over N gates with a full stop between each | Implemented, offline-validated |
| **2** | Optimized course: association, vector planning, arc transitions | Implemented, offline-validated |

Phase 1B remains the trustworthy fallback, and Phase 2 is built so that its **worst case
is 1B's normal case**: every path that cannot plan a transition becomes a 1B leg and says
so in the log.

---

## 2. File inventory

### Modified

| File | Lines | Responsibility |
|---|---|---|
| `navigation/navigation.py` | ~2250 | MAVLink I/O, links, `VehicleState`, setpoint streamer, `move_to_target`, `slew_yaw`, `land`, `GateMission` UDP listener. |

### Phase 1 stack (`navigation/missions/`)

| File | Responsibility |
|---|---|
| `contracts.py` | Every dataclass/enum crossing a seam. No I/O. Also the tolerance/timeout/deadline consistency arithmetic. |
| `frames.py` | **Pure geometry, zero I/O.** Circular mean, normal-flip resolution, altitude clamp, plane-crossing, `localize_gate`, `localize_gates`, `evaluate_commit`. |
| `gate_leg.py` | **THE shared routine.** `approach_and_cross_one_gate()`. 1A, 1B and 2 all call it unchanged. |
| `pad.py` | Link, telemetry rates, vision check, banner, operator confirm, arm wait, OFFBOARD verify, takeoff. |
| `cli.py` | Shared argparse surface + object construction. |
| `phase1a_single_gate.py` | Runnable mission. Linear sequence, one `_abort`, one `gate_leg` call. |
| `phase1b_course.py` | Runnable mission. Pad + loop, whole-gate retries, consecutive-failure cap. |
| `offline_backend.py` | `OfflineNavigationController` — kinematic model for `--offline`, optional attitude model. |
| `udp_injector.py` | Synthetic vision publisher. Real wire format, measurement noise, **17 cases**. |

### Phase 2 stack (new)

| File | Responsibility |
|---|---|
| `association.py` | **Pure.** `GateTracker` — nearest-neighbour association with an age-growing gating radius, an ambiguity veto, and confidence = evidence × freshness. |
| `course.py` | **Pure.** `PassVector`, `minimum_turn_radius_m`, `plan_turn` (arc tangent to two gate vectors), `plan_leg` (the degrade decision), `order_gates_along_course`. |
| `vector_leg.py` | Flies a planned arc. Same control loop as the crossing leg. Never re-plans. |
| `phase2_course.py` | Runnable mission. Track → plan → fly → degrade-and-log. |

### Unchanged legacy (kept as reference)

`navigation/missions/single_gate.py`, `multi_stage_gate.py` — byte-identical.
`plan_standoff_horizon` / `limit_target_step` are **imported** by `gate_leg.py`, never copied.
`navigation.py`, `navigation_Vmanual.py`, `navigation_closed_loop.py` at the repo root are
**dead prototypes**, shadowed by the `navigation/` package. Not in scope.

### Tests (`tests/`) — 269 tests, all green

| File | Tests | Covers |
|---|---|---|
| `test_frames_and_rotation.py` | 39 | 3-2-1 rotation (incl. roll+pitch cross terms, length preservation), circular mean, normal flip, cone/offset, altitude envelope, crossing, commit gate |
| `test_navigation_regressions.py` | 43 | One per bug fixed 2026-08-10, plus the links, frozen feed, estimator, state history, multi-gate capture |
| `test_course_planning.py` | 38 | Turn geometry against hand-derived values, radius bounds, every refusal, every degrade decision |
| `test_phase2_course.py` | 30 | `exit_leg`/`expected_gate` seams, vector leg, next-gate selection, mission loop |
| `test_pad.py` | 26 | Climb invariant, ordering constraints, every refusal-to-fly, takeoff, detection-wait states |
| `test_gate_leg.py` | 25 | Commit/cross, approach/align, all three recovery rungs, every abort |
| `test_association.py` | 22 | Row-swap immunity, gating radius, ambiguity veto, confidence, track lifecycle |
| `test_move_timeout_and_setpoints.py` | 19 | `estimate_move_timeout`, `type_mask_for`, `is_transient`, `is_valid_detection` |
| `test_phase1b_course.py` | 14 | Course loop policy with per-gate work scripted out |
| `test_navigation_geometry.py` | 6 | Pre-existing |
| `test_single_gate_mission.py` | 4 | Pre-existing |
| `test_flask_streaming.py` | 3 | Pre-existing, unrelated |
| `support.py` | — | Shared doubles: virtual clock, `FakeNav`, `ScriptedMission` |

---

## 3. Conventions in force

**Frames.** body FRD (`+x` forward, `+y` right, `+z` down) → local NED. `d` is **down**, so
`d = -1.5` is 1.5 m *above* the origin.

**Units.** Metres, radians — unless the name ends `_deg`.

**Vision wire format** (UDP `127.0.0.1:5050`, JSON):
`{"fps": float, "gates": [[dist, forward, right, down, roll, pitch, yaw], ×3]}` —
always exactly 3 rows, sorted nearest-first, absent gates padded all-`999.0`.
**Row index is not a stable gate identity.** Gate model is a 0.97155 m square.

**Ownership rule:** the mission file owns the **sequence**; `gate_leg` owns one **step**.

**Returns are named, not boolean.** `GateResult` enum, not `bool`.

**Every mission exit lands.** Success, failure, deadline, exception, Ctrl-C.

**Lock order:** `_vehicle_lock → _setpoint_lock → _master_lock`. Never hold
`_setpoint_lock` across a transmit.

**One seam out.** Every outbound byte goes through a `Link`. Nothing tests `self.dry_run`
before transmitting — a test asserts this at source level.

**One rotation matrix.** `navigation.body_to_local`. `frames.rotate_body_to_ned` aliases it.

---

## 4. What was fixed on 2026-08-10 (commits `8348cda`, `d7c9ede`, `5ae3b74`, `94be44a`)

### Review bugs — all fixed, each with a named regression test

| Sev | Bug | Fix |
|---|---|---|
| HIGH | `land()` used ~15 s of its 40 s budget and printed "NOT confirmed" on good landings; 1A exited 1 | Loop bounded by the deadline only. LAND re-issued every 5 s *inside* it. Pre-arm abort no longer claims a landing; dry-run returns immediately. |
| HIGH | Deadman false-fired on the pad every run | Alarm gated on `armed AND in_offboard`. Below that the prime is re-stamped and announced once as expected quiescence. In-flight behaviour unchanged. |
| MED | `observe_gate` interleaved position and velocity masks; vertical drift every window | Vestigial 25 Hz velocity send deleted. One setpoint kind for the window. |
| MED | Only `HEARTBEAT` filtered by source | `_is_autopilot` gates every message, sysid **and** compid. |
| MED | Tolerance 0.10 m / floor 15 s / deadline 75 s were mutually inconsistent | 0.15 m / 5.0 s / 90 s; tolerance threaded through `move_to_target` and into its own budget. `contracts.attempt_budget_s` states the relationship; the banner prints it and any inconsistency. |
| LOW | Ctrl-C discarded the per-gate report | `results` hoisted to `main`. |
| LOW | `land()` said "Landed and disarmed" on a pre-arm abort | Now says "no landing required". |
| LOW | Unused `field` import in `pad.py` | Removed. |

### Gaps closed from the old §4 table

1. **Frozen-feed detection** — payload fingerprint over 15 frames, the third staleness
   signal. Excludes the all-999 case (an empty sky is not a frozen feed). `--case frozen`
   is now **refused on the pad**.
2. **Unit tests for `gate_leg`/`pad`** — 51 tests across both.
3. **Yaw-rate limiting** — `slew_yaw` ramps the commanded heading. `turn_around_180` and
   the recovery sweep both use it. No longer relies on `MPC_YAWRAUTO_MAX`.
4. **`ESTIMATOR_STATUS`** — subscribed, folded into `VehicleState`, and `estimator_note()`
   is appended to every offboard-loss message so flow degradation is not read as link loss.
5. **3-2-1 in `navigation.py`** — one matrix, aliased by `frames.py`, wired into the legacy
   `detection_to_gate_local`.
6. **`DryRunLink`** — `PymavlinkLink` / `DryRunLink` / `NullLink` replace five inline branches.
7. **State history / deskew** — 30-sample ring buffer, `state_at(timestamp)`, used by
   `gate_leg` and Phase 2 localization.
8. **`POSITION_TARGET_LOCAL_NED` (85)** — requested at 5 Hz, logged on mask change.
9. **`MissionOutcome`** — returned by `run_mission`, with an `exit_code`.
10. **`GateDetection.row_index` / `source_fps`** — carried through the listener.
11. **`expected_gate=`** — implemented, optional, defaults off.
12. **`exit_leg=`** — implemented, optional, defaults off. This is how Phase 2 degrades.

Plus: **all three gate rows are consumed**, not just `gates[0]`.

---

## 5. Known gaps and open questions

| # | Item | Note |
|---|---|---|
| 1 | **`--dry-run` cannot pass STEP 6** | `request_offboard` cannot confirm a mode it never transmitted, so a bench run aborts before the gate approach. Fixing it means a "pretend OFFBOARD succeeded" branch in a safety path. **Deliberately not done — needs a human decision.** |
| 2 | **`observe_gate` horizontal hold** | The vertical axis is now a true position hold. The horizontal axes remain a zero-*velocity* brake, deliberately: a position setpoint chases optical-flow drift at up to `MPC_XY_VEL_MAX` (12 m/s default). Changing this is a safety decision. |
| 3 | **Takeoff tolerance** | `takeoff_tolerance_m = 0.15` means a 1.50 m target reports success at 1.35 m, and the mission then flies 0.15 m low against a 0.80 m floor. In spec; tightening it is a flight-envelope decision. |
| 4 | **Camera mount offsets** | Still never measured. Zeroed offline. **The single largest unvalidated item.** |
| 5 | **Course ordering is a heuristic** | `order_gates_along_course` chains forward and is stated as a heuristic. Wrong for a course that doubles back through the same volume; there is no way to tell without gate IDs. |
| 6 | **Association identity does not survive a long dropout** | Past `max_gate_radius_m` a returning observation becomes a new track. Geometry survives; identity does not. Documented in `association.py`. |

### What `--offline` does **not** exercise

- **Position drift** — the model is drift-free. Real optical flow drifts and N/E is not
  globally consistent. **Still the single biggest untested risk**, and it is the input
  Phase 2's association and planning both depend on most.
- **Camera mount offsets** — zeroed offline; the offset path is never exercised.
- **PX4 itself** — no controller, no estimator, no failsafes, no ground effect, no latency.
- **Open-loop injector geometry** — the injector emits vehicle-frame poses and has no idea
  where the drone is, so gate positions in NED move as the drone advances. Phase 2's
  association and transition planning therefore see a course that is not world-fixed. This
  is why the offline Phase 2 run degrades to 1B on most legs: the degrade is correct
  behaviour on the synthetic input, not a planning bug.
- **Yaw rate** — now rate-limited in the real code, and the model also limits it, so
  offline still cannot distinguish the two.

---

## 6. How to run

Use the venv: `.venv/bin/python`.

### Tests
```bash
.venv/bin/python -m unittest discover -s tests -v      # 269 tests, all green
.venv/bin/python -m compileall -q navigation scripts vision debug
```

### Offline — laptop only, no drone, no camera (two terminals)
```bash
# Phase 1A
.venv/bin/python -m navigation.missions.udp_injector --case dead-ahead --duration-s 80
.venv/bin/python -m navigation.missions.phase1a_single_gate --offline

# Phase 1B
.venv/bin/python -m navigation.missions.udp_injector --case course --duration-s 200
.venv/bin/python -m navigation.missions.phase1b_course --gates 3 --offline

# Phase 2
.venv/bin/python -m navigation.missions.udp_injector --case multi-gate-course --duration-s 200
.venv/bin/python -m navigation.missions.phase2_course --gates 3 --offline

# Phase 2 forced to Phase 1B behaviour, for an A/B comparison
.venv/bin/python -m navigation.missions.phase2_course --gates 3 --offline --no-planning

# Exercise the full 3-2-1 rotation (the injector cannot compensate; that is the point)
.venv/bin/python -m navigation.missions.phase1a_single_gate --offline --offline-tilt

# Frozen-feed detection: this MUST be refused on the pad
.venv/bin/python -m navigation.missions.udp_injector --case frozen --duration-s 40
.venv/bin/python -m navigation.missions.phase1a_single_gate --offline
```

Injector cases: `.venv/bin/python -m navigation.missions.udp_injector --list`
→ `dead-ahead`, `offset-left`, `offset-right`, `above`, `below`, `normal-as-reported`,
`normal-reversed`, `edge-on`, `multiple-gates`, `dropout`, `frozen`, `course`,
`multi-gate-course`, `row-swap`, `close-gates`, `association-dropout`, `non-level`.

> The injector binds UDP 5050 — the same port `debug/telemetry_reader.py` uses. **One
> publisher at a time**: two injectors running at once interleave, and no case behaves
> as documented.

### Dry run — real vehicle, real telemetry, **transmits nothing** (props off, bench)
```bash
.venv/bin/python -m navigation.missions.phase1a_single_gate --dry-run
```
Note gap 5.1: this currently stops at STEP 6.

### Real flight
```bash
.venv/bin/python -m navigation.missions.phase1a_single_gate
.venv/bin/python -m navigation.missions.phase1b_course --gates 3
.venv/bin/python -m navigation.missions.phase2_course --gates 3
```

`--offline` and `--dry-run` are mutually exclusive. Every flag is in `cli.py` (plus the
Phase 2 groups in `phase2_course.py`) and is printed in the pre-flight banner.

---

## 7. Validation status

| Claim | Validated by | Confidence |
|---|---|---|
| Geometry: rotation (incl. roll+pitch), circular mean, normal flip, clamp, crossing, commit | 39 unit tests against **hand-derived arithmetic** | **High** |
| Turn planning: radius bounds, tangent points, arc, heading profile, every refusal | 38 unit tests against hand-derived arithmetic | **High** |
| Association: row-swap immunity, gating, ambiguity veto, confidence, lifecycle | 22 unit tests | **High** |
| Move-timeout arithmetic; `type_mask` values; detection validity | 19 unit tests | **High** |
| `gate_leg` state machine and recovery ladder | 25 unit tests, every branch | **High** |
| Pad sequence, ordering constraints, refusals, takeoff | 26 unit tests | **High** |
| The 2026-08-10 bug fixes | 43 regression tests | **High** |
| Course loop policy (1B and 2): retries, failure counter, abort, always-lands | 44 unit tests | **High** |
| Phase 1A/1B/2 end-to-end sequencing | Instrumented `--offline` runs, 2026-08-10 | **Medium** — logic only |
| Camera mount offsets / sign conventions | **Nothing.** Zeroed offline; never measured. | **None** |
| Anything about the aircraft — control, estimator, failsafes, flow drift | **Nothing.** Never flown. | **None** |

---

## 8. Next steps, in order

1. **Bench test with `--dry-run` against a tape measure** — `forward` vs measured range,
   body lateral vs measured sideways offset, gate `d` vs measured height difference,
   `cone_angle` vs measured skew, `normal_flipped` vs which way the gate faces. The
   0.97155 m square is an in-frame scale reference. **This is the only way to validate the
   camera offsets.** Decide gap 5.1 first, or the run stops at STEP 6.
2. **Confirm the PX4 parameter checklist** on the airframe — read every value off the
   vehicle with `param show`; airframe startup scripts override firmware defaults. The
   dangerous defaults: `COM_LOW_BAT_ACT = 0` (no battery action at all),
   `EKF2_OF_CTRL = 0` (optical flow **off**), `MPC_XY_VEL_MAX = 12 m/s`.
3. **First flight: Phase 1A, one gate, netted/tethered, safety pilot on the RC.**
4. **Phase 1B, repeatably.**
5. **Phase 2 — only after 1A and 1B fly repeatably.** Fly it first with `--no-planning`,
   which is 1B behaviour through the Phase 2 code path, then enable planning.

### Before any flight — human safety review

- Every number in the config banner, especially the altitude envelope and speeds
- **The `attempt budget` banner row** — if it prints `!! INCONSISTENT`, the flags are
  mis-tuned and the advertised attempt count is not reachable
- Camera-offset signs, verified on the bench against a tape measure
- The PX4 parameter checklist
- The commit predicate thresholds — the one place the drone stops trusting vision
- **Phase 2 only:** the yaw-rate and lateral-acceleration limits, the transition lead-in,
  and the confidence thresholds. These decide how tight a turn the aircraft will attempt
  between two gates, and nothing about them has been validated in the air.
- That the RC arm switch reliably kills the mission at every phase
