"""PHASE 2 -- optimized course: track every gate, plan the turn, fly it.

WHAT THIS DOES THAT PHASE 1B DOES NOT
    1B flies gate, full stop, re-acquire, gate.  Phase 2 keeps a track on every
    gate it can see, and when it knows where the *next* one is well enough, it
    plans a constant-radius transition off the current gate's exit vector onto
    the next gate's entry vector and flies that instead of stopping.

WHAT IT DOES NOT CHANGE
    Everything that decides whether the aircraft flies at a gate: localization,
    the commit predicate, the recovery ladder, the altitude envelope, the pad
    sequence, and landing.  Those come from the identical shared code 1A and 1B
    use, called through the identical ``approach_and_cross_one_gate``.  Phase 2
    supplies two optional arguments to it -- ``expected_gate`` and ``exit_leg``
    -- and nothing else.

THE FALLBACK IS A FEATURE, NOT AN ERROR PATH
    Phase 2 degrades to Phase 1B behaviour whenever it cannot justify doing
    better, and says so every time:

      * no next gate tracked;
      * next gate tracked but confidence below threshold;
      * next gate's *heading* not trusted (position known, normal not);
      * the turn is infeasible -- too tight for the yaw-rate and tilt limits, or
        no room between the gates for it;
      * the planned transition failed in flight.

    In each case the leg becomes exactly a 1B leg: cross, hold, re-acquire.
    That makes the worst case of Phase 2 equal to the normal case of Phase 1B,
    which is the only basis on which flying it is a reasonable thing to do.

RUN IT

    python -m navigation.missions.udp_injector --case multi-gate-course --duration-s 200
    python -m navigation.missions.phase2_course --gates 3 --offline

    python -m navigation.missions.phase2_course --gates 3 --dry-run
    python -m navigation.missions.phase2_course --gates 3

PRECONDITIONS
    Identical to Phase 1A/1B, and printed in the banner before you confirm.
    Phase 2 should not be flown until 1A and 1B fly repeatably.
"""

import argparse
import signal
import sys
import time
from dataclasses import dataclass
from typing import List, Optional

from .association import AssociationConfig, GateTrack, GateTracker
from .cli import add_common_arguments, build, check_mode_flags
from .contracts import GateLegConfig, GateOutcome, GateResult, summarize_config
from .course import (
    CoursePlanConfig,
    LegPlan,
    PassVector,
    pass_vector,
    plan_leg,
    track_pass_vector,
)
from .frames import localize_gates
from .gate_leg import approach_and_cross_one_gate
from .pad import PadConfig, run_pad_sequence
from .vector_leg import describe_heading_profile, fly_turn

MISSION_TITLE = "PHASE 2 -- OPTIMIZED GATE COURSE"


@dataclass(frozen=True)
class Phase2Config:
    """Course-level bounds.  Per-gate bounds live in :class:`GateLegConfig`."""

    gate_count: int = 3
    gate_retries: int = 1
    max_consecutive_failures: int = 2
    course_timeout_s: float = 600.0
    settle_between_gates_s: float = 1.0
    # Whole-course kill switch: after this many transitions fail, stop planning
    # and fly the rest of the course as Phase 1B. A planner that keeps failing
    # is telling you something about the course, and repeatedly re-attempting a
    # manoeuvre that has not worked is not a recovery strategy.
    max_transition_failures: int = 2

    def __post_init__(self) -> None:
        if self.gate_count < 1:
            raise ValueError("gate_count must be at least 1")
        if self.gate_retries < 0:
            raise ValueError("gate_retries must be non-negative")
        if self.max_consecutive_failures < 1:
            raise ValueError("max_consecutive_failures must be at least 1")
        if self.course_timeout_s <= 0.0:
            raise ValueError("course_timeout_s must be positive")
        if self.max_transition_failures < 0:
            raise ValueError("max_transition_failures must be non-negative")


@dataclass
class CourseState:
    """Mutable course bookkeeping, owned by ``main`` so Ctrl-C keeps the report."""

    results: List[GateOutcome]
    crossed: int = 0
    transitions_flown: int = 0
    transitions_failed: int = 0
    degradations: List[str] = None

    def __post_init__(self):
        if self.degradations is None:
            self.degradations = []


# =========================
# TRACKING
# =========================


def refresh_tracks(nav, mission, tracker: GateTracker, cfg: GateLegConfig, log) -> None:
    """Fold the newest vision packet's gates into the tracker.

    Every gate in the packet, not just the nearest: the whole reason Phase 2 can
    plan a transition is that it saw gate N+1 while it was still flying gate N.
    """
    detections = mission.get_latest_detections_snapshot()
    if not detections:
        return
    if mission.feed_frozen:
        log("[!] Not updating gate tracks: the vision feed is frozen")
        return

    envelope = nav.altitude_envelope or cfg.altitude
    # Deskew against the pose at capture time, not the pose now.
    state = nav.state_at(detections[0].timestamp)
    fixes = localize_gates(
        detections,
        state,
        envelope,
        cam_offset_right_m=mission.cam_offset_right_m,
        cam_offset_down_m=mission.cam_offset_down_m,
        cam_yaw_offset_deg=mission.cam_yaw_offset_deg,
        max_cone_deg=cfg.commit_max_cone_deg,
    )
    result = tracker.update(fixes, time.time())
    if result.created or result.dropped or result.ambiguous:
        log(f"[*] Gate tracks: {result.describe()}")
    for note in result.notes:
        log(f"    {note}")


def choose_next_gate(
    tracker: GateTracker,
    current: Optional[PassVector],
    now_s: float,
    plan_cfg: CoursePlanConfig,
) -> Optional[GateTrack]:
    """The most plausible "gate after this one" among the live tracks.

    Ahead of the current gate along its own vector, confident enough to plan
    against, and nearest of whatever is left.  Returns None rather than a guess:
    None is what triggers the documented, logged fallback to Phase 1B, and a
    guess is what would fly the aircraft at the wrong gate.
    """
    candidates = []
    for track in tracker.tracks:
        if track.confidence(now_s, tracker.cfg) < plan_cfg.min_next_gate_confidence:
            continue
        if current is not None:
            # Must be beyond the current gate's plane; otherwise it is the gate
            # being flown, or one already behind the aircraft.
            if current.along(track.n, track.e) <= plan_cfg.exit_clearance_m:
                continue
        candidates.append(track)

    if not candidates:
        return None
    if current is None:
        return None
    return min(candidates, key=lambda track: current.along(track.n, track.e))


# =========================
# THE MISSION
# =========================


def _abort(nav, reason: str, course: CourseState) -> int:
    """The single exit for every failure.  Brake, pause, land, report."""
    try:
        nav.safe_shutdown(reason)
    except KeyboardInterrupt:
        # A further interrupt during the shutdown must not escape as a
        # traceback, and must not trigger a second abort that re-runs the whole
        # sequence. safe_shutdown has already commanded LAND by this point.
        print(
            "[!] Interrupted during safe shutdown. LAND was already commanded -- "
            "TAKE MANUAL CONTROL AND CHECK THE AIRCRAFT. Not retrying."
        )
    except Exception as exc:
        print(f"[!] safe_shutdown itself failed: {exc}")
    _report(course, reason=reason, ok=False)
    return 1


def _report(course: CourseState, *, reason: str, ok: bool) -> None:
    print("\n" + "=" * 72)
    print(f"[RESULT] {'COURSE COMPLETE' if ok else 'COURSE ENDED EARLY'}: {reason}")
    print("-" * 72)
    if not course.results:
        print("  no gates were attempted")
    for outcome in course.results:
        print(
            f"  gate {outcome.gate_index}: {outcome.result.value:<10} "
            f"attempts={outcome.attempts:<3} reacquires={outcome.reacquires:<3} "
            f"{outcome.elapsed_s:6.1f}s  {outcome.reason}"
        )
        if outcome.result is not GateResult.CROSSED and outcome.fix is not None:
            fix = outcome.fix
            print(
                f"            last known gate pose: N={fix.n:+.2f} E={fix.e:+.2f} "
                f"D={fix.d:+.2f} range={fix.range_m:.2f}m"
            )
    print("-" * 72)
    print(
        f"  crossed {course.crossed} | transitions flown "
        f"{course.transitions_flown}, failed {course.transitions_failed}"
    )
    if course.degradations:
        print("  DEGRADED TO PHASE 1B:")
        for note in course.degradations:
            print(f"    - {note}")
    else:
        print("  no degradations: every transition was planned and flown")
    print("=" * 72)


def run(
    nav,
    mission,
    leg_cfg: GateLegConfig,
    pad_cfg: PadConfig,
    course_cfg: Phase2Config,
    plan_cfg: CoursePlanConfig,
    assoc_cfg: AssociationConfig,
    course: CourseState,
) -> int:
    """The whole course, in order.  Returns a process exit code."""

    started_s = time.time()
    tracker = GateTracker(assoc_cfg)
    consecutive_failures = 0
    planning_enabled = True

    # ---- 1. pad sequence: link, telemetry, vision, stream, confirm, arm, climb
    pad = run_pad_sequence(
        nav,
        mission,
        pad_cfg,
        banner_rows=list(summarize_config(leg_cfg)) + [
            ("course gates", f"{course_cfg.gate_count}"),
            ("retries per gate", f"{course_cfg.gate_retries}"),
            ("give up after", f"{course_cfg.max_consecutive_failures} consecutive failures"),
            ("course timeout", f"{course_cfg.course_timeout_s:.0f} s"),
            ("cruise speed", f"{plan_cfg.cruise_speed_m_s:.2f} m/s"),
            ("turn speed floor", f"{plan_cfg.min_turn_speed_m_s:.2f} m/s"),
            ("max yaw rate", f"{plan_cfg.max_yaw_rate_deg_s:.0f} deg/s"),
            ("max lateral accel", f"{plan_cfg.max_lateral_accel_m_s2:.2f} m/s^2"),
            ("transition lead-in", f"{plan_cfg.entry_lead_in_m:.2f} m"),
            ("next-gate confidence", f"{plan_cfg.min_next_gate_confidence:.2f}"),
            ("association radius", f"{assoc_cfg.base_gate_radius_m:.2f} m "
                                   f"+{assoc_cfg.radius_growth_m_per_s:.2f}/s "
                                   f"(max {assoc_cfg.max_gate_radius_m:.2f} m)"),
            ("ambiguity margin", f"{assoc_cfg.ambiguity_margin_m:.2f} m"),
            ("fallback", "Phase 1B stop-and-acquire, logged every time"),
        ],
        title=MISSION_TITLE,
    )
    if not pad.ok:
        return _abort(nav, f"pad sequence failed: {pad.reason}", course)

    envelope = pad.envelope or leg_cfg.altitude

    # ---- 2. gate by gate, planning the transition out of each ----------------
    for gate_index in range(course_cfg.gate_count):
        if time.time() - started_s > course_cfg.course_timeout_s:
            return _abort(
                nav,
                f"course timeout of {course_cfg.course_timeout_s:.0f}s exceeded "
                f"at gate {gate_index}",
                course,
            )

        print("\n" + "=" * 72)
        print(f"  GATE {gate_index + 1} OF {course_cfg.gate_count}")
        print("=" * 72)

        refresh_tracks(nav, mission, tracker, leg_cfg, print)

        # The exit leg is built per gate. It closes over the tracker so that the
        # decision of where to go next is made with the freshest fix of the gate
        # being crossed -- which is the last moment vision is trusted.
        exit_leg = None
        if planning_enabled:
            exit_leg = _build_exit_leg(
                nav, mission, tracker, leg_cfg, plan_cfg, envelope, course, gate_index
            )

        outcome = _attempt_gate(
            nav, mission, leg_cfg, course_cfg, gate_index, course, exit_leg
        )

        if outcome.result is GateResult.CROSSED:
            course.crossed += 1
            consecutive_failures = 0
            if course.transitions_failed >= course_cfg.max_transition_failures > 0:
                if planning_enabled:
                    planning_enabled = False
                    note = (
                        f"{course.transitions_failed} transitions failed; planning "
                        f"disabled for the rest of the course, flying Phase 1B"
                    )
                    print(f"[!] {note}")
                    course.degradations.append(note)
            if "exit leg flown" not in outcome.reason:
                # No transition was flown out of this gate, so this is a 1B leg:
                # stop, settle, and re-acquire from a stable pose.
                nav.hold_position(label=f"settled after gate {gate_index}")
                _settle(nav, course_cfg.settle_between_gates_s)
            continue

        if outcome.result is GateResult.ABORTED:
            return _abort(nav, f"gate {gate_index} aborted: {outcome.reason}", course)

        consecutive_failures += 1
        print(
            f"[!] Gate {gate_index} failed ({outcome.result.value}). "
            f"{consecutive_failures} consecutive failure(s)."
        )
        if consecutive_failures >= course_cfg.max_consecutive_failures:
            return _abort(
                nav,
                f"{consecutive_failures} consecutive gate failures, "
                f"last at gate {gate_index}: {outcome.reason}",
                course,
            )

    # ---- 3. land -------------------------------------------------------------
    print(f"\n[*] Reached the end of the course with {course.crossed}/"
          f"{course_cfg.gate_count} gates crossed; landing")
    landed = nav.land()

    _report(
        course,
        reason=f"{course.crossed}/{course_cfg.gate_count} gates in "
               f"{time.time() - started_s:.1f}s"
               + ("" if landed else " -- LANDING NOT CONFIRMED, CHECK THE AIRCRAFT"),
        ok=landed and course.crossed == course_cfg.gate_count,
    )
    return 0 if (landed and course.crossed == course_cfg.gate_count) else 1


def _build_exit_leg(
    nav, mission, tracker, leg_cfg, plan_cfg, envelope, course, gate_index
):
    """Build the ``exit_leg`` callback handed to ``approach_and_cross_one_gate``.

    Called with the frozen fix the crossing was flown on, immediately after the
    gate plane is cleared.  Returning False means "the transition did not
    happen"; the gate is still crossed, and the mission settles instead.
    """

    def exit_leg(controller, fix) -> bool:
        refresh_tracks(controller, mission, tracker, leg_cfg, print)
        current = pass_vector(fix)
        now = time.time()

        next_track = choose_next_gate(tracker, current, now, plan_cfg)
        plan: LegPlan = plan_leg(
            current,
            envelope,
            plan_cfg,
            next_gate=track_pass_vector(next_track) if next_track else None,
            next_confidence=(
                next_track.confidence(now, tracker.cfg) if next_track else 0.0
            ),
            next_heading_confidence=(
                next_track.heading_confidence if next_track else 0.0
            ),
            pass_distance_m=leg_cfg.pass_distance_m,
        )

        if plan.degraded:
            note = f"gate {gate_index}: {plan.degrade_reason}"
            print(f"[*] PHASE 1B FALLBACK -- {note}")
            course.degradations.append(note)
            return False

        print(f"[*] Transition out of gate {gate_index}: {plan.turn.describe()}")
        print(describe_heading_profile(plan.turn))

        flown = fly_turn(
            controller,
            plan.turn,
            envelope,
            plan_cfg,
            label=f"gate {gate_index} -> {gate_index + 1} transition",
        )
        if flown:
            course.transitions_flown += 1
        else:
            course.transitions_failed += 1
            note = f"gate {gate_index}: planned transition did not complete in flight"
            course.degradations.append(note)
        return flown

    return exit_leg


def _attempt_gate(
    nav, mission, leg_cfg, course_cfg, gate_index, course, exit_leg
) -> GateOutcome:
    """Try one gate, with whole-gate retries on top of gate_leg's own ladder."""
    outcome = None
    for attempt in range(course_cfg.gate_retries + 1):
        if attempt:
            print(f"[*] Retrying gate {gate_index} ({attempt}/{course_cfg.gate_retries})")
        outcome = approach_and_cross_one_gate(
            nav, mission, leg_cfg, gate_index=gate_index, exit_leg=exit_leg
        )
        course.results.append(outcome)
        if outcome.result in (GateResult.CROSSED, GateResult.ABORTED):
            return outcome
    return outcome


def _settle(nav, duration_s: float) -> None:
    """Hold still so the next acquisition starts from a stable pose."""
    deadline = time.time() + duration_s
    while nav.running and time.time() < deadline:
        time.sleep(0.05)


# =========================
# CLI
# =========================


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Phase 2: fly a course of N gates with planned transitions.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    course = parser.add_argument_group("course")
    course.add_argument("--gates", type=int, default=3)
    course.add_argument("--gate-retries", type=int, default=1)
    course.add_argument("--max-consecutive-failures", type=int, default=2)
    course.add_argument("--course-timeout-s", type=float, default=600.0)
    course.add_argument("--settle-between-gates-s", type=float, default=1.0)
    course.add_argument("--max-transition-failures", type=int, default=2)

    planning = parser.add_argument_group("phase 2 planning")
    planning.add_argument("--cruise-speed-m-s", type=float, default=0.45)
    planning.add_argument("--min-turn-speed-m-s", type=float, default=0.25)
    planning.add_argument("--max-lateral-accel-m-s2", type=float, default=1.47,
                          help="0.15 g; a multirotor makes lateral accel by tilting, "
                               "and tilt swings the nose-mounted camera off the gate")
    planning.add_argument("--entry-lead-in-m", type=float, default=1.0,
                          help="straight run into a gate before the commit decision")
    planning.add_argument("--min-next-gate-confidence", type=float, default=0.5)
    planning.add_argument("--min-heading-confidence", type=float, default=0.5)
    planning.add_argument(
        "--no-planning",
        action="store_true",
        help="track gates but never plan a transition; flies exactly Phase 1B",
    )

    association = parser.add_argument_group("association")
    association.add_argument("--assoc-radius-m", type=float, default=0.6)
    association.add_argument("--assoc-radius-growth-m-s", type=float, default=0.25)
    association.add_argument("--assoc-max-radius-m", type=float, default=2.0)
    association.add_argument("--assoc-ambiguity-margin-m", type=float, default=0.5)
    association.add_argument("--assoc-track-age-s", type=float, default=5.0)

    add_common_arguments(parser)
    return parser.parse_args(argv)


def build_plan_config(args) -> CoursePlanConfig:
    return CoursePlanConfig(
        cruise_speed_m_s=args.cruise_speed_m_s,
        min_turn_speed_m_s=args.min_turn_speed_m_s,
        max_yaw_rate_deg_s=args.max_yaw_rate_deg_s,
        max_lateral_accel_m_s2=args.max_lateral_accel_m_s2,
        exit_clearance_m=args.exit_clearance_m,
        entry_lead_in_m=args.entry_lead_in_m,
        min_next_gate_confidence=args.min_next_gate_confidence,
        min_heading_confidence=args.min_heading_confidence,
    )


def build_association_config(args) -> AssociationConfig:
    return AssociationConfig(
        base_gate_radius_m=args.assoc_radius_m,
        radius_growth_m_per_s=args.assoc_radius_growth_m_s,
        max_gate_radius_m=args.assoc_max_radius_m,
        ambiguity_margin_m=args.assoc_ambiguity_margin_m,
        max_track_age_s=args.assoc_track_age_s,
    )


def main(argv=None) -> int:
    args = parse_args(argv)
    if not check_mode_flags(args):
        return 2

    nav, mission, leg_cfg, pad_cfg = build(args)
    course_cfg = Phase2Config(
        gate_count=args.gates,
        gate_retries=args.gate_retries,
        max_consecutive_failures=args.max_consecutive_failures,
        course_timeout_s=args.course_timeout_s,
        settle_between_gates_s=args.settle_between_gates_s,
        max_transition_failures=0 if args.no_planning else args.max_transition_failures,
    )
    plan_cfg = build_plan_config(args)
    assoc_cfg = build_association_config(args)

    # Owned here so an interrupt still reports which gates were flown, and how
    # many transitions degraded.
    course = CourseState(results=[])

    def on_signal(signum, _frame):
        print(f"\n[!] Signal {signum} received -- shutting down safely")
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    if args.no_planning:
        print("[*] --no-planning: transitions disabled; this run is Phase 1B behaviour")

    mission.start()
    try:
        if args.no_planning:
            plan_cfg = CoursePlanConfig(
                cruise_speed_m_s=plan_cfg.cruise_speed_m_s,
                min_turn_speed_m_s=plan_cfg.min_turn_speed_m_s,
                max_yaw_rate_deg_s=plan_cfg.max_yaw_rate_deg_s,
                max_lateral_accel_m_s2=plan_cfg.max_lateral_accel_m_s2,
                exit_clearance_m=plan_cfg.exit_clearance_m,
                entry_lead_in_m=plan_cfg.entry_lead_in_m,
                # Unreachable confidence: every leg degrades, and says so.
                min_next_gate_confidence=2.0,
                min_heading_confidence=2.0,
            )
        return run(
            nav, mission, leg_cfg, pad_cfg, course_cfg, plan_cfg, assoc_cfg, course
        )
    except KeyboardInterrupt:
        return _abort(nav, "operator interrupt", course)
    except Exception as exc:  # never leave the aircraft flying because of a bug here
        return _abort(nav, f"unhandled error: {exc!r}", course)
    finally:
        mission.stop()
        nav.stop()


if __name__ == "__main__":
    sys.exit(main())
