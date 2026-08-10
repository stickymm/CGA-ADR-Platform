"""PHASE 1B -- barebones N-gate course.  Deliberately unoptimized.

This is the reliable fallback: the mission to fly if the optimized Phase 2
version misbehaves, and the reference its behaviour gets checked against.  It is
Phase 1A's approach repeated gate by gate, with a full stop and a fresh
acquisition between every one.  No trajectory smoothing, no turn anticipation,
no lookahead to the next gate -- all of that is Phase 2's job, and keeping it out
of here is what makes this version trustworthy.

SHARED, NOT COPIED
    The per-gate work is ``approach_and_cross_one_gate``, called unchanged --
    the identical function Phase 1A uses.  Only the loop around it is new, so
    there is exactly one implementation of "cross a gate" in the codebase and
    fixing it fixes every phase at once.

RUN IT

    python -m navigation.missions.udp_injector --case course --duration-s 200
    python -m navigation.missions.phase1b_course --gates 3 --offline

    python -m navigation.missions.phase1b_course --gates 4 --dry-run
    python -m navigation.missions.phase1b_course --gates 4

RECOVERY POLICY
    Per gate, the ladder inside ``gate_leg`` runs first: re-observe, then a
    bounded yaw sweep, then back off and look again.  If a gate is still not
    crossed, this loop retries the whole gate up to ``--gate-retries`` times.
    ``--max-consecutive-failures`` gates in a row failing ends the course.
    Every exit path lands -- including the deadline, the give-up, an exception,
    and Ctrl-C.
"""

import argparse
import signal
import sys
import time
from dataclasses import dataclass
from typing import List

from .cli import add_common_arguments, build, check_mode_flags
from .contracts import GateLegConfig, GateOutcome, GateResult, summarize_config
from .gate_leg import approach_and_cross_one_gate
from .pad import PadConfig, run_pad_sequence

MISSION_TITLE = "PHASE 1B -- GATE COURSE (barebones fallback)"


@dataclass(frozen=True)
class CourseConfig:
    """Course-level bounds.  Per-gate bounds live in :class:`GateLegConfig`."""

    gate_count: int = 3
    gate_retries: int = 1
    max_consecutive_failures: int = 2
    course_timeout_s: float = 600.0
    settle_between_gates_s: float = 1.0

    def __post_init__(self) -> None:
        if self.gate_count < 1:
            raise ValueError("gate_count must be at least 1")
        if self.gate_retries < 0:
            raise ValueError("gate_retries must be non-negative")
        if self.max_consecutive_failures < 1:
            raise ValueError("max_consecutive_failures must be at least 1")
        if self.course_timeout_s <= 0.0:
            raise ValueError("course_timeout_s must be positive")


def _abort(nav, reason: str, results: List[GateOutcome], crossed: int) -> int:
    """The single exit for every failure.  Brake, pause, land, report."""
    try:
        nav.safe_shutdown(reason)
    except Exception as exc:
        print(f"[!] safe_shutdown itself failed: {exc}")
    _report(results, crossed, reason=reason, ok=False)
    return 1


def _report(results: List[GateOutcome], crossed: int, *, reason: str, ok: bool) -> None:
    """Per-gate outcomes, so a failed course says exactly which gate failed."""
    print("\n" + "=" * 68)
    print(f"[RESULT] {'COURSE COMPLETE' if ok else 'COURSE ENDED EARLY'}: {reason}")
    print("-" * 68)
    if not results:
        print("  no gates were attempted")
    for outcome in results:
        line = (
            f"  gate {outcome.gate_index}: {outcome.result.value:<10} "
            f"attempts={outcome.attempts:<3} reacquires={outcome.reacquires:<3} "
            f"{outcome.elapsed_s:6.1f}s  {outcome.reason}"
        )
        print(line)
        if outcome.result is not GateResult.CROSSED and outcome.fix is not None:
            fix = outcome.fix
            print(
                f"            last known gate pose: N={fix.n:+.2f} E={fix.e:+.2f} "
                f"D={fix.d:+.2f} heading={fix.yaw_rad:+.3f}rad "
                f"range={fix.range_m:.2f}m"
            )
    print("-" * 68)
    print(f"  crossed {crossed} of {len({o.gate_index for o in results}) or crossed} attempted")
    print("=" * 68)


def run(
    nav,
    mission,
    leg_cfg: GateLegConfig,
    pad_cfg: PadConfig,
    course_cfg: CourseConfig,
) -> int:
    """The whole course, in order.  Returns a process exit code."""

    started_s = time.time()
    results: List[GateOutcome] = []
    crossed = 0
    consecutive_failures = 0

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
        ],
        title=MISSION_TITLE,
    )
    if not pad.ok:
        return _abort(nav, f"pad sequence failed: {pad.reason}", results, crossed)

    # ---- 2. one gate at a time, stopping and re-acquiring between each --------
    for gate_index in range(course_cfg.gate_count):
        if time.time() - started_s > course_cfg.course_timeout_s:
            return _abort(
                nav,
                f"course timeout of {course_cfg.course_timeout_s:.0f}s exceeded "
                f"at gate {gate_index}",
                results,
                crossed,
            )

        print("\n" + "=" * 68)
        print(f"  GATE {gate_index + 1} OF {course_cfg.gate_count}")
        print("=" * 68)

        outcome = _attempt_gate(nav, mission, leg_cfg, course_cfg, gate_index, results)

        if outcome.result is GateResult.CROSSED:
            crossed += 1
            consecutive_failures = 0
            # Full stop and settle before looking for the next gate. Phase 2
            # replaces exactly this with a planned transition; keeping it dumb
            # here is the point of having a fallback.
            nav.hold_position(label=f"settled after gate {gate_index}")
            _settle(nav, course_cfg.settle_between_gates_s)
            continue

        if outcome.result is GateResult.ABORTED:
            return _abort(
                nav, f"gate {gate_index} aborted: {outcome.reason}", results, crossed
            )

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
                results,
                crossed,
            )

    # ---- 3. land -------------------------------------------------------------
    if crossed == course_cfg.gate_count:
        print(f"\n[*] All {crossed} gates crossed; landing")
    else:
        print(
            f"\n[*] Reached the end of the course with {crossed}/"
            f"{course_cfg.gate_count} gates crossed; landing"
        )
    landed = nav.land()

    # ---- 4. report -----------------------------------------------------------
    _report(
        results,
        crossed,
        reason=f"{crossed}/{course_cfg.gate_count} gates in {time.time() - started_s:.1f}s"
               + ("" if landed else " -- LANDING NOT CONFIRMED, CHECK THE AIRCRAFT"),
        ok=landed and crossed == course_cfg.gate_count,
    )
    return 0 if (landed and crossed == course_cfg.gate_count) else 1


def _attempt_gate(
    nav,
    mission,
    leg_cfg: GateLegConfig,
    course_cfg: CourseConfig,
    gate_index: int,
    results: List[GateOutcome],
) -> GateOutcome:
    """Try one gate, with whole-gate retries on top of gate_leg's own ladder."""
    outcome = None
    for attempt in range(course_cfg.gate_retries + 1):
        if attempt:
            print(f"[*] Retrying gate {gate_index} ({attempt}/{course_cfg.gate_retries})")
        outcome = approach_and_cross_one_gate(
            nav, mission, leg_cfg, gate_index=gate_index
        )
        results.append(outcome)
        if outcome.result in (GateResult.CROSSED, GateResult.ABORTED):
            return outcome
    return outcome


def _settle(nav, duration_s: float) -> None:
    """Hold still between gates so the next acquisition starts from a stable pose."""
    deadline = time.time() + duration_s
    while nav.running and time.time() < deadline:
        time.sleep(0.05)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Phase 1B: fly a course of N gates, one at a time.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    course = parser.add_argument_group("course")
    course.add_argument("--gates", type=int, default=3, help="number of gates to fly")
    course.add_argument("--gate-retries", type=int, default=1,
                        help="whole-gate retries after gate_leg's own recovery ladder")
    course.add_argument("--max-consecutive-failures", type=int, default=2)
    course.add_argument("--course-timeout-s", type=float, default=600.0)
    course.add_argument("--settle-between-gates-s", type=float, default=1.0)

    add_common_arguments(parser)
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    if not check_mode_flags(args):
        return 2

    nav, mission, leg_cfg, pad_cfg = build(args)
    course_cfg = CourseConfig(
        gate_count=args.gates,
        gate_retries=args.gate_retries,
        max_consecutive_failures=args.max_consecutive_failures,
        course_timeout_s=args.course_timeout_s,
        settle_between_gates_s=args.settle_between_gates_s,
    )

    def on_signal(signum, _frame):
        # Raise into the main thread so the finally block runs, which is what
        # lands the aircraft. Safe because the setpoint streamer is a separate
        # daemon thread that keeps publishing through the unwind, degrading to a
        # position hold rather than leaving a velocity latched.
        print(f"\n[!] Signal {signum} received -- shutting down safely")
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    mission.start()
    try:
        return run(nav, mission, leg_cfg, pad_cfg, course_cfg)
    except KeyboardInterrupt:
        return _abort(nav, "operator interrupt", [], 0)
    except Exception as exc:  # never leave the aircraft flying because of a bug here
        return _abort(nav, f"unhandled error: {exc!r}", [], 0)
    finally:
        mission.stop()
        nav.stop()


if __name__ == "__main__":
    sys.exit(main())
