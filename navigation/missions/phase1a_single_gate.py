"""PHASE 1A -- barebones single gate: take off, find one gate, cross it, land.

This is the first thing that will ever run on the real aircraft, so it is
written to be read top to bottom by someone standing at a test site with a
laptop in one hand.  The whole mission is a numbered sequence in :func:`run`,
every abort goes through the single ``_abort`` helper below it, and the only
piece delegated elsewhere is one call to ``approach_and_cross_one_gate``.

Deliberately NOT here: receding-horizon replanning, multi-gate logic, vector
planning, or any dependency on a second gate existing.  Those are Phase 1B and
Phase 2.  This one does the simplest thing that can fly.

RUN IT

    # laptop only, no drone, against the synthetic injector (two terminals)
    python -m navigation.missions.udp_injector --case dead-ahead
    python -m navigation.missions.phase1a_single_gate --offline

    # bench, props off, armed, real vision -- prints commands, transmits nothing
    python -m navigation.missions.phase1a_single_gate --dry-run

    # for real
    python -m navigation.missions.phase1a_single_gate

PRECONDITIONS (also printed in the banner before you are asked to confirm)
    * the vision process is already running and publishing to UDP 127.0.0.1:5050
    * the drone is DISARMED, props fitted, on level ground, clear of the gate
    * the PX4 parameter checklist has been confirmed on this airframe
    * a safety pilot has the RC with mode switch and kill switch ready
"""

import argparse
import signal
import sys
import time

from .cli import add_common_arguments, build, check_mode_flags
from .contracts import GateLegConfig, GateResult, summarize_config
from .gate_leg import approach_and_cross_one_gate
from .pad import PadConfig, run_pad_sequence

# CODE-LOAD CANARY. Fires the instant this module finishes importing --
# before argparse, before any mission logic. If you do NOT see this line,
# Python never successfully loaded this file: the problem is the environment
# (git pull, venv, working directory, `python -m` invocation), not anything
# below this point. If you DO see it, everything downstream is real code
# running, and `__file__` tells you exactly which copy of it.
print(f"[CODE-CHECK] phase1a_single_gate.py loaded OK from: {__file__}")

MISSION_TITLE = "PHASE 1A -- SINGLE GATE"


def _abort(nav, reason: str) -> int:
    """The single exit for every failure.  Brake, pause, land, report.

    One function so that a reader can find every abort path by looking at its
    call sites, and so that "what happens when it goes wrong" is one behaviour
    rather than one per branch.
    """
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
    print(f"\n[RESULT] MISSION ABORTED: {reason}")
    return 1


def run(nav, mission, leg_cfg: GateLegConfig, pad_cfg: PadConfig, mission_deadline_s: float) -> int:
    """The whole mission, in order.  Returns a process exit code."""

    # ---- 1. pad sequence: link, telemetry, vision, stream, confirm, arm, climb
    pad = run_pad_sequence(
        nav,
        mission,
        pad_cfg,
        banner_rows=summarize_config(leg_cfg),
        title=MISSION_TITLE,
    )
    if not pad.ok:
        return _abort(nav, f"pad sequence failed: {pad.reason}")

    # The deadline is a FLIGHT budget, so the clock starts here -- after the
    # aircraft is airborne -- not when the process started. Started earlier, it
    # was consumed by ground time nobody can bound: reading the banner, typing
    # GO, and the arm wait, which alone is allowed 300 s. An operator who took
    # their time on the pad could have the gate leg aborted on arrival for
    # "exceeding" a budget it had never been given any of.
    started_s = time.time()

    # ---- 2. approach, align, commit, cross -- one call, six named outcomes ---
    print("\n[STEP 8] Approaching the gate")
    outcome = approach_and_cross_one_gate(nav, mission, leg_cfg, gate_index=0)

    elapsed = time.time() - started_s
    if elapsed > mission_deadline_s:
        return _abort(
            nav,
            f"flight deadline of {mission_deadline_s:.0f}s exceeded "
            f"({elapsed:.0f}s airborne)",
        )

    if outcome.result is not GateResult.CROSSED:
        return _abort(nav, f"gate not crossed ({outcome.result.value}): {outcome.reason}")

    # ---- 3. land -------------------------------------------------------------
    print("\n[STEP 9] Landing")
    landed = nav.land()

    # ---- 4. report -----------------------------------------------------------
    print("\n" + "=" * 62)
    print(f"[RESULT] gate 0: {outcome.result.value}")
    print(f"         attempts={outcome.attempts} reacquires={outcome.reacquires} "
          f"gate time={outcome.elapsed_s:.1f}s flight time={elapsed:.1f}s")
    if outcome.fix is not None:
        fix = outcome.fix
        print(f"         last fix: N={fix.n:+.2f} E={fix.e:+.2f} D={fix.d:+.2f} "
              f"normal=({fix.normal_n:+.3f},{fix.normal_e:+.3f}) "
              f"{'flipped' if fix.normal_flipped else 'as reported'}")
    print(f"         landing confirmed: {'yes' if landed else 'NO -- CHECK THE AIRCRAFT'}")
    print("=" * 62)
    return 0 if landed else 1


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Phase 1A: take off, cross one gate, land.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    # Airborne budget only -- the clock starts after takeoff, not at launch,
    # so ground time is not charged against it. Sized to cover the gate leg
    # deadline (150 s) plus the crossing and a landing.
    parser.add_argument("--mission-timeout-s", type=float, default=240.0)
    add_common_arguments(parser)
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    if not check_mode_flags(args):
        return 2

    nav, mission, leg_cfg, pad_cfg = build(args)

    def on_signal(signum, _frame):
        # Raise into the main thread so the finally block below runs, which is
        # what actually lands the aircraft. This is safe because the setpoint
        # streamer is a separate daemon thread: it keeps publishing throughout
        # the unwind, and its deadman degrades to a position hold rather than
        # leaving the last commanded velocity latched.
        print(f"\n[!] Signal {signum} received -- shutting down safely")
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    mission.start()
    try:
        return run(nav, mission, leg_cfg, pad_cfg, args.mission_timeout_s)
    except KeyboardInterrupt:
        return _abort(nav, "operator interrupt")
    except Exception as exc:  # never leave the aircraft flying because of a bug here
        return _abort(nav, f"unhandled error: {exc!r}")
    finally:
        mission.stop()
        nav.stop()


if __name__ == "__main__":
    sys.exit(main())
