"""Synthetic gate detections over UDP, for testing with no drone and no camera.

Emits the real wire format so the mission stack cannot tell the difference:

    {"fps": 30.0, "gates": [[dist, forward, right, down, roll, pitch, yaw], x3]}

Exactly three rows every packet, sorted nearest-first, absent gates padded with
an all-999.0 row -- because that is what ``vision/opencv_processing.py`` does,
and code that only ever sees tidy input is not actually being tested.

RUN IT

    python -m navigation.missions.udp_injector --list
    python -m navigation.missions.udp_injector --case dead-ahead
    python -m navigation.missions.udp_injector --case normal-reversed --closing

Pair it with ``--offline`` in another terminal::

    python -m navigation.missions.phase1a_single_gate --offline

NOTE
    This binds the same port the mission listens on (5050), which is also the
    port ``debug/telemetry_reader.py`` binds.  Only one consumer at a time.
"""

import argparse
import json
import math
import random
import socket
import sys
import time

NO_DETECTION_ROW = [999.0, 999.0, 999.0, 999.0, 999.0, 999.0, 999.0]

# Measurement noise, metres and degrees of 1-sigma.
#
# NOT COSMETIC.  A real PnP pose estimate jitters in the third decimal every
# frame; a noiseless injector does not, so once the simulated range reaches its
# floor every payload becomes bit-identical and the mission's frozen-feed
# detector fires -- correctly, on synthetic data that is genuinely frozen.  That
# would make the one signal that catches a stalled camera untrustworthy offline.
# ``--case frozen`` sets these to zero, because there the whole point is that
# nothing changes.
DEFAULT_POSITION_NOISE_M = 0.003
DEFAULT_ANGLE_NOISE_DEG = 0.05

_noise_position_m = DEFAULT_POSITION_NOISE_M
_noise_angle_deg = DEFAULT_ANGLE_NOISE_DEG


def _jitter(scale: float) -> float:
    return random.gauss(0.0, scale) if scale > 0.0 else 0.0


def gate_row(forward, right=0.0, down=0.0, yaw_deg=0.0, roll=0.0, pitch=0.0):
    """One detection row in the vision process's own format and rounding.

    ``dist`` is the 3-D norm, matching how the real publisher computes it.
    """
    forward += _jitter(_noise_position_m)
    right += _jitter(_noise_position_m)
    down += _jitter(_noise_position_m)
    yaw_deg += _jitter(_noise_angle_deg)
    roll += _jitter(_noise_angle_deg)
    pitch += _jitter(_noise_angle_deg)

    dist = math.sqrt(forward * forward + right * right + down * down)
    return [
        round(dist, 3),
        round(forward, 3),
        round(right, 3),
        round(down, 3),
        round(roll, 2),
        round(pitch, 2),
        round(yaw_deg, 2),
    ]


def gate_row_exact(forward, right=0.0, down=0.0, yaw_deg=0.0, roll=0.0, pitch=0.0):
    """A detection row with no measurement noise at all.

    Only the ``frozen`` case uses this: it models a stalled camera republishing
    one captured pose, and a stalled camera does not add fresh noise.
    """
    dist = math.sqrt(forward * forward + right * right + down * down)
    return [
        round(dist, 3), round(forward, 3), round(right, 3), round(down, 3),
        round(roll, 2), round(pitch, 2), round(yaw_deg, 2),
    ]


# Each case is a function of elapsed seconds and the current closing range,
# returning the list of visible gates (before padding).
def _case_dead_ahead(t, distance):
    return [gate_row(distance)]


def _case_offset_left(t, distance):
    return [gate_row(distance, right=-0.6)]


def _case_offset_right(t, distance):
    return [gate_row(distance, right=+0.6)]


def _case_above(t, distance):
    return [gate_row(distance, down=-0.5)]


def _case_below(t, distance):
    return [gate_row(distance, down=+0.5)]


def _case_normal_as_reported(t, distance):
    return [gate_row(distance, yaw_deg=-10.0)]


def _case_normal_reversed(t, distance):
    """The same physical gate with its plane normal reported 180 degrees round.

    The mission must fly this identically to ``normal-as-reported``.  If the
    normal is taken at face value the standoff point lands behind the gate and
    the approach runs backwards.
    """
    return [gate_row(distance, yaw_deg=170.0)]


def _case_edge_on(t, distance):
    """A gate seen almost edge-on, where the plane fit is not trustworthy."""
    return [gate_row(distance, right=distance * 0.9, yaw_deg=75.0)]


def _case_multiple_gates(t, distance):
    return [
        gate_row(distance),
        gate_row(distance + 4.0, right=1.5),
        gate_row(distance + 9.0, right=-2.0),
    ]


# ---------------------------------------------------------------------------
# PHASE 2A cases -- association and full attitude
# ---------------------------------------------------------------------------


def _case_multi_gate_course(t, distance):
    """Three well-separated gates with distinct headings, all closing together.

    Phase 2 has to localize all three from one packet and keep their identities
    while their nearest-first row order changes underneath it.  The headings
    differ so a planner has something real to turn toward.
    """
    return [
        gate_row(distance, yaw_deg=0.0),
        gate_row(distance + 3.5, right=+2.0, yaw_deg=+25.0),
        gate_row(distance + 7.0, right=-1.5, yaw_deg=-20.0),
    ]


def _case_row_swap(t, distance):
    """Two gates that exchange nearest-first row order halfway through.

    The exact condition that makes row index useless as an identity: the
    publisher re-sorts every frame, so row 0 becomes a different physical gate
    with no announcement.  Association must not notice, and the track ids must
    not move.
    """
    left = gate_row(distance, right=-1.2, yaw_deg=0.0)
    right = gate_row(distance + 0.02 * (t - 10.0), right=+1.2, yaw_deg=0.0)
    near_first = sorted([left, right], key=lambda row: row[0])
    return near_first


def _case_close_gates(t, distance):
    """Two gates 0.45 m apart -- inside the default ambiguity margin.

    The tracker must REFUSE to associate rather than guess, and Phase 2 must
    degrade to Phase 1B behaviour instead of confidently turning toward whichever
    one it picked.  A run of this case that reports clean association is a bug,
    not a success.
    """
    return [
        gate_row(distance, right=-0.225),
        gate_row(distance, right=+0.225),
    ]


def _case_association_dropout(t, distance):
    """Three gates that vanish for 2.5 s and return slightly displaced.

    Models an occlusion or a pipeline stall mid-course.  Inside the tracker's
    grown gating radius the identities survive; the 0.35 m displacement on
    return is deliberately sized to sit near that boundary, because the useful
    question is what happens at the edge, not in the easy case.
    """
    if 8.0 <= t < 10.5:
        return []
    displaced = 0.35 if t >= 10.5 else 0.0
    return [
        gate_row(distance, right=displaced),
        gate_row(distance + 3.5, right=+2.0 + displaced, yaw_deg=+25.0),
        gate_row(distance + 7.0, right=-1.5 + displaced, yaw_deg=-20.0),
    ]


def _case_non_level(t, distance):
    """A gate whose plane is rolled and pitched relative to the camera.

    Exercises the roll/pitch fields of the detection, which Phase 1 averaged and
    then ignored.

    NOTE: this tilts the GATE, not the vehicle.  To exercise the vehicle-attitude
    terms of the 3-2-1 rotation, run the mission with ``--offline-tilt``, which
    makes the offline model derive roll and pitch from its own horizontal
    acceleration.  The injector cannot do that for you: it is open loop and has
    no idea how the vehicle is oriented.
    """
    roll = 18.0 * math.sin(t * 0.7)
    pitch = 12.0 * math.cos(t * 0.5)
    return [gate_row(distance, roll=roll, pitch=pitch)]


def _case_dropout(t, distance):
    """Detector drops out for three seconds mid-approach, then returns."""
    if 6.0 <= t < 9.0:
        return []
    return [gate_row(distance)]


def _case_frozen(t, distance):
    """Publisher keeps sending, but the pose never changes.

    Models a stalled camera or a wedged pipeline.  The distance argument is
    ignored on purpose -- the whole point is that the drone moves and the
    reported gate does not.  Uses the noiseless row builder: a stalled camera
    republishes one captured pose, it does not keep generating fresh noise.
    """
    return [gate_row_exact(3.0)]


def _case_course(t, distance):
    """A fresh gate every cycle -- the closing range sawtooths back to the start.

    The injector is open loop: it has no idea where the drone actually is, so it
    cannot present a world-fixed course.  Resetting the range on a timer is the
    honest approximation -- it exercises the Phase 1B loop (cross, settle,
    re-acquire, cross again) without pretending to model course geometry.
    ``--course-cycle-s`` should be a little longer than one gate takes to fly.
    """
    return [gate_row(distance)]


CASES = {
    "course": _case_course,
    "dead-ahead": _case_dead_ahead,
    "offset-left": _case_offset_left,
    "offset-right": _case_offset_right,
    "above": _case_above,
    "below": _case_below,
    "normal-as-reported": _case_normal_as_reported,
    "normal-reversed": _case_normal_reversed,
    "edge-on": _case_edge_on,
    "multiple-gates": _case_multiple_gates,
    "dropout": _case_dropout,
    "frozen": _case_frozen,
    # Phase 2A
    "multi-gate-course": _case_multi_gate_course,
    "row-swap": _case_row_swap,
    "close-gates": _case_close_gates,
    "association-dropout": _case_association_dropout,
    "non-level": _case_non_level,
}

CASE_HELP = {
    "course": "a fresh gate every --course-cycle-s, for the Phase 1B loop",
    "dead-ahead": "gate straight ahead on the boresight",
    "offset-left": "gate 0.6 m to the left -- must not commit until aligned",
    "offset-right": "gate 0.6 m to the right",
    "above": "gate 0.5 m above the drone",
    "below": "gate 0.5 m below the drone -- exercises the altitude floor",
    "normal-as-reported": "plane normal reported pointing away (yaw -10 deg)",
    "normal-reversed": "SAME gate, normal reported reversed (yaw 170 deg)",
    "edge-on": "gate nearly edge-on -- must be rejected as low confidence",
    "multiple-gates": "three gates in one packet, nearest first",
    "dropout": "detector goes silent for 3 s mid-approach",
    "frozen": "publisher alive but the pose never changes (stalled camera)",
    "multi-gate-course": "three separated gates with distinct headings, all localized",
    "row-swap": "two gates that exchange nearest-first row order mid-run",
    "close-gates": "two gates 0.45 m apart -- association MUST refuse, not guess",
    "association-dropout": "three gates vanish for 2.5 s and return displaced 0.35 m",
    "non-level": "one gate with a rolled and pitched plane (see also --offline-tilt)",
}


def build_payload(gates, fps):
    rows = [list(row) for row in gates[:3]]
    while len(rows) < 3:
        rows.append(list(NO_DETECTION_ROW))
    return {"fps": round(fps, 1), "gates": rows}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Publish synthetic gate detections for offline mission testing.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--case", default="dead-ahead", choices=sorted(CASES))
    parser.add_argument("--list", action="store_true", help="describe every case and exit")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5050)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--start-distance-m", type=float, default=4.0)
    parser.add_argument(
        "--closing-speed-m-s",
        type=float,
        default=0.3,
        help="how fast the simulated gate approaches; 0 holds it at a fixed range",
    )
    parser.add_argument("--min-distance-m", type=float, default=0.35)
    parser.add_argument(
        "--course-cycle-s",
        type=float,
        default=16.0,
        help="for --case course: how often the range resets, i.e. a new gate",
    )
    parser.add_argument("--duration-s", type=float, default=0.0, help="0 runs until Ctrl-C")
    parser.add_argument(
        "--noise-m",
        type=float,
        default=DEFAULT_POSITION_NOISE_M,
        help="1-sigma position noise. 0 makes every payload bit-identical once "
             "the range stops changing, which the frozen-feed detector will "
             "correctly flag",
    )
    parser.add_argument("--noise-deg", type=float, default=DEFAULT_ANGLE_NOISE_DEG)
    parser.add_argument(
        "--seed", type=int, default=None, help="seed the noise for a repeatable run"
    )
    args = parser.parse_args(argv)

    global _noise_position_m, _noise_angle_deg
    _noise_position_m = max(0.0, args.noise_m)
    _noise_angle_deg = max(0.0, args.noise_deg)
    if args.seed is not None:
        random.seed(args.seed)

    if args.list:
        width = max(len(name) for name in CASES)
        for name in sorted(CASES):
            print(f"  {name:<{width}}  {CASE_HELP[name]}")
        return 0

    case = CASES[args.case]
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    period = 1.0 / args.fps
    started = time.time()
    distance = args.start_distance_m
    sent = 0

    print(f"[*] Injecting '{args.case}' to {args.host}:{args.port} at {args.fps:.0f} Hz")
    print(f"[*] {CASE_HELP[args.case]}")
    print(f"[*] Gate closes from {args.start_distance_m:.2f} m at "
          f"{args.closing_speed_m_s:.2f} m/s (floor {args.min_distance_m:.2f} m)")
    print("[*] Ctrl-C to stop.\n")

    try:
        while True:
            elapsed = time.time() - started
            if args.duration_s > 0.0 and elapsed >= args.duration_s:
                break

            # For the course case the range sawtooths, so each cycle reads as a
            # newly acquired gate rather than one gate that keeps getting closer.
            closing_time = elapsed % args.course_cycle_s if args.case == "course" else elapsed
            distance = max(
                args.min_distance_m,
                args.start_distance_m - args.closing_speed_m_s * closing_time,
            )
            payload = build_payload(case(elapsed, distance), args.fps)
            sock.sendto(json.dumps(payload).encode("utf-8"), (args.host, args.port))

            sent += 1
            if sent % int(max(1.0, args.fps)) == 0:
                first = payload["gates"][0]
                shown = "no detection" if first[0] == 999.0 else (
                    f"dist={first[0]:.2f} fwd={first[1]:+.2f} right={first[2]:+.2f} "
                    f"down={first[3]:+.2f} yaw={first[6]:+.1f}"
                )
                print(f"  t={elapsed:6.1f}s  {shown}")

            time.sleep(period)
    except KeyboardInterrupt:
        print("\n[*] Injector stopped.")
    finally:
        sock.close()

    print(f"[*] Sent {sent} packets.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
