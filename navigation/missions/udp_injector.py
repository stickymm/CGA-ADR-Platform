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
import socket
import sys
import time

NO_DETECTION_ROW = [999.0, 999.0, 999.0, 999.0, 999.0, 999.0, 999.0]


def gate_row(forward, right=0.0, down=0.0, yaw_deg=0.0, roll=0.0, pitch=0.0):
    """One detection row in the vision process's own format and rounding.

    ``dist`` is the 3-D norm, matching how the real publisher computes it.
    """
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


def _case_dropout(t, distance):
    """Detector drops out for three seconds mid-approach, then returns."""
    if 6.0 <= t < 9.0:
        return []
    return [gate_row(distance)]


def _case_frozen(t, distance):
    """Publisher keeps sending, but the pose never changes.

    Models a stalled camera or a wedged pipeline.  The distance argument is
    ignored on purpose -- the whole point is that the drone moves and the
    reported gate does not.
    """
    return [gate_row(3.0)]


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
    args = parser.parse_args(argv)

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
