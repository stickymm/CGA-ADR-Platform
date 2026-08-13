"""Tests for the Phase 1A mission wrapper.

Small file, because Phase 1A is deliberately a linear sequence: pad, one gate
leg, land, report.  What is worth pinning down is the part that is NOT obvious
from reading it -- which clock the mission deadline runs on, and that every exit
lands.
"""

import unittest
from unittest.mock import patch

from support import FakeNav, ScriptedMission  # noqa: E402

from navigation.missions import phase1a_single_gate  # noqa: E402
from navigation.missions.contracts import (  # noqa: E402
    AltitudeEnvelope,
    GateLegConfig,
    GateOutcome,
    GateResult,
)
from navigation.missions.pad import PadConfig, PadResult  # noqa: E402


def gate_outcome(result, reason="scripted") -> GateOutcome:
    return GateOutcome(result=result, gate_index=0, reason=reason)


class MissionDeadlineTests(unittest.TestCase):
    """REGRESSION -- the deadline was charging flight time for ground time.

    ``started_s`` was taken before ``run_pad_sequence``, so everything the
    operator does on the pad -- reading the banner, typing GO, and the arm wait,
    which alone is allowed 300 s -- was billed against a 180 s *flight* budget.
    An unhurried pad session could have the gate leg aborted the instant it
    returned, for exceeding a budget it had never been given any of.

    The clock now starts after the pad sequence completes.
    """

    def setUp(self):
        self.nav = FakeNav()
        self.leg_cfg = GateLegConfig()
        self.pad_cfg = PadConfig()

    def _run(self, *, pad_seconds, leg_seconds, deadline_s, result=GateResult.CROSSED):
        clock = _StepClock()

        def pad_sequence(*args, **kwargs):
            clock.advance(pad_seconds)
            return PadResult(True, "ready", AltitudeEnvelope())

        def gate_leg(*args, **kwargs):
            clock.advance(leg_seconds)
            return gate_outcome(result)

        with patch.object(phase1a_single_gate, "time", clock), \
             patch.object(phase1a_single_gate, "run_pad_sequence", pad_sequence), \
             patch.object(phase1a_single_gate, "approach_and_cross_one_gate", gate_leg):
            return phase1a_single_gate.run(
                self.nav, ScriptedMission([]), self.leg_cfg, self.pad_cfg, deadline_s
            )

    def test_a_long_pad_session_does_not_consume_the_flight_budget(self):
        # 400 s on the pad -- an operator taking their time, well inside the
        # 300 s arm wait plus banner reading -- then a 20 s gate leg against a
        # 240 s budget. Before the fix this aborted; now it succeeds.
        code = self._run(pad_seconds=400.0, leg_seconds=20.0, deadline_s=240.0)

        self.assertEqual(code, 0)
        self.assertEqual(self.nav.safe_shutdowns, [])
        self.assertEqual(self.nav.landings, 1)

    def test_a_genuinely_long_flight_still_trips_the_deadline(self):
        # The budget must still bound time in the air.
        code = self._run(pad_seconds=5.0, leg_seconds=300.0, deadline_s=240.0)

        self.assertEqual(code, 1)
        self.assertEqual(len(self.nav.safe_shutdowns), 1)
        self.assertIn("flight deadline", self.nav.safe_shutdowns[0])
        self.assertIn("airborne", self.nav.safe_shutdowns[0])

    def test_the_deadline_is_measured_from_takeoff_not_from_launch(self):
        # Directly: identical flight, wildly different pad times, same verdict.
        for pad_seconds in (1.0, 250.0, 900.0):
            with self.subTest(pad_seconds=pad_seconds):
                self.nav = FakeNav()
                code = self._run(
                    pad_seconds=pad_seconds, leg_seconds=30.0, deadline_s=240.0
                )
                self.assertEqual(code, 0)


class MissionExitTests(unittest.TestCase):
    """Every exit path lands."""

    def setUp(self):
        self.nav = FakeNav()
        self.leg_cfg = GateLegConfig()
        self.pad_cfg = PadConfig()

    def _run(self, pad_ok=True, result=GateResult.CROSSED):
        def pad_sequence(*args, **kwargs):
            return PadResult(pad_ok, "ready" if pad_ok else "no detections",
                             AltitudeEnvelope())

        with patch.object(phase1a_single_gate, "run_pad_sequence", pad_sequence), \
             patch.object(phase1a_single_gate, "approach_and_cross_one_gate",
                          lambda *a, **k: gate_outcome(result)):
            return phase1a_single_gate.run(
                self.nav, ScriptedMission([]), self.leg_cfg, self.pad_cfg, 240.0
            )

    def test_a_crossed_gate_lands_and_exits_zero(self):
        self.assertEqual(self._run(), 0)
        self.assertEqual(self.nav.landings, 1)

    def test_a_failed_pad_sequence_still_lands(self):
        self.assertEqual(self._run(pad_ok=False), 1)
        self.assertEqual(self.nav.landings, 1)

    def test_every_non_crossed_outcome_aborts_and_lands(self):
        for result in (
            GateResult.NOT_FOUND,
            GateResult.LOST,
            GateResult.NO_COMMIT,
            GateResult.TIMED_OUT,
            GateResult.ABORTED,
        ):
            with self.subTest(result=result):
                self.nav = FakeNav()
                self.assertEqual(self._run(result=result), 1)
                self.assertEqual(self.nav.landings, 1)
                self.assertIn(result.value, self.nav.safe_shutdowns[0])


class _StepClock:
    """A clock that only moves when a test tells it to."""

    def __init__(self, start=1_000.0):
        self.now = start

    def time(self):
        return self.now

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds

    def advance(self, seconds):
        self.now += seconds


if __name__ == "__main__":
    unittest.main()
