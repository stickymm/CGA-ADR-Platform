"""Tests for gate association across frames.

The payload carries no gate ID and row index is not an identity, so everything
here is the tracker deciding which observation is which physical gate.  A silent
identity swap is the failure that matters: it does not look like an error, it
looks like Phase 2 confidently turning toward the wrong gate.
"""

import unittest

import support  # noqa: F401  (installs the pymavlink stub)

from navigation.missions.association import (  # noqa: E402
    AssociationConfig,
    GateTracker,
)
from navigation.missions.contracts import GateFix  # noqa: E402


def fix(n, e, d=-1.5, *, normal=(1.0, 0.0), low_confidence=False, cone=0.0) -> GateFix:
    return GateFix(
        n=n, e=e, d=d,
        normal_n=normal[0], normal_e=normal[1],
        yaw_rad=0.0,
        range_m=3.0,
        lateral_body_m=0.0,
        vertical_body_m=0.0,
        lateral_axis_m=0.0,
        cone_angle_deg=cone,
        normal_flipped=False,
        altitude_clamped=False,
        low_confidence=low_confidence,
        reason="edge-on" if low_confidence else "",
    )


class BasicAssociationTests(unittest.TestCase):
    def setUp(self):
        self.tracker = GateTracker()

    def test_the_first_frame_creates_one_track_per_gate(self):
        result = self.tracker.update([fix(3.0, 0.0), fix(7.0, 1.5)], now_s=0.0)

        self.assertEqual(len(result.tracks), 2)
        self.assertEqual(result.created, (0, 1))
        self.assertEqual(result.matched, ())

    def test_a_gate_seen_again_updates_its_track_rather_than_making_a_new_one(self):
        self.tracker.update([fix(3.0, 0.0)], now_s=0.0)
        result = self.tracker.update([fix(2.85, 0.05)], now_s=0.1)

        self.assertEqual(len(result.tracks), 1)
        self.assertEqual(result.created, ())
        self.assertEqual(result.matched, ((0, 0),))
        self.assertEqual(result.tracks[0].hits, 2)

    def test_row_order_swapping_does_not_swap_identity(self):
        # THE WHOLE POINT. The publisher sorts nearest-first, so the moment the
        # drone passes between two gates the rows exchange places. Association
        # is geometric, so the track ids do not.
        self.tracker.update([fix(3.0, 0.0), fix(7.0, 2.0)], now_s=0.0)
        # Same two gates, reported in the opposite order.
        result = self.tracker.update([fix(7.0, 2.0), fix(3.0, 0.0)], now_s=0.1)

        self.assertEqual(len(result.tracks), 2)
        self.assertEqual(result.created, ())
        self.assertEqual(dict(result.matched), {0: 1, 1: 0})

    def test_positions_are_smoothed_not_replaced(self):
        # A single noisy PnP solve should nudge a track, not teleport it.
        cfg = AssociationConfig(position_smoothing=0.5)
        tracker = GateTracker(cfg)
        tracker.update([fix(3.0, 0.0)], now_s=0.0)
        tracker.update([fix(3.4, 0.0)], now_s=0.1)

        self.assertAlmostEqual(tracker.tracks[0].n, 3.2, places=6)

    def test_a_genuinely_new_gate_gets_its_own_track(self):
        self.tracker.update([fix(3.0, 0.0)], now_s=0.0)
        result = self.tracker.update([fix(3.0, 0.0), fix(9.0, 4.0)], now_s=0.1)

        self.assertEqual(len(result.tracks), 2)
        self.assertEqual(result.created, (1,))


class GatingRadiusTests(unittest.TestCase):
    def test_an_observation_beyond_the_gating_radius_is_a_new_gate(self):
        tracker = GateTracker(AssociationConfig(base_gate_radius_m=0.5))
        tracker.update([fix(3.0, 0.0)], now_s=0.0)
        result = tracker.update([fix(3.0, 2.0)], now_s=0.05)

        self.assertEqual(result.created, (1,))
        self.assertEqual(result.matched, ())

    def test_the_radius_grows_while_a_track_goes_unseen(self):
        # Sized to plausible optical-flow drift: a gate last seen 3 s ago has
        # genuinely moved in the drifting local frame, even though it has not
        # moved in the room.
        cfg = AssociationConfig(
            base_gate_radius_m=0.5, radius_growth_m_per_s=0.25, max_gate_radius_m=2.0
        )
        self.assertAlmostEqual(cfg.gate_radius_m(0.0), 0.5, places=9)
        self.assertAlmostEqual(cfg.gate_radius_m(2.0), 1.0, places=9)
        self.assertAlmostEqual(cfg.gate_radius_m(100.0), 2.0, places=9)  # capped

    def test_a_gate_reacquired_after_a_short_dropout_keeps_its_identity(self):
        tracker = GateTracker(
            AssociationConfig(base_gate_radius_m=0.5, radius_growth_m_per_s=0.25)
        )
        tracker.update([fix(3.0, 0.0)], now_s=0.0)
        for step in range(1, 8):           # 1.4 s of nothing
            tracker.update([], now_s=step * 0.2)
        # Comes back 0.8 m from where it was: outside the base radius, inside
        # the radius that 1.6 s of ageing has grown.
        result = tracker.update([fix(3.0, 0.8)], now_s=1.6)

        self.assertEqual(result.matched, ((0, 0),))
        self.assertEqual(result.created, ())

    def test_after_a_long_dropout_identity_is_not_preserved(self):
        # DOCUMENTED FAILURE MODE. Past max_gate_radius_m a returning
        # observation is a new gate, not the old one seen from further away.
        # Gate geometry survives a long dropout; gate identity does not, and a
        # caller that pinned a plan to a track id must re-derive its ordering.
        tracker = GateTracker(AssociationConfig(max_track_age_s=2.0))
        tracker.update([fix(3.0, 0.0)], now_s=0.0)
        result = tracker.update([fix(3.0, 3.0)], now_s=3.0)

        self.assertEqual(result.created, (1,))
        self.assertEqual(result.dropped, (0,))
        self.assertTrue(any("dropped" in note for note in result.notes))


class AmbiguityTests(unittest.TestCase):
    """Closely-spaced gates: refuse rather than guess."""

    def test_two_candidates_too_close_to_call_are_refused(self):
        # DOCUMENTED FAILURE MODE. Two tracks 0.4 m apart, an observation
        # between them. Guessing does not give a 50/50 chance of being right --
        # it gives a confident, silent identity swap, and Phase 2 then plans a
        # turn toward the wrong gate. Refusing degrades to Phase 1B, which is
        # exactly what the fallback exists for.
        tracker = GateTracker(
            AssociationConfig(base_gate_radius_m=1.0, ambiguity_margin_m=0.5)
        )
        tracker.update([fix(3.0, 0.0), fix(3.0, 0.4)], now_s=0.0)
        result = tracker.update([fix(3.0, 0.2)], now_s=0.1)

        self.assertEqual(result.ambiguous, (0,))
        self.assertEqual(result.matched, ())
        self.assertEqual(result.created, ())  # not invented as a third gate
        self.assertTrue(any("ambiguous" in note.lower() for note in result.notes))

    def test_a_clear_winner_is_not_treated_as_ambiguous(self):
        tracker = GateTracker(
            AssociationConfig(base_gate_radius_m=2.0, ambiguity_margin_m=0.5)
        )
        tracker.update([fix(3.0, 0.0), fix(3.0, 1.8)], now_s=0.0)
        result = tracker.update([fix(3.0, 0.05)], now_s=0.1)

        self.assertEqual(result.ambiguous, ())
        self.assertEqual(result.matched, ((0, 0),))

    def test_well_separated_gates_are_never_ambiguous(self):
        tracker = GateTracker()
        tracker.update([fix(3.0, 0.0), fix(8.0, 0.0), fix(13.0, 0.0)], now_s=0.0)
        result = tracker.update(
            [fix(3.02, 0.0), fix(8.01, 0.0), fix(12.98, 0.0)], now_s=0.1
        )

        self.assertEqual(result.ambiguous, ())
        self.assertEqual(len(result.matched), 3)


class ConfidenceTests(unittest.TestCase):
    def setUp(self):
        self.cfg = AssociationConfig(hits_for_full_confidence=3, max_track_age_s=5.0)
        self.tracker = GateTracker(self.cfg)

    def test_one_sighting_is_not_enough_to_plan_against(self):
        # One sighting is as likely to be a reflection, a doorway, or a single
        # bad PnP solve as it is to be a gate.
        self.tracker.update([fix(3.0, 0.0)], now_s=0.0)
        track = self.tracker.tracks[0]

        self.assertAlmostEqual(track.confidence(0.0, self.cfg), 1 / 3, places=6)
        self.assertEqual(self.tracker.confident_tracks(0.0, threshold=0.5), ())

    def test_confidence_rises_with_repeated_sightings(self):
        for step in range(3):
            self.tracker.update([fix(3.0, 0.0)], now_s=step * 0.1)

        track = self.tracker.tracks[0]
        self.assertAlmostEqual(track.confidence(0.2, self.cfg), 1.0, places=2)
        self.assertEqual(len(self.tracker.confident_tracks(0.2)), 1)

    def test_confidence_decays_with_staleness(self):
        for step in range(3):
            self.tracker.update([fix(3.0, 0.0)], now_s=step * 0.1)

        # Half the max age has elapsed: freshness halves.
        self.assertAlmostEqual(
            self.tracker.tracks[0].confidence(2.7, self.cfg), 0.5, places=2
        )
        self.assertEqual(self.tracker.confident_tracks(5.5, threshold=0.1), ())

    def test_edge_on_sightings_localize_the_gate_but_not_its_heading(self):
        # A gate seen edge-on gives a good centre and a worthless normal. The
        # position evidence counts; the heading evidence does not, because a
        # planner that flies THROUGH a gate needs the normal to be real.
        self.tracker.update([fix(3.0, 0.0)], now_s=0.0)
        self.tracker.update([fix(3.0, 0.0, low_confidence=True, cone=85.0)], now_s=0.1)

        track = self.tracker.tracks[0]
        self.assertEqual(track.hits, 2)
        self.assertEqual(track.confident_hits, 1)
        self.assertAlmostEqual(track.heading_confidence, 0.5, places=6)

    def test_a_low_confidence_fix_does_not_drag_the_heading_around(self):
        tracker = GateTracker(AssociationConfig(position_smoothing=0.0))
        tracker.update([fix(3.0, 0.0, normal=(1.0, 0.0))], now_s=0.0)
        tracker.update(
            [fix(3.0, 0.0, normal=(0.0, 1.0), low_confidence=True)], now_s=0.1
        )

        # Position followed the new fix; heading did not.
        self.assertAlmostEqual(tracker.tracks[0].normal_n, 1.0, places=6)
        self.assertAlmostEqual(tracker.tracks[0].normal_e, 0.0, places=6)


class TrackLifecycleTests(unittest.TestCase):
    def test_a_missed_track_survives_briefly_then_is_dropped(self):
        tracker = GateTracker(AssociationConfig(max_track_age_s=1.0))
        tracker.update([fix(3.0, 0.0)], now_s=0.0)

        result = tracker.update([], now_s=0.5)
        self.assertEqual(len(result.tracks), 1)
        self.assertEqual(result.tracks[0].misses, 1)

        result = tracker.update([], now_s=1.5)
        self.assertEqual(result.dropped, (0,))
        self.assertEqual(result.tracks, ())

    def test_track_ids_are_never_reused(self):
        tracker = GateTracker(AssociationConfig(max_track_age_s=0.5))
        tracker.update([fix(3.0, 0.0)], now_s=0.0)
        tracker.update([], now_s=1.0)                     # id 0 dropped
        result = tracker.update([fix(3.0, 0.0)], now_s=1.1)

        self.assertEqual(result.created, (1,))

    def test_reset_clears_everything(self):
        tracker = GateTracker()
        tracker.update([fix(3.0, 0.0)], now_s=0.0)
        tracker.reset()
        self.assertEqual(tracker.tracks, ())

    def test_the_result_describes_itself_for_a_log_line(self):
        tracker = GateTracker()
        result = tracker.update([fix(3.0, 0.0), fix(9.0, 0.0)], now_s=0.0)
        self.assertIn("2 tracks", result.describe())
        self.assertIn("2 new", result.describe())


class ConfigValidationTests(unittest.TestCase):
    def test_impossible_configurations_are_rejected_at_construction(self):
        for kwargs in (
            {"position_smoothing": 1.0},
            {"position_smoothing": -0.1},
            {"base_gate_radius_m": 0.0},
            {"base_gate_radius_m": 3.0, "max_gate_radius_m": 1.0},
            {"hits_for_full_confidence": 0},
            {"max_track_age_s": 0.0},
        ):
            with self.subTest(**kwargs):
                with self.assertRaises(ValueError):
                    AssociationConfig(**kwargs)


if __name__ == "__main__":
    unittest.main()
