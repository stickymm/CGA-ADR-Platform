"""Track gates across frames when the payload carries no gate identity.

THE PROBLEM
    The vision wire format is three rows, sorted nearest-first, padded with an
    all-999 sentinel.  There is no gate ID.  Row index is therefore *not* an
    identity: the instant two gates swap order -- which happens every time the
    drone passes between them, or when one is briefly occluded -- row 0 becomes
    a different physical gate with no announcement.

    Phase 1 got away with this because it only ever consumed row 0 and crossed
    each gate before looking for the next.  Phase 2 plans a transition toward
    gate N+1 while still flying gate N, so it has to know which observation is
    which across frames.

THE APPROACH
    Nearest-neighbour association in local NED, greedy by ascending distance,
    with three deliberate refusals:

    1. **A gating radius that grows with track age.**  A track seen 40 ms ago
       should match within centimetres.  A track last seen 4 s ago has had time
       for the drone's own flow-based position estimate to drift, so it is
       matched within a wider radius.  Past ``max_gate_radius_m`` the match is
       refused outright rather than made at any distance.
    2. **An ambiguity margin.**  If the best and second-best candidate for an
       observation are within ``ambiguity_margin_m`` of each other, NO match is
       made.  Guessing between two closely-spaced gates does not produce a
       50/50 chance of being right -- it produces a confident, silent identity
       swap, and Phase 2 would then plan a turn toward the wrong gate.  Refusing
       degrades cleanly: the mission falls back to Phase 1B behaviour, which is
       the whole point of having a fallback.
    3. **Confidence, not presence.**  A track that has been seen once is not
       evidence.  ``GateTrack.confidence`` combines how often a track has been
       confirmed with how recently, and Phase 2 requires a threshold before it
       will plan against a gate.

    Everything here is pure: no I/O, no controller, no clock of its own -- the
    caller passes ``now_s``.  The whole thing is unit-testable on a laptop,
    which is the standing requirement in this repo for anything that decides
    where the aircraft goes.

FAILURE MODES -- these are real, and none of them are fixed by tuning
    * **Closely-spaced gates.**  Two gates less than about
      ``2 * ambiguity_margin_m`` apart are systematically refused rather than
      guessed.  Cost: fewer confirmed tracks and more Phase 1B fallback near
      tight gate pairs.  This is the intended trade; the alternative is an
      identity swap that reads as success.
    * **Recovery after a dropout.**  The gating radius grows with age, but past
      ``max_gate_radius_m`` the returning observation becomes a NEW track and
      the old one ages out.  Gate *identity* does not survive a long dropout --
      only gate *geometry* does.  A caller that has pinned a plan to a track id
      must re-derive its ordering, which is why ``update`` reports
      ``created``/``dropped`` explicitly instead of only returning the track
      set.
    * **Estimator drift.**  Positions live in PX4's local NED frame, which on an
      optical-flow airframe drifts with no global correction available.  Two
      observations of the same physical gate far apart in time can be metres
      apart in NED.  ``max_track_age_s`` bounds how long the tracker is willing
      to pretend otherwise; it cannot detect the drift itself.
    * **Symmetric geometry.**  Nearest-neighbour cannot distinguish two
      identical gates if the drone's own position estimate has slipped by more
      than half the gate spacing.  The ambiguity margin turns that from a wrong
      answer into no answer, which is the best this layer can do without a gate
      ID in the payload.
    * **A gate seen edge-on.**  Its ``normal`` is unreliable, so the smoothed
      heading of a track fed by edge-on views is unreliable too.  Low-confidence
      fixes are still associated (position is fine) but are flagged, and
      ``GateTrack.heading_confidence`` reports how much of the track's evidence
      came from usable geometry.
"""

import math
from dataclasses import dataclass, field, replace
from typing import Dict, List, Optional, Sequence, Tuple

from .contracts import GateFix


@dataclass(frozen=True)
class AssociationConfig:
    """Tunables for :class:`GateTracker`.  Every one is a documented trade."""

    # Radius for a track seen in the immediately preceding frame.
    base_gate_radius_m: float = 0.60
    # How fast that radius grows while a track goes unseen. Sized to plausible
    # optical-flow position drift rather than to gate spacing.
    radius_growth_m_per_s: float = 0.25
    # Hard cap. Past this, a returning observation is a new gate, not the old
    # one seen from further away.
    max_gate_radius_m: float = 2.00
    # Best and second-best candidates closer together than this -> refuse.
    ambiguity_margin_m: float = 0.50
    # Position smoothing. 0 trusts only the newest fix, 1 never updates.
    position_smoothing: float = 0.6
    # Observations needed before a track is worth planning against.
    hits_for_full_confidence: int = 3
    # A track unseen for longer than this is dropped.
    max_track_age_s: float = 5.0

    def __post_init__(self) -> None:
        if not 0.0 <= self.position_smoothing < 1.0:
            raise ValueError("position_smoothing must be in [0, 1)")
        if self.base_gate_radius_m <= 0.0 or self.max_gate_radius_m <= 0.0:
            raise ValueError("gate radii must be positive")
        if self.max_gate_radius_m < self.base_gate_radius_m:
            raise ValueError("max_gate_radius_m must be at least base_gate_radius_m")
        if self.hits_for_full_confidence < 1:
            raise ValueError("hits_for_full_confidence must be at least 1")
        if self.max_track_age_s <= 0.0:
            raise ValueError("max_track_age_s must be positive")

    def gate_radius_m(self, age_s: float) -> float:
        """Association radius for a track last seen ``age_s`` ago."""
        grown = self.base_gate_radius_m + self.radius_growth_m_per_s * max(0.0, age_s)
        return min(self.max_gate_radius_m, grown)


@dataclass(frozen=True)
class GateTrack:
    """One physical gate, followed across frames.

    ``track_id`` is assigned by the tracker and is stable for as long as the
    track survives.  It is emphatically **not** a course position: a course
    ordering has to be derived from geometry by the planner, because the tracker
    has no idea which gate the drone is supposed to fly next.
    """

    track_id: int
    n: float
    e: float
    d: float
    normal_n: float
    normal_e: float
    hits: int
    misses: int
    first_seen_s: float
    last_seen_s: float
    last_fix: GateFix
    confident_hits: int = 0     # hits from fixes that were not low-confidence

    @property
    def yaw_rad(self) -> float:
        return math.atan2(self.normal_e, self.normal_n)

    def age_s(self, now_s: float) -> float:
        return max(0.0, now_s - self.last_seen_s)

    def confidence(self, now_s: float, cfg: AssociationConfig) -> float:
        """How much this track should be trusted, in ``[0, 1]``.

        Two independent linear factors, kept linear so a number in a log can be
        reasoned about without reaching for the source:

        * **evidence** -- ``hits / hits_for_full_confidence``, capped at 1.  One
          sighting is not a gate; it is as likely to be a reflection, a doorway,
          or a single bad PnP solve.
        * **freshness** -- falls linearly to zero across ``max_track_age_s``.  A
          gate localized 4 seconds ago in a drifting frame is a guess about
          where it *was*.
        """
        evidence = min(1.0, self.hits / cfg.hits_for_full_confidence)
        freshness = max(0.0, 1.0 - self.age_s(now_s) / cfg.max_track_age_s)
        return evidence * freshness

    @property
    def heading_confidence(self) -> float:
        """Fraction of this track's sightings that had usable plane geometry.

        A gate observed edge-on gives a good centre and a worthless normal.  A
        planner that wants to fly *through* a gate needs the normal, so it needs
        this as well as :meth:`confidence`.
        """
        if self.hits <= 0:
            return 0.0
        return self.confident_hits / self.hits

    def range_from(self, n: float, e: float) -> float:
        return math.hypot(self.n - n, self.e - e)


@dataclass
class AssociationResult:
    """What one :meth:`GateTracker.update` did, in enough detail to log it."""

    tracks: Tuple[GateTrack, ...] = ()
    matched: Tuple[Tuple[int, int], ...] = ()      # (track_id, observation index)
    created: Tuple[int, ...] = ()                  # new track ids
    dropped: Tuple[int, ...] = ()                  # aged-out track ids
    ambiguous: Tuple[int, ...] = ()                # observation indices refused
    notes: Tuple[str, ...] = ()

    def describe(self) -> str:
        parts = [
            f"{len(self.tracks)} tracks",
            f"{len(self.matched)} matched",
        ]
        if self.created:
            parts.append(f"{len(self.created)} new {list(self.created)}")
        if self.dropped:
            parts.append(f"{len(self.dropped)} dropped {list(self.dropped)}")
        if self.ambiguous:
            parts.append(f"{len(self.ambiguous)} AMBIGUOUS (refused)")
        return " | ".join(parts)


def _distance(fix: GateFix, track: GateTrack) -> float:
    return math.sqrt(
        (fix.n - track.n) ** 2 + (fix.e - track.e) ** 2 + (fix.d - track.d) ** 2
    )


class GateTracker:
    """Maintains the set of gates believed to exist, from frame to frame.

    Stateful, but with no I/O and no clock: ``update`` takes the observations
    and the time, so a test can drive a dropout, a re-acquisition and an
    ambiguity in three lines with no threads involved.
    """

    def __init__(self, cfg: Optional[AssociationConfig] = None):
        self.cfg = cfg or AssociationConfig()
        self._tracks: Dict[int, GateTrack] = {}
        self._next_id = 0

    @property
    def tracks(self) -> Tuple[GateTrack, ...]:
        """Every live track, nearest-to-origin first is NOT assumed; id order."""
        return tuple(self._tracks[key] for key in sorted(self._tracks))

    def confident_tracks(self, now_s: float, threshold: float = 0.5) -> Tuple[GateTrack, ...]:
        return tuple(
            track for track in self.tracks
            if track.confidence(now_s, self.cfg) >= threshold
        )

    def reset(self) -> None:
        self._tracks.clear()

    def update(self, fixes: Sequence[GateFix], now_s: float) -> AssociationResult:
        """Fold one frame's worth of localized gates into the track set."""
        matched_pairs, ambiguous = self._match(fixes, now_s)

        matched_tracks = {track_id for track_id, _ in matched_pairs}
        matched_observations = {index for _, index in matched_pairs}

        for track_id, index in matched_pairs:
            self._tracks[track_id] = self._merge(
                self._tracks[track_id], fixes[index], now_s
            )

        created: List[int] = []
        for index, fix in enumerate(fixes):
            if index in matched_observations or index in ambiguous:
                continue
            track = self._create(fix, now_s)
            created.append(track.track_id)

        dropped: List[int] = []
        for track_id, track in list(self._tracks.items()):
            if track_id in matched_tracks or track_id in created:
                continue
            aged = replace(track, misses=track.misses + 1)
            if aged.age_s(now_s) > self.cfg.max_track_age_s:
                del self._tracks[track_id]
                dropped.append(track_id)
            else:
                self._tracks[track_id] = aged

        notes = []
        if ambiguous:
            notes.append(
                f"refused {len(ambiguous)} observation(s) as ambiguous: two candidate "
                f"gates within {self.cfg.ambiguity_margin_m:.2f} m of each other. "
                f"Guessing here swaps gate identity silently."
            )
        if dropped:
            notes.append(
                f"dropped {len(dropped)} track(s) unseen for more than "
                f"{self.cfg.max_track_age_s:.1f}s"
            )

        return AssociationResult(
            tracks=self.tracks,
            matched=tuple(matched_pairs),
            created=tuple(created),
            dropped=tuple(dropped),
            ambiguous=tuple(sorted(ambiguous)),
            notes=tuple(notes),
        )

    # ------------------------------------------------------------------
    # matching
    # ------------------------------------------------------------------

    def _match(
        self, fixes: Sequence[GateFix], now_s: float
    ) -> Tuple[List[Tuple[int, int]], set]:
        """Greedy nearest-neighbour with a gating radius and an ambiguity veto.

        Greedy rather than globally optimal (Hungarian): with at most three
        observations and a handful of tracks the two agree except in exactly the
        cases the ambiguity veto refuses anyway, and greedy is auditable from a
        log line.
        """
        candidates: List[Tuple[float, int, int]] = []
        for index, fix in enumerate(fixes):
            for track_id, track in self._tracks.items():
                distance = _distance(fix, track)
                if distance <= self.cfg.gate_radius_m(track.age_s(now_s)):
                    candidates.append((distance, track_id, index))

        ambiguous = self._find_ambiguous(fixes, candidates)

        candidates.sort()
        matched: List[Tuple[int, int]] = []
        used_tracks, used_observations = set(), set()
        for distance, track_id, index in candidates:
            if index in ambiguous:
                continue
            if track_id in used_tracks or index in used_observations:
                continue
            used_tracks.add(track_id)
            used_observations.add(index)
            matched.append((track_id, index))

        return matched, ambiguous

    def _find_ambiguous(self, fixes, candidates) -> set:
        """Observations whose two best candidate tracks are too close to call."""
        by_observation: Dict[int, List[float]] = {}
        for distance, _track_id, index in candidates:
            by_observation.setdefault(index, []).append(distance)

        ambiguous = set()
        for index, distances in by_observation.items():
            if len(distances) < 2:
                continue
            distances.sort()
            if distances[1] - distances[0] < self.cfg.ambiguity_margin_m:
                ambiguous.add(index)
        return ambiguous

    # ------------------------------------------------------------------
    # track maintenance
    # ------------------------------------------------------------------

    def _create(self, fix: GateFix, now_s: float) -> GateTrack:
        track = GateTrack(
            track_id=self._next_id,
            n=fix.n,
            e=fix.e,
            d=fix.d,
            normal_n=fix.normal_n,
            normal_e=fix.normal_e,
            hits=1,
            misses=0,
            first_seen_s=now_s,
            last_seen_s=now_s,
            last_fix=fix,
            confident_hits=0 if fix.low_confidence else 1,
        )
        self._tracks[track.track_id] = track
        self._next_id += 1
        return track

    def _merge(self, track: GateTrack, fix: GateFix, now_s: float) -> GateTrack:
        """Blend a new fix into a track.

        Position is an exponential moving average.  The normal is summed as a
        *vector* and renormalized rather than averaged as an angle, which is the
        same reason the detection averaging uses a circular mean: averaging
        +179 and -179 arithmetically gives 0, and a gate normal sits near the
        wrap routinely once the flip resolution has done its work.

        A low-confidence fix (edge-on, or altitude-clamped) still updates the
        position, which is well observed, but does NOT contribute to the
        heading evidence, which is not.
        """
        alpha = self.cfg.position_smoothing
        normal_n = track.normal_n
        normal_e = track.normal_e
        if not fix.low_confidence:
            summed_n = alpha * track.normal_n + (1.0 - alpha) * fix.normal_n
            summed_e = alpha * track.normal_e + (1.0 - alpha) * fix.normal_e
            length = math.hypot(summed_n, summed_e)
            if length > 1e-9:
                normal_n, normal_e = summed_n / length, summed_e / length

        return replace(
            track,
            n=alpha * track.n + (1.0 - alpha) * fix.n,
            e=alpha * track.e + (1.0 - alpha) * fix.e,
            d=alpha * track.d + (1.0 - alpha) * fix.d,
            normal_n=normal_n,
            normal_e=normal_e,
            hits=track.hits + 1,
            last_seen_s=now_s,
            last_fix=fix,
            confident_hits=track.confident_hits + (0 if fix.low_confidence else 1),
        )
