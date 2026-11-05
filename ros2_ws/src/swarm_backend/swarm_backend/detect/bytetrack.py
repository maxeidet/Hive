"""ByteTrack (Zhang et al. 2022): keeps an id on each target from image to image, also
through images where the detector is unsure of it.

The idea: a detector's low-score detections are mostly clutter, but some are real targets
that are blurred, small or partly hidden. Throwing them away (a score threshold) breaks
the tracks; keeping them starts tracks on clutter. ByteTrack uses them only to continue
tracks that already exist, where a track's Kalman filter (kalman.py) predicts it to be:

  each image
    0. compensate every track for the camera turning (if the orientation is known)
       and predict where it is now (kalman.py)
    1. split the detections: high (score >= high), low (low <= score < high);
       the rest are dropped
    2. match the high ones to the tracks being followed and to the lost ones
    3. match the low ones to the followed tracks still without a detection;
       low detections left over are dropped, they never start a track
    4. match the high ones left to the tentative tracks (seen in the last image only);
       a tentative track that is not matched is deleted (it was probably clutter)
    5. followed tracks still without a detection become lost; lost tracks are kept
       (predicted on) for max_lost images and can come back in step 2
    6. high detections left with score >= new start new tentative tracks;
       a tentative track becomes confirmed after k images in a row

Matching: the paper uses the overlap (IoU) of the predicted box and the detection. The
drones here are a few px across and move more than their own size between images, so the
boxes often do not overlap at all; the default is therefore the distance between the centres
measured in the filter's own uncertainty (Mahalanobis distance; 3 = three standard
deviations), cost='iou' gives the paper's. Each step picks the matching with the lowest
total cost (Hungarian method, scipy; greedy nearest-first if scipy is missing). No ROS here.
"""
from dataclasses import dataclass, field
from enum import Enum

import numpy as np

from swarm_backend.detect.detectors import Detection
from swarm_backend.detect.kalman import BoxKalman, Noise

try:
    from scipy.optimize import linear_sum_assignment
except ImportError:  # not in the container yet: greedy matching instead
    linear_sum_assignment = None


class State(Enum):
    TENTATIVE = 'tentative'  # new, not seen k images in a row yet
    TRACKED = 'tracked'  # confirmed and followed
    LOST = 'lost'  # confirmed, but no detection lately; still predicted on


@dataclass
class Settings:
    high: float = 0.5  # score from which a detection is "high"
    low: float = 0.1  # below this a detection is dropped
    new: float = 0.6  # lowest score that starts a track
    k: int = 3  # images in a row before a track is confirmed
    max_lost: int = 30  # images a lost track is kept
    cost: str = 'distance'  # 'distance' (centre, in standard deviations) or 'iou'
    gate: float = 3.0  # largest cost of a match: standard deviations ('distance') ...
    gate_low: float = 3.0  # ... for the low detections (step 3)
    min_iou: float = 0.2  # ... or the smallest overlap ('iou'); the paper's 1 - 0.8
    min_iou_low: float = 0.5  # the paper's 1 - 0.5 for the low ones
    noise: Noise = field(default_factory=Noise)
    hungarian: bool = True  # False: greedy nearest-first (as the old Confirmer)


@dataclass
class Track:
    id: int
    kalman: BoxKalman
    detection: Detection | None  # matched in the latest image, None after a miss
    state: State = State.TENTATIVE
    streak: int = 1  # images in a row with a detection
    misses: int = 0  # images in a row without one
    age: int = 1  # images since the track started
    score: float = 0.0  # of the latest matched detection

    @property
    def box(self) -> tuple[float, float, float, float]:
        """u, v (centre), w, h: the filter's, corrected with the latest detection."""
        return self.kalman.box

    @property
    def position(self) -> tuple[float, float]:
        """Centre, px (as detect.tracking.Track.position)."""
        return self.kalman.box[:2]

    @property
    def confirmed(self) -> bool:
        """Followed and seen in this image: what goes on to the attack."""
        return self.state == State.TRACKED and self.misses == 0


class ByteTracker:

    def __init__(self, settings: Settings | None = None):
        self.settings = settings or Settings()
        self.tracks: list[Track] = []  # tentative, tracked and lost
        self._next_id = 1

    def update(self, detections: list[Detection],
               homography: np.ndarray | None = None) -> list[Track]:
        """Feeds one image's detections; `homography` moves the tracks from the last image's
        pixels to this one's when the camera turned (kalman.rotation_homography), None when
        the camera's orientation is unknown. Returns the live tracks (not the lost ones)."""
        s = self.settings
        # 0. camera motion, then the filter's prediction
        if homography is not None:
            self.tracks = [t for t in self.tracks if t.kalman.compensate(homography)]
        for t in self.tracks:
            t.kalman.predict()
            t.age += 1
            t.detection = None

        # 1. high and low detections
        high = [d for d in detections if d.score >= s.high]
        low = [d for d in detections if s.low <= d.score < s.high]
        tracked = [t for t in self.tracks if t.state == State.TRACKED]
        lost = [t for t in self.tracks if t.state == State.LOST]
        tentative = [t for t in self.tracks if t.state == State.TENTATIVE]
        matched: set[int] = set()  # ids of tracks that got a detection

        # 2. high detections to followed and lost tracks
        followed = tracked + lost
        pairs, high = self._match(followed, high, low_step=False)
        for t, d in pairs:
            self._hit(t, d)
            matched.add(t.id)

        # 3. low detections to followed tracks that are still unmatched
        waiting = [t for t in tracked if t.id not in matched]
        pairs, _ = self._match(waiting, low, low_step=True)  # leftover low ones: dropped
        for t, d in pairs:
            self._hit(t, d)
            matched.add(t.id)

        # 4. high detections left to the tentative tracks; unmatched tentative ones go
        pairs, high = self._match(tentative, high, low_step=False)
        for t, d in pairs:
            self._hit(t, d)
            matched.add(t.id)

        # 5. misses: followed -> lost, too long lost -> deleted, tentative -> deleted
        kept = []
        for t in self.tracks:
            if t.id not in matched:
                t.streak = 0
                t.misses += 1
                if t.state == State.TENTATIVE or t.misses > s.max_lost:
                    continue
                t.state = State.LOST
            kept.append(t)
        self.tracks = kept

        # 6. new tracks from confident detections nobody claimed
        for d in high:
            if d.score >= s.new:
                self.tracks.append(Track(self._next_id, BoxKalman(_box(d), s.noise), d,
                                         score=d.score))
                self._next_id += 1
        for t in self.tracks:
            if t.state == State.TENTATIVE and t.streak >= s.k:
                t.state = State.TRACKED
        return [t for t in self.tracks if t.state != State.LOST]

    @property
    def confirmed(self) -> list[Track]:
        return [t for t in self.tracks if t.confirmed]

    def reset(self) -> None:
        self.tracks.clear()

    # --- matching ---------------------------------------------------------------------

    def _hit(self, track: Track, detection: Detection) -> None:
        track.kalman.update(_box(detection))
        track.detection, track.score = detection, detection.score
        track.streak += 1
        track.misses = 0
        if track.state == State.LOST:
            track.state = State.TRACKED  # back; it was confirmed before

    def _match(self, tracks: list[Track], detections: list[Detection], low_step: bool
               ) -> tuple[list[tuple[Track, Detection]], list[Detection]]:
        """The cheapest matching inside the gate; returns the pairs and the detections
        left over (in their order)."""
        if not tracks or not detections:
            return [], list(detections)
        cost, gate = self._costs(tracks, detections, low_step)
        if self.settings.hungarian and linear_sum_assignment is not None:
            big = gate + 1e6  # outside the gate: never worth taking
            rows, cols = linear_sum_assignment(np.where(cost <= gate, cost, big))
            chosen = [(r, c) for r, c in zip(rows, cols) if cost[r, c] <= gate]
        else:
            chosen, used_t, used_d = [], set(), set()
            for r, c in sorted(np.ndindex(cost.shape), key=lambda rc: cost[rc]):
                if cost[r, c] <= gate and r not in used_t and c not in used_d:
                    chosen.append((r, c))
                    used_t.add(r)
                    used_d.add(c)
        taken = {c for _, c in chosen}
        return ([(tracks[r], detections[c]) for r, c in chosen],
                [d for i, d in enumerate(detections) if i not in taken])

    def _costs(self, tracks: list[Track], detections: list[Detection], low_step: bool
               ) -> tuple[np.ndarray, float]:
        """Cost matrix (tracks x detections) and the largest cost that may be matched."""
        s = self.settings
        if s.cost == 'iou':
            cost = np.array([[1.0 - iou(t.box, _box(d)) for d in detections] for t in tracks])
            return cost, 1.0 - (s.min_iou_low if low_step else s.min_iou)
        if s.cost != 'distance':
            raise ValueError(f'Unknown cost {s.cost!r}: distance or iou')
        cost = np.array([[t.kalman.centre_distance(d.u, d.v) for d in detections]
                         for t in tracks])
        return cost, s.gate_low if low_step else s.gate


def iou(a: tuple[float, float, float, float], b: tuple[float, float, float, float]) -> float:
    """Overlap of two (u, v, w, h) boxes: intersection area / union area, 0 to 1."""
    ax0, ay0, ax1, ay1 = a[0] - a[2] / 2, a[1] - a[3] / 2, a[0] + a[2] / 2, a[1] + a[3] / 2
    bx0, by0, bx1, by1 = b[0] - b[2] / 2, b[1] - b[3] / 2, b[0] + b[2] / 2, b[1] + b[3] / 2
    w = max(0.0, min(ax1, bx1) - max(ax0, bx0))
    h = max(0.0, min(ay1, by1) - max(ay0, by0))
    inter = w * h
    union = a[2] * a[3] + b[2] * b[3] - inter
    return inter / union if union > 0 else 0.0


def _box(d: Detection) -> tuple[float, float, float, float]:
    return (d.u, d.v, float(d.width), float(d.height))
