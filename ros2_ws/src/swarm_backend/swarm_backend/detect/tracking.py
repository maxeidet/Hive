"""Associate detections between images and confirm targets seen k images in a row.

Positions are plain vectors and the gate is a distance on them, so the same tracker runs
on pixel centres today and on world directions (unit vectors, angle gate) once the camera
math exists. Matching in world direction is the goal: when the gimbal turns, the target
jumps across the image but stays put in world direction.

Greedy nearest neighbour inside the gate; ByteTrack replaces it behind the same interface
when YOLO is added.
"""
import math
from dataclasses import dataclass, field
from typing import Callable, Sequence

Position = Sequence[float]


def euclidean(a: Position, b: Position) -> float:
    return math.dist(a, b)


def angle_between(a: Position, b: Position) -> float:
    """Radians between two unit vectors."""
    dot = sum(x * y for x, y in zip(a, b))
    return math.acos(max(-1.0, min(1.0, dot)))


@dataclass
class Track:
    id: int
    position: Position  # of the latest detection
    detection: object = None  # the latest detection itself (e.g. a Box), None after a miss
    streak: int = 1  # images in a row with a detection
    misses: int = 0  # images in a row without one
    confirmed: bool = False
    age: int = 1  # images since the track started


@dataclass
class Confirmer:
    k: int = 3  # images in a row before a track is confirmed
    gate: float = 20.0  # largest distance between consecutive detections of one target
    max_misses: int = 2  # a track is dropped after this many images in a row without a hit
    distance: Callable[[Position, Position], float] = euclidean
    tracks: list[Track] = field(default_factory=list)
    _next_id: int = 1

    def update(self, positions: list[Position], detections: list | None = None) -> list[Track]:
        """Feed one image's detections; returns the live tracks.

        A miss resets the streak, so a confirmed track that is not seen in one image is no
        longer confirmed until it has been seen k images in a row again.
        """
        if detections is None:
            detections = [None] * len(positions)
        pairs = sorted(
            (self.distance(track.position, p), ti, di)
            for ti, track in enumerate(self.tracks)
            for di, p in enumerate(positions))
        used_tracks, used_detections = set(), set()
        for d, ti, di in pairs:
            if d > self.gate or ti in used_tracks or di in used_detections:
                continue
            used_tracks.add(ti)
            used_detections.add(di)
            track = self.tracks[ti]
            track.position, track.detection = positions[di], detections[di]
            track.streak += 1
            track.misses = 0

        for ti, track in enumerate(self.tracks):
            track.age += 1
            if ti not in used_tracks:
                track.detection = None
                track.streak = 0
                track.misses += 1
        self.tracks = [t for t in self.tracks if t.misses <= self.max_misses]

        for di, p in enumerate(positions):
            if di not in used_detections:
                self.tracks.append(Track(self._next_id, p, detections[di]))
                self._next_id += 1

        for track in self.tracks:
            track.confirmed = track.streak >= self.k
        return self.tracks

    @property
    def confirmed(self) -> list[Track]:
        return [t for t in self.tracks if t.confirmed]
