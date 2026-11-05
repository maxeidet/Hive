"""Which sightings are the same target. The drones' cameras do not name what they see, so
the base station gives the target ids: a sighting near a recent one is the same target,
anything else is new. Pure Python, world positions in metres."""
import math
from dataclasses import dataclass

Vector3 = tuple[float, float, float]


@dataclass
class _Track:
    position: Vector3
    time: float


class TargetAssociator:

    def __init__(self, gate_m: float, expiry_s: float):
        self.gate_m = gate_m
        self.expiry_s = expiry_s
        self._tracks: dict[str, _Track] = {}
        self._count = 0

    def identify(self, position: Vector3, now: float) -> str:
        """The id of the target at `position`, new if no recent target is within the gate."""
        self._tracks = {k: t for k, t in self._tracks.items() if now - t.time <= self.expiry_s}
        best, best_distance = None, self.gate_m
        for target_id, track in self._tracks.items():
            distance = math.dist(position, track.position)
            if distance <= best_distance:
                best, best_distance = target_id, distance
        if best is None:
            self._count += 1
            best = f'T{self._count}'
        self._tracks[best] = _Track(position, now)
        return best

    def forget(self, target_id: str) -> None:
        self._tracks.pop(target_id, None)
