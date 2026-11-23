"""Friendly drones: the hivemind's broadcast of where they all are, and the check that a
detection is one of them, so the detector never locks onto our own drones.

The hivemind node (nodes/hivemind_node.py) publishes every friendly drone's position and
velocity, parked ones too, on TOPIC at about 10 Hz, stamped with the simulation time. Each
drone's detector (detect/live.py) keeps the latest message (FriendlyBook) and checks its
own tracks locally against it, on every image (friendly_for):

  direction  the box must point at the friendly: its true direction from the camera,
             predicted to the image's time with its velocity, within a gate of
             MIN_GATE_DEG plus the angle the friendly and the position error take up
             at its distance (the direction from a pixel is good to about 0.1 deg);
  distance   and the box-size distance must roughly fit (within RANGE_FACTOR), so a
             hostile far behind a friendly, in the same direction, is still a hostile.
             The box-size distance is rough (+-30 %), hence the wide factor.

A topic and a local check, not a question to the hivemind per track: no round trip
before each lock, the camera pose at the image's exact time is only known on the drone,
and the check keeps working on the last positions (predicted on, MAX_AGE_S) if a
message is late. World NED, metres, seconds. Pure Python, no ROS.
"""
import json
import math
from dataclasses import dataclass

from swarm_backend.config import Vector3

TOPIC = '/hive/friendlies'  # std_msgs/String, JSON (see encode)
RATE_HZ = 10.0
MAX_AGE_S = 2.0  # older positions are not used
FRIEND_RADIUS_M = 0.5  # half an x500 across, with its propellers
POSITION_ERROR_M = 1.0  # where the friendly may be off by, on top of ...
AGE_ERROR_M_S = 2.0  # ... this per second of prediction (it may turn or brake)
MIN_GATE_DEG = 2.0  # pointing and timing error of the camera itself
RANGE_FACTOR = 2.0  # box-size distance within 1/2 .. 2 x the friendly's: it is the friendly


@dataclass(frozen=True)
class Friendly:
    id: int
    position: Vector3  # world NED, m
    velocity: Vector3  # m/s


def encode(stamp: float, friendlies: list[Friendly]) -> str:
    return json.dumps({'t': round(stamp, 3), 'drones': [
        {'id': f.id, 'p': [round(c, 2) for c in f.position],
         'v': [round(c, 2) for c in f.velocity]} for f in friendlies]})


def decode(text: str) -> tuple[float, list[Friendly]]:
    d = json.loads(text)
    return float(d['t']), [Friendly(int(f['id']), tuple(f['p']), tuple(f['v']))
                           for f in d['drones']]


class FriendlyBook:
    """The latest broadcast, predicted to any time."""

    def __init__(self):
        self.stamp: float | None = None
        self.friendlies: list[Friendly] = []

    def update(self, stamp: float, friendlies: list[Friendly]) -> None:
        if self.stamp is None or stamp >= self.stamp:
            self.stamp, self.friendlies = stamp, list(friendlies)

    def age(self, t: float) -> float:
        return math.inf if self.stamp is None else t - self.stamp

    def at(self, t: float) -> dict[int, Vector3]:
        """id -> predicted position at `t`; empty when the broadcast is too old."""
        age = self.age(t)
        if abs(age) > MAX_AGE_S:
            return {}
        return {f.id: tuple(p + v * age for p, v in zip(f.position, f.velocity))
                for f in self.friendlies}


def gate_deg(distance: float, age_s: float = 0.0) -> float:
    """How far off (degrees) a detection's direction may be from a friendly at
    `distance` m, predicted `age_s` seconds, and still be that friendly."""
    error = FRIEND_RADIUS_M + POSITION_ERROR_M + AGE_ERROR_M_S * abs(age_s)
    return MIN_GATE_DEG + math.degrees(math.atan2(error, max(distance, 1e-6)))


def friendly_for(direction: Vector3, box_range: float | None, camera: Vector3,
                 friends: dict[int, Vector3], age_s: float = 0.0) -> int | None:
    """The id of the friendly the detection is, or None: it points at the friendly
    (`direction`, a unit vector from `camera`, within gate_deg) and its box-size distance
    `box_range` fits the friendly's (within RANGE_FACTOR; None: not checked, so it counts
    as the friendly). The closest in angle when several fit."""
    best = None
    for fid, position in friends.items():
        to = tuple(p - c for p, c in zip(position, camera))
        distance = math.hypot(*to)
        if distance < 1e-6:
            continue  # the camera's own drone
        cos = sum(a * b for a, b in zip(direction, to)) / distance
        angle = math.degrees(math.acos(max(-1.0, min(1.0, cos))))
        if angle > gate_deg(distance, age_s):
            continue
        if box_range is not None and not (distance / RANGE_FACTOR <= box_range
                                          <= distance * RANGE_FACTOR):
            continue  # same direction, but much nearer or further: not this one
        if best is None or angle < best[0]:
            best = (angle, fid)
    return None if best is None else best[1]
