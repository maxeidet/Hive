"""Friendly drones: the hivemind's broadcast of where they all are, and the check that a
detection is one of them, so the detector never locks onto our own drones.

The hivemind node (nodes/hivemind_node.py) publishes every friendly drone's position and
velocity, parked ones too, on TOPIC at about 10 Hz. Each drone carries its own stamp: the
simulation time its odometry arrived, not the time of the broadcast (which is up to a
timer period later: at 15 m/s that alone was 1.5 m). Each drone's detector (detect/live.py)
keeps the latest message (FriendlyBook) and checks its own tracks locally against it, on
every image (friendly_for):

  direction  the box must point at the friendly: its true direction from the camera,
             predicted to the image's time with its velocity, within a gate of
             MIN_GATE_DEG plus the angle the friendly and the position error take up
             at its distance (the direction from a pixel is good to about 0.1 deg);
  distance   and the box-size distance must roughly fit (within RANGE_FACTOR), so a
             hostile far behind a friendly, in the same direction, is still a hostile.
             The box-size distance is rough (+-30 %), hence the wide factor.

A track becomes friendly after CONFIRM_FRAMES images in a row that fit the same friendly,
and stops being friendly after RELEASE_FRAMES images in a row (with a box) that fit none
(FriendlyMarks): one bad image, or a tracker that hands a friendly's track id to a hostile,
no longer makes a hostile friendly for good.

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
RESTART_S = 1.0  # a message this much older than the last: the simulation was restarted
FRIEND_RADIUS_M = 0.5  # half an x500 across, with its propellers
POSITION_ERROR_M = 1.0  # where the friendly may be off by, on top of ...
AGE_ERROR_M_S = 2.0  # ... this per second of prediction (it may turn or brake)
MIN_GATE_DEG = 2.0  # pointing and timing error of the camera itself
RANGE_FACTOR = 2.0  # box-size distance within 1/2 .. 2 x the friendly's: it is the friendly
CONFIRM_FRAMES = 3  # images in a row that fit a friendly before its track is friendly
RELEASE_FRAMES = 5  # images in a row (with a box) that fit no friendly before it is not


@dataclass(frozen=True)
class Friendly:
    id: int
    position: Vector3  # world NED, m
    velocity: Vector3  # m/s
    stamp: float | None = None  # simulation time of the position; None: the message's


def encode(stamp: float, friendlies: list[Friendly]) -> str:
    def one(f: Friendly) -> dict:
        d = {'id': f.id, 'p': [round(c, 2) for c in f.position],
             'v': [round(c, 2) for c in f.velocity]}
        if f.stamp is not None:
            d['t'] = round(f.stamp, 3)
        return d
    return json.dumps({'t': round(stamp, 3), 'drones': [one(f) for f in friendlies]})


def decode(text: str) -> tuple[float, list[Friendly]]:
    d = json.loads(text)
    return float(d['t']), [Friendly(int(f['id']), tuple(f['p']), tuple(f['v']),
                                    float(f['t']) if 't' in f else None)
                           for f in d['drones']]


class FriendlyBook:
    """The latest broadcast, predicted to any time."""

    def __init__(self):
        self.stamp: float | None = None
        self.friendlies: list[Friendly] = []

    def update(self, stamp: float, friendlies: list[Friendly]) -> None:
        """Keep the newest; one more than RESTART_S older means the clock started over
        (Gazebo restarted), and it replaces the old one."""
        if self.stamp is None or stamp >= self.stamp or stamp < self.stamp - RESTART_S:
            self.stamp, self.friendlies = stamp, list(friendlies)

    def age(self, t: float) -> float:
        """Seconds since the broadcast was sent."""
        return math.inf if self.stamp is None else t - self.stamp

    def get(self, drone_id: int) -> tuple[float, Friendly] | None:
        """(the time its position is from, the friendly) as last broadcast, or None."""
        for f in self.friendlies:
            if f.id == drone_id:
                return (self.stamp if f.stamp is None else f.stamp), f
        return None

    def ages(self, t: float) -> dict[int, float]:
        """id -> seconds from each friendly's position to `t`, the ones not too old."""
        if self.stamp is None:
            return {}
        out = {}
        for f in self.friendlies:
            age = t - (self.stamp if f.stamp is None else f.stamp)
            if abs(age) <= MAX_AGE_S:
                out[f.id] = age
        return out

    def at(self, t: float) -> dict[int, Vector3]:
        """id -> position predicted to `t` from its own time; too old ones left out."""
        ages = self.ages(t)
        return {f.id: tuple(p + v * ages[f.id] for p, v in zip(f.position, f.velocity))
                for f in self.friendlies if f.id in ages}


def gate_deg(distance: float, age_s: float = 0.0) -> float:
    """How far off (degrees) a detection's direction may be from a friendly at
    `distance` m, predicted `age_s` seconds, and still be that friendly."""
    error = FRIEND_RADIUS_M + POSITION_ERROR_M + AGE_ERROR_M_S * abs(age_s)
    return MIN_GATE_DEG + math.degrees(math.atan2(error, max(distance, 1e-6)))


def friendly_for(direction: Vector3, box_range: float | None, camera: Vector3,
                 friends: dict[int, Vector3],
                 age_s: float | dict[int, float] = 0.0) -> int | None:
    """The id of the friendly the detection is, or None: it points at the friendly
    (`direction`, a unit vector from `camera`, within gate_deg) and its box-size distance
    `box_range` fits the friendly's (within RANGE_FACTOR; None: not checked, so it counts
    as the friendly). The closest in angle when several fit. `age_s`: how far each
    position was predicted, one for all or per id (FriendlyBook.ages)."""
    best = None
    for fid, position in friends.items():
        age = age_s.get(fid, 0.0) if isinstance(age_s, dict) else age_s
        to = tuple(p - c for p, c in zip(position, camera))
        distance = math.hypot(*to)
        if distance < 1e-6:
            continue  # the camera's own drone
        cos = sum(a * b for a, b in zip(direction, to)) / distance
        angle = math.degrees(math.acos(max(-1.0, min(1.0, cos))))
        if angle > gate_deg(distance, age):
            continue
        if box_range is not None and not (distance / RANGE_FACTOR <= box_range
                                          <= distance * RANGE_FACTOR):
            continue  # same direction, but much nearer or further: not this one
        if best is None or angle < best[0]:
            best = (angle, fid)
    return None if best is None else best[1]


class FriendlyMarks:
    """Which tracks are friendly, with CONFIRM_FRAMES to become one and RELEASE_FRAMES to
    stop: update() gets, for each track with a box in this image, the friendly it fits
    (friendly_for) or None. Tracks without a box keep what they are."""

    def __init__(self, confirm: int = CONFIRM_FRAMES, release: int = RELEASE_FRAMES):
        self.confirm, self.release = confirm, release
        self.marked: dict[int, int] = {}  # track id -> friendly drone id
        self._hits: dict[int, tuple[int, int]] = {}  # track id -> (friendly, images in a row)
        self._misses: dict[int, int] = {}  # marked track id -> images in a row fitting none

    def clear(self) -> None:
        self.marked.clear()
        self._hits.clear()
        self._misses.clear()

    def prune(self, live: set[int]) -> dict[int, int]:
        """Forget tracks that are gone; the marked ones."""
        for d in (self.marked, self._hits, self._misses):
            for track_id in [i for i in d if i not in live]:
                del d[track_id]
        return dict(self.marked)

    def update(self, fits: dict[int, int | None], live: set[int]) -> dict[int, int]:
        """`fits`: track id -> the friendly it fits in this image, or None. Returns the
        marked tracks (track id -> friendly id)."""
        self.prune(live)
        for track_id, friend in fits.items():
            if friend is None:
                self._hits.pop(track_id, None)
                if track_id in self.marked:
                    misses = self._misses.get(track_id, 0) + 1
                    if misses >= self.release:
                        del self.marked[track_id]
                        self._misses.pop(track_id, None)
                    else:
                        self._misses[track_id] = misses
                continue
            self._misses.pop(track_id, None)
            if track_id in self.marked:
                self.marked[track_id] = friend
                continue
            last, count = self._hits.get(track_id, (friend, 0))
            count = count + 1 if last == friend else 1
            if count >= self.confirm:
                self.marked[track_id] = friend
                self._hits.pop(track_id, None)
            else:
                self._hits[track_id] = (friend, count)
        return dict(self.marked)
