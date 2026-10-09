"""Flight patterns for the hostile drone: where it should be at time t, at constant speed.

Every pattern is a closed loop that starts at `anchor` (the drone's current position),
leaves it in the direction `heading` and stays at the anchor's altitude. Pure math,
world NED like the rest of the backend.
"""
import math
import random
from bisect import bisect_right

from swarm_backend.config import Vector3

PATTERNS = ('line', 'circle', 'figure8', 'random')
CURVE_SAMPLES = 360  # points per smooth curve; a few cm of error on an 80 m pattern
RANDOM_WAYPOINTS = 30
TURN_RADIUS = 8.0  # metres; the tightest turn of the random pattern
STEP = 1.0  # metres between points of the random pattern


class Path:
    """A closed polyline flown at constant speed, parametrised by time."""

    def __init__(self, points: list[Vector3], speed: float):
        if len(points) < 2:
            raise ValueError('A path needs at least two points')
        if speed <= 0:
            raise ValueError(f'speed must be positive, got {speed}')
        self.points = list(points) + [points[0]]  # closed: the last leg returns to the start
        self.speed = speed
        self._starts = [0.0]  # arc length at the start of each leg
        for a, b in zip(self.points, self.points[1:]):
            self._starts.append(self._starts[-1] + math.dist(a, b))
        self.length = self._starts[-1]
        if self.length == 0:
            raise ValueError('A path needs points that are not all the same')

    def at(self, t: float) -> tuple[Vector3, Vector3]:
        """(position, velocity) at `t` seconds after the start; loops forever."""
        s = (t * self.speed) % self.length
        leg = min(bisect_right(self._starts, s) - 1, len(self.points) - 2)
        while self._starts[leg + 1] == self._starts[leg]:  # skip zero-length legs
            leg += 1
        a, b = self.points[leg], self.points[leg + 1]
        leg_length = self._starts[leg + 1] - self._starts[leg]
        f = (s - self._starts[leg]) / leg_length
        position = tuple(pa + f * (pb - pa) for pa, pb in zip(a, b))
        velocity = tuple(self.speed * (pb - pa) / leg_length for pa, pb in zip(a, b))
        return position, velocity


def make_path(pattern: str, anchor: Vector3, heading: float, speed: float, size: float,
              seed: int = 0) -> Path:
    """`size` is the line length, circle diameter, or figure-8 / random-area width (m).
    `heading` is radians from north."""
    if pattern == 'line':
        # A racetrack: two straights `size` long joined by half circles, the way a real
        # drone turns round (an instant reversal is not flyable).
        r = size / 8
        turn = [math.pi * i / (CURVE_SAMPLES // 4) for i in range(CURVE_SAMPLES // 4 + 1)]
        local = ([(size * i / 20, 0.0) for i in range(20)]
                 + [(size + r * math.sin(a), r - r * math.cos(a)) for a in turn]
                 + [(size - size * i / 20, 2 * r) for i in range(1, 20)]
                 + [(-r * math.sin(a), r + r * math.cos(a)) for a in turn[:-1]])
    elif pattern == 'circle':
        r = size / 2
        local = [(r * math.sin(a), r - r * math.cos(a)) for a in _angles()]
    elif pattern == 'figure8':
        # Lissajous 1:2 through the start point, size wide and size / 2 across.
        a = size / 2
        local = [(a * math.sin(u), a / 2 * math.sin(2 * u)) for u in _angles()]
    elif pattern == 'random':
        rng = random.Random(seed)
        waypoints = [(rng.uniform(0, size), rng.uniform(-size / 2, size / 2))
                     for _ in range(RANDOM_WAYPOINTS)]
        local = _fly_through(waypoints + [(0.0, 0.0)])
    else:
        raise ValueError(f'Unknown pattern {pattern!r}; use one of {", ".join(PATTERNS)}')
    return Path(_place(local, anchor, heading), speed)


def _angles() -> list[float]:
    return [2 * math.pi * i / CURVE_SAMPLES for i in range(CURVE_SAMPLES)]


def _fly_through(waypoints: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """The track of a vehicle that starts at the origin facing forward and steers to each
    waypoint in turn, never turning tighter than TURN_RADIUS."""
    x, y, heading = 0.0, 0.0, 0.0
    points = [(x, y)]
    max_turn = STEP / TURN_RADIUS
    for wx, wy in waypoints:
        # Give up on a waypoint we keep circling (it is inside our turn circle).
        for _ in range(int(4 * math.dist((x, y), (wx, wy)) / STEP) + 50):
            if math.dist((x, y), (wx, wy)) < 2 * STEP:
                break
            wanted = math.atan2(wy - y, wx - x)
            turn = (wanted - heading + math.pi) % (2 * math.pi) - math.pi
            heading += max(-max_turn, min(max_turn, turn))
            x, y = x + STEP * math.cos(heading), y + STEP * math.sin(heading)
            points.append((x, y))
    return points


def _place(local: list[tuple[float, float]], anchor: Vector3, heading: float) -> list[Vector3]:
    """Turn (forward, right) points starting at the origin into world points: rotated so
    the first leg points along `heading`, moved to `anchor`, at the anchor's altitude."""
    fx, fy = local[1]
    turn = heading - math.atan2(fy, fx)
    c, s = math.cos(turn), math.sin(turn)
    ax, ay, az = anchor
    return [(ax + c * x - s * y, ay + s * x + c * y, az) for x, y in local]
