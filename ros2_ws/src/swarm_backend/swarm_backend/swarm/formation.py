"""Formation geometry: target positions for N drones.

Pure math on (x, y, z) tuples in the swarm world frame (NED: x north, y east,
z down). Nothing here knows that ROS 2 or PX4 exist.
"""
import itertools
import math
from collections.abc import Sequence

Point = tuple[float, float, float]

# Above this many drones, trying every assignment gets slow; fall back to greedy.
MAX_EXACT_ASSIGNMENT = 8


class Formation:

    def __init__(self, size: int):
        if size < 1:
            raise ValueError('A formation needs at least one drone')
        self.size = size

    def grid(self, center: Point, spacing: float) -> list[Point]:
        """Rows along north, as square as possible, centred on `center`."""
        cols = math.ceil(math.sqrt(self.size))
        rows = math.ceil(self.size / cols)
        cx, cy, cz = center
        points = []
        for i in range(self.size):
            row, col = divmod(i, cols)
            points.append((
                cx + (row - (rows - 1) / 2) * spacing,
                cy + (col - (cols - 1) / 2) * spacing,
                cz,
            ))
        return points

    def line(self, center: Point, spacing: float, heading: float = math.pi / 2) -> list[Point]:
        """A straight line through `center` pointing along `heading` (radians from north).

        The default heading is east, i.e. drones side by side when the swarm flies north.
        """
        cx, cy, cz = center
        dx, dy = math.cos(heading), math.sin(heading)
        return [
            (cx + offset * dx, cy + offset * dy, cz)
            for offset in ((i - (self.size - 1) / 2) * spacing for i in range(self.size))
        ]

    def circle(self, center: Point, radius: float) -> list[Point]:
        """Evenly spaced on a horizontal circle, the first drone due north of `center`."""
        cx, cy, cz = center
        if self.size == 1:
            return [(cx, cy, cz)]
        return [
            (cx + radius * math.cos(angle), cy + radius * math.sin(angle), cz)
            for angle in (2 * math.pi * i / self.size for i in range(self.size))
        ]


def circle_radius_for_spacing(size: int, spacing: float) -> float:
    """Radius at which `size` drones on a circle are `spacing` apart from their neighbours."""
    if size < 2:
        return 0.0
    return spacing / (2 * math.sin(math.pi / size))


def translate(points: Sequence[Point], offset: Point) -> list[Point]:
    ox, oy, oz = offset
    return [(x + ox, y + oy, z + oz) for x, y, z in points]


def centroid(points: Sequence[Point]) -> Point:
    n = len(points)
    return (
        sum(p[0] for p in points) / n,
        sum(p[1] for p in points) / n,
        sum(p[2] for p in points) / n,
    )


def assign_targets(positions: Sequence[Point], targets: Sequence[Point]) -> list[Point]:
    """Reorder `targets` so targets[i] is the one drone i should fly to.

    Minimises the total distance flown, which also keeps paths from crossing when
    switching formations. Exact for small swarms, greedy (closest pair first) above that.
    """
    if len(positions) != len(targets):
        raise ValueError(f'{len(positions)} drones but {len(targets)} targets')
    n = len(positions)
    if n <= MAX_EXACT_ASSIGNMENT:
        best = min(
            itertools.permutations(range(n)),
            key=lambda perm: sum(math.dist(positions[i], targets[perm[i]]) for i in range(n)),
        )
        return [targets[j] for j in best]

    pairs = sorted(
        (math.dist(p, t), i, j) for i, p in enumerate(positions) for j, t in enumerate(targets))
    assigned: dict[int, int] = {}
    used_targets = set()
    for _, i, j in pairs:
        if i not in assigned and j not in used_targets:
            assigned[i] = j
            used_targets.add(j)
    return [targets[assigned[i]] for i in range(n)]
