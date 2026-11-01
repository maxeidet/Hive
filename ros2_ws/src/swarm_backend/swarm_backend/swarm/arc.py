"""The surveillance arc: where the drones hold their points and which slice each one sweeps.

Pure math on (x, y, z) tuples in the swarm world frame (NED: x north, y east, z down),
like formation.py. Angles are degrees from the threat heading, positive to the right.
Nothing here knows that ROS 2 or PX4 exist.
"""
import math
from collections.abc import Sequence

from swarm_backend.swarm.formation import Point

DEFAULT_ARC_DEG = 90.0


def slot_angles(n: int, arc_deg: float = DEFAULT_ARC_DEG) -> list[float]:
    """The angle of each of `n` slots: the centre of each of `n` equal slices of the arc
    (-30, 0, 30 for three slots on 90 degrees)."""
    if n < 1:
        raise ValueError('An arc needs at least one slot')
    width = arc_deg / n
    return [(i - (n - 1) / 2) * width for i in range(n)]


def arc_points(center: Point, heading: float, radius: float, n: int,
               arc_deg: float = DEFAULT_ARC_DEG) -> list[Point]:
    """Hold points for `n` drones on a horizontal arc of `radius` around `center`, at the
    same height as `center`. `heading` is the threat heading, radians from north."""
    cx, cy, cz = center
    points = []
    for angle in slot_angles(n, arc_deg):
        direction = heading + math.radians(angle)
        points.append((cx + radius * math.cos(direction), cy + radius * math.sin(direction), cz))
    return points


def sectors(slots: Sequence[int], arc_deg: float = DEFAULT_ARC_DEG) -> dict[int, tuple[float, float]]:
    """Split the arc evenly among the drones still watching it, in slot order.

    Returns slot -> (from, to) in degrees from the threat heading. Three slots give 30
    degrees each; when the middle one is gone the two others get 45 each (option A: nobody
    moves, the sweeps widen)."""
    ordered = sorted(slots)
    if not ordered:
        return {}
    width = arc_deg / len(ordered)
    return {slot: (-arc_deg / 2 + k * width, -arc_deg / 2 + (k + 1) * width)
            for k, slot in enumerate(ordered)}


def aim_bearing_deg(drone: Point, base: tuple[float, float], heading: float, phi_deg: float,
                    aim_distance: float) -> float:
    """The direction (degrees from north) from a drone to the point `aim_distance` from the
    base in direction `phi_deg` off the threat heading (`heading`, radians). Sectors are
    angles seen from the base, so a drone that sits on the arc aims its camera at that
    point rather than along `phi_deg`."""
    direction = heading + math.radians(phi_deg)
    north = base[0] + aim_distance * math.cos(direction) - drone[0]
    east = base[1] + aim_distance * math.sin(direction) - drone[1]
    return math.degrees(math.atan2(east, north))
