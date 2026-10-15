"""What the operator UI shows and sets, as plain data. No ROS, so it can be tested anywhere.

The UI only watches the drones. The one thing it sets is the search area: an arc around
the base that the defenders should watch. How the drones cover it is up to them (or the
base station), not the operator.

Angles are degrees clockwise from north, positions world NED (north, east) in metres,
the same frame as the rest of the backend.
"""
import math
from dataclasses import asdict, dataclass

from swarm_backend.px4.drone import pitch_yaw_deg, wrap_deg

# Limits on what the operator may set.
MIN_WIDTH_DEG = 10.0
MAX_WIDTH_DEG = 180.0
MIN_RADIUS_M = 10.0
MAX_RADIUS_M = 5000.0


@dataclass(frozen=True)
class SearchArea:
    """An arc (circle sector) around the base: heading ± width/2, out to radius."""
    heading_deg: float  # centre of the arc, degrees clockwise from north
    width_deg: float  # opening angle, 90 = the quarter circle
    radius_m: float  # how far out the area goes
    centre: tuple[float, float] = (0.0, 0.0)  # the base, world NED (north, east)
    revision: int = 0  # +1 on every change, so a listener can tell old from new

    def to_dict(self) -> dict:
        return asdict(self)

    def contains(self, north: float, east: float) -> bool:
        dn, de = north - self.centre[0], east - self.centre[1]
        if math.hypot(dn, de) > self.radius_m:
            return False
        bearing = math.degrees(math.atan2(de, dn))
        return abs(wrap_deg(bearing - self.heading_deg)) <= self.width_deg / 2


def update_search_area(current: SearchArea, data: dict) -> SearchArea:
    """The area the operator asked for: `current` with the fields in `data` changed.

    Raises ValueError on unknown fields or values outside the limits. The centre (the
    base) is not something the operator moves.
    """
    allowed = {'heading_deg', 'width_deg', 'radius_m'}
    unknown = set(data) - allowed
    if unknown:
        raise ValueError(f'Unknown search area fields: {sorted(unknown)}')
    try:
        values = {k: float(v) for k, v in data.items()}
    except (TypeError, ValueError):
        raise ValueError('Search area values must be numbers') from None
    if any(not math.isfinite(v) for v in values.values()):
        raise ValueError('Search area values must be finite')

    heading = values.get('heading_deg', current.heading_deg) % 360.0
    width = values.get('width_deg', current.width_deg)
    radius = values.get('radius_m', current.radius_m)
    if not MIN_WIDTH_DEG <= width <= MAX_WIDTH_DEG:
        raise ValueError(f'width_deg must be {MIN_WIDTH_DEG:g}..{MAX_WIDTH_DEG:g}')
    if not MIN_RADIUS_M <= radius <= MAX_RADIUS_M:
        raise ValueError(f'radius_m must be {MIN_RADIUS_M:g}..{MAX_RADIUS_M:g}')
    return SearchArea(heading, width, radius, current.centre, current.revision + 1)


def drone_view(drone_id: int, position, velocity, attitude, gimbal_q, gimbal_earth_frame: bool,
               armed: bool | None, nav_state: str | None, age_s: float | None) -> dict:
    """One drone as the UI shows it. Any input may be None while PX4 has not sent it yet.

    gimbal_q is the gimbal's attitude quaternion (w, x, y, z) as PX4 reports it; its yaw is
    in the earth frame if gimbal_earth_frame, else relative to the drone's nose.
    """
    view = {'id': drone_id, 'armed': armed, 'nav_state': nav_state,
            'age_s': None if age_s is None else round(age_s, 2),
            'north': None, 'east': None, 'altitude': None, 'speed': None,
            'heading_deg': None, 'gimbal_heading_deg': None, 'gimbal_pitch_deg': None}
    if position is not None:
        view['north'], view['east'] = round(position[0], 2), round(position[1], 2)
        view['altitude'] = round(-position[2], 2)
    if velocity is not None:
        view['speed'] = round(math.hypot(velocity[0], velocity[1]), 2)
    heading = None
    if attitude is not None:
        heading = pitch_yaw_deg(attitude)[1] % 360.0
        view['heading_deg'] = round(heading, 1)
    if gimbal_q is not None:
        pitch, yaw = pitch_yaw_deg(gimbal_q)
        if not gimbal_earth_frame:
            yaw = None if heading is None else yaw + heading
        view['gimbal_pitch_deg'] = round(pitch, 1)
        view['gimbal_heading_deg'] = None if yaw is None else round(yaw % 360.0, 1)
    return view
