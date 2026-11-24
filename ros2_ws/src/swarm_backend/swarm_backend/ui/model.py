"""What the operator UI shows and sets, as plain data. No ROS, so it can be tested anywhere.

The UI only watches the drones. What it sets is the search area: an arc around the base
that the defenders should watch. How the drones cover it is up to them (or the base
station), not the operator. And the area to map: a polygon a drone should scan into the
3D map (only stored for now; nothing flies it yet).

Angles are degrees clockwise from north, positions world NED (north, east) in metres,
the same frame as the rest of the backend.
"""
import json
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


# Limits on the area to map.
MAX_MAP_POINTS = 64
MAX_MAP_REACH_M = 5000.0  # no corner further than this from the base
MIN_MAP_AREA_M2 = 25.0
MIN_MAP_RADIUS_M = 3.0


@dataclass(frozen=True)
class MapArea:
    """The area the operator wants a drone to scan into the 3D map: a simple polygon or a
    circle, or nothing.

    points: polygon corners in world NED (north, east) metres, in drawing order.
    circle: (north, east, radius_m) instead. At most one of the two is set.
    The operator UI sends it to the exploration node (explore/area.py), which maps it with DAEP.
    """
    points: tuple[tuple[float, float], ...] = ()
    circle: tuple[float, float, float] | None = None
    revision: int = 0  # +1 on every change, like SearchArea.revision

    @property
    def shape(self) -> str | None:
        return 'circle' if self.circle else 'polygon' if self.points else None

    @property
    def area_m2(self) -> float:
        return math.pi * self.circle[2] ** 2 if self.circle else polygon_area_m2(self.points)

    def describe(self) -> str:
        """For the event log: what the area is, or '' for none."""
        if self.circle:
            return f'circle r {self.circle[2]:.0f} m, {self.area_m2:.0f} m²'
        return f'{len(self.points)} corners, {self.area_m2:.0f} m²' if self.points else ''

    def to_dict(self) -> dict:
        circle = None if not self.circle else {'centre': list(self.circle[:2]),
                                               'radius_m': self.circle[2]}
        return {'shape': self.shape, 'points': [list(p) for p in self.points], 'circle': circle,
                'revision': self.revision, 'area_m2': round(self.area_m2, 1)}


def polygon_area_m2(points) -> float:
    """Area of a simple polygon (shoelace), any winding."""
    n = len(points)
    return abs(sum(points[i][0] * points[(i + 1) % n][1] - points[(i + 1) % n][0] * points[i][1]
                   for i in range(n))) / 2 if n >= 3 else 0.0


def _segments_cross(p1, p2, q1, q2) -> bool:
    """True if segment p1-p2 properly crosses q1-q2 (touching ends do not count)."""
    def side(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    d1, d2 = side(q1, q2, p1), side(q1, q2, p2)
    d3, d4 = side(p1, p2, q1), side(p1, p2, q2)
    return d1 * d2 < 0 and d3 * d4 < 0


def update_map_area(current: MapArea, data: dict) -> MapArea:
    """The area to map the operator asked for: data = {'points': [[north, east], ...]} or
    {'circle': {'centre': [north, east], 'radius_m': r}}.

    An empty points list clears it. Raises ValueError on anything that is not a simple
    polygon of 3..MAX_MAP_POINTS corners, or a circle of at least MIN_MAP_RADIUS_M, all
    within MAX_MAP_REACH_M of the base.
    """
    unknown = set(data) - {'points', 'circle'}
    if unknown:
        raise ValueError(f'Unknown map area fields: {sorted(unknown)}')
    if len(data) != 1:
        raise ValueError('Give either points or circle')
    if 'circle' in data:
        return MapArea(circle=_circle(data['circle']), revision=current.revision + 1)
    raw = data.get('points')
    if not isinstance(raw, list):
        raise ValueError('points must be a list of [north, east]')
    try:
        points = tuple((float(p[0]), float(p[1])) for p in raw if len(p) == 2)
    except (TypeError, ValueError, KeyError):
        raise ValueError('Each point must be [north, east] in metres') from None
    if len(points) != len(raw):
        raise ValueError('Each point must be [north, east] in metres')
    if not points:
        return MapArea(revision=current.revision + 1)
    if not 3 <= len(points) <= MAX_MAP_POINTS:
        raise ValueError(f'A map area needs 3..{MAX_MAP_POINTS} corners')
    if any(not (math.isfinite(n) and math.isfinite(e)) or math.hypot(n, e) > MAX_MAP_REACH_M
           for n, e in points):
        raise ValueError(f'Corners must be finite and within {MAX_MAP_REACH_M:g} m of the base')
    edges = [(points[i], points[(i + 1) % len(points)]) for i in range(len(points))]
    for i, (a, b) in enumerate(edges):
        for c, d in edges[i + 2:]:
            if _segments_cross(a, b, c, d):
                raise ValueError('The map area’s edges cross each other')
    if polygon_area_m2(points) < MIN_MAP_AREA_M2:
        raise ValueError(f'The map area must cover at least {MIN_MAP_AREA_M2:g} m²')
    return MapArea(points, revision=current.revision + 1)


def _circle(raw) -> tuple[float, float, float]:
    """(north, east, radius_m) from {'centre': [north, east], 'radius_m': r}, checked."""
    try:
        if set(raw) != {'centre', 'radius_m'} or len(raw['centre']) != 2:
            raise ValueError
        n, e, r = float(raw['centre'][0]), float(raw['centre'][1]), float(raw['radius_m'])
    except (TypeError, ValueError, KeyError):
        raise ValueError('circle must be {"centre": [north, east], "radius_m": r}') from None
    if not all(math.isfinite(v) for v in (n, e, r)):
        raise ValueError('The circle must be finite')
    if r < MIN_MAP_RADIUS_M:
        raise ValueError(f'The circle needs a radius of at least {MIN_MAP_RADIUS_M:g} m')
    if math.hypot(n, e) + r > MAX_MAP_REACH_M:
        raise ValueError(f'The circle must stay within {MAX_MAP_REACH_M:g} m of the base')
    return n, e, r


EARTH_RADIUS_M = 6371000.0


def latlon_offset_m(lat: float, lon: float, lat0: float, lon0: float) -> tuple[float, float]:
    """(north, east) metres from (lat0, lon0) to (lat, lon), degrees. Flat earth: good to
    centimetres over the few km of a Gazebo world."""
    north = math.radians(lat - lat0) * EARTH_RADIUS_M
    east = math.radians(lon - lon0) * EARTH_RADIUS_M * math.cos(math.radians(lat0))
    return north, east


def sdf_origin(sdf_text: str) -> tuple[float, float] | None:
    """The (lat, lon) of world (0, 0) from a world SDF's <spherical_coordinates>: what
    Gazebo's NavSat, and so every PX4's GPS, is based on. Exact, unlike world_origin's
    estimate from the spawn points in swarm.yaml, which are wrong whenever the drones were
    started with another config. None if the world has none, or turns its frame (a
    heading other than 0 is not handled here)."""
    import xml.etree.ElementTree as ET
    try:
        root = ET.fromstring(sdf_text)
    except ET.ParseError:
        return None
    sc = root.find('.//world/spherical_coordinates')
    if sc is None:
        return None

    def number(tag, default=None):
        el = sc.find(tag)
        try:
            return float(el.text) if el is not None and el.text else default
        except ValueError:
            return None

    lat, lon = number('latitude_deg'), number('longitude_deg')
    heading = number('heading_deg', 0.0)
    if lat is None or lon is None or heading is None or abs(heading) > 1e-9:
        return None
    return lat, lon


def world_origin(refs: dict, spawns: dict, skip=()) -> tuple[float, float] | None:
    """The (lat, lon) of world NED (0, 0), from the drones' local frame origins.

    `refs`: drone id -> (ref_lat, ref_lon), where each PX4's local frame starts (its
    VehicleLocalPosition). `spawns`: drone id -> spawn_ned from swarm.yaml. A drone still
    where it was spawned gives origin = its ref minus its spawn. `skip`: drones that may
    have been moved and restarted (the hostile drone); the median of the rest, so one
    that was moved anyway does not shift the others. None while no drone has reported."""
    candidates = []
    for drone_id, (lat, lon) in refs.items():
        if drone_id in skip or drone_id not in spawns:
            continue
        north, east = spawns[drone_id][0], spawns[drone_id][1]
        lat0 = lat - math.degrees(north / EARTH_RADIUS_M)
        lon0 = lon - math.degrees(east / (EARTH_RADIUS_M * math.cos(math.radians(lat0))))
        candidates.append((lat0, lon0))
    if not candidates:
        return None
    lats, lons = sorted(c[0] for c in candidates), sorted(c[1] for c in candidates)
    middle = len(candidates) // 2
    return lats[middle], lons[middle]


def drone_home(spawn_ned, ref, origin) -> tuple[float, float, float]:
    """Where this drone's PX4 local frame starts, world NED: from its ref (lat, lon) when
    it and the world origin are known, so a drone that was moved and restarted (the
    scenarios do this with the hostile drone every run) is placed right; else its spawn."""
    if ref is None or origin is None:
        return tuple(spawn_ned)
    north, east = latlon_offset_m(ref[0], ref[1], origin[0], origin[1])
    return (north, east, spawn_ned[2])


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


# What the attack node's states mean to the operator.
ATTACK_STATUS = {'search': 'searching', 'attack': 'intercept', 'coast': 'intercept',
                 'lost': 'target lost', 'hit': 'hit'}
# PX4 flight modes worth naming; any other mode while armed is just 'flying'.
NAV_STATUS = {'AUTO_TAKEOFF': 'taking off', 'AUTO_LAND': 'landing', 'AUTO_RTL': 'returning'}


def drone_status(attack_state: str | None, armed: bool | None, nav_state: str | None,
                 age_s: float | None, stale_s: float) -> str:
    """The one word the operator needs: what the drone is doing right now.

    The attack node's state wins while it has one (it says 'idle' between attacks); without
    it, the status comes from PX4.
    """
    if age_s is None or age_s > stale_s:
        return 'no link'
    if attack_state in ATTACK_STATUS:
        return ATTACK_STATUS[attack_state]
    if not armed:
        return 'landed'
    return NAV_STATUS.get(nav_state, 'flying')


def attack_view(raw: str) -> dict | None:
    """The attack node's status (JSON of intercept.attack.attack_status) as the UI shows it,
    or None if it is not valid JSON. The target becomes north/east/altitude like a drone."""
    try:
        status = json.loads(raw)
    except (TypeError, ValueError):
        return None
    if not isinstance(status, dict):
        return None
    target = status.get('target')
    if isinstance(target, list) and len(target) == 3:
        status['target'] = {'north': target[0], 'east': target[1], 'altitude': -target[2]}
    else:
        status['target'] = None
    velocity = status.get('velocity')
    if isinstance(velocity, list) and len(velocity) == 3:
        status['velocity'] = {'north': velocity[0], 'east': velocity[1], 'up': -velocity[2]}
    else:
        status['velocity'] = None
    return status


MAP_KEYS = ('north_min', 'north_max', 'east_min', 'east_max')


def load_map(folder) -> dict | None:
    """The background map's extent (map.json from map_snapshot.py) if folder has map.png
    and a valid map.json, else None. Adds 'version' (the PNG's mtime) so the page reloads
    the picture when it changes."""
    from pathlib import Path
    folder = Path(folder)
    png, meta = folder / 'map.png', folder / 'map.json'
    if not png.is_file() or not meta.is_file():
        return None
    try:
        info = json.loads(meta.read_text())
        extent = {k: float(info[k]) for k in MAP_KEYS}
    except (ValueError, KeyError, TypeError):
        return None
    if not (extent['north_max'] > extent['north_min'] and extent['east_max'] > extent['east_min']):
        return None
    return {**extent, 'world': info.get('world'), 'version': int(png.stat().st_mtime)}


# --- Event log -------------------------------------------------------------------------
# What happened during the run, for the operator: found by comparing each drone's view
# with the one before, so no other node has to send anything. Kinds, for the page's
# filters: 'fleet' (link, arming, flight mode), 'detect', 'attack', 'operator'. Level
# 'good' / 'bad' / None colours the entry.

ENGAGED = ('intercept', 'target lost')  # statuses that belong to an attack
FLIGHT_EVENTS = {'taking off': 'taking off', 'landing': 'landing', 'returning': 'returning home'}


@dataclass(frozen=True)
class Event:
    id: int  # 1, 2, ... so the page can ask for the ones after the last it has
    t: float  # seconds since the log started
    kind: str  # 'fleet' | 'detect' | 'attack' | 'operator'
    text: str
    drone: int | None = None  # the drone it is about, if any
    detail: str = ''  # numbers that go with it, shown smaller
    level: str | None = None  # 'good' | 'bad' | None

    def to_dict(self) -> dict:
        return asdict(self)


def _who(view: dict) -> str:
    # The target too is just 'Drone N': the UI shows it as unknown until a defender spots it.
    return f"Drone {view['id']}"


class EventLog:
    """Keeps the last `size` events. Feed it every drone view (update) about as often as
    the page polls; add() is for events that do not come from the views (search area)."""

    def __init__(self, size: int = 500):
        self.size = size
        self.events: list[Event] = []
        self._next_id = 1
        self._last: dict[int, dict] = {}  # drone id -> the view update() saw last
        self._seen_at: dict[int, float] = {}  # drone id -> when it first saw this target

    def add(self, t: float, kind: str, text: str, drone: int | None = None, detail: str = '',
            level: str | None = None) -> Event:
        event = Event(self._next_id, round(t, 1), kind, text, drone, detail, level)
        self._next_id += 1
        self.events.append(event)
        del self.events[:-self.size]
        return event

    def after(self, event_id: int = 0, limit: int = 200) -> list[dict]:
        """The events newer than event_id, oldest first, at most the last `limit`."""
        return [e.to_dict() for e in self.events if e.id > event_id][-limit:]

    def update(self, t: float, views: list[dict]) -> None:
        for view in views:
            before = self._last.get(view['id'])
            self._compare(t, before, view)
            self._last[view['id']] = view

    def _compare(self, t: float, before: dict | None, now: dict) -> None:
        who, drone = _who(now), now['id']
        linked = now.get('status') != 'no link'
        was_linked = before is not None and before.get('status') != 'no link'
        if linked and not was_linked:
            self.add(t, 'fleet', f'{who} link up' if before is None or before.get('armed') is None
                     else f'{who} link back', drone, level='good')
        elif was_linked and not linked:
            self.add(t, 'fleet', f'{who} link lost', drone,
                     f'last message {now.get("age_s") or 0:.0f} s ago', level='bad')
        if not linked or before is None or not was_linked:
            return

        if now.get('armed') and not before.get('armed'):
            self.add(t, 'fleet', f'{who} armed', drone)
        elif before.get('armed') and now.get('armed') is False:
            self.add(t, 'fleet', f'{who} landed and disarmed', drone)

        status, old = now.get('status'), before.get('status')
        if status != old and status in FLIGHT_EVENTS:
            self.add(t, 'fleet', f'{who} {FLIGHT_EVENTS[status]}', drone)

        if now.get('detection') and not before.get('detection'):
            if drone not in self._seen_at:
                self._seen_at[drone] = t
            d = now['detection']
            self.add(t, 'detect', f'{who} sees the target', drone,
                     f'{d["altitude"]:.0f} m up' if d.get('altitude') is not None else '')
        elif before.get('detection') and not now.get('detection'):
            self.add(t, 'detect', f'{who} lost sight of the target', drone)

        if status != old:
            self._attack_change(t, who, drone, old, status, now.get('attack') or {})

    def _attack_change(self, t, who, drone, old, status, attack) -> None:
        range_m = attack.get('range_m')
        rng = f'range {range_m:.0f} m' if range_m is not None else ''
        if status == 'intercept':
            self._seen_at.setdefault(drone, t)
            self.add(t, 'attack', f'{who} {old} → intercept', drone, rng, 'bad')
        elif status == 'target lost':
            seen_ago = attack.get('seen_s_ago')
            self.add(t, 'attack', f'{who} lost the target', drone,
                     f'last seen {seen_ago:.1f} s ago' if seen_ago is not None else '', 'bad')
        elif status == 'hit':
            seen = self._seen_at.pop(drone, None)
            self.add(t, 'attack', f'{who} hit the target', drone,
                     f'{t - seen:.1f} s after it first saw it' if seen is not None else '', 'good')
        elif status == 'searching' and old in ENGAGED + ('hit',):
            self._seen_at.pop(drone, None)
            self.add(t, 'attack', f'{who} back to searching', drone)
