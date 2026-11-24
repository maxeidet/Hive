"""The exploration node's messages and logic, without ROS (explore_node.py does the ROS part).

Topics (JSON in std_msgs/String, like swarm/messages.py):

  /explore/command   -> explore_node   start mapping an area, or stop
  /explore/status    <- explore_node   what the exploration is doing (latched)

A start command carries the area the way the operator UI's Map area tab holds it (world NED,
north and east in metres, see ui/model.py MapArea), and optionally the heights:

  {"command": "start", "area": {"circle": {"centre": [n, e], "radius_m": r}}}
  {"command": "start", "area": {"points": [[n, e], ...]}, "alt": 23, "z_min": 2, "z_max": 24}
  {"command": "stop"}

DAEP explores an axis-aligned box only, so a circle or polygon becomes the box around it:
the drone maps a little more than was drawn.
"""
import json
import math
import re
from dataclasses import dataclass, field, replace

COMMAND_TOPIC = '/explore/command'
STATUS_TOPIC = '/explore/status'

# Heights in metres above the ground, used when a start command leaves them out: low enough
# for the vehicles and tents around the camp, as in the test flights. Over trees, send higher
# ones (alt above the tallest tree, see scenarios/hive_base_forest.txt).
DEFAULT_ALT = 4.0  # flight to the area, and where DAEP starts
DEFAULT_Z_MIN = 0.5
DEFAULT_Z_MAX = 5.0
MIN_HALF_SIZE_M = 2.0  # a smaller box leaves DAEP no room to sample

# What the exploration is doing, in order. DONE, STOPPED and FAILED end it.
IDLE, STARTING, TAKING_OFF, FLYING_TO_AREA, EXPLORING, LANDING, DONE, STOPPING, STOPPED, \
    FAILED = ('idle', 'starting', 'taking_off', 'flying_to_area', 'exploring', 'landing',
              'done', 'stopping', 'stopped', 'failed')
FINISHED = (IDLE, DONE, STOPPED, FAILED)


@dataclass(frozen=True)
class Box:
    """The box DAEP explores, in the Gazebo world frame (ENU: x east, y north, z up)."""
    center: tuple[float, float]  # x, y
    half: tuple[float, float]  # half the side along x and y
    z_min: float
    z_max: float
    alt: float

    def launch_args(self) -> list[str]:
        """Arguments for launch/daep_exploration.launch.py."""
        return [f'center:={self.center[0]:g},{self.center[1]:g}',
                f'radius:={self.half[0]:g},{self.half[1]:g}',
                f'z_min:={self.z_min:g}', f'z_max:={self.z_max:g}', f'alt:={self.alt:g}']

    def to_dict(self) -> dict:
        (x, y), (hx, hy) = self.center, self.half
        return {'min': [x - hx, y - hy, self.z_min], 'max': [x + hx, y + hy, self.z_max],
                'alt': self.alt}


@dataclass(frozen=True)
class Command:
    kind: str  # 'start' or 'stop'
    box: Box | None = None


def box_around(area: dict, alt: float = DEFAULT_ALT, z_min: float = DEFAULT_Z_MIN,
               z_max: float = DEFAULT_Z_MAX) -> Box:
    """The ENU box around an area in NED: {'circle': {'centre': [n, e], 'radius_m': r}} or
    {'points': [[n, e], ...]}. Raises ValueError on a bad area or heights."""
    if not z_min < alt < z_max:
        raise ValueError(f'Need z_min < alt < z_max, got {z_min:g} < {alt:g} < {z_max:g}')
    if z_min < 0:
        raise ValueError('z_min must be above the ground')
    circle, points = area.get('circle'), area.get('points')
    try:
        if circle:
            north, east = (float(v) for v in circle['centre'])
            r = float(circle['radius_m'])
            center, half = (east, north), (r, r)
        elif points:
            norths, easts = zip(*((float(p[0]), float(p[1])) for p in points))
            if len(norths) < 3:
                raise ValueError
            center = ((min(easts) + max(easts)) / 2, (min(norths) + max(norths)) / 2)
            half = ((max(easts) - min(easts)) / 2, (max(norths) - min(norths)) / 2)
        else:
            raise ValueError
    except (TypeError, ValueError, KeyError):
        raise ValueError('The area must be a circle {"centre": [n, e], "radius_m": r} or '
                         'points [[n, e], ...] (at least 3)') from None
    if not all(math.isfinite(v) for v in (*center, *half)):
        raise ValueError('The area must be finite')
    if min(half) < MIN_HALF_SIZE_M:
        raise ValueError(f'The area must be at least {2 * MIN_HALF_SIZE_M:g} m across')
    return Box(center, half, float(z_min), float(z_max), float(alt))


def parse_command(text: str) -> Command:
    """A Command from the JSON on /explore/command. Raises ValueError if it is not one."""
    try:
        data = json.loads(text)
    except json.JSONDecodeError:
        raise ValueError('The command is not JSON') from None
    if not isinstance(data, dict) or data.get('command') not in ('start', 'stop'):
        raise ValueError('command must be "start" or "stop"')
    if data['command'] == 'stop':
        return Command('stop')
    if not isinstance(data.get('area'), dict):
        raise ValueError('A start command needs an area')
    heights = {k: float(data[k]) for k in ('alt', 'z_min', 'z_max') if data.get(k) is not None}
    return Command('start', box_around(data['area'], **heights))


@dataclass(frozen=True)
class Status:
    """What /explore/status says."""
    state: str = IDLE
    detail: str = ''
    box: Box | None = None
    coverage_pct: float | None = None
    map_path: str | None = None
    run: int = 0  # +1 for every exploration started

    def to_json(self) -> str:
        return json.dumps({'state': self.state, 'detail': self.detail, 'run': self.run,
                           'box': self.box.to_dict() if self.box else None,
                           'coverage_pct': self.coverage_pct, 'map': self.map_path})


# Lines of the launch's output that move the exploration on (daep_bridge, rpl_exploration,
# save_map), and the state they lead to.
_PROGRESS = [
    (re.compile(r'daep\.px4_bridge\]: Taking off'), TAKING_OFF),
    (re.compile(r'daep\.px4_bridge\]: Flying to the area'), FLYING_TO_AREA),
    (re.compile(r'daep\.px4_bridge\]: In the air'), EXPLORING),
    (re.compile(r'exploration complete!', re.IGNORECASE), LANDING),
]
_MAP_SAVED = re.compile(r'Map saved: (\S+)')
_LANDED = re.compile(r'daep\.px4_bridge\.drone\d+\]: disarmed')
_DIED = re.compile(r'\[ERROR\] \[([\w.]+)-\d+\]: process has died')


@dataclass
class Progress:
    """Follows one exploration through the launch's output lines."""
    status: Status = field(default_factory=Status)
    landed: bool = False

    def line(self, text: str) -> None:
        s = self.status
        if s.state in FINISHED or s.state == STOPPING and not _LANDED.search(text):
            return
        for pattern, state in _PROGRESS:
            if pattern.search(text):
                self.status = replace(s, state=state, detail='')
                return
        if m := _MAP_SAVED.search(text):
            self.status = replace(s, map_path=m.group(1))
        elif _LANDED.search(text):
            self.landed = True
        elif m := _DIED.search(text):
            if s.state != STOPPING:
                self.status = replace(s, state=FAILED, detail=f'{m.group(1)} stopped unexpectedly')

    @property
    def finished_flying(self) -> bool:
        """DAEP is done, the map is saved and the drone is down: the launch can be stopped."""
        return self.status.state == LANDING and self.landed and self.status.map_path is not None


def last_coverage(csv_text: str) -> float | None:
    """The latest coverage (percent) in rpl_exploration's coverage.csv, None if none yet."""
    for line in reversed(csv_text.strip().splitlines()[1:]):
        cells = line.split(',')
        if len(cells) >= 3:
            try:
                return round(float(cells[2]), 1)
            except ValueError:
                continue
    return None
