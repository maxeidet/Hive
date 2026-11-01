"""The messages between the defenders and the base station, as JSON in std_msgs/String.

Plain strings keep us from a new message package for a few small messages. Topics:

  /surveillance/sighting     drone -> base   a target the drone's camera has confirmed
  /surveillance/status       drone -> base   heartbeat: mode and distance from the base
  /surveillance/event        drone -> base   contact, target_lost or returned
  /surveillance/assignment   base -> drone   PATROL, ENGAGE or RETURN, resent every second

Pure Python, no ROS. Times are not sent: the base station uses when a message arrives.
"""
import json
from dataclasses import dataclass

from swarm_backend.intercept.base_station import Mode, Order

SIGHTING_TOPIC = '/surveillance/sighting'
STATUS_TOPIC = '/surveillance/status'
EVENT_TOPIC = '/surveillance/event'
ASSIGNMENT_TOPIC = '/surveillance/assignment'

EVENT_KINDS = ('contact', 'target_lost', 'returned')


@dataclass(frozen=True)
class SightingMsg:
    drone_id: int
    position: tuple[float, float, float]  # the target, world NED
    confirmed: bool


@dataclass(frozen=True)
class StatusMsg:
    drone_id: int
    mode: Mode
    distance_m: float  # from the base


@dataclass(frozen=True)
class EventMsg:
    drone_id: int
    kind: str


def encode_sighting(m: SightingMsg) -> str:
    return json.dumps({'drone': m.drone_id, 'position': list(m.position),
                       'confirmed': m.confirmed})


def decode_sighting(text: str) -> SightingMsg:
    d = json.loads(text)
    return SightingMsg(int(d['drone']), tuple(float(v) for v in d['position']),
                       bool(d['confirmed']))


def encode_status(m: StatusMsg) -> str:
    return json.dumps({'drone': m.drone_id, 'mode': m.mode.value, 'distance_m': m.distance_m})


def decode_status(text: str) -> StatusMsg:
    d = json.loads(text)
    return StatusMsg(int(d['drone']), Mode(d['mode']), float(d['distance_m']))


def encode_event(m: EventMsg) -> str:
    if m.kind not in EVENT_KINDS:
        raise ValueError(f'Unknown event {m.kind!r}; use one of {", ".join(EVENT_KINDS)}')
    return json.dumps({'drone': m.drone_id, 'kind': m.kind})


def decode_event(text: str) -> EventMsg:
    d = json.loads(text)
    if d['kind'] not in EVENT_KINDS:
        raise ValueError(f'Unknown event {d["kind"]!r}')
    return EventMsg(int(d['drone']), d['kind'])


def encode_order(order: Order) -> str:
    return json.dumps({'drone': order.drone_id, 'mode': order.mode.value,
                       'target': order.target_id,
                       'sector': None if order.sector is None else list(order.sector),
                       'may_start': order.may_start})


def decode_order(text: str) -> Order:
    d = json.loads(text)
    sector = None if d['sector'] is None else (float(d['sector'][0]), float(d['sector'][1]))
    return Order(int(d['drone']), Mode(d['mode']), d['target'], sector, bool(d['may_start']))
