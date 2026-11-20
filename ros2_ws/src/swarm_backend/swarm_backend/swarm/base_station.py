"""The base station's decisions: who attacks which target, who holds back, and how the
arc is shared among the drones that are left (surveillance plan, section 4).

Pure logic, no ROS. Events go in (a sighting, a status, a hit or a miss) and orders come
out, each only when it changes. The ROS node that sends the orders is a later step.

  PATROL  holds its slot and sweeps its sector
  ENGAGE  flies at its target
  RETURN  flies back to its slot, then reports `on_returned`
  LOST    gone for good

Distances are metres from the base, times are seconds on one clock.
"""
from dataclasses import dataclass, field
from enum import Enum

from swarm_backend.swarm.arc import DEFAULT_ARC_DEG, sectors


class Mode(Enum):
    PATROL = 'patrol'
    ENGAGE = 'engage'
    RETURN = 'return'
    LOST = 'lost'


@dataclass(frozen=True)
class BaseConfig:
    arc_deg: float = DEFAULT_ARC_DEG
    leash_m: float = 1000.0  # an attacker further from the base than this turns back
    close_radius_m: float = 500.0  # inside this, a confirmed target may take the reserve
    reserve: int = 1  # drones that stay on patrol unless the target is confirmed and close
    resweep_delay_s: float = 10.0  # wait before a returned drone gets its sector back


@dataclass(frozen=True)
class Sighting:
    drone_id: int
    target_id: str
    time: float
    target_range_m: float  # target to base
    confirmed: bool = False


@dataclass(frozen=True)
class Order:
    drone_id: int
    mode: Mode  # PATROL, ENGAGE or RETURN
    target_id: str | None = None  # ENGAGE
    sector: tuple[float, float] | None = None  # PATROL: degrees from the threat heading;
    # None = hold the slot without sweeping (just back, waiting out the delay)
    may_start: bool = False  # PATROL: may the drone turn on a target by itself


@dataclass
class _Drone:
    slot: int
    mode: Mode = Mode.PATROL
    target: str | None = None
    back_since: float | None = None


@dataclass
class _Target:
    range_m: float
    confirmed: bool = False
    owner: int | None = None
    seen_by: dict[int, float] = field(default_factory=dict)  # drone id -> first sighting


class BaseStation:

    def __init__(self, drone_ids, config: BaseConfig | None = None):
        self.config = config or BaseConfig()
        self.drones = {d: _Drone(slot) for slot, d in enumerate(sorted(drone_ids))}
        self.targets: dict[str, _Target] = {}
        self._sent_patrol: dict[int, tuple] = {}

    # Events. Each returns the orders that changed.

    def tick(self, now: float) -> list[Order]:
        """Call at start and regularly: serves waiting targets and ends the resweep delay."""
        return self._finish(now, [])

    def on_sightings(self, sightings: list[Sighting], now: float) -> list[Order]:
        """Reports that arrived together. The earliest wins a target; ties go to the lowest
        drone id."""
        for s in sorted(sightings, key=lambda s: (s.time, s.drone_id)):
            target = self.targets.setdefault(s.target_id, _Target(s.target_range_m))
            target.range_m = s.target_range_m
            target.confirmed = target.confirmed or s.confirmed
            target.seen_by.setdefault(s.drone_id, s.time)
        return self._finish(now, [])

    def on_status(self, drone_id: int, distance_m: float, now: float) -> list[Order]:
        """An attacker past the leash turns back."""
        drone = self.drones[drone_id]
        orders = []
        if drone.mode is Mode.ENGAGE and distance_m > self.config.leash_m:
            orders = self._call_off(drone_id)
        return self._finish(now, orders)

    def on_target_lost(self, drone_id: int, now: float) -> list[Order]:
        """The attacker no longer sees its target. The target is forgotten until it is
        sighted again."""
        drone = self.drones[drone_id]
        orders = self._call_off(drone_id) if drone.mode is Mode.ENGAGE else []
        return self._finish(now, orders)

    def on_hit(self, drone_id: int, now: float) -> list[Order]:
        """The attack hit: the target is gone and the attacker is lost."""
        drone = self.drones[drone_id]
        if drone.target is not None:
            self.targets.pop(drone.target, None)
        drone.mode, drone.target = Mode.LOST, None
        return self._finish(now, [])

    def on_kill(self, drone_id: int, now: float) -> list[Order]:
        """The attacker destroyed its target and survived: the target is gone and the
        attacker flies back to its slot (then `on_returned`)."""
        drone = self.drones[drone_id]
        orders = self._call_off(drone_id) if drone.mode is Mode.ENGAGE else []
        return self._finish(now, orders)

    def on_miss(self, drone_id: int, drone_alive: bool, target_inbound: bool,
                now: float) -> list[Order]:
        """A missed attack. A surviving attacker goes back; a target that is no longer on
        its way in is forgotten, otherwise it is served again (by another drone if the
        attacker is gone)."""
        drone = self.drones[drone_id]
        target_id = drone.target
        if not target_inbound and target_id is not None:
            self.targets.pop(target_id, None)
        if drone_alive:
            orders = self._recall(drone_id)
        else:
            self._release(drone)
            drone.mode = Mode.LOST
            orders = []
        return self._finish(now, orders)

    def on_returned(self, drone_id: int, now: float) -> list[Order]:
        drone = self.drones[drone_id]
        self._sent_patrol.pop(drone_id, None)  # tell it its sector again
        drone.mode, drone.back_since = Mode.PATROL, now
        return self._finish(now, [])

    def on_respawned(self, drone_id: int, now: float) -> list[Order]:
        """A crashed defender was started again (the scenarios do it between runs), also
        one counted as LOST: it patrols its sector again at once, and the arc is shared
        out among all the patrolling drones again."""
        drone = self.drones[drone_id]
        if drone.target is not None:
            self.targets.pop(drone.target, None)
        self._release(drone)
        self._sent_patrol.pop(drone_id, None)
        drone.mode, drone.back_since = Mode.PATROL, None
        return self._finish(now, [])

    def drop_target(self, target_id: str, now: float) -> list[Order]:
        """The target left (out of range, landed). Its attacker goes back."""
        target = self.targets.pop(target_id, None)
        orders = []
        if target is not None and target.owner is not None:
            orders = self._recall(target.owner)
        return self._finish(now, orders)

    # Internals

    def _patrol_ids(self) -> list[int]:
        return sorted(d for d, drone in self.drones.items() if drone.mode is Mode.PATROL)

    def _recall(self, drone_id: int) -> list[Order]:
        drone = self.drones[drone_id]
        self._release(drone)
        drone.mode = Mode.RETURN
        return [Order(drone_id, Mode.RETURN)]

    def _call_off(self, drone_id: int) -> list[Order]:
        target_id = self.drones[drone_id].target
        orders = self._recall(drone_id)
        self.targets.pop(target_id, None)
        return orders

    def _release(self, drone: _Drone) -> None:
        target = self.targets.get(drone.target) if drone.target is not None else None
        if target is not None:
            target.owner = None
        drone.target = None

    def _may_take(self, target: _Target, patrol_count: int) -> bool:
        c = self.config
        if patrol_count > c.reserve:
            return True
        return patrol_count > 0 and target.confirmed and target.range_m <= c.close_radius_m

    def _dispatch(self) -> list[Order]:
        """Give every unserved target a drone if the rules allow, closest to the base first."""
        orders = []
        waiting = sorted((t.range_m, tid) for tid, t in self.targets.items()
                         if t.owner is None and t.range_m <= self.config.leash_m)
        for _, target_id in waiting:
            target = self.targets[target_id]
            patrol = self._patrol_ids()
            if not self._may_take(target, len(patrol)):
                continue
            seen = sorted((t, d) for d, t in target.seen_by.items() if d in patrol)
            drone_id = seen[0][1] if seen else patrol[0]
            target.owner = drone_id
            drone = self.drones[drone_id]
            drone.mode, drone.target, drone.back_since = Mode.ENGAGE, target_id, None
            orders.append(Order(drone_id, Mode.ENGAGE, target_id))
        return orders

    def _patrol_orders(self, now: float) -> list[Order]:
        c = self.config
        patrol = self._patrol_ids()
        sweeping = [d for d in patrol if self.drones[d].back_since is None
                    or now - self.drones[d].back_since >= c.resweep_delay_s]
        split = sectors([self.drones[d].slot for d in sweeping], c.arc_deg)
        may_start = len(patrol) > c.reserve
        orders = []
        for d in patrol:
            sector = split.get(self.drones[d].slot) if d in sweeping else None
            key = (sector, may_start)
            if self._sent_patrol.get(d) != key:
                self._sent_patrol[d] = key
                orders.append(Order(d, Mode.PATROL, sector=sector, may_start=may_start))
        for d, drone in self.drones.items():
            if drone.mode is not Mode.PATROL:
                self._sent_patrol.pop(d, None)
        return orders

    def _finish(self, now: float, orders: list[Order]) -> list[Order]:
        orders = orders + self._dispatch()
        return orders + self._patrol_orders(now)
