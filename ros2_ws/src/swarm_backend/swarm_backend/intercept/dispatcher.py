"""The base station's message handling, without ROS: what the defenders report goes in,
orders come out (surveillance plan, sections 3 and 4). nodes/base_station_node.py runs it
on ROS topics; the scenarios' fast simulation calls it directly.

Times are when a message arrives, on whatever clock the caller uses. A defender that has
not reported for `status_timeout_s` is counted as lost. A `kill` event (sent by the
scenario's referee, which scores hits from the truth) means the attacker destroyed its
target and survives: the target is forgotten and the attacker flies back to its slot. A
`contact` from an attacker (its detector saw the target within contact_m) is only logged:
the base station does not see the hostile drone itself, and a miss is not detected on
real drones yet. A `respawned` event (a crashed defender started again by the scenario) puts
the drone back on patrol, even one counted as lost.
"""
from collections.abc import Callable

from swarm_backend.config import SurveillanceConfig
from swarm_backend.intercept.base_station import BaseConfig, BaseStation, Mode, Order, Sighting
from swarm_backend.intercept.surveillance_msgs import EventMsg, SightingMsg, StatusMsg
from swarm_backend.intercept.target_association import TargetAssociator


class Dispatcher:

    def __init__(self, config: SurveillanceConfig,
                 log: Callable[[str, str], None] = lambda level, text: None):
        """`log(level, text)`, level 'info' or 'warning'."""
        self.config = c = config
        self.log = log
        self.station = BaseStation(c.drone_ids, BaseConfig(
            arc_deg=c.arc_deg, leash_m=c.leash_m, close_radius_m=c.close_radius_m,
            reserve=c.reserve, resweep_delay_s=c.resweep_delay_s))
        self.associator = TargetAssociator(c.gate_m, c.track_expiry_s)
        self.orders: dict[int, Order] = {}  # the latest order to each drone
        self.last_heard = {d: None for d in c.drone_ids}

    def start(self, now: float) -> list[Order]:
        return self._record(self.station.tick(now))

    def on_sighting(self, s: SightingMsg, now: float) -> list[Order]:
        self.last_heard[s.drone_id] = now
        bx, by = self.config.base_position
        target_id = self.associator.identify(s.position, now)
        distance = ((s.position[0] - bx) ** 2 + (s.position[1] - by) ** 2) ** 0.5
        return self._record(self.station.on_sightings(
            [Sighting(s.drone_id, target_id, now, distance, s.confirmed)], now))

    def on_status(self, s: StatusMsg, now: float) -> list[Order]:
        self.last_heard[s.drone_id] = now
        orders = self.station.on_status(s.drone_id, s.distance_m, now)
        # A drone that turned on a target by itself (decision 6) but was not given it.
        recall = Order(s.drone_id, Mode.RETURN)
        if (s.mode is Mode.ENGAGE and self.station.drones[s.drone_id].mode is not Mode.ENGAGE
                and self.orders.get(s.drone_id) != recall):
            orders.append(recall)
        return self._record(orders)

    def on_event(self, e: EventMsg, now: float) -> list[Order]:
        self.last_heard[e.drone_id] = now
        if e.kind == 'respawned':  # also for a drone counted as lost: it is back
            target = self.station.drones[e.drone_id].target
            if target is not None:
                self.associator.forget(target)
            self.log('info', f'drone {e.drone_id} respawned: back on patrol')
            return self._record(self.station.on_respawned(e.drone_id, now))
        if self.station.drones[e.drone_id].mode is Mode.LOST:
            return []
        if e.kind == 'kill':
            target = self.station.drones[e.drone_id].target
            if target is not None:
                self.associator.forget(target)
            self.log('info', f'drone {e.drone_id} killed its target: returning to its slot')
            orders = self.station.on_kill(e.drone_id, now)
        elif e.kind == 'contact':
            self.log('info', f'drone {e.drone_id} made contact')
            orders = []
        elif e.kind == 'target_lost':
            orders = self.station.on_target_lost(e.drone_id, now)
        else:
            orders = self.station.on_returned(e.drone_id, now)
        return self._record(orders)

    def tick(self, now: float) -> list[Order]:
        """The base station's own clock: lost defenders and timed rules."""
        orders = []
        for drone_id, heard in self.last_heard.items():
            silent = heard is not None and now - heard > self.config.status_timeout_s
            if silent and self.station.drones[drone_id].mode is not Mode.LOST:
                self.log('warning', f'No word from drone {drone_id} for '
                                    f'{self.config.status_timeout_s:.0f} s: counted as lost')
                orders += self.station.on_hit(drone_id, now)
        orders += self.station.tick(now)
        return self._record(orders)

    def _record(self, orders: list[Order]) -> list[Order]:
        for order in orders:
            self.orders[order.drone_id] = order
            if order.mode is not Mode.PATROL:
                self.log('info', f'drone {order.drone_id}: {order.mode.value.upper()}'
                                 + (f' {order.target_id}' if order.target_id else ''))
        return orders
