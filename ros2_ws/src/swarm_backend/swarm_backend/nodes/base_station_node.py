"""The base station (`mount-backend.sh basestation`): reads what the defenders report,
decides with intercept/base_station.py and sends the orders back (surveillance plan,
sections 3 and 4).

It does not fly and has no camera. Settings come from the `surveillance:` section of
swarm.yaml.

    python3 -m swarm_backend.nodes.base_station_node --config <swarm.yaml>

Times are when a message arrives here, not the drones' own clocks. A defender that has not
reported for `status_timeout_s` is counted as lost, and a `contact` from an attacker is
counted as a hit: the base station does not see the hostile drone itself, and a miss is not
detected on real drones yet.
"""
import argparse
import sys
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from swarm_backend.config import default_config_path, load_config
from swarm_backend.intercept.base_station import BaseConfig, BaseStation, Mode, Order, Sighting
from swarm_backend.intercept import surveillance_msgs as msgs
from swarm_backend.intercept.target_association import TargetAssociator

TICK_HZ = 5.0
RESEND_S = 1.0  # every current order is sent again this often, so a late drone gets it


class BaseStationNode:

    def __init__(self, node, config):
        self.node = node
        self.log = node.get_logger()
        self.config = config.surveillance
        c = self.config
        self.station = BaseStation(c.drone_ids, BaseConfig(
            arc_deg=c.arc_deg, leash_m=c.leash_m, close_radius_m=c.close_radius_m,
            reserve=c.reserve, resweep_delay_s=c.resweep_delay_s))
        self.associator = TargetAssociator(c.gate_m, c.track_expiry_s)
        self.orders: dict[int, Order] = {}
        self.last_heard = {d: None for d in c.drone_ids}
        self._last_sent = 0.0
        self._publisher = node.create_publisher(String, msgs.ASSIGNMENT_TOPIC, 10)
        node.create_subscription(String, msgs.SIGHTING_TOPIC, self._on_sighting, 10)
        node.create_subscription(String, msgs.STATUS_TOPIC, self._on_status, 10)
        node.create_subscription(String, msgs.EVENT_TOPIC, self._on_event, 10)
        node.create_timer(1.0 / TICK_HZ, self._tick)
        self._publish(self.station.tick(self.now()))

    @staticmethod
    def now() -> float:
        return time.monotonic()

    # --- what the drones say ---------------------------------------------------

    def _on_sighting(self, msg) -> None:
        s = msgs.decode_sighting(msg.data)
        now = self.now()
        self.last_heard[s.drone_id] = now
        bx, by = self.config.base_position
        target_id = self.associator.identify(s.position, now)
        distance = ((s.position[0] - bx) ** 2 + (s.position[1] - by) ** 2) ** 0.5
        self._publish(self.station.on_sightings(
            [Sighting(s.drone_id, target_id, now, distance, s.confirmed)], now))

    def _on_status(self, msg) -> None:
        s = msgs.decode_status(msg.data)
        now = self.now()
        self.last_heard[s.drone_id] = now
        orders = self.station.on_status(s.drone_id, s.distance_m, now)
        # A drone that turned on a target by itself (decision 6) but was not given it.
        recall = Order(s.drone_id, Mode.RETURN)
        if (s.mode is Mode.ENGAGE and self.station.drones[s.drone_id].mode is not Mode.ENGAGE
                and self.orders.get(s.drone_id) != recall):
            orders.append(recall)
        self._publish(orders)

    def _on_event(self, msg) -> None:
        e = msgs.decode_event(msg.data)
        now = self.now()
        self.last_heard[e.drone_id] = now
        if self.station.drones[e.drone_id].mode is Mode.LOST:
            return
        if e.kind == 'contact':
            target = self.station.drones[e.drone_id].target
            if target is not None:
                self.associator.forget(target)
            self.log.info(f'drone {e.drone_id} made contact: counted as a hit')
            orders = self.station.on_hit(e.drone_id, now)
        elif e.kind == 'target_lost':
            orders = self.station.on_target_lost(e.drone_id, now)
        else:
            orders = self.station.on_returned(e.drone_id, now)
        self._publish(orders)

    # --- the base station's own clock ---------------------------------------------

    def _tick(self) -> None:
        now = self.now()
        orders = []
        for drone_id, heard in self.last_heard.items():
            silent = heard is not None and now - heard > self.config.status_timeout_s
            if silent and self.station.drones[drone_id].mode is not Mode.LOST:
                self.log.warning(f'No word from drone {drone_id} for '
                                 f'{self.config.status_timeout_s:.0f} s: counted as lost')
                orders += self.station.on_hit(drone_id, now)
        orders += self.station.tick(now)
        self._publish(orders)
        if now - self._last_sent >= RESEND_S:
            self._last_sent = now
            for order in self.orders.values():
                self._send(order)

    def _publish(self, orders: list[Order]) -> None:
        for order in orders:
            self.orders[order.drone_id] = order
            self._send(order)
            if order.mode is not Mode.PATROL:
                self.log.info(f'drone {order.drone_id}: {order.mode.value.upper()}'
                              + (f' {order.target_id}' if order.target_id else ''))

    def _send(self, order: Order) -> None:
        self._publisher.publish(String(data=msgs.encode_order(order)))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--config', default=None, help='swarm.yaml (default: the installed one)')
    args = parser.parse_args(argv)
    config = load_config(args.config or default_config_path())

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('base_station')
    BaseStationNode(node, config)
    node.get_logger().info(
        f'Base station for drones {list(config.surveillance.drone_ids)}. Ctrl-C to stop.')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
