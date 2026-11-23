"""The base station (`mount-backend.sh basestation`): reads what the defenders report,
decides with swarm/dispatcher.py (the rules are in swarm/base_station.py) and sends
the orders back (surveillance plan, sections 3 and 4).

It does not fly and has no camera. Settings come from the `surveillance:` section of
swarm.yaml.

    python3 -m swarm_backend.nodes.base_station_node --config <swarm.yaml>

Times are when a message arrives here, not the drones' own clocks (wall clock, or the
`clock` the scenarios give: Gazebo's). See swarm/dispatcher.py for lost defenders and
the `kill`, `contact` and `target_lost` events.
"""
import argparse
import sys
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from swarm_backend.config import default_config_path, load_config
from swarm_backend.swarm import messages as msgs
from swarm_backend.swarm.base_station import Order
from swarm_backend.swarm.dispatcher import Dispatcher

TICK_HZ = 5.0
RESEND_S = 1.0  # every current order is sent again this often, so a late drone gets it


class BaseStationNode:

    def __init__(self, node, config, clock=None):
        """`clock` gives seconds (default: time.monotonic)."""
        self.node = node
        self._clock = clock or time.monotonic
        self._last_now = 0.0
        self.log = node.get_logger()
        self.config = config.surveillance
        self.dispatcher = Dispatcher(self.config, self._log)
        self._last_sent = 0.0
        self._publisher = node.create_publisher(String, msgs.ASSIGNMENT_TOPIC, 10)
        node.create_subscription(String, msgs.SIGHTING_TOPIC, self._on_sighting, 10)
        node.create_subscription(String, msgs.STATUS_TOPIC, self._on_status, 10)
        node.create_subscription(String, msgs.EVENT_TOPIC, self._on_event, 10)
        node.create_timer(1.0 / TICK_HZ, self._tick)
        self._publish(self.dispatcher.start(self.now()))

    @property
    def station(self):
        return self.dispatcher.station

    @property
    def orders(self) -> dict[int, Order]:
        return self.dispatcher.orders

    def _log(self, level: str, text: str) -> None:
        # One call per level: rclpy raises 'Logger severity cannot be changed between
        # calls' when one line of code logs at two levels.
        if level == 'error':
            self.log.error(text)
        elif level == 'warning':
            self.log.warning(text)
        elif level == 'debug':
            self.log.debug(text)
        else:
            self.log.info(text)

    def now(self) -> float:
        """The clock's time; while it does not know the time yet, the last time it gave."""
        now = self._clock()
        if now is not None:
            self._last_now = now
        return self._last_now

    # --- what the drones say ---------------------------------------------------

    def _on_sighting(self, msg) -> None:
        self._publish(self.dispatcher.on_sighting(msgs.decode_sighting(msg.data), self.now()))

    def _on_status(self, msg) -> None:
        self._publish(self.dispatcher.on_status(msgs.decode_status(msg.data), self.now()))

    def _on_event(self, msg) -> None:
        self._publish(self.dispatcher.on_event(msgs.decode_event(msg.data), self.now()))

    # --- the base station's own clock ---------------------------------------------

    def _tick(self) -> None:
        now = self.now()
        self._publish(self.dispatcher.tick(now))
        if now - self._last_sent >= RESEND_S:
            self._last_sent = now
            for order in self.orders.values():
                self._send(order)

    def _publish(self, orders: list[Order]) -> None:
        for order in orders:
            self._send(order)

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
