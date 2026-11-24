"""The hivemind: broadcasts where every friendly drone is, so that no drone's detector
locks onto one of our own (swarm/friendlies.py).

    python3 -m swarm_backend.nodes.hivemind_node --config <yaml> --ids 0 1

Listens (never commands) to each friendly PX4's odometry and publishes all of them on
/hive/friendlies (std_msgs/String, JSON: position and velocity, world NED) at --rate Hz,
parked drones too. Each drone is stamped with Gazebo's simulation time when its odometry
arrived (the clock the camera images carry), so a detector can predict each friendly to
the time of its image; the message itself with the time it is sent. Without Gazebo
(no /clock) the node's own ROS clock is used. --ids: the friendly drones, required: a
config usually lists the hostile drones too (scenario2.yaml does), and a hostile listed
here would never be attacked.
"""
import argparse
import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String

from swarm_backend.config import default_config_path, load_config
from swarm_backend.swarm.friendlies import RATE_HZ, TOPIC, Friendly, encode
from swarm_backend.px4.observer import DroneObserver


class Hivemind:

    def __init__(self, node: Node, observers: list, rate_hz: float = RATE_HZ):
        """observers: DroneObserver or PX4Drone (.id, .position, .velocity, .clock, .arrived)."""
        self.node = node
        self.observers = observers
        self._pub = node.create_publisher(String, TOPIC, 10)
        self._sim_now: float | None = None
        self._lock = threading.Lock()
        self._gz = None
        try:
            from gz.msgs10.clock_pb2 import Clock
            from gz.transport13 import Node as GzNode
            self._gz = GzNode()
            self._gz.subscribe(Clock, '/clock', self._on_clock)
        except ImportError:
            pass  # no Gazebo: the ROS clock
        for o in observers:  # each position is stamped with when it arrived (friendlies.py)
            o.clock = self.now
        node.create_timer(1.0 / rate_hz, self._publish)

    def _on_clock(self, msg) -> None:
        with self._lock:
            self._sim_now = msg.sim.sec + msg.sim.nsec * 1e-9

    def now(self) -> float:
        with self._lock:
            if self._sim_now is not None:
                return self._sim_now
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _publish(self) -> None:
        friendlies = [Friendly(o.id, o.position, o.velocity or (0.0, 0.0, 0.0),
                               getattr(o, 'arrived', None))
                      for o in self.observers if o.position is not None]
        if friendlies:
            self._pub.publish(String(data=encode(self.now(), friendlies)))


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--config', help='swarm.yaml (default: the installed one)')
    parser.add_argument('--ids', type=int, nargs='+', required=True,
                        help='the friendly drones (never the hostiles)')
    parser.add_argument('--rate', type=float, default=RATE_HZ, help='Hz (default %(default)s)')
    cli = parser.parse_args(remove_ros_args(sys.argv)[1:])
    config = load_config(cli.config or default_config_path())
    ids = cli.ids

    rclpy.init(args=args)
    node = Node('hivemind')
    Hivemind(node, [DroneObserver(node, config.drone(i)) for i in ids], cli.rate)
    node.get_logger().info(f'Friendly drones {ids} on {TOPIC} at {cli.rate:g} Hz')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
