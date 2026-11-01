"""One defender on the arc (`mount-backend.sh patrol ID`): takes off, flies to its point,
sweeps its camera over the sector the base station gives it and attacks when told to
(intercept/patrol.py). Start the base station (`basestation`) and a fake or real detector
for this drone (`fakedetect --id ID`, `detect ID`) first.

Lands where it ends up on Ctrl-C or --duration; after a hit the drone is gone anyway.
Settings come from the `surveillance:` and `attack:` sections of swarm.yaml.

    python3 -m swarm_backend.nodes.patrol_node --config <swarm.yaml> --id 1
"""
import argparse
import sys
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from swarm_backend.config import default_config_path, load_config
from swarm_backend.intercept import surveillance_msgs as msgs
from swarm_backend.intercept.attack import DetectionInbox
from swarm_backend.intercept.detection import OFFSET_TOPIC, POSITION_TOPIC, topic
from swarm_backend.intercept.patrol import PatrolAction
from swarm_backend.nodes.attack_node import (
    ALTITUDE_TOLERANCE_M, CLIMB_TIMEOUT_S, POLL_S, RETURN_TIMEOUT_S, stamp_seconds)
from swarm_backend.px4.px4_drone import PX4Drone
from swarm_backend.runner import LAND_TIMEOUT_S, spinning, wait_until
from swarm_backend.swarm.manager import SwarmManager
from swarm_backend.swarm.swarm import Swarm

TO_SLOT_TIMEOUT_S = 240.0  # slow when Gazebo runs below real time
CONNECT_TIMEOUT_S = 180.0  # three freshly spawned drones take a while to reach ROS


class PatrolNode:
    """A PX4Drone, the detection and surveillance topics and the patrol, on one ROS node."""

    def __init__(self, node, swarm_config, drone_id):
        from geometry_msgs.msg import PointStamped, Vector3Stamped

        self.config = swarm_config.surveillance
        if drone_id not in self.config.drone_ids:
            raise ValueError(f'Drone {drone_id} is not in surveillance.drone_ids '
                             f'{list(self.config.drone_ids)}')
        self.log = node.get_logger()
        self.drone = PX4Drone(node, swarm_config.drone(drone_id), rate_hz=swarm_config.rate_hz)
        self.manager = SwarmManager(Swarm([self.drone]), ALTITUDE_TOLERANCE_M)
        self.inbox = DetectionInbox()
        self.clock = lambda: self.drone.stamp
        slot = self.config.drone_ids.index(drone_id)
        self.patrol = PatrolAction(self.drone, self.inbox, swarm_config.attack, self.config,
                                   slot, self.clock)
        node.create_subscription(PointStamped, topic(drone_id, POSITION_TOPIC),
                                 self._on_position, 10)
        node.create_subscription(Vector3Stamped, topic(drone_id, OFFSET_TOPIC),
                                 self._on_offset, 10)
        node.create_subscription(String, msgs.ASSIGNMENT_TOPIC, self._on_order, 10)
        self._sightings = node.create_publisher(String, msgs.SIGHTING_TOPIC, 10)
        self._status = node.create_publisher(String, msgs.STATUS_TOPIC, 10)
        self._events = node.create_publisher(String, msgs.EVENT_TOPIC, 10)
        node.create_timer(1.0 / swarm_config.rate_hz, self._step)
        node.create_timer(1.0 / self.config.status_hz, self._send_status)

    def _step(self) -> None:
        self.patrol.step()
        for s in self.patrol.take_sightings():
            self._sightings.publish(String(data=msgs.encode_sighting(s)))
        for e in self.patrol.take_events():
            self.log.info(f'{e.kind}')
            self._events.publish(String(data=msgs.encode_event(e)))

    def _send_status(self) -> None:
        status = self.patrol.status()
        if status is not None and self.patrol.running:
            self._status.publish(String(data=msgs.encode_status(status)))

    def _on_order(self, msg) -> None:
        order = msgs.decode_order(msg.data)
        if order.drone_id == self.drone.id:
            before = self.patrol.mode
            self.patrol.apply(order)
            if self.patrol.mode is not before:
                self.log.info(f'{self.patrol.mode.value.upper()}')

    def _on_position(self, msg) -> None:
        now = self.clock()
        if now is not None:
            p = msg.point
            self.inbox.put_position((p.x, p.y, p.z), stamp_seconds(msg.header), now)

    def _on_offset(self, msg) -> None:
        now = self.clock()
        if now is not None:
            v = msg.vector
            self.inbox.put_offset((v.x, v.y), stamp_seconds(msg.header), now)

    def fly(self, duration: float) -> None:
        c = self.config
        wait_until(self.manager.connected, CONNECT_TIMEOUT_S,
                   f'status and position from drone {self.drone.id}')
        wait_until(self.manager.ready, CONNECT_TIMEOUT_S, 'pre-flight checks to pass')
        self.log.info(f'Taking off to {c.altitude} m')
        self.manager.takeoff(c.altitude)
        wait_until(self.manager.all_reached, CLIMB_TIMEOUT_S, f'{c.altitude} m')
        self.log.info(f'Flying to the slot at {self.patrol.slot_angle:+.0f} deg on the arc')
        self.patrol.start()
        wait_until(lambda: self.drone.reached(2.0), TO_SLOT_TIMEOUT_S, 'the slot')
        self.log.info('On patrol. Ctrl-C to stop.')
        start = time.monotonic()
        while duration <= 0 or time.monotonic() - start < duration:
            time.sleep(POLL_S)

    def stop_and_land(self) -> None:
        self.patrol.stop()
        if not self.drone.armed:
            return
        try:
            if self.drone.offboard:
                self.log.info('Returning to the slot')
                self.drone.set_position(*self.patrol.slot_point)
                wait_until(lambda: self.drone.reached(2.0), RETURN_TIMEOUT_S, 'the slot')
        except (TimeoutError, KeyboardInterrupt) as e:
            self.log.warning(f'Landing where it is ({e or "Ctrl-C"})')
        self.log.info('Landing')
        self.manager.land()
        wait_until(self.manager.all_landed, max(LAND_TIMEOUT_S, 900.0),
                   'the drone to land and disarm')
        self.log.info('Landed')


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--config', default=None, help='swarm.yaml (default: the installed one)')
    parser.add_argument('--id', type=int, required=True, help='PX4 instance of this defender')
    parser.add_argument('--duration', type=float, default=0.0,
                        help='seconds (wall clock) before landing; 0 = until Ctrl-C')
    args = parser.parse_args(argv)
    swarm_config = load_config(args.config or default_config_path())

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node(f'patrol_{args.id}')
    try:
        run = PatrolNode(node, swarm_config, args.id)
    except ValueError as e:
        node.get_logger().error(str(e))
        rclpy.try_shutdown()
        return 1
    code = 0
    with spinning(node):
        try:
            run.fly(args.duration)
        except KeyboardInterrupt:
            run.log.info('Ctrl-C')
        except TimeoutError as e:
            run.log.error(str(e))
            code = 1
        finally:
            try:
                run.stop_and_land()
            except (TimeoutError, KeyboardInterrupt) as e:
                run.log.error(f'Did not land cleanly: {e or "Ctrl-C"}')
                code = 1
    node.destroy_node()
    rclpy.try_shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
