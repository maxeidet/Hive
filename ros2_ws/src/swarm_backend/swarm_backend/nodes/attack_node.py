"""Baseline attack node (`mount-backend.sh attack`): the defender flies into the hostile
drone with pure pursuit, on the two detection topics only (intercept/attack.py).

It takes off to the guard altitude, searches, and attacks whatever the detector (or the
fake one, `mount-backend.sh fakedetect`) reports, until Ctrl-C or --duration. Then it
flies back above its start point and lands. It does not score hits; the attack test
(work plan task 4) does that. Settings come from the `attack:` section of swarm.yaml.

    python3 -m swarm_backend.nodes.attack_node --config <swarm.yaml> [--id 2]
"""
import argparse
import dataclasses
import sys
import time

import rclpy
from rclpy.signals import SignalHandlerOptions

from swarm_backend.config import default_config_path, load_config
from swarm_backend.intercept.attack import STATE_TOPIC, AttackAction, DetectionInbox, state_label
from swarm_backend.intercept.detection import OFFSET_TOPIC, POSITION_TOPIC, topic
from swarm_backend.px4.px4_drone import PX4Drone
from swarm_backend.runner import LAND_TIMEOUT_S, spinning, wait_until
from swarm_backend.swarm.manager import SwarmManager
from swarm_backend.swarm.swarm import Swarm

ALTITUDE_TOLERANCE_M = 1.0
CLIMB_TIMEOUT_S = 240.0  # slow when Gazebo runs below real time
RETURN_TIMEOUT_S = 180.0
POLL_S = 0.1
STATE_REPEAT_S = 1.0  # the state is published on every change and at least this often


def stamp_seconds(header) -> float:
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class AttackNode:
    """A PX4Drone, the detection subscriptions and the attack, on one ROS node."""

    def __init__(self, node, swarm_config, config, clock=None):
        """`clock` gives the time the attack runs on (seconds); default the drone's PX4
        clock. The attack test passes Gazebo's simulation clock, which never jumps."""
        from geometry_msgs.msg import PointStamped, Vector3Stamped
        from std_msgs.msg import String

        self.config = config
        self.log = node.get_logger()
        self.drone = PX4Drone(node, swarm_config.drone(config.attacker_id),
                              rate_hz=swarm_config.rate_hz)
        self.manager = SwarmManager(Swarm([self.drone]), ALTITUDE_TOLERANCE_M)
        self.inbox = DetectionInbox()
        self.clock = clock or (lambda: self.drone.stamp)
        self.attack = AttackAction(self.drone, self.inbox, config, self.clock)
        self.position_topic = topic(config.attacker_id, POSITION_TOPIC)
        self.offset_topic = topic(config.attacker_id, OFFSET_TOPIC)
        node.create_subscription(PointStamped, self.position_topic, self._on_position, 10)
        node.create_subscription(Vector3Stamped, self.offset_topic, self._on_offset, 10)
        self._string = String
        self._state_pub = node.create_publisher(String, topic(config.attacker_id, STATE_TOPIC), 10)
        self._state_sent: tuple[str, float] | None = None  # (label, time.monotonic())
        node.create_timer(1.0 / swarm_config.rate_hz, self._step)

    def new_attack(self) -> AttackAction:
        """A fresh attack (state, times) on the same drone and topics, e.g. for each test run."""
        self.attack = AttackAction(self.drone, self.inbox, self.config, self.clock)
        return self.attack

    def _step(self) -> None:
        self.attack.step()
        self._publish_state()

    def _publish_state(self) -> None:
        label, now = state_label(self.attack), time.monotonic()
        if self._state_sent is None or self._state_sent[0] != label \
                or now - self._state_sent[1] >= STATE_REPEAT_S:
            self._state_pub.publish(self._string(data=label))
            self._state_sent = (label, now)

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
        try:
            wait_until(self.manager.connected, 30, f'status and position from drone {c.attacker_id}')
        except TimeoutError:
            raise TimeoutError(f'No data from PX4 for drone {c.attacker_id}. Start it with '
                               '`./mount-backend.sh spawn`') from None
        wait_until(self.manager.ready, 60, 'pre-flight checks to pass')
        self.log.info(f'Taking off to the guard altitude, {c.guard_altitude} m')
        self.manager.takeoff(c.guard_altitude)
        wait_until(self.manager.all_reached, CLIMB_TIMEOUT_S, f'{c.guard_altitude} m')

        self.log.info(f'Attacking on {self.position_topic} and {self.offset_topic} at '
                      f'{c.max_speed} m/s. Searching toward {c.search_heading_deg} deg, gimbal '
                      f'{c.search_pitch_deg} deg up. Ctrl-C to stop.')
        self.attack.start()
        start = time.monotonic()
        seen = 0
        while duration <= 0 or time.monotonic() - start < duration:
            for t, state in self.attack.transitions[seen:]:
                distance = ''
                message = self.inbox.position
                if message is not None:
                    distance = f', target {sum(v * v for v in message.value) ** 0.5:.1f} m away'
                self.log.info(f'{state.value.upper()}{distance}')
            seen = len(self.attack.transitions)
            time.sleep(POLL_S)

    def stop_and_land(self) -> None:
        self.attack.stop()
        if not self.drone.armed:
            return
        try:
            if self.drone.offboard:
                self.log.info('Returning above the start point')
                self.drone.set_position(*self.attack.guard)
                wait_until(lambda: self.drone.reached(2.0), RETURN_TIMEOUT_S, 'the start point')
        except (TimeoutError, KeyboardInterrupt) as e:
            self.log.warning(f'Landing where it is ({e or "Ctrl-C"})')
        self.log.info('Landing')
        self.manager.land()
        # Wall clock: minutes from the guard height when Gazebo runs slowly.
        wait_until(self.manager.all_landed, max(LAND_TIMEOUT_S, 900.0),
                   'the drone to land and disarm')
        self.log.info('Landed')

    def summary(self) -> str:
        times = ', '.join(f'{s.value} {t:.1f} s' for s, t in self.attack.state_time.items() if t)
        return f'Time in each state: {times or "none"}'


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--config', default=None, help='swarm.yaml (default: the installed one)')
    parser.add_argument('--id', type=int, default=None, help='attacker drone (gimbal drone)')
    parser.add_argument('--duration', type=float, default=0.0,
                        help='seconds (wall clock) before landing; 0 = until Ctrl-C')
    parser.add_argument('--speed', type=float, default=None, help='m/s, overrides max_speed')
    args = parser.parse_args(argv)

    swarm_config = load_config(args.config or default_config_path())
    overrides = {k: v for k, v in (('attacker_id', args.id), ('max_speed', args.speed))
                 if v is not None}
    config = dataclasses.replace(swarm_config.attack, **overrides)

    # Ctrl-C must reach our handler, so the drone lands instead of the node just dying.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node(f'attack_{config.attacker_id}')
    run = AttackNode(node, swarm_config, config)
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
            run.log.info(run.summary())
    node.destroy_node()
    rclpy.try_shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
