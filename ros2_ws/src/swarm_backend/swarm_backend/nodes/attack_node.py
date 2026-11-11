"""Baseline attack node (`mount-backend.sh attack`): the defender flies into the hostile
drone with pure pursuit, on the two detection topics only (intercept/attack.py).

With --guidance mpc it flies the MPC instead (intercept/mpc_action.py), on the same
topics: an InboxDetector turns target_position into world positions. Same take-off,
search direction, limits and landing, so the two can be compared on the same runs.

It takes off to the guard altitude, searches, and attacks whatever the detector (or the
fake one, `mount-backend.sh fakedetect`) reports, until Ctrl-C or --duration. Then it
flies back above its start point and lands. It does not score hits; the attack test
(work plan task 4) does that. Settings come from the `attack:` section of swarm.yaml.

    python3 -m swarm_backend.nodes.attack_node --config <swarm.yaml> [--id 2] [--guidance mpc]
"""
import argparse
import dataclasses
import sys
import time

import rclpy
from rclpy.signals import SignalHandlerOptions

from swarm_backend.config import default_config_path, load_config
from swarm_backend.intercept.attack import AttackAction, DetectionInbox, limit_velocity
from swarm_backend.intercept.detection import OFFSET_TOPIC, POSITION_TOPIC, topic
from swarm_backend.intercept.inbox_detector import InboxDetector
from swarm_backend.intercept.mpc_action import MpcIntercept
from swarm_backend.px4.px4_drone import PX4Drone
from swarm_backend.runner import LAND_TIMEOUT_S, spinning, wait_until
from swarm_backend.swarm.manager import SwarmManager
from swarm_backend.swarm.swarm import Swarm

ALTITUDE_TOLERANCE_M = 1.0
CLIMB_TIMEOUT_S = 240.0  # slow when Gazebo runs below real time
RETURN_TIMEOUT_S = 180.0
POLL_S = 0.1
GUIDANCE = ('pursuit', 'mpc')


def stamp_seconds(header) -> float:
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class AttackNode:
    """A PX4Drone, the detection subscriptions and the attack, on one ROS node."""

    def __init__(self, node, swarm_config, config, clock=None, guidance='pursuit'):
        """`clock` gives the time the attack runs on (seconds); default the drone's PX4
        clock. The attack test passes Gazebo's simulation clock, which never jumps.
        `guidance` is 'pursuit' (the baseline) or 'mpc'."""
        if guidance not in GUIDANCE:
            raise ValueError(f'guidance must be one of {GUIDANCE}, not {guidance!r}')
        from geometry_msgs.msg import PointStamped, Vector3Stamped

        self.config = config
        self.guidance = guidance
        self.swarm_config = swarm_config
        self.log = node.get_logger()
        self.drone = PX4Drone(node, swarm_config.drone(config.attacker_id),
                              rate_hz=swarm_config.rate_hz)
        self.manager = SwarmManager(Swarm([self.drone]), ALTITUDE_TOLERANCE_M)
        self.inbox = DetectionInbox()
        self.clock = clock or (lambda: self.drone.stamp)
        self.detector = InboxDetector(self.drone, self.inbox, self.clock)
        self.attack = self._make_attack()
        self.position_topic = topic(config.attacker_id, POSITION_TOPIC)
        self.offset_topic = topic(config.attacker_id, OFFSET_TOPIC)
        node.create_subscription(PointStamped, self.position_topic, self._on_position, 10)
        node.create_subscription(Vector3Stamped, self.offset_topic, self._on_offset, 10)
        node.create_timer(1.0 / swarm_config.rate_hz, self._step)

    def new_attack(self) -> AttackAction | MpcIntercept:
        """A fresh attack (state, times) on the same drone and topics, e.g. for each test run."""
        self.detector = InboxDetector(self.drone, self.inbox, self.clock)
        self.attack = self._make_attack()
        return self.attack

    def _make_attack(self) -> AttackAction | MpcIntercept:
        c = self.config
        if self.guidance == 'pursuit':
            return AttackAction(self.drone, self.inbox, c, self.clock)
        # The MPC's speed is the attack's (swarm.yaml may keep it low for testing), and it
        # flies within the baseline's limits: minimum height, climb and descent speed.
        intercept = dataclasses.replace(self.swarm_config.intercept,
                                        interceptor_max_speed=c.max_speed)
        return MpcIntercept(self.drone, self.detector, intercept, self.clock,
                            search=(c.search_pitch_deg, c.search_heading_deg),
                            limit=lambda v: limit_velocity(v, self.drone.altitude, c))

    @property
    def guard(self):
        """Where the drone hovers while searching and returns to: above its start point."""
        x, y, z = self.drone.home
        return (x, y, z - self.config.guard_altitude)

    def _step(self) -> None:
        if self.guidance == 'mpc':
            now = self.clock()
            if now is not None:
                self.detector.update(now)  # newest message -> world position
        self.attack.step()

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

        self.log.info(f'Attacking ({self.guidance}) on {self.position_topic} and '
                      f'{self.offset_topic} at {c.max_speed} m/s. Searching toward '
                      f'{c.search_heading_deg} deg, gimbal {c.search_pitch_deg} deg up. '
                      'Ctrl-C to stop.')
        self.detector.start()
        self.attack.start()
        start = time.monotonic()
        seen = 0
        while duration <= 0 or time.monotonic() - start < duration:
            for t, state in self.attack.transitions[seen:]:
                distance = ''
                message = self.inbox.position
                if message is not None:
                    distance = f', target {sum(v * v for v in message.value) ** 0.5:.1f} m away'
                self.log.info(f'{getattr(state, "value", state).upper()}{distance}')
            seen = len(self.attack.transitions)
            time.sleep(POLL_S)

    def stop_and_land(self) -> None:
        self.attack.stop()
        if not self.drone.armed:
            return
        try:
            if self.drone.offboard:
                self.log.info('Returning above the start point')
                self.drone.set_position(*self.guard)
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
        if self.guidance == 'mpc':
            steps = ', '.join(f'{what} at {t:.1f} s' for t, what in self.attack.transitions)
            return f'MPC: {steps or "nothing"}'
        times = ', '.join(f'{s.value} {t:.1f} s' for s, t in self.attack.state_time.items() if t)
        return f'Time in each state: {times or "none"}'


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--config', default=None, help='swarm.yaml (default: the installed one)')
    parser.add_argument('--id', type=int, default=None, help='attacker drone (gimbal drone)')
    parser.add_argument('--duration', type=float, default=0.0,
                        help='seconds (wall clock) before landing; 0 = until Ctrl-C')
    parser.add_argument('--speed', type=float, default=None, help='m/s, overrides max_speed')
    parser.add_argument('--guidance', choices=GUIDANCE, default='pursuit',
                        help='pursuit (the baseline) or mpc')
    args = parser.parse_args(argv)

    swarm_config = load_config(args.config or default_config_path())
    overrides = {k: v for k, v in (('attacker_id', args.id), ('max_speed', args.speed))
                 if v is not None}
    config = dataclasses.replace(swarm_config.attack, **overrides)

    # Ctrl-C must reach our handler, so the drone lands instead of the node just dying.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node(f'attack_{config.attacker_id}')
    run = AttackNode(node, swarm_config, config, guidance=args.guidance)
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
