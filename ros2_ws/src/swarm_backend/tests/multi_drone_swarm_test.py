"""Milestone 2 flight test: every drone in the config takes off, hovers and lands together.

Needs one PX4 instance per drone in swarm.yaml (see the package README). Inside the
container, after `colcon build` and sourcing install/setup.bash:

    python3 /workspace/ros2_ws/src/swarm_backend/tests/multi_drone_swarm_test.py
    python3 .../multi_drone_swarm_test.py --num-drones 2

If anything times out or you press Ctrl-C, all drones are told to land.
"""
import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from swarm_backend.config import default_config_path, load_config
from swarm_backend.px4.px4_drone import PX4Drone
from swarm_backend.runner import LAND_TIMEOUT_S, spinning, wait_until
from swarm_backend.swarm.manager import SwarmManager
from swarm_backend.swarm.swarm import Swarm


def fly(manager: SwarmManager, log, altitude: float, hover_s: float) -> None:
    log.info(f'Waiting for PX4 on drones {manager.swarm.ids}...')
    try:
        wait_until(manager.connected, 30, 'status and position from every drone')
    except TimeoutError:
        raise TimeoutError(
            f'No data from PX4 for drones {manager.not_connected()}. Are they running? '
            'Start them with `./mount-backend.sh spawn`, or use fewer drones') from None
    wait_until(manager.ready, 60, 'pre-flight checks to pass on every drone')

    log.info(f'Taking off to {altitude} m')
    manager.takeoff(altitude)
    wait_until(manager.all_armed, 20, 'every drone to arm')
    wait_until(manager.all_offboard, 20, 'every drone to enter offboard')
    wait_until(manager.all_reached, 30, 'every drone to reach takeoff altitude')

    log.info(f'Hovering for {hover_s} s')
    time.sleep(hover_s)

    land_and_wait(manager, log)
    log.info('Multi-drone test PASSED')


def land_and_wait(manager: SwarmManager, log) -> None:
    log.info('Landing all drones')
    manager.land()
    wait_until(manager.all_landed, LAND_TIMEOUT_S, 'every drone to land and disarm')
    log.info('All drones landed and disarmed')


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--config', help='swarm.yaml (default: the installed one)')
    parser.add_argument('--num-drones', type=int, help='fly only the first N drones in the config')
    parser.add_argument('--altitude', type=float, help='metres (default: from config)')
    parser.add_argument('--hover', type=float, default=5.0, help='seconds to hover')
    args = parser.parse_args()

    config = load_config(args.config or default_config_path(), args.num_drones)
    altitude = args.altitude or config.takeoff_altitude

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('multi_drone_swarm_test')
    log = node.get_logger()
    swarm = Swarm(PX4Drone(node, c, rate_hz=config.rate_hz) for c in config.drones)
    manager = SwarmManager(swarm, config.position_tolerance)

    with spinning(node):
        try:
            fly(manager, log, altitude, args.hover)
        except (TimeoutError, KeyboardInterrupt) as e:
            log.error(f'Aborting ({e or "Ctrl-C"})')
            if manager.any_armed():
                land_and_wait(manager, log)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
