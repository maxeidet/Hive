"""Milestone 1 flight test: one PX4 drone flies arm -> offboard -> takeoff -> move -> hover -> land.

Needs PX4 SITL, Gazebo and the DDS agent running. Inside the container, after
`colcon build` and sourcing install/setup.bash:

    python3 /workspace/ros2_ws/src/swarm_backend/tests/single_drone_test.py
    python3 .../single_drone_test.py --drone-id 1 --distance 3

If anything times out or you press Ctrl-C, the drone is told to land.
"""
import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from swarm_backend.config import default_config_path, load_config
from swarm_backend.px4.px4_drone import PX4Drone
from swarm_backend.runner import LAND_TIMEOUT_S, spinning, wait_until


def fly(drone: PX4Drone, log, altitude: float, distance: float, hover_s: float,
        tolerance: float) -> None:
    log.info('Waiting for PX4...')
    wait_until(lambda: drone.connected and drone.position is not None, 30, 'PX4 status and position')
    wait_until(lambda: drone.preflight_ok, 60, 'pre-flight checks to pass')

    log.info('Arming')
    drone.arm()
    wait_until(lambda: drone.armed, 15, 'arming')

    log.info('Entering offboard mode')
    drone.start_offboard()
    wait_until(lambda: drone.offboard, 15, 'offboard mode')

    log.info(f'Taking off to {altitude} m')
    drone.takeoff(altitude)
    wait_until(lambda: drone.reached(tolerance), 30, 'takeoff altitude')

    x, y, z = drone.target
    log.info(f'Moving {distance} m north')
    drone.set_position(x + distance, y, z)
    wait_until(lambda: drone.reached(tolerance), 30, 'the move target')

    log.info(f'Hovering for {hover_s} s')
    time.sleep(hover_s)

    land_and_wait(drone, log)
    log.info('Single-drone test PASSED')


def land_and_wait(drone: PX4Drone, log) -> None:
    log.info('Landing')
    drone.land()
    wait_until(lambda: not drone.armed, LAND_TIMEOUT_S, 'landing and auto-disarm')
    log.info('Landed and disarmed')


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--config', help='swarm.yaml (default: the installed one)')
    parser.add_argument('--drone-id', type=int, default=0)
    parser.add_argument('--altitude', type=float, help='metres (default: from config)')
    parser.add_argument('--distance', type=float, default=5.0, help='metres to fly north')
    parser.add_argument('--hover', type=float, default=5.0, help='seconds to hover')
    args = parser.parse_args()

    config = load_config(args.config or default_config_path())
    altitude = args.altitude or config.takeoff_altitude

    # Keep Ctrl-C as a KeyboardInterrupt in this thread, so we can still land.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('single_drone_test')
    log = node.get_logger()
    drone = PX4Drone(node, config.drone(args.drone_id), rate_hz=config.rate_hz)

    with spinning(node):
        try:
            fly(drone, log, altitude, args.distance, args.hover, config.position_tolerance)
        except (TimeoutError, KeyboardInterrupt) as e:
            log.error(f'Aborting ({e or "Ctrl-C"})')
            if drone.armed:
                land_and_wait(drone, log)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
