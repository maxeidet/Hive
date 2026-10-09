"""Main ROS 2 entry point: builds the swarm from the config and flies a formation demo.

    ros2 run swarm_backend swarm_node
    ros2 run swarm_backend swarm_node --ros-args -p num_drones:=2 -p config_file:=/path/swarm.yaml

The demo sequence below is a stand-in until mission/mission.py (the swarm state
machine) exists; the node itself should stay limited to wiring components together.
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from swarm_backend.config import default_config_path, load_config
from swarm_backend.px4.px4_drone import PX4Drone
from swarm_backend.runner import LAND_TIMEOUT_S, spinning, wait_until
from swarm_backend.swarm.formation import Formation, centroid, circle_radius_for_spacing
from swarm_backend.swarm.manager import SwarmManager
from swarm_backend.swarm.swarm import Swarm

FORMATION_HOLD_S = 3.0


class SwarmNode(Node):

    def __init__(self):
        super().__init__('swarm')
        self.declare_parameter('config_file', '')
        self.declare_parameter('num_drones', 0)  # 0 = every drone in the config

        config_file = self.get_parameter('config_file').value or default_config_path()
        num_drones = self.get_parameter('num_drones').value or None
        self.config = load_config(config_file, num_drones)
        self.get_logger().info(f'Loaded {len(self.config.drones)} drones from {config_file}')

        self.swarm = Swarm(PX4Drone(self, c, rate_hz=self.config.rate_hz) for c in self.config.drones)
        self.manager = SwarmManager(self.swarm, self.config.position_tolerance)
        self.formation = Formation(len(self.swarm))


def run_demo(node: SwarmNode) -> None:
    log = node.get_logger()
    manager, formation, config = node.manager, node.formation, node.config

    log.info('Waiting for every drone to connect and pass pre-flight checks...')
    try:
        wait_until(manager.connected, 30, 'status and position from every drone')
    except TimeoutError:
        raise TimeoutError(
            f'No data from PX4 for drones {manager.not_connected()}. Are they running? '
            'Start them with `./mount-backend.sh spawn`, or use fewer drones') from None
    wait_until(manager.ready, 60, 'pre-flight checks to pass on every drone')

    log.info(f'Taking off to {config.takeoff_altitude} m')
    manager.takeoff(config.takeoff_altitude)
    wait_until(manager.all_reached, 45, 'every drone to reach takeoff altitude')

    # Form up above the middle of the spawn points.
    hx, hy, hz = centroid([d.home for d in manager.swarm])
    center = (hx, hy, hz - config.takeoff_altitude)
    spacing = config.formation_spacing
    for name, points in (
        ('grid', formation.grid(center, spacing)),
        ('line', formation.line(center, spacing)),
        ('circle', formation.circle(center, circle_radius_for_spacing(formation.size, spacing))),
    ):
        log.info(f'Forming {name}')
        manager.move_to_formation(points)
        wait_until(manager.all_reached, 45, f'the {name} formation')
        time.sleep(FORMATION_HOLD_S)

    log.info('Landing')
    manager.land()
    wait_until(manager.all_landed, LAND_TIMEOUT_S, 'every drone to land and disarm')
    log.info('Demo complete')


def main(args=None):
    # Keep Ctrl-C as a KeyboardInterrupt in this thread, so we can still land.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = SwarmNode()
    log = node.get_logger()
    with spinning(node):
        try:
            run_demo(node)
        except (TimeoutError, KeyboardInterrupt) as e:
            log.error(f'Aborting ({e or "Ctrl-C"}); landing every drone')
            node.manager.emergency_land()
            try:
                wait_until(node.manager.all_landed, LAND_TIMEOUT_S, 'every drone to land')
            except (TimeoutError, KeyboardInterrupt):
                log.error('Gave up waiting for the landing; PX4 lands on offboard loss anyway')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
