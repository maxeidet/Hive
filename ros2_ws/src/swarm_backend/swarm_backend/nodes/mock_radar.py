"""Mock base-station radar: measures the hostile drone (range, azimuth, elevation,
radial velocity, with noise) and publishes radar_msgs/RadarScan.

    ros2 run swarm_backend mock_radar [--config swarm.yaml]

Settings come from the `intercept:` section of swarm.yaml (hostile_id, radar_topic,
radar_rate_hz, radar_position, the radar_*_std errors, seed). It only listens to the
hostile drone, so it can run next to whatever is flying it. tests/intercept_test.py and
chase() in the shell start their own copy, so this node is for running the chase from
other code.
"""
import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from swarm_backend.config import default_config_path, load_config
from swarm_backend.intercept.radar import MockRadar
from swarm_backend.px4.observer import DroneObserver


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--config', help='swarm.yaml (default: the installed one)')
    cli = parser.parse_args(remove_ros_args(sys.argv)[1:])
    config = load_config(cli.config or default_config_path())
    settings = config.intercept

    rclpy.init(args=args)
    node = Node('mock_radar')
    hostile = DroneObserver(node, config.drone(settings.hostile_id))
    MockRadar(node, hostile, settings)
    node.get_logger().info(
        f'Reporting drone {settings.hostile_id} on {settings.radar_topic} at '
        f'{settings.radar_rate_hz} Hz from a radar at {settings.radar_position}')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
