"""Saves the 3D map as a .bt file when DAEP reports the exploration completed.

Waits for True on /exploration_completed, then runs octomap_server's octomap_saver_node
against the octomap_server in `map_ns` and exits. `path` is relative to the working
directory, like rpl_exploration's data/ folder (coverage.csv, ...), so the map ends up
next to those logs. Started by launch/daep_exploration.launch.py.

    ros2 run swarm_backend save_map --ros-args -p map_ns:=mapping -p path:=data/map.bt
"""
import os
import subprocess
import sys
import threading

import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.node import Node
from std_msgs.msg import Bool

SAVE_TIMEOUT_S = 120.0  # the saver asks octomap_server for the map over a service


class MapSaver(Node):
    def __init__(self):
        super().__init__('save_map')
        self.map_ns = self.declare_parameter('map_ns', 'mapping').value.strip('/')
        self.path = os.path.abspath(self.declare_parameter('path', 'data/map.bt').value)
        self.done = threading.Event()
        self._started = False
        # rpl_exploration publishes False with every pose until it is done.
        self.create_subscription(Bool, '/exploration_completed', self._on_completed, 1)
        self.get_logger().info(f'Will save /{self.map_ns} to {self.path} when the exploration completes')

    def _on_completed(self, msg: Bool) -> None:
        if msg.data and not self._started:
            self._started = True
            threading.Thread(target=self._save, daemon=True).start()

    def _save(self) -> None:
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        saver = os.path.join(get_package_prefix('octomap_server'), 'lib', 'octomap_server',
                             'octomap_saver_node')
        cmd = [saver, '--ros-args', '-r', f'__ns:=/{self.map_ns}', '-p', f'octomap_path:={self.path}']
        self.get_logger().info('Exploration completed: saving the map')
        try:
            subprocess.run(cmd, timeout=SAVE_TIMEOUT_S, stdout=subprocess.DEVNULL,
                           stderr=subprocess.STDOUT, check=False)
        except subprocess.TimeoutExpired:
            self.get_logger().error(f'octomap_saver_node did not finish in {SAVE_TIMEOUT_S:.0f} s')
        if os.path.exists(self.path):
            self.get_logger().info(
                f'Map saved: {self.path} ({os.path.getsize(self.path) / 1e3:.0f} kB)')
        else:
            self.get_logger().error(f'Could not save the map to {self.path}')
        self.done.set()


def main() -> int:
    rclpy.init(args=sys.argv)
    node = MapSaver()
    try:
        while rclpy.ok() and not node.done.is_set():
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
