"""Read-only view of a PX4 drone: position and clock, no commands.

For processes that watch a drone someone else flies (the hivemind node). A PX4Drone
would stream its own setpoints and fight the process that is flying it.
"""
import math

from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node

from swarm_backend.config import DroneConfig
from swarm_backend.px4.px4_drone import PX4_OUT_QOS
from swarm_backend.px4.topics import PX4Topics


class DroneObserver:

    def __init__(self, node: Node, config: DroneConfig):
        self.id = config.id
        self.home = config.spawn_ned
        self.position = None  # world NED, metres
        self.velocity = None
        self.stamp = None  # PX4 clock, seconds (simulation time in SITL)
        self.clock = None  # () -> the receiver's time (the hivemind's Gazebo clock), if set
        self.arrived = None  # self.clock() when the latest odometry arrived
        node.create_subscription(
            VehicleOdometry, PX4Topics(config.namespace).vehicle_odometry, self._on_odometry,
            PX4_OUT_QOS)

    def _on_odometry(self, msg: VehicleOdometry) -> None:
        local = [float(v) for v in msg.position]
        if any(math.isnan(v) for v in local):
            return
        hx, hy, hz = self.home
        self.position = (local[0] + hx, local[1] + hy, local[2] + hz)
        self.velocity = tuple(float(v) for v in msg.velocity)
        self.stamp = msg.timestamp * 1e-6
        if self.clock is not None:
            self.arrived = self.clock()
