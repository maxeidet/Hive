import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from px4_msgs.msg import VehicleOdometry

# PX4's uXRCE-DDS topics publish BEST_EFFORT/TRANSIENT_LOCAL, not the
# default RELIABLE — a mismatched QoS means the subscription connects
# but silently never receives anything.
PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


class SwarmBackend(Node):

    def __init__(self):
        super().__init__('swarm_backend')
        self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.on_odometry,
            PX4_QOS,
        )

    def on_odometry(self, msg: VehicleOdometry) -> None:
        self.get_logger().info(f'position: {msg.position}')


def main(args=None):
    rclpy.init(args=args)
    node = SwarmBackend()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
