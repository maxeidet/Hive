import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleOdometry,
)

# PX4's uXRCE-DDS output topics (/fmu/out/...) publish BEST_EFFORT/TRANSIENT_LOCAL,
# not the default RELIABLE — a mismatched QoS means the subscription connects
# but silently never receives anything.
PX4_OUT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

# PX4's input topics (/fmu/in/...) expect BEST_EFFORT/VOLATILE instead —
# confirmed via `ros2 topic info /fmu/in/... --verbose`, not assumed.
PX4_IN_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
VEHICLE_CMD_DO_SET_MODE = 176
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6

TAKEOFF_ALTITUDE_M = 5.0  # NED frame, so target z is negative
SETPOINTS_BEFORE_ARMING = 10  # ~1s at 10Hz — PX4 needs a live setpoint stream first


class SwarmBackend(Node):

    def __init__(self):
        super().__init__('swarm_backend')

        self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.on_odometry,
            PX4_OUT_QOS,
        )

        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', PX4_IN_QOS)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', PX4_IN_QOS)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', PX4_IN_QOS)

        self.setpoint_count = 0
        self.create_timer(0.1, self.on_timer)  # 10 Hz, PX4 requires >=2Hz

    def on_odometry(self, msg: VehicleOdometry) -> None:
        self.get_logger().info(f'position: {msg.position}')

    def on_timer(self) -> None:
        self.publish_offboard_heartbeat()
        self.publish_takeoff_setpoint()

        # PX4 refuses to enter offboard mode until it has already seen a
        # live setpoint stream, so only arm + switch mode after a short warm-up.
        if self.setpoint_count == SETPOINTS_BEFORE_ARMING:
            self.arm()
            self.switch_to_offboard()

        self.setpoint_count += 1

    def publish_offboard_heartbeat(self) -> None:
        msg = OffboardControlMode()
        msg.position = True
        msg.timestamp = self.now_us()
        self.offboard_mode_pub.publish(msg)

    def publish_takeoff_setpoint(self) -> None:
        msg = TrajectorySetpoint()
        msg.position = [0.0, 0.0, -TAKEOFF_ALTITUDE_M]  # NED: negative z = up
        msg.timestamp = self.now_us()
        self.setpoint_pub.publish(msg)

    def arm(self) -> None:
        self.send_command(VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('arm command sent')

    def switch_to_offboard(self) -> None:
        self.send_command(
            VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=float(PX4_CUSTOM_MAIN_MODE_OFFBOARD))
        self.get_logger().info('offboard mode command sent')

    def send_command(self, command: int, param1: float = 0.0, param2: float = 0.0) -> None:
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.now_us()
        self.command_pub.publish(msg)

    def now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)


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
