import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleCommandAck,
    VehicleOdometry,
    VehicleStatus,
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

COMMAND_NAMES = {
    VEHICLE_CMD_COMPONENT_ARM_DISARM: 'arm',
    VEHICLE_CMD_DO_SET_MODE: 'offboard mode',
}
ACK_RESULTS = {
    VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED: 'accepted',
    VehicleCommandAck.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED: 'temporarily rejected',
    VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED: 'denied',
    VehicleCommandAck.VEHICLE_CMD_RESULT_UNSUPPORTED: 'unsupported',
    VehicleCommandAck.VEHICLE_CMD_RESULT_FAILED: 'failed',
    VehicleCommandAck.VEHICLE_CMD_RESULT_IN_PROGRESS: 'in progress',
}

HOVER_ALTITUDE_M = 5.0  # NED frame, so the target z is negative
ALTITUDE_TOLERANCE_M = 0.3
TIMER_PERIOD_S = 0.1  # 10 Hz, PX4 requires the offboard heartbeat at >= 2 Hz
SETPOINTS_BEFORE_ARMING = 10  # ~1 s — PX4 needs a live setpoint stream first
RETRY_EVERY_TICKS = 10  # ~1 s between arm/offboard attempts


class SwarmBackend(Node):

    def __init__(self):
        super().__init__('swarm_backend')

        self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.on_odometry, PX4_OUT_QOS)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v4', self.on_status, PX4_OUT_QOS)
        self.create_subscription(
            VehicleCommandAck, '/fmu/out/vehicle_command_ack_v1', self.on_ack, PX4_OUT_QOS)

        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', PX4_IN_QOS)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', PX4_IN_QOS)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', PX4_IN_QOS)

        self.tick = 0
        self.armed = False
        self.offboard = False
        self.preflight_ok = False
        self.status_seen = False
        self.flight_started = False
        self.reached_altitude = False

        self.create_timer(TIMER_PERIOD_S, self.on_timer)

    # --- what PX4 tells us -------------------------------------------------

    def on_status(self, msg: VehicleStatus) -> None:
        self.status_seen = True
        armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        offboard = msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

        if armed != self.armed:
            self.get_logger().info('PX4 is now ' + ('ARMED' if armed else 'disarmed'))
        if offboard != self.offboard:
            if offboard:
                self.get_logger().info('PX4 is now in OFFBOARD mode')
            else:
                self.get_logger().warning(
                    'PX4 left offboard mode (switched from QGroundControl, or a failsafe). '
                    'Not taking control back; restart the node to fly again.')
        if msg.pre_flight_checks_pass != self.preflight_ok:
            self.get_logger().info(
                'PX4 pre-flight checks now ' + ('PASS' if msg.pre_flight_checks_pass else 'FAIL'))

        self.armed = armed
        self.offboard = offboard
        self.preflight_ok = msg.pre_flight_checks_pass
        if armed and offboard:
            self.flight_started = True

    def on_ack(self, msg: VehicleCommandAck) -> None:
        name = COMMAND_NAMES.get(msg.command)
        if name is None:
            return
        result = ACK_RESULTS.get(msg.result, f'result {msg.result}')
        if msg.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
            self.get_logger().info(f'PX4 {result} the {name} command')
        else:
            self.get_logger().warning(
                f'PX4 {result} the {name} command (see the PX4 terminal for why)',
                throttle_duration_sec=5.0)

    def on_odometry(self, msg: VehicleOdometry) -> None:
        altitude = -float(msg.position[2])  # NED: negative z is up
        self.get_logger().info(
            f'position x={msg.position[0]:.2f} y={msg.position[1]:.2f} altitude={altitude:.2f} m',
            throttle_duration_sec=1.0)
        if (not self.reached_altitude and self.armed
                and abs(altitude - HOVER_ALTITUDE_M) < ALTITUDE_TOLERANCE_M):
            self.reached_altitude = True
            self.get_logger().info(f'Reached {HOVER_ALTITUDE_M} m, hovering')

    # --- what we tell PX4 --------------------------------------------------

    def on_timer(self) -> None:
        self.publish_offboard_heartbeat()
        self.publish_hover_setpoint()
        self.tick += 1

        # Retry only until the flight has started once. After that, whoever changed
        # the mode (QGroundControl, a failsafe) is in charge and we do not fight it.
        if self.tick < SETPOINTS_BEFORE_ARMING or self.flight_started:
            return
        if self.tick % RETRY_EVERY_TICKS != 0:
            return

        if not self.status_seen:
            self.get_logger().warning(
                'No vehicle status from PX4 yet — is PX4 running and the DDS agent connected?',
                throttle_duration_sec=5.0)
        elif not self.preflight_ok:
            # PX4 would refuse to arm and print "Arming denied", so wait instead.
            self.get_logger().warning(
                'Waiting: PX4 pre-flight checks are failing (see the "Preflight Fail" '
                'lines in the PX4 terminal). Trying again every second.',
                throttle_duration_sec=5.0)
        else:
            self.send_command(
                VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=float(PX4_CUSTOM_MAIN_MODE_OFFBOARD))
            self.send_command(VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def publish_offboard_heartbeat(self) -> None:
        msg = OffboardControlMode()
        msg.position = True
        msg.timestamp = self.now_us()
        self.offboard_mode_pub.publish(msg)

    def publish_hover_setpoint(self) -> None:
        nan = float('nan')
        msg = TrajectorySetpoint()
        msg.position = [0.0, 0.0, -HOVER_ALTITUDE_M]  # above the start point
        # NaN means "do not control this". The message default is 0.0, which
        # would tell PX4 to hold zero velocity/acceleration while climbing.
        msg.velocity = [nan, nan, nan]
        msg.acceleration = [nan, nan, nan]
        msg.jerk = [nan, nan, nan]
        msg.yaw = 0.0  # face north
        msg.yawspeed = nan
        msg.timestamp = self.now_us()
        self.setpoint_pub.publish(msg)

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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
