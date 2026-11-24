"""A Drone backed by one PX4 SITL instance over ROS 2 (uXRCE-DDS)."""
import math
import time

from px4_msgs.msg import VehicleCommand, VehicleCommandAck, VehicleOdometry, VehicleStatus
from px4_msgs.msg import GimbalDeviceAttitudeStatus, OffboardControlMode, TrajectorySetpoint
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from swarm_backend.config import DroneConfig, Vector3
from swarm_backend.px4 import messages
from swarm_backend.px4.drone import Drone, pitch_yaw_deg, wrap_deg
from swarm_backend.px4.topics import PX4Topics

# PX4's output topics (/fmu/out/...) publish BEST_EFFORT/TRANSIENT_LOCAL, not the
# default RELIABLE. A mismatched QoS connects but silently never receives anything.
PX4_OUT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

# PX4's input topics (/fmu/in/...) expect BEST_EFFORT/VOLATILE.
PX4_IN_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

COMMAND_NAMES = {
    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM: 'arm/disarm',
    VehicleCommand.VEHICLE_CMD_DO_SET_MODE: 'offboard mode',
    VehicleCommand.VEHICLE_CMD_NAV_LAND: 'land',
    VehicleCommand.VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE: 'gimbal control',
    VehicleCommand.VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW: 'gimbal pitch/yaw',
}
ACK_RESULTS = {
    VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED: 'accepted',
    VehicleCommandAck.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED: 'temporarily rejected',
    VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED: 'denied',
    VehicleCommandAck.VEHICLE_CMD_RESULT_UNSUPPORTED: 'unsupported',
    VehicleCommandAck.VEHICLE_CMD_RESULT_FAILED: 'failed',
    VehicleCommandAck.VEHICLE_CMD_RESULT_IN_PROGRESS: 'in progress',
}

RETRY_PERIOD_S = 1.0  # between repeated arm/offboard/land attempts
STAMP_JUMP_S = 2.0  # odometry comes at 10+ Hz; a bigger step in its time stamp is a jump
GIMBAL_TRIES = 5  # gimbal commands are re-sent until PX4 acks them, at most this often
GIMBAL_STREAM_HZ = 5.0  # how often aim_gimbal re-sends pitch/yaw
STREAM_BEFORE_OFFBOARD_S = 1.0  # PX4 rejects offboard without a live setpoint stream

# After our land command, a drone this low and this still for this long counts as down.
# PX4's own land detector often never confirms touchdown in our (slower than real
# time) sim, which leaves the drone armed on the ground, pushing into it.
TOUCHDOWN_ALTITUDE_M = 0.4
TOUCHDOWN_SPEED_MS = 0.2
TOUCHDOWN_TIME_S = 3.0


class PX4Drone(Drone):
    """Streams the offboard heartbeat and the current setpoint, and turns the
    arm/offboard/land requests into VehicleCommands that are repeated until PX4's
    status shows they took effect.

    Once a request has taken effect it is not re-sent: if someone else changes the
    mode later (QGroundControl, a failsafe), they are in charge and we do not fight it.
    """

    def __init__(self, node: Node, config: DroneConfig, rate_hz: float = 10.0):
        super().__init__(config.id, home=config.spawn_ned)
        self.config = config
        self.nav_state: int | None = None
        self.clock = None  # () -> the receiver's time (the hivemind's Gazebo clock), if set
        self.arrived = None  # self.clock() when the latest odometry arrived

        self._node = node
        self._log = node.get_logger().get_child(f'drone{config.id}')
        topics = PX4Topics(config.namespace)

        self._offboard_mode_pub = node.create_publisher(
            OffboardControlMode, topics.offboard_control_mode, PX4_IN_QOS)
        self._setpoint_pub = node.create_publisher(
            TrajectorySetpoint, topics.trajectory_setpoint, PX4_IN_QOS)
        self._command_pub = node.create_publisher(
            VehicleCommand, topics.vehicle_command, PX4_IN_QOS)

        node.create_subscription(
            VehicleOdometry, topics.vehicle_odometry, self._on_odometry, PX4_OUT_QOS)
        node.create_subscription(
            VehicleStatus, topics.vehicle_status, self._on_status, PX4_OUT_QOS)
        node.create_subscription(
            VehicleCommandAck, topics.vehicle_command_ack, self._on_ack, PX4_OUT_QOS)
        node.create_subscription(
            GimbalDeviceAttitudeStatus, topics.gimbal_attitude, self._on_gimbal, PX4_OUT_QOS)

        # What we are streaming: ('position', local NED, velocity feed-forward or None) or
        # ('velocity', NED, None), None until the first odometry arrives. Until someone
        # commands a setpoint (and again after landing) it follows the current position, so
        # switching to offboard never makes the drone jump to a stale target.
        self._setpoint: tuple[str, Vector3, Vector3 | None] | None = None
        self._tracking = True
        self._yaw = 0.0  # radians from north

        # Pending requests, cleared by _on_status once PX4 reports them done.
        self._want_armed: bool | None = None
        self._want_offboard = False
        self._want_land = False
        self._disarm_on_touchdown = False  # set by land(), until the drone disarms
        self._want_gimbal: tuple[float, float] | None = None  # (pitch, yaw) degrees
        self._gimbal_tries = 0
        self._gimbal_aim: tuple[float, float] | None = None  # streamed by aim_gimbal
        self._gimbal_controlled = False  # PX4 acked us as the gimbal's controller
        self._still_on_ground_since: float | None = None

        self._ticks_per_retry = max(1, round(RETRY_PERIOD_S * rate_hz))
        self._ticks_before_offboard = max(1, round(STREAM_BEFORE_OFFBOARD_S * rate_hz))
        self._ticks_per_gimbal_aim = max(1, round(rate_hz / GIMBAL_STREAM_HZ))
        self._tick = 0
        self._streamed_ticks = 0
        node.create_timer(1.0 / rate_hz, self._on_timer)

        self._log.info(f'Using topics under "{topics.namespace or "/"}", system id {config.system_id}')

    def reset(self, home: Vector3) -> None:
        """Forget everything about the PX4 behind this drone: it was stopped and a new one
        started (same instance, so the same topics) at `home`, world NED on the ground."""
        self.home = home
        self.position = self.velocity = self.attitude = self.stamp = None
        self.arrived = None  # self.clock() when the latest odometry arrived
        self.connected = self.preflight_ok = self.armed = self.offboard = self.landing = False
        self.target = None
        self.gimbal = None
        self.nav_state = None
        self._setpoint = None
        self._tracking = True
        self._yaw = 0.0
        self._want_armed = None
        self._want_offboard = self._want_land = self._disarm_on_touchdown = False
        self._want_gimbal = self._gimbal_aim = None
        self._gimbal_tries = 0
        self._gimbal_controlled = False
        self._still_on_ground_since = None
        self._streamed_ticks = 0

    # --- commands ----------------------------------------------------------

    def arm(self) -> None:
        self._disarm_on_touchdown = False
        self._want_armed = True

    def disarm(self) -> None:
        self._want_armed = False

    def start_offboard(self) -> None:
        self._want_land = False
        self._disarm_on_touchdown = False
        self._want_offboard = True

    def land(self) -> None:
        self._want_offboard = False
        self._want_armed = None
        self._want_land = True
        self._disarm_on_touchdown = True
        self._tracking = True
        self.target = None

    def set_position(self, x: float, y: float, z: float, yaw: float | None = None,
                     velocity: Vector3 | None = None) -> None:
        self.target = (float(x), float(y), float(z))
        self._tracking = False
        hx, hy, hz = self.home
        feed_forward = None if velocity is None else tuple(float(v) for v in velocity)
        self._setpoint = ('position', (x - hx, y - hy, z - hz), feed_forward)
        if yaw is not None:
            self._yaw = float(yaw)

    def set_velocity(self, vx: float, vy: float, vz: float, yaw: float | None = None) -> None:
        self.target = None
        self._tracking = False
        self._setpoint = ('velocity', (float(vx), float(vy), float(vz)), None)
        if yaw is not None:
            self._yaw = float(yaw)

    def point_gimbal(self, pitch: float, yaw: float) -> None:
        self._gimbal_aim = None
        self._want_gimbal = (float(pitch), float(yaw))
        self._gimbal_tries = 0

    def aim_gimbal(self, pitch: float, yaw: float) -> None:
        if self._gimbal_aim is None:
            self._gimbal_controlled = False
        self._want_gimbal = None
        self._gimbal_aim = (float(pitch), float(yaw))

    def release_gimbal(self) -> None:
        self._gimbal_aim = None

    # --- what PX4 tells us -------------------------------------------------

    def _on_odometry(self, msg: VehicleOdometry) -> None:
        local = [float(v) for v in msg.position]
        if any(math.isnan(v) for v in local):
            return
        hx, hy, hz = self.home
        self.position = (local[0] + hx, local[1] + hy, local[2] + hz)
        self.velocity = tuple(float(v) for v in msg.velocity)
        self.attitude = tuple(float(v) for v in msg.q)
        stamp = msg.timestamp * 1e-6
        if self.stamp is not None and not -0.5 < stamp - self.stamp < STAMP_JUMP_S:
            self._log.warning(f'PX4 clock jumped {stamp - self.stamp:+.1f} s between two '
                              'odometry messages (time sync?)', throttle_duration_sec=5.0)
        self.stamp = stamp
        if self.clock is not None:
            self.arrived = self.clock()
        if self._tracking:
            self._setpoint = ('position', tuple(local), None)

    def _on_status(self, msg: VehicleStatus) -> None:
        if not self.connected:
            self._log.info('Connected to PX4')
        armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        offboard = msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        landing = msg.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND

        if armed != self.armed:
            self._log.info('ARMED' if armed else 'disarmed')
        if offboard != self.offboard:
            if offboard:
                self._log.info('In OFFBOARD mode')
            elif not landing and armed:
                self._log.warning(
                    'Left offboard mode (QGroundControl or a failsafe); not taking control back')
        if msg.pre_flight_checks_pass != self.preflight_ok:
            self._log.info('Pre-flight checks ' + ('PASS' if msg.pre_flight_checks_pass else 'FAIL'))

        self.connected = True
        self.armed = armed
        self.offboard = offboard
        self.landing = landing
        self.preflight_ok = msg.pre_flight_checks_pass
        self.nav_state = msg.nav_state

        if self._want_armed is not None and self._want_armed == armed:
            self._want_armed = None
        if self._want_offboard and offboard:
            self._want_offboard = False
        if self._want_land and (landing or not armed):
            self._want_land = False
        if not armed:
            self._disarm_on_touchdown = False

    def _on_gimbal(self, msg: GimbalDeviceAttitudeStatus) -> None:
        q = [float(v) for v in msg.q]
        if any(math.isnan(v) for v in q):
            return
        pitch, yaw = pitch_yaw_deg(q)
        # PX4's sim gimbal reports yaw in the earth frame (NED, from north); make it
        # relative to the nose like the command.
        if msg.device_flags & GimbalDeviceAttitudeStatus.DEVICE_FLAGS_YAW_IN_EARTH_FRAME:
            if self.attitude is None:
                return
            yaw -= pitch_yaw_deg(self.attitude)[1]
        self.gimbal = (pitch, wrap_deg(yaw))

    def _on_ack(self, msg: VehicleCommandAck) -> None:
        name = COMMAND_NAMES.get(msg.command)
        if name is None:
            return
        accepted = msg.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED
        if msg.command == VehicleCommand.VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW and accepted:
            self._want_gimbal = None
            if self._gimbal_aim is not None:
                return  # streamed several times a second; not worth a log line each
        if msg.command == VehicleCommand.VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE and accepted:
            self._gimbal_controlled = True
        result = ACK_RESULTS.get(msg.result, f'result {msg.result}')
        if msg.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
            self._log.info(f'PX4 {result} the {name} command')
        else:
            self._log.warning(
                f'PX4 {result} the {name} command (see the PX4 terminal for why)',
                throttle_duration_sec=5.0)

    # --- what we tell PX4 --------------------------------------------------

    def _on_timer(self) -> None:
        self._stream_setpoint()
        self._tick += 1
        if self._tick % self._ticks_per_gimbal_aim == 0:
            self._stream_gimbal_aim()
        if self._tick % self._ticks_per_retry == 0:
            self._send_pending_commands()
            self._send_gimbal_command()
            self._disarm_if_touched_down()

    def _stream_setpoint(self) -> None:
        if self._setpoint is None:
            return
        now = self._now_us()
        kind, value, feed_forward = self._setpoint
        if kind == 'position':
            mode = messages.offboard_control_mode(now, position=True)
            setpoint = messages.position_setpoint(now, value, self._yaw, feed_forward)
        else:
            mode = messages.offboard_control_mode(now, velocity=True)
            setpoint = messages.velocity_setpoint(now, value, self._yaw)
        self._offboard_mode_pub.publish(mode)
        self._setpoint_pub.publish(setpoint)
        self._streamed_ticks += 1

    def _send_pending_commands(self) -> None:
        if not (self._want_land or self._want_armed is not None or self._want_offboard):
            return
        if not self.connected:
            self._log.warning(
                'No vehicle status from PX4 yet; is this instance running and the DDS agent up?',
                throttle_duration_sec=5.0)
            return
        system = self.config.system_id
        now = self._now_us()

        if self._want_land:
            self._command_pub.publish(messages.land_command(now, system))
            return

        if self._want_armed is True:
            if self.preflight_ok:
                self._command_pub.publish(messages.arm_command(now, system))
            else:
                # PX4 would refuse and print "Arming denied", so wait instead.
                self._log.warning(
                    'Waiting to arm: pre-flight checks are failing (see "Preflight Fail" '
                    'in the PX4 terminal)', throttle_duration_sec=5.0)
        elif self._want_armed is False:
            self._command_pub.publish(messages.disarm_command(now, system))

        if self._want_offboard and self._streamed_ticks >= self._ticks_before_offboard:
            self._command_pub.publish(messages.offboard_mode_command(now, system))

    def _send_gimbal_command(self) -> None:
        if self._gimbal_aim is not None and not self._gimbal_controlled and self.connected:
            # aim_gimbal's pitch/yaw stream is ignored until we are the gimbal's controller.
            self._command_pub.publish(
                messages.gimbal_take_control_command(self._now_us(), self.config.system_id))
        if self._want_gimbal is None or not self.connected:
            return
        if self._gimbal_tries >= GIMBAL_TRIES:
            self._want_gimbal = None
            if self.gimbal is None:
                self._log.warning('No gimbal answered. Only gimbal drones (spawn gimbal, '
                                  'model gz_x500_gimbal) have one.')
            return
        self._gimbal_tries += 1
        system = self.config.system_id
        now = self._now_us()
        self._command_pub.publish(messages.gimbal_take_control_command(now, system))
        self._command_pub.publish(messages.gimbal_pitch_yaw_command(now, system, *self._want_gimbal))

    def _stream_gimbal_aim(self) -> None:
        if self._gimbal_aim is None or not self.connected:
            return
        self._command_pub.publish(messages.gimbal_pitch_yaw_command(
            self._now_us(), self.config.system_id, *self._gimbal_aim))

    def _disarm_if_touched_down(self) -> None:
        on_ground = (
            self._disarm_on_touchdown and self.armed and self.landing
            and self.altitude is not None and self.altitude < TOUCHDOWN_ALTITUDE_M
            and self.velocity is not None and abs(self.velocity[2]) < TOUCHDOWN_SPEED_MS)
        if not on_ground:
            self._still_on_ground_since = None
            return
        now = time.monotonic()
        if self._still_on_ground_since is None:
            self._still_on_ground_since = now
        elif now - self._still_on_ground_since >= TOUCHDOWN_TIME_S:
            self._log.info('On the ground but PX4 has not disarmed; forcing disarm',
                           throttle_duration_sec=5.0)
            self._command_pub.publish(
                messages.disarm_command(self._now_us(), self.config.system_id, force=True))

    def _now_us(self) -> int:
        return self._node.get_clock().now().nanoseconds // 1000
