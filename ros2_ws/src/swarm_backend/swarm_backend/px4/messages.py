"""Builders for the PX4 messages the backend sends, so no other module fills in fields."""
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand

from swarm_backend.config import Vector3

NAN = float('nan')
NAN3 = [NAN, NAN, NAN]

PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
FORCE_DISARM_MAGIC = 21196.0  # param2 value that makes PX4 disarm even if it thinks it is flying
SOURCE_SYSTEM = 1  # MAVLink system/component our commands claim to come from
SOURCE_COMPONENT = 1


def offboard_control_mode(timestamp_us: int, *, position: bool = False,
                          velocity: bool = False) -> OffboardControlMode:
    """The offboard heartbeat. PX4 drops out of offboard if it stops (< 2 Hz)."""
    msg = OffboardControlMode()
    msg.position = position
    msg.velocity = velocity
    msg.timestamp = timestamp_us
    return msg


def position_setpoint(timestamp_us: int, position: Vector3, yaw: float,
                      velocity: Vector3 | None = None) -> TrajectorySetpoint:
    """Fly to a position in PX4's local NED frame. `velocity` is an optional feed-forward
    that PX4 adds to its position controller's output, for setpoints that move."""
    msg = _setpoint(timestamp_us)
    msg.position = [float(v) for v in position]
    if velocity is not None:
        msg.velocity = [float(v) for v in velocity]
    msg.yaw = float(yaw)
    return msg


def velocity_setpoint(timestamp_us: int, velocity: Vector3, yaw: float) -> TrajectorySetpoint:
    """Fly at a velocity in PX4's local NED frame."""
    msg = _setpoint(timestamp_us)
    msg.velocity = [float(v) for v in velocity]
    msg.yaw = float(yaw)
    return msg


def _setpoint(timestamp_us: int) -> TrajectorySetpoint:
    # NaN means "do not control this". The message default is 0.0, which would
    # tell PX4 to hold zero velocity/acceleration while flying to a position.
    msg = TrajectorySetpoint()
    msg.position = list(NAN3)
    msg.velocity = list(NAN3)
    msg.acceleration = list(NAN3)
    msg.jerk = list(NAN3)
    msg.yaw = NAN
    msg.yawspeed = NAN
    msg.timestamp = timestamp_us
    return msg


def vehicle_command(timestamp_us: int, target_system: int, command: int,
                    param1: float = 0.0, param2: float = 0.0, param3: float = 0.0,
                    param4: float = 0.0, param5: float = 0.0, param7: float = 0.0) -> VehicleCommand:
    msg = VehicleCommand()
    msg.command = command
    msg.param1 = float(param1)
    msg.param2 = float(param2)
    msg.param3 = float(param3)
    msg.param4 = float(param4)
    msg.param5 = float(param5)
    msg.param7 = float(param7)
    msg.target_system = target_system
    msg.target_component = 1
    msg.source_system = SOURCE_SYSTEM
    msg.source_component = SOURCE_COMPONENT
    msg.from_external = True
    msg.timestamp = timestamp_us
    return msg


def arm_command(timestamp_us: int, target_system: int) -> VehicleCommand:
    return vehicle_command(
        timestamp_us, target_system, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)


def disarm_command(timestamp_us: int, target_system: int, force: bool = False) -> VehicleCommand:
    return vehicle_command(
        timestamp_us, target_system, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
        param1=0.0, param2=FORCE_DISARM_MAGIC if force else 0.0)


def offboard_mode_command(timestamp_us: int, target_system: int) -> VehicleCommand:
    return vehicle_command(
        timestamp_us, target_system, VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
        param1=1.0, param2=float(PX4_CUSTOM_MAIN_MODE_OFFBOARD))


def land_command(timestamp_us: int, target_system: int) -> VehicleCommand:
    return vehicle_command(timestamp_us, target_system, VehicleCommand.VEHICLE_CMD_NAV_LAND)


def gimbal_take_control_command(timestamp_us: int, target_system: int) -> VehicleCommand:
    """Make us the gimbal's primary controller. PX4's gimbal manager (MAVLink gimbal v2,
    MNT_MODE_IN=4 in the gz_x500_gimbal airframe) ignores pitch/yaw from anyone else.
    Our ids are given explicitly: PX4 resolves "-2 = the sender" to its own id for commands
    arriving over DDS. param3/4 = -1 leave the secondary controller unchanged."""
    return vehicle_command(
        timestamp_us, target_system, VehicleCommand.VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
        param1=SOURCE_SYSTEM, param2=SOURCE_COMPONENT, param3=-1.0, param4=-1.0)


def gimbal_pitch_yaw_command(timestamp_us: int, target_system: int,
                             pitch_deg: float, yaw_deg: float) -> VehicleCommand:
    """Point the gimbal. Pitch 0 is level, -90 straight down; yaw is relative to the
    drone's nose (flags 0 = yaw follows the vehicle). Rates NaN = go there at full speed.
    param7 = 0 addresses every gimbal on the drone."""
    return vehicle_command(
        timestamp_us, target_system, VehicleCommand.VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
        param1=pitch_deg, param2=yaw_deg, param3=NAN, param4=NAN, param5=0.0, param7=0.0)
