"""Builders for the PX4 messages the backend sends, so no other module fills in fields."""
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand

from swarm_backend.config import Vector3

NAN = float('nan')
NAN3 = [NAN, NAN, NAN]

PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
FORCE_DISARM_MAGIC = 21196.0  # param2 value that makes PX4 disarm even if it thinks it is flying


def offboard_control_mode(timestamp_us: int, *, position: bool = False,
                          velocity: bool = False) -> OffboardControlMode:
    """The offboard heartbeat. PX4 drops out of offboard if it stops (< 2 Hz)."""
    msg = OffboardControlMode()
    msg.position = position
    msg.velocity = velocity
    msg.timestamp = timestamp_us
    return msg


def position_setpoint(timestamp_us: int, position: Vector3, yaw: float) -> TrajectorySetpoint:
    """Fly to a position in PX4's local NED frame."""
    msg = _setpoint(timestamp_us)
    msg.position = [float(v) for v in position]
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
                    param1: float = 0.0, param2: float = 0.0) -> VehicleCommand:
    msg = VehicleCommand()
    msg.command = command
    msg.param1 = float(param1)
    msg.param2 = float(param2)
    msg.target_system = target_system
    msg.target_component = 1
    msg.source_system = 1
    msg.source_component = 1
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
