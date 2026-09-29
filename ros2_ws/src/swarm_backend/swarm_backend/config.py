"""Swarm configuration, loaded from config/swarm.yaml.

Plain data only: nothing here imports ROS, so it can be used (and tested) anywhere.
"""
from dataclasses import dataclass
from pathlib import Path

import yaml

Vector3 = tuple[float, float, float]


@dataclass(frozen=True)
class DroneConfig:
    id: int  # PX4 instance number
    namespace: str  # ROS namespace of this instance's /fmu topics, '' for instance 0
    system_id: int  # MAVLink system id PX4 uses; VehicleCommand.target_system must match
    spawn_pose: Vector3  # Gazebo world position, ENU (same as PX4_GZ_MODEL_POSE)

    @property
    def spawn_ned(self) -> Vector3:
        """Spawn position in the swarm world frame (NED).

        PX4's local frame has its origin where the drone started, so adding this
        converts a local position to a world position.
        """
        east, north, up = self.spawn_pose
        return (float(north), float(east), float(-up))


@dataclass(frozen=True)
class SwarmConfig:
    rate_hz: float
    takeoff_altitude: float
    position_tolerance: float
    formation_spacing: float
    drones: tuple[DroneConfig, ...]

    def drone(self, drone_id: int) -> DroneConfig:
        for drone in self.drones:
            if drone.id == drone_id:
                return drone
        raise KeyError(f'No drone with id {drone_id} in the config')


SWARM_DEFAULTS = {
    'rate_hz': 10.0,
    'takeoff_altitude': 5.0,
    'position_tolerance': 0.3,
    'formation_spacing': 4.0,
}


def default_namespace(instance: int) -> str:
    return '' if instance == 0 else f'px4_{instance}'


def default_config_path() -> Path:
    """The swarm.yaml installed with the package (needs a sourced ROS workspace)."""
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('swarm_backend')) / 'config' / 'swarm.yaml'


def load_config(path, num_drones: int | None = None) -> SwarmConfig:
    with open(path) as f:
        return parse_config(yaml.safe_load(f), num_drones)


def parse_config(data: dict, num_drones: int | None = None) -> SwarmConfig:
    """Build a SwarmConfig from parsed YAML.

    num_drones keeps only the first N drones, so a launch argument can fly a
    smaller swarm without editing the file.
    """
    swarm = {**SWARM_DEFAULTS, **(data.get('swarm') or {})}
    entries = data.get('drones') or []
    if not entries:
        raise ValueError('Config has no drones')
    if num_drones is not None:
        if not 1 <= num_drones <= len(entries):
            raise ValueError(f'num_drones={num_drones}, but the config has {len(entries)} drones')
        entries = entries[:num_drones]

    drones = tuple(_parse_drone(entry) for entry in entries)
    for field in ('id', 'namespace', 'system_id'):
        values = [getattr(d, field) for d in drones]
        if len(set(values)) != len(values):
            raise ValueError(f'Drone {field}s must be unique, got {values}')

    return SwarmConfig(
        rate_hz=float(swarm['rate_hz']),
        takeoff_altitude=float(swarm['takeoff_altitude']),
        position_tolerance=float(swarm['position_tolerance']),
        formation_spacing=float(swarm['formation_spacing']),
        drones=drones,
    )


def _parse_drone(entry: dict) -> DroneConfig:
    drone_id = int(entry['id'])
    spawn = entry.get('spawn_pose', [0.0, 0.0, 0.0])
    if len(spawn) != 3:
        raise ValueError(f'Drone {drone_id}: spawn_pose must be [x, y, z], got {spawn}')
    return DroneConfig(
        id=drone_id,
        namespace=str(entry.get('namespace', default_namespace(drone_id))).strip('/'),
        system_id=int(entry.get('system_id', drone_id + 1)),
        spawn_pose=tuple(float(v) for v in spawn),
    )
