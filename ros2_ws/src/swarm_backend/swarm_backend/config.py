"""Swarm configuration, loaded from config/swarm.yaml.

Plain data only: nothing here imports ROS, so it can be used (and tested) anywhere.
"""
import dataclasses
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
class InterceptConfig:
    """One drone chasing a hostile drone (the `intercept:` section of swarm.yaml)."""
    interceptor_id: int = 2
    hostile_id: int = 0
    altitude: float = 50.0  # metres; both drones fly at this fixed altitude
    standoff: float = 10.0  # metres behind the hostile drone while shadowing
    standoff_sigmas: float = 3.0  # but at least this many sigmas of position uncertainty
    capture_margin: float = 3.0  # intercept counts as done within standoff + this
    min_separation: float = 5.0  # never command a point closer than this to the estimate
    max_speed: float = 10.0  # m/s the interceptor is assumed to fly, for the lead point
    track_timeout: float = 3.0  # seconds without a fix before the target counts as lost
    use_gimbal: bool = True  # keep the gimbal camera pointed at the target
    radar_topic: str = '/base/radar/hostile'
    radar_rate_hz: float = 2.0
    radar_position: Vector3 = (0.0, 0.0, 0.0)  # world NED
    # Mock radar errors, 1 sigma.
    radar_range_std: float = 3.5  # metres
    radar_azimuth_std_deg: float = 1.0
    radar_elevation_std_deg: float = 2.0
    radar_radial_velocity_std: float = 0.5  # m/s
    hostile_pattern: str = 'circle'  # line, circle, figure8 or random
    hostile_speed: float = 4.0  # m/s
    pattern_size: float = 80.0  # metres: line length, circle diameter, figure-8 width
    seed: int = 0  # random waypoints and radar noise


@dataclass(frozen=True)
class AttackConfig:
    """The baseline attack: one drone flies into a hostile drone it sees with its camera
    (the `attack:` section of swarm.yaml). The camera detector, or the fake one, publishes
    the target on two topics (intercept/detection.py); the attack node flies on them."""
    attacker_id: int = 2  # the defender: a gimbal drone
    target_id: int = 0  # the hostile drone (the fake detector looks for this one)
    guard_altitude: float = 20.0  # metres above the attacker's start point, while searching
    search_heading_deg: float = 0.0  # where the hostile drone comes from, degrees from north
    search_pitch_deg: float = 10.0  # gimbal pitch while searching, degrees above the horizon
    max_speed: float = 12.0  # m/s; pure pursuit always flies at this speed (PX4's default limit)
    gimbal_gain: float = 0.5  # share of target_offset the gimbal turns by on each message
    coast_after: float = 0.5  # seconds without a message before ATTACK becomes COAST
    lost_after: float = 1.5  # seconds in COAST without a message before LOST
    lost_hold: float = 3.0  # seconds LOST hovers, looking where the target was, before SEARCH
    # Fake detector (nodes/fake_detection.py). All errors off by default: a perfect detector.
    fake_rate_hz: float = 4.0  # images per second, like the baseline detector on a Mac
    fake_delay: float = 0.0  # seconds from image to message (the real one: about 0.25)
    fake_angle_noise_deg: float = 0.0  # 1 sigma on each image angle
    fake_range_noise: float = 0.0  # 1 sigma, as a share of the distance (0.2 = 20 %)
    fake_blind_range: float = 0.0  # metres; closer than this nothing is seen (real: about 5)
    fake_max_range: float = 0.0  # metres; 0 = where the target is fake_min_pixels wide
    fake_min_pixels: float = 4.0  # smallest detectable target, used when fake_max_range is 0
    fake_confirm_frames: int = 3  # images in a row before the target counts as confirmed
    seed: int = 0  # fake detector noise


@dataclass(frozen=True)
class SwarmConfig:
    rate_hz: float
    takeoff_altitude: float
    position_tolerance: float
    formation_spacing: float
    drones: tuple[DroneConfig, ...]
    intercept: InterceptConfig = dataclasses.field(default_factory=InterceptConfig)
    attack: AttackConfig = dataclasses.field(default_factory=AttackConfig)

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
        intercept=_parse_intercept(data.get('intercept') or {}),
        attack=_parse_attack(data.get('attack') or {}),
    )


def _parse_intercept(entry: dict) -> InterceptConfig:
    types = {f.name: type(f.default) for f in dataclasses.fields(InterceptConfig)}
    unknown = set(entry) - set(types)
    if unknown:
        raise ValueError(f'Unknown intercept settings: {sorted(unknown)}')
    values = {k: types[k](v) for k, v in entry.items()}
    if 'radar_position' in values:
        if len(values['radar_position']) != 3:
            raise ValueError('intercept: radar_position must be [x, y, z]')
        values['radar_position'] = tuple(float(v) for v in values['radar_position'])
    config = InterceptConfig(**values)
    if config.interceptor_id == config.hostile_id:
        raise ValueError('intercept: interceptor_id and hostile_id must differ')
    return config


def _parse_attack(entry: dict) -> AttackConfig:
    types = {f.name: type(f.default) for f in dataclasses.fields(AttackConfig)}
    unknown = set(entry) - set(types)
    if unknown:
        raise ValueError(f'Unknown attack settings: {sorted(unknown)}')
    config = AttackConfig(**{k: types[k](v) for k, v in entry.items()})
    if config.attacker_id == config.target_id:
        raise ValueError('attack: attacker_id and target_id must differ')
    if config.max_speed <= 0 or config.fake_rate_hz <= 0:
        raise ValueError('attack: max_speed and fake_rate_hz must be positive')
    return config


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
