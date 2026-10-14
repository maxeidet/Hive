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
    # Baseline evaluation (planning report, Interception > Evaluation and Tables
    # "assumptions" / "target-assumptions"). `altitude` above is the interceptor's h.
    hit_radius: float = 1.5  # metres: a hit is the interceptor this close to the target
    goal_radius: float = 100.0  # metres: x, the mission fails once the target is this close to base
    perimeter_radius: float = 1000.0  # metres: R_p, where the interceptor starts
    interceptor_max_speed: float = 12.0  # m/s: v_i
    hostile_altitude_min: float = 20.0  # metres: h_h, drawn per run between min and max
    hostile_altitude_max: float = 120.0
    camera_fov_deg: float = 115.0  # theta_0 of the x500_gimbal camera
    # Range a 0.5 m drone is p_min = 4 px wide in a 1280 px, 115 deg image; the
    # scenario starts here, at first detection.
    detect_range: float = 51.0
    detect_bearing_std_deg: float = 0.5  # noise of the stand-in detector, 1 sigma
    detect_range_std_frac: float = 0.1  # range noise as a fraction of the range, 1 sigma
    lock_k: int = 3  # lock-on: confirmed in at least k ...
    lock_n: int = 5  # ... of every n consecutive images
    lock_distance: float = 20.0  # metres: d_lock
    lock_time: float = 3.0  # seconds: t_lock without a break
    eval_aspects: tuple = (0.0, 30.0, 60.0, 90.0)  # degrees: 0 = head-on, 90 = crossing
    eval_speeds: tuple = (4.0, 8.0, 12.0, 14.0, 17.0)  # m/s: up to and beyond v_i, incl. v_h
    eval_runs: int = 30  # seeded runs per (aspect, speed)
    eval_timeout: float = 300.0  # seconds: a run ends here if nothing else ended it


@dataclass(frozen=True)
class DetectConfig:
    """Detection evaluation (the `detect:` section of swarm.yaml; planning report,
    Drone Detection Using Sensors > Evaluation and Table "assumptions")."""
    num_defenders: int = 12  # N on the ring (the report's feasibility table: 10-12)
    perimeter_radius: float = 1000.0  # metres: R_p
    altitude: float = 50.0  # metres: h, the defenders' patrol altitude
    camera_fov_deg: float = 115.0  # theta_0 of the x500_gimbal camera
    image_width: int = 1280  # px: W_0
    image_height: int = 720  # px: H_0
    target_size: float = 0.5  # metres: s, the hostile drone
    bird_size: float = 0.5  # metres
    detect_px: float = 4.0  # p_min: the smallest detectable size
    classify_px: float = 20.0  # pixels needed to classify
    confirm_k: int = 3  # k consecutive images above tau before it counts as a drone
    confidence_threshold: float = 0.5  # tau
    match_radius_px: float = 3.0  # a detection is correct within this of the true centre
    frame_rate_hz: float = 10.0  # images per second per camera
    scan_dwell: float = 0.5  # seconds: t_g, gimbal turn and settle time per position
    hostile_speed: float = 17.0  # m/s: v_h
    hostile_altitude_min: float = 20.0  # metres: h_h, drawn per run
    hostile_altitude_max: float = 120.0
    start_margin: float = 1000.0  # metres outside the ring where the hostile drone spawns
    num_birds: int = 3
    bird_speed: float = 10.0  # m/s
    range_bins: tuple = (0.0, 25.0, 50.0, 100.0, 200.0, 400.0, 800.0)  # metres, edges
    size_bins_px: tuple = (0.0, 2.0, 4.0, 8.0, 20.0, 50.0, 100.0)  # px, edges
    eval_runs: int = 30  # seeded runs
    eval_timeout: float = 300.0  # seconds


@dataclass(frozen=True)
class SwarmConfig:
    rate_hz: float
    takeoff_altitude: float
    position_tolerance: float
    formation_spacing: float
    drones: tuple[DroneConfig, ...]
    intercept: InterceptConfig = dataclasses.field(default_factory=InterceptConfig)
    detect: DetectConfig = dataclasses.field(default_factory=DetectConfig)

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
        detect=_parse_section(DetectConfig, 'detect', data.get('detect') or {}),
    )


def _parse_section(cls, name: str, entry: dict):
    """A config dataclass from a YAML section: every key must be a field, and is
    converted to the field's type (lists to tuples of floats)."""
    types = {f.name: type(f.default) for f in dataclasses.fields(cls)}
    unknown = set(entry) - set(types)
    if unknown:
        raise ValueError(f'Unknown {name} settings: {sorted(unknown)}')
    values = {k: types[k](v) for k, v in entry.items()}
    for key, value in values.items():
        if isinstance(value, tuple):
            values[key] = tuple(float(v) for v in value)
    return cls(**values)


def _parse_intercept(entry: dict) -> InterceptConfig:
    if 'radar_position' in entry and len(entry['radar_position']) != 3:
        raise ValueError('intercept: radar_position must be [x, y, z]')
    config = _parse_section(InterceptConfig, 'intercept', entry)
    if config.interceptor_id == config.hostile_id:
        raise ValueError('intercept: interceptor_id and hostile_id must differ')
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
