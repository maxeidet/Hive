"""Build a 3D map (OctoMap) from one drone's depth camera or lidar (`mount-backend.sh map ID`).

Bridges the drone's point cloud from Gazebo to ROS 2, publishes TF from Gazebo's true poses
(world -> drone -> base_link and the sensor link) and runs octomap_server on that cloud. The
map comes out in the Gazebo world frame (ENU: x east, y north, z up) on <ns>/octomap_full,
with the occupied voxels as markers on <ns>/occupied_cells_vis_array for RViz.

A depth camera gives no point where it sees nothing within its range (the sky), so
octomap_server would leave that space unknown. Its cloud therefore goes through FreeRays
first, which turns those pixels into far points that clear the space along their rays.

Like `view`, TF and the sim clock go on topics of their own (/mapping_<id>/tf, .../clock),
because every drone has frames with the same names. Only octomap_server and the optional RViz
are remapped to them. Plan: hive-vault/mapping/3D mapping plan.md.
"""
import argparse
import math
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from swarm_backend.sim.sensor_view import (
    BODY_FRAME, DEPTH_TOPIC, QOS, WORLD_FRAME, Sensor, TFPublisher, bridge_config, find_drone,
    find_sensors, gz_topics, rviz_config)

DEFAULT_NS = 'mapping'
DEFAULT_RESOLUTION = 0.1  # metres per voxel
DEFAULT_MAX_RANGE = 8.0  # metres; points further away are only used to clear free space
DEFAULT_WAIT = 60.0  # seconds to wait for the drone's sensor in Gazebo
DEFAULT_CLOUD_STEP = 4  # every 4th pixel of the depth image each way: 7 cm apart at 8 m


Point = tuple[float, float, float]


@dataclass(frozen=True)
class MapStats:
    """Size of a map: number of occupied voxels and the box around them (ENU, metres)."""
    occupied: int
    min: Point | None
    max: Point | None


def map_stats(points: list[Point]) -> MapStats:
    """Count and bounding box of the occupied voxel centres (octomap_server's markers)."""
    if not points:
        return MapStats(0, None, None)
    xs, ys, zs = zip(*points)
    return MapStats(len(points), (min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs)))


def ned_to_enu(p: Point) -> Point:
    """Swarm world NED (x north, y east, z down) -> Gazebo world ENU (x east, y north, z up).
    Both frames have their origin at the Gazebo world origin (see swarm.yaml)."""
    north, east, down = p
    return (east, north, -down)


def reach(points: list[Point], origin: Point, bearing_deg: float) -> float | None:
    """How far the map reaches from `origin` along a horizontal bearing (degrees from north),
    in metres: the largest projection of an occupied voxel on that direction. ENU points."""
    if not points:
        return None
    east, north = math.sin(math.radians(bearing_deg)), math.cos(math.radians(bearing_deg))
    return max((x - origin[0]) * east + (y - origin[1]) * north for x, y, _ in points)


def point_clouds(sensors: list[Sensor]) -> list[Sensor]:
    """The sensors that give point clouds (lidars and the depth camera), not images."""
    return [s for s in sensors if s.kind == 'points']


def is_depth_camera(sensor: Sensor) -> bool:
    return sensor.gz_topic.startswith(DEPTH_TOPIC)


def pixel_scale(points: np.ndarray, min_returns: int = 100) -> float | None:
    """How far the rays of two neighbouring pixels spread per metre (the tangent of the angle
    between them), fitted to the returns of a depth camera's cloud: (h, w, 3) with x forward,
    y left, z up and the optical axis through the image centre, as Gazebo gives it. None when
    the points do not fit that within 2 pixels (a lidar), or there are too few returns."""
    h, w, _ = points.shape
    ok = np.isfinite(points).all(axis=-1) & (points[..., 0] > 0)
    if ok.sum() < min_returns:
        return None
    du = np.broadcast_to((w - 1) / 2.0 - np.arange(w)[None, :], (h, w))[ok]
    dv = np.broadcast_to((h - 1) / 2.0 - np.arange(h)[:, None], (h, w))[ok]
    y, z = points[..., 1][ok] / points[..., 0][ok], points[..., 2][ok] / points[..., 0][ok]
    spread = (du * du + dv * dv).sum()
    if spread == 0:
        return None
    scale = float((du * y + dv * z).sum() / spread)
    error = max(np.abs(y - scale * du).max(), np.abs(z - scale * dv).max())
    return scale if scale > 0 and error < 2 * scale else None


def with_free_rays(points: np.ndarray, scale: float, free_range: float,
                   step: int = 1) -> np.ndarray:
    """Every `step`-th point of a depth camera's cloud (see pixel_scale), as (n, 3) float32.
    A pixel that saw nothing within the camera's range (+inf) becomes a point `free_range`
    metres out along its ray. octomap_server marks a ray free up to its max_range when the
    point is further away than that, so `free_range` must be. Pixels that are too close
    (-inf) are dropped."""
    h, w, _ = points.shape
    p = points[::step, ::step].astype(np.float32)
    rays = np.empty_like(p)
    rays[..., 0] = 1.0
    rays[..., 1] = scale * ((w - 1) / 2.0 - np.arange(0, w, step))[None, :]
    rays[..., 2] = scale * ((h - 1) / 2.0 - np.arange(0, h, step))[:, None]
    rays *= free_range / np.linalg.norm(rays, axis=-1, keepdims=True)
    out = np.where(np.isposinf(p[..., :1]), rays, p)
    return out[np.isfinite(out).all(axis=-1)]


class FreeRays:
    """Republishes a depth camera's cloud on `out_topic` for octomap_server, through
    with_free_rays. Until a cloud has enough returns to fit the camera, clouds pass through
    unchanged. Needs rclpy initialised (TFPublisher does that)."""

    def __init__(self, name: str, in_topic: str, out_topic: str, free_range: float, step: int):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import PointCloud2

        self._free_range, self._step = free_range, step
        self._scale: float | None = None
        self._node = rclpy.create_node(name)
        self._pub = self._node.create_publisher(PointCloud2, out_topic, 1)
        # Depth 1 and best effort: clouds that arrive while one is being handled are dropped.
        self._node.create_subscription(
            PointCloud2, in_topic, self._on_cloud,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        threading.Thread(target=self._executor.spin, daemon=True).start()

    def _on_cloud(self, msg) -> None:
        from sensor_msgs.msg import PointCloud2, PointField

        offsets = {f.name: f.offset for f in msg.fields}
        if msg.is_bigendian or msg.row_step != msg.width * msg.point_step:
            self._pub.publish(msg)
            return
        raw = np.frombuffer(msg.data, dtype=np.dtype({
            'names': ['x', 'y', 'z'], 'formats': ['<f4'] * 3,
            'offsets': [offsets[n] for n in 'xyz'], 'itemsize': msg.point_step}))
        points = np.stack([raw['x'], raw['y'], raw['z']], axis=-1).reshape(
            msg.height, msg.width, 3)
        if self._scale is None:
            self._scale = pixel_scale(points)
            if self._scale is None:
                self._pub.publish(msg)
                return
        out = with_free_rays(points, self._scale, self._free_range, self._step)
        cloud = PointCloud2(header=msg.header, height=1, width=len(out), is_dense=True,
                            point_step=12, row_step=12 * len(out), data=out.tobytes())
        cloud.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                        for i, n in enumerate('xyz')]
        self._pub.publish(cloud)

    def close(self) -> None:
        self._executor.shutdown()
        self._node.destroy_node()


def wait_for_clouds(drone_id: int, timeout: float, topics=None, sleep=time.sleep,
                    clock=time.monotonic):
    """The drone and its point cloud sensors (find_drone, point_clouds), polling Gazebo's
    topic list for up to `timeout` seconds. Right after a drone spawns, Gazebo advertises its
    sensors a few seconds after the drone itself."""
    topics = topics or gz_topics
    deadline = clock() + timeout
    while True:
        names = topics()
        drone = find_drone(names, drone_id)
        clouds = point_clouds(find_sensors(names, drone_id)) if drone else []
        if clouds or clock() >= deadline:
            return drone, clouds
        sleep(1.0)


def ros_executable(package: str, executable: str) -> str:
    """Path of a ROS executable. Started directly rather than with `ros2 run`, because
    `ros2 run` runs it as a child process, and stopping `ros2 run` leaves the child running."""
    from ament_index_python.packages import get_package_prefix
    return str(Path(get_package_prefix(package)) / 'lib' / package / executable)


def octomap_args(cloud_topic: str, ns: str, tf_topic: str, clock_topic: str,
                 resolution: float = DEFAULT_RESOLUTION,
                 max_range: float = DEFAULT_MAX_RANGE) -> list[str]:
    """ROS arguments for octomap_server on `cloud_topic`, in namespace `ns`, with TF and the
    sim clock read from the private topics."""
    return [
        '--ros-args',
        '-r', f'__ns:=/{ns.strip("/")}',
        '-r', f'cloud_in:={cloud_topic}',
        '-r', f'/tf:={tf_topic}',
        '-r', f'/tf_static:={tf_topic}_static',
        '-r', f'/clock:={clock_topic}',
        '-p', 'use_sim_time:=true',
        '-p', f'frame_id:={WORLD_FRAME}',
        '-p', f'base_frame_id:={BODY_FRAME}',
        '-p', f'resolution:={resolution}',
        '-p', f'sensor_model.max_range:={max_range}',
        '-p', 'publish_free_space:=true',
    ]


def saver_args(ns: str, path: str) -> list[str]:
    """ROS arguments for octomap_saver_node to write the map of octomap_server in `ns` to
    `path` (.bt or .ot). In the same namespace, its service request finds that server."""
    return ['--ros-args', '-r', f'__ns:=/{ns.strip("/")}', '-p', f'octomap_path:={path}']


def start_bridge(config: list[dict], name: str) -> subprocess.Popen:
    """ros_gz_bridge running `config` (sensor_view.bridge_config), logging to /tmp/<name>.log.
    Like sensor_view.start_bridge, but started directly so terminate() stops it."""
    config_file = Path(f'/tmp/{name}.yaml')
    config_file.write_text(yaml.safe_dump(config, sort_keys=False))
    with open(f'/tmp/{name}.log', 'w') as log:
        return subprocess.Popen(
            [ros_executable('ros_gz_bridge', 'parameter_bridge'),
             '--ros-args', '-p', f'config_file:={config_file}'],
            stdout=log, stderr=subprocess.STDOUT)


def mapping_rviz_config(template: dict, sensors: list[Sensor], ns: str) -> dict:
    """The sensor view layout plus the map's occupied voxels."""
    config = rviz_config(template, sensors)
    config['Visualization Manager']['Displays'].append({
        'Class': 'rviz_default_plugins/MarkerArray',
        'Name': '3D map (occupied voxels)',
        'Enabled': True,
        'Topic': {'Value': f'/{ns.strip("/")}/occupied_cells_vis_array', **QOS,
                  'Reliability Policy': 'Reliable'},
    })
    return config


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--id', type=int, default=0, help='PX4 instance number of the drone')
    parser.add_argument('--ns', default=DEFAULT_NS,
                        help=f'ROS namespace of octomap_server (default {DEFAULT_NS})')
    parser.add_argument('--resolution', type=float, default=DEFAULT_RESOLUTION,
                        help=f'voxel size in metres (default {DEFAULT_RESOLUTION})')
    parser.add_argument('--max-range', type=float, default=DEFAULT_MAX_RANGE,
                        help=f'sensor range used for the map in metres (default {DEFAULT_MAX_RANGE})')
    parser.add_argument('--wait', type=float, default=DEFAULT_WAIT,
                        help='seconds to wait for the drone and its depth camera or lidar in '
                             f'Gazebo (default {DEFAULT_WAIT:g})')
    parser.add_argument('--cloud-step', type=int, default=DEFAULT_CLOUD_STEP,
                        help='use every Nth pixel of a depth camera each way '
                             f'(default {DEFAULT_CLOUD_STEP})')
    parser.add_argument('--rviz', type=Path,
                        help='RViz layout template; opens RViz with the map when given')
    args = parser.parse_args(argv)

    drone, clouds = wait_for_clouds(args.id, 0.0)
    if not clouds:
        print(f'Waiting up to {args.wait:g} s for drone {args.id} and its depth camera or lidar '
              'in Gazebo...', flush=True)
        drone, clouds = wait_for_clouds(args.id, args.wait)
    if drone is None:
        print(f'Drone {args.id} is not in Gazebo. Start one with a depth camera or lidar, '
              'e.g. ./mount-backend.sh spawn depth (or lidar).')
        return 1
    if not clouds:
        print(f'Drone {args.id} ({drone.name}) has no depth camera or lidar, so it cannot map. '
              'Use a model such as gz_x500_depth or gz_x500_lidar_3d.')
        return 1
    cloud = clouds[0]
    frames = list(dict.fromkeys([BODY_FRAME] + [s.frame for s in clouds]))
    private = f'/mapping_{args.id}'
    tf_topic, clock_topic = f'{private}/tf', f'{private}/clock'
    ns = args.ns.strip('/')
    # A depth camera's cloud goes to octomap_server through FreeRays, a lidar's directly.
    map_cloud = f'{private}/cloud' if is_depth_camera(cloud) else cloud.ros_topic

    print(f'Drone {args.id} ({drone.name}) maps with {cloud.name}:')
    print(f'  cloud: {cloud.gz_topic} -> {cloud.ros_topic}')
    if map_cloud != cloud.ros_topic:
        print(f'         -> {map_cloud} (every {args.cloud_step}. pixel; pixels without a '
              'return clear free space)')
    print(f'  TF:    {WORLD_FRAME} -> {drone.name} -> {", ".join(frames)} on {tf_topic}')
    print(f'  map:   /{ns}/octomap_full, voxels /{ns}/occupied_cells_vis_array '
          f'({args.resolution} m, range {args.max_range} m)')
    print('  Save it: ros2 run octomap_server octomap_saver_node '
          + ' '.join(saver_args(ns, '$PWD/map.bt')))
    print('Ctrl-C to stop.')

    def stop(*_):  # SIGTERM (e.g. from mapping_test) stops like Ctrl-C, so the children go too
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)

    tf = TFPublisher(drone, frames, tf_topic)
    free_rays = None
    if map_cloud != cloud.ros_topic:
        free_rays = FreeRays(f'mapping_free_rays_{args.id}', cloud.ros_topic, map_cloud,
                             args.max_range + 1.0, args.cloud_step)
    bridge = start_bridge(bridge_config(clouds[:1], clock_topic=clock_topic),
                          f'mapping_bridge_{args.id}')
    log_dir = Path('/tmp')
    procs = [bridge]
    with open(log_dir / f'mapping_octomap_{args.id}.log', 'w') as octo_log:
        procs.append(subprocess.Popen(
            [ros_executable('octomap_server', 'octomap_server_node')]
            + octomap_args(map_cloud, ns, tf_topic, clock_topic,
                           args.resolution, args.max_range),
            stdout=octo_log, stderr=subprocess.STDOUT))
        try:
            if args.rviz:
                rviz_file = log_dir / f'mapping_{args.id}.rviz'
                template = yaml.safe_load(args.rviz.read_text())
                rviz_file.write_text(yaml.safe_dump(
                    mapping_rviz_config(template, clouds[:1], ns), sort_keys=False))
                with open(log_dir / f'mapping_rviz_{args.id}.log', 'w') as rviz_log:
                    subprocess.run(
                        ['rviz2', '-d', str(rviz_file), '--ros-args', '-p', 'use_sim_time:=true',
                         '-r', f'/tf:={tf_topic}', '-r', f'/tf_static:={tf_topic}_static',
                         '-r', f'/clock:={clock_topic}'],
                        stdout=rviz_log, stderr=subprocess.STDOUT)
            else:
                procs[-1].wait()
        except KeyboardInterrupt:
            pass
        finally:
            for proc in procs:
                proc.terminate()
            for proc in procs:
                proc.wait()
            if free_rays is not None:
                free_rays.close()
            tf.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
