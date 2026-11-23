"""Show one drone in RViz with its cameras and lidars (`mount-backend.sh view ID`).

Finds the drone and its sensor topics in Gazebo (`gz topic -l`), bridges the sensors to
ROS 2 with ros_gz_bridge, publishes TF from Gazebo's poses (world -> drone -> base_link and
sensor links) and opens RViz with one display per sensor. Works for every drone type: one
without sensors still shows where it is. Everything above `main` is plain data and string
handling without ROS, so it is unit tested in tests/.

TF and the sim clock go on topics of their own (/sensor_view_<id>/tf, .../clock) that only
this RViz is remapped to, because every drone has frames with the same names (base_link,
camera_link). So several views can run at once without mixing their frames.
"""
import argparse
import copy
import re
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import yaml

from swarm_backend.config import default_namespace

# PX4 names each drone's Gazebo model <model>_<instance>, e.g. x500_mono_cam_1. Sensors
# without their own <topic> publish under the model, with the link name as frame id.
SENSOR_TOPIC = re.compile(
    r'^/world/[^/]+/model/(?P<model>[^/]+)_(?P<id>\d+)/link/(?P<link>[^/]+)'
    r'/sensor/(?P<sensor>[^/]+)/(?P<kind>image|scan/points)$'
)
MODEL_TOPIC = re.compile(r'^/world/(?P<world>[^/]+)/model/(?P<model>[^/]+)_(?P<id>\d+)/')

# The OakD-Lite depth camera (gz_x500_depth) sets <topic>depth_camera</topic>, so it
# publishes on one global topic that every x500_depth drone shares.
DEPTH_MODEL = 'x500_depth'
DEPTH_TOPIC = '/depth_camera'
DEPTH_FRAME = 'camera_link'

WORLD_FRAME = 'world'
BODY_FRAME = 'base_link'
TF_RATE_HZ = 30.0

BRIDGE_TYPES = {
    'image': ('sensor_msgs/msg/Image', 'gz.msgs.Image'),
    'points': ('sensor_msgs/msg/PointCloud2', 'gz.msgs.PointCloudPacked'),
}

QOS = {
    'Depth': 5,
    'Durability Policy': 'Volatile',
    'History Policy': 'Keep Last',
    'Reliability Policy': 'Best Effort',
}


@dataclass(frozen=True)
class Sensor:
    name: str  # Gazebo sensor name, e.g. camera, lidar_3d
    kind: str  # 'image' or 'points'
    gz_topic: str
    ros_topic: str
    frame: str  # frame_id of the messages (the sensor's link)


@dataclass(frozen=True)
class Drone:
    world: str  # Gazebo world name
    model: str  # model name without the instance, e.g. x500_gimbal
    id: int

    @property
    def name(self) -> str:
        """Gazebo model name, also the drone's TF frame, e.g. x500_gimbal_2."""
        return f'{self.model}_{self.id}'


# (name, entity id, (x, y, z), (qx, qy, qz, qw)): one entry of Gazebo's pose list.
Pose = tuple[str, int, tuple[float, float, float], tuple[float, float, float, float]]
# (parent frame, child frame, translation, rotation)
Transform = tuple[str, str, tuple[float, float, float], tuple[float, float, float, float]]


def find_drone(gz_topics: list[str], drone_id: int) -> Drone | None:
    """PX4 instance drone_id in Gazebo, if it is there."""
    for topic in gz_topics:
        match = MODEL_TOPIC.match(topic)
        if match and int(match['id']) == drone_id:
            return Drone(match['world'], match['model'], drone_id)
    return None


def find_sensors(gz_topics: list[str], drone_id: int) -> list[Sensor]:
    """Cameras (images) and lidars (point clouds) of PX4 instance drone_id."""
    ns = default_namespace(drone_id)
    prefix = f'/{ns}' if ns else ''
    sensors = []
    for topic in sorted(gz_topics):
        match = SENSOR_TOPIC.match(topic)
        if not match or int(match['id']) != drone_id:
            continue
        kind = 'image' if match['kind'] == 'image' else 'points'
        sensors.append(Sensor(
            name=match['sensor'],
            kind=kind,
            gz_topic=topic,
            ros_topic=f"{prefix}/{match['sensor']}/{kind}",
            frame=match['link'],
        ))
    drone = find_drone(gz_topics, drone_id)
    if drone is not None and drone.model == DEPTH_MODEL:
        for gz_topic, kind in ((DEPTH_TOPIC, 'image'), (f'{DEPTH_TOPIC}/points', 'points')):
            if gz_topic in gz_topics:
                sensors.append(Sensor(
                    name='depth_camera',
                    kind=kind,
                    gz_topic=gz_topic,
                    ros_topic=f'{prefix}/depth_camera/{kind}',
                    frame=DEPTH_FRAME,
                ))
    return sensors


def drone_transforms(poses: list[Pose], drone_name: str, frames: list[str]) -> list[Transform]:
    """world -> drone and drone -> each of `frames` (link names), from Gazebo's pose list.

    Gazebo lists models with world poses and links with poses relative to their model, by
    name only, and every drone has a base_link. Entity ids are handed out in creation order
    and a model's links are created right after it, so a drone's link is the first entity
    with that name after the drone's own id.
    """
    drone = next((p for p in poses if p[0] == drone_name), None)
    if drone is None:
        return []
    transforms = [(WORLD_FRAME, drone_name, drone[2], drone[3])]
    for frame in frames:
        links = [p for p in poses if p[0] == frame and p[1] > drone[1]]
        if links:
            link = min(links, key=lambda p: p[1])
            transforms.append((drone_name, frame, link[2], link[3]))
    return transforms


def bridge_config(sensors: list[Sensor], clock_topic: str | None = None) -> list[dict]:
    """ros_gz_bridge parameter_bridge config_file contents, Gazebo -> ROS only."""
    config = []
    for sensor in sensors:
        ros_type, gz_type = BRIDGE_TYPES[sensor.kind]
        config.append({
            'ros_topic_name': sensor.ros_topic,
            'gz_topic_name': sensor.gz_topic,
            'ros_type_name': ros_type,
            'gz_type_name': gz_type,
            'direction': 'GZ_TO_ROS',
        })
    if clock_topic is not None:
        config.append({
            'ros_topic_name': clock_topic,
            'gz_topic_name': '/clock',
            'ros_type_name': 'rosgraph_msgs/msg/Clock',
            'gz_type_name': 'gz.msgs.Clock',
            'direction': 'GZ_TO_ROS',
        })
    return config


def detection_debug_topic(drone_id: int) -> str:
    """Where the camera detector (detect/live.py, YOLO or the baseline) publishes its image
    with the boxes drawn on: /<namespace>/detection/debug (live.offset_topic)."""
    ns = default_namespace(drone_id)
    return f'/{ns}/detection/debug' if ns else '/detection/debug'


def rviz_config(template: dict, sensors: list[Sensor], detection_topic: str | None = None) -> dict:
    """The template layout with its PointCloud2 display repeated once per point cloud and
    an Image display per camera. With `detection_topic` and a camera, first an Image
    display of the detector's picture with its boxes: it shows while a detector runs on
    the drone (the scenarios start one per defender), and waits for one otherwise. The
    camera displays are then there but off (tick them in RViz for the raw picture)."""
    config = copy.deepcopy(template)
    manager = config['Visualization Manager']
    displays = manager['Displays']
    cloud_template = next(d for d in displays if d['Class'] == 'rviz_default_plugins/PointCloud2')
    displays = [d for d in displays if d['Class'] != 'rviz_default_plugins/PointCloud2']

    detection = bool(detection_topic) and any(s.kind == 'image' for s in sensors)
    if detection:
        displays.append({
            'Class': 'rviz_default_plugins/Image',
            'Name': 'detection (boxes)',
            'Enabled': True,
            'Normalize Range': False,
            'Topic': {'Value': detection_topic, **QOS},
        })
    for sensor in sensors:
        if sensor.kind == 'points':
            display = copy.deepcopy(cloud_template)
            display['Name'] = f'{sensor.name} points'
            display['Topic'] = {'Value': sensor.ros_topic, **QOS}
        else:
            display = {
                'Class': 'rviz_default_plugins/Image',
                'Name': f'{sensor.name} image',
                'Enabled': not detection,  # the boxes show the same picture
                'Normalize Range': True,  # needed for float depth images
                'Topic': {'Value': sensor.ros_topic, **QOS},
            }
        displays.append(display)
    manager['Displays'] = displays
    return config


def gz_topics() -> list[str]:
    return subprocess.run(
        ['gz', 'topic', '-l'], capture_output=True, text=True, check=True).stdout.split()


def start_bridge(config: list[dict], name: str) -> subprocess.Popen:
    """ros_gz_bridge running `config` (bridge_config), logging to /tmp/<name>.log."""
    config_file = Path(f'/tmp/{name}.yaml')
    config_file.write_text(yaml.safe_dump(config, sort_keys=False))
    with open(f'/tmp/{name}.log', 'w') as log:
        return subprocess.Popen(
            ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
             '--ros-args', '-p', f'config_file:={config_file}'],
            stdout=log, stderr=subprocess.STDOUT)


class TFPublisher:
    """Republishes the drone's transforms from Gazebo's pose stream as ROS TF."""

    def __init__(self, drone: Drone, frames: list[str], tf_topic: str):
        import rclpy
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node as GzNode
        from tf2_msgs.msg import TFMessage

        rclpy.init()
        self._node = rclpy.create_node(f'sensor_view_{drone.id}')
        self._pub = self._node.create_publisher(TFMessage, tf_topic, 10)
        self._drone = drone
        self._frames = frames
        self._last = 0.0
        self._lock = threading.Lock()
        self._gz = GzNode()
        topic = f'/world/{drone.world}/dynamic_pose/info'
        if not self._gz.subscribe(Pose_V, topic, self._on_poses):
            raise RuntimeError(f'Could not subscribe to {topic}')

    def _on_poses(self, msg) -> None:
        from geometry_msgs.msg import TransformStamped
        from tf2_msgs.msg import TFMessage

        now = time.monotonic()
        with self._lock:
            if now - self._last < 1.0 / TF_RATE_HZ:
                return
            self._last = now
        poses = [(p.name, p.id, (p.position.x, p.position.y, p.position.z),
                  (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))
                 for p in msg.pose]
        out = TFMessage()
        for parent, child, (x, y, z), (qx, qy, qz, qw) in drone_transforms(
                poses, self._drone.name, self._frames):
            t = TransformStamped()
            t.header.stamp.sec = msg.header.stamp.sec  # sim time, like the sensor messages
            t.header.stamp.nanosec = msg.header.stamp.nsec
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = x, y, z
            t.transform.rotation.x, t.transform.rotation.y = qx, qy
            t.transform.rotation.z, t.transform.rotation.w = qz, qw
            out.transforms.append(t)
        if out.transforms:
            self._pub.publish(out)

    def close(self) -> None:
        import rclpy
        self._node.destroy_node()
        rclpy.shutdown()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--id', type=int, default=1, help='PX4 instance number of the drone')
    parser.add_argument('--rviz', type=Path, required=True, help='RViz layout template')
    args = parser.parse_args(argv)

    topics = gz_topics()
    drone = find_drone(topics, args.id)
    if drone is None:
        print(f'Drone {args.id} is not in Gazebo. Start one with: ./mount-backend.sh spawn camera '
              '(or lidar, depth, plain, ...), or give it a model: in swarm.yaml and run spawn.')
        return 1
    sensors = find_sensors(topics, args.id)
    frames = list(dict.fromkeys([BODY_FRAME] + [s.frame for s in sensors]))

    ns = f'/sensor_view_{args.id}'
    rviz_file = Path(f'/tmp/sensors_{args.id}.rviz')
    template = yaml.safe_load(args.rviz.read_text())
    detection = detection_debug_topic(args.id)
    rviz_file.write_text(yaml.safe_dump(rviz_config(template, sensors, detection), sort_keys=False))

    print(f'Drone {args.id} ({drone.name}):')
    for sensor in sensors:
        print(f'  {sensor.name:14} {sensor.gz_topic}\n  {"":14} -> {sensor.ros_topic}')
    if not sensors:
        print('  no camera or lidar; RViz shows where the drone is')
    if any(s.kind == 'image' for s in sensors):
        print(f'  detection      {detection} (the detector\'s boxes, while one runs on this drone)')
    if drone.model == DEPTH_MODEL:
        print(f'  Note: every {DEPTH_MODEL} drone publishes its depth image on the same '
              f'{DEPTH_TOPIC} topic.')
    print(f'  TF: {WORLD_FRAME} -> {drone.name} -> {", ".join(frames)} on {ns}/tf')
    print('RViz is opening on the GUI display. Close it or press Ctrl-C here to stop.')

    tf = TFPublisher(drone, frames, f'{ns}/tf')
    bridge = start_bridge(bridge_config(sensors, clock_topic=f'{ns}/clock'),
                          f'sensors_bridge_{args.id}')
    with open(f'/tmp/rviz_{args.id}.log', 'w') as rviz_log:
        try:
            subprocess.run(
                ['rviz2', '-d', str(rviz_file), '--ros-args', '-p', 'use_sim_time:=true',
                 '-r', f'/tf:={ns}/tf', '-r', f'/tf_static:={ns}/tf_static',
                 '-r', f'/clock:={ns}/clock'],
                stdout=rviz_log, stderr=subprocess.STDOUT)
        except KeyboardInterrupt:
            pass
        finally:
            bridge.terminate()
            bridge.wait()
            tf.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
