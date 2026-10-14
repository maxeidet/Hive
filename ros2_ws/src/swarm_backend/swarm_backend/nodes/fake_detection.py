"""Fake camera detector (`mount-backend.sh fakedetect`): the two detection topics from
where the drones really are, so the attack can be built before the real detector.

Reads Gazebo's poses (as `snap` does), works out what the attacker's gimbal camera sees of
the target drone (intercept/detection.py) and publishes target_position and target_offset
exactly like the real detector will. Settings come from the `attack:` section of
swarm.yaml; flags override them. All errors are off by default (a perfect detector);
--realistic turns on the real detector's delay, noise and blind zone.

    python3 -m swarm_backend.nodes.fake_detection --config <swarm.yaml> [--realistic]

It cheats on purpose (true positions, true camera orientation), and says so in the log.
"""
import argparse
import dataclasses
import subprocess
import sys
import threading

from swarm_backend import ground_truth
from swarm_backend.config import default_config_path, load_config
from swarm_backend.intercept.detection import (
    FRAME_ID, OFFSET_TOPIC, POSITION_TOPIC, REALISTIC, FakeDetector, topic)
from swarm_backend.save_images import drone_names
from swarm_backend.sensor_view import find_sensors

LOG_PERIOD_S = 2.0  # wall clock


class FakeDetectionNode:

    def __init__(self, node, config, world: str, names: dict[int, str], camera_link: str):
        from geometry_msgs.msg import PointStamped, Vector3Stamped
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node as GzNode

        self.config = config
        self.detector = FakeDetector(config)
        self.camera_name = names[config.attacker_id]
        self.target_name = names[config.target_id]
        self.camera_link = camera_link
        self.log = node.get_logger()
        self._Point, self._Vector = PointStamped, Vector3Stamped
        self.position_topic = topic(config.attacker_id, POSITION_TOPIC)
        self.offset_topic = topic(config.attacker_id, OFFSET_TOPIC)
        self._position_pub = node.create_publisher(PointStamped, self.position_topic, 10)
        self._offset_pub = node.create_publisher(Vector3Stamped, self.offset_topic, 10)
        self._lock = threading.Lock()
        self.last_seen: tuple[float, float] | None = None  # (stamp, distance)
        self._gz = GzNode()
        pose_topic = f'/world/{world}/dynamic_pose/info'
        if not self._gz.subscribe(Pose_V, pose_topic, self._on_poses):
            raise RuntimeError(f'Could not subscribe to {pose_topic}')

    def _on_poses(self, msg) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        with self._lock:
            if self.detector.wants_image(stamp):
                poses = [(p.name, p.id, (p.position.x, p.position.y, p.position.z),
                          (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))
                         for p in msg.pose]
                camera = ground_truth.camera_pose(poses, self.camera_name, self.camera_link)
                target = ground_truth.body_position(poses, self.target_name)
                self.detector.observe(stamp, camera, target)
            for image_stamp, found in self.detector.due(stamp):
                self._publish(image_stamp, found)

    def _publish(self, stamp: float, found) -> None:
        position, offset = self._Point(), self._Vector()
        for msg in (position, offset):
            msg.header.stamp.sec = int(stamp)
            msg.header.stamp.nanosec = int(round((stamp - int(stamp)) * 1e9)) % 1_000_000_000
            msg.header.frame_id = FRAME_ID
        position.point.x, position.point.y, position.point.z = found.position
        offset.vector.x, offset.vector.y = found.offset
        offset.vector.z = 0.0
        self._position_pub.publish(position)
        self._offset_pub.publish(offset)
        self.last_seen = (stamp, sum(c * c for c in found.position) ** 0.5)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--config', default=None, help='swarm.yaml (default: the installed one)')
    parser.add_argument('--id', type=int, default=None, help='attacker (camera) drone')
    parser.add_argument('--target', type=int, default=None, help='hostile (target) drone')
    parser.add_argument('--realistic', action='store_true',
                        help='0.25 s delay, 1 deg and 20 %% noise, blind below 5 m')
    parser.add_argument('--rate', type=float, default=None, help='images per second')
    parser.add_argument('--delay', type=float, default=None, help='seconds')
    parser.add_argument('--angle-noise', type=float, default=None, help='degrees, 1 sigma')
    parser.add_argument('--range-noise', type=float, default=None, help='share, 1 sigma')
    parser.add_argument('--blind', type=float, default=None, help='metres')
    parser.add_argument('--max-range', type=float, default=None, help='metres')
    parser.add_argument('--seed', type=int, default=None)
    args = parser.parse_args(argv)

    config = load_config(args.config or default_config_path()).attack
    overrides = dict(REALISTIC) if args.realistic else {}
    for flag, field in (('id', 'attacker_id'), ('target', 'target_id'), ('rate', 'fake_rate_hz'),
                        ('delay', 'fake_delay'), ('angle_noise', 'fake_angle_noise_deg'),
                        ('range_noise', 'fake_range_noise'), ('blind', 'fake_blind_range'),
                        ('max_range', 'fake_max_range'), ('seed', 'seed')):
        if getattr(args, flag) is not None:
            overrides[field] = getattr(args, flag)
    config = dataclasses.replace(config, **overrides)

    gz_topics = subprocess.run(
        ['gz', 'topic', '-l'], capture_output=True, text=True, check=True).stdout.split()
    world, names = drone_names(gz_topics)
    missing = [i for i in (config.attacker_id, config.target_id) if i not in names]
    if missing:
        print(f'Drone(s) {missing} are not in Gazebo. Start them with ./mount-backend.sh spawn.')
        return 1
    cameras = [s for s in find_sensors(gz_topics, config.attacker_id)
               if s.kind == 'image' and s.name == 'camera']
    if not cameras:
        print(f'Drone {config.attacker_id} has no camera. The attacker must be a gimbal drone '
              '(model gz_x500_gimbal).')
        return 1

    import rclpy

    rclpy.init()
    node = rclpy.create_node(f'fake_detection_{config.attacker_id}')
    fake = FakeDetectionNode(node, config, world, names, cameras[0].frame)
    d = fake.detector
    print(f'FAKE detector (true positions, not the camera): drone {config.attacker_id} '
          f'({fake.camera_name}) looking for drone {config.target_id} ({fake.target_name})\n'
          f'  {fake.position_topic}\n  {fake.offset_topic}\n'
          f'  {1 / d.period:.1f} images/s, confirmed after {d.confirm_frames} in a row, '
          f'range {d.blind_range:.1f}-{d.max_range:.1f} m, delay {d.delay:.2f} s, '
          f'noise {config.fake_angle_noise_deg:.1f} deg / {100 * d.range_noise:.0f} %',
          flush=True)

    def log_status() -> None:
        seen = fake.last_seen
        where = 'nothing' if seen is None else f'last at t={seen[0]:.1f} s, {seen[1]:.1f} m'
        print(f'{d.images} images, {d.published} published, in view {d.streak} in a row; '
              f'{where}', flush=True)

    node.create_timer(LOG_PERIOD_S, log_status)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
