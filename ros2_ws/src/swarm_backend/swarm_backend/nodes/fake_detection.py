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


class GazeboTruth:
    """Gazebo's pose stream, read directly (no ROS bridge): calls every listener with
    (simulation time, poses) for each message. Listeners run on Gazebo's thread."""

    def __init__(self, world: str):
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node as GzNode

        self.listeners = []
        self.stamp: float | None = None  # simulation time of the latest poses
        self._gz = GzNode()
        topic = f'/world/{world}/dynamic_pose/info'
        if not self._gz.subscribe(Pose_V, topic, self._on_poses):
            raise RuntimeError(f'Could not subscribe to {topic}')

    def _on_poses(self, msg) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        self.stamp = stamp
        poses = [(p.name, p.id, (p.position.x, p.position.y, p.position.z),
                  (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))
                 for p in msg.pose]
        for listener in list(self.listeners):
            listener(stamp, poses)


    def clock(self) -> float | None:
        """Gazebo's simulation time, seconds: steady however slowly Gazebo runs."""
        return self.stamp


class FakeDetectionNode:
    """Publishes the two detection topics for `config.attacker_id` from GazeboTruth."""

    def __init__(self, node, config, truth: GazeboTruth, names: dict[int, str],
                 camera_link: str):
        from geometry_msgs.msg import PointStamped, Vector3Stamped

        self.config = config
        self.detector = FakeDetector(config)
        self.camera_name = names[config.attacker_id]
        self.target_name = names[config.target_id]
        self.camera_link = camera_link
        self._Point, self._Vector = PointStamped, Vector3Stamped
        self.position_topic = topic(config.attacker_id, POSITION_TOPIC)
        self.offset_topic = topic(config.attacker_id, OFFSET_TOPIC)
        self._position_pub = node.create_publisher(PointStamped, self.position_topic, 10)
        self._offset_pub = node.create_publisher(Vector3Stamped, self.offset_topic, 10)
        self._lock = threading.Lock()
        self.last_seen: tuple[float, float] | None = None  # (stamp, distance)
        self.truth = truth  # keeps Gazebo's subscription alive
        truth.listeners.append(self._on_poses)

    def describe(self) -> str:
        d, c = self.detector, self.config
        return (f'FAKE detector (true positions, not the camera): drone {c.attacker_id} '
                f'({self.camera_name}) looking for drone {c.target_id} ({self.target_name})\n'
                f'  {self.position_topic}\n  {self.offset_topic}\n'
                f'  {1 / d.period:.1f} images/s, confirmed after {d.confirm_frames} in a row, '
                f'range {d.blind_range:.1f}-{d.max_range:.1f} m, delay {d.delay:.2f} s, '
                f'noise {c.fake_angle_noise_deg:.1f} deg / {100 * d.range_noise:.0f} %')

    def _on_poses(self, stamp: float, poses) -> None:
        with self._lock:
            if self.detector.wants_image(stamp):
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


def fake_config(config, realistic: bool = False, **overrides):
    """The attack config with the fake detector's errors: --realistic first, then flags."""
    changes = dict(REALISTIC) if realistic else {}
    changes.update({k: v for k, v in overrides.items() if v is not None})
    return dataclasses.replace(config, **changes)


def find_scene(config) -> tuple[str, dict[int, str], str]:
    """(world, model names by drone id, the attacker's camera link) from Gazebo, or a
    RuntimeError saying what to start."""
    gz_topics = subprocess.run(
        ['gz', 'topic', '-l'], capture_output=True, text=True, check=True).stdout.split()
    world, names = drone_names(gz_topics)
    missing = [i for i in (config.attacker_id, config.target_id) if i not in names]
    if missing:
        raise RuntimeError(f'Drone(s) {missing} are not in Gazebo. Start them with '
                           './mount-backend.sh spawn.')
    cameras = [s for s in find_sensors(gz_topics, config.attacker_id)
               if s.kind == 'image' and s.name == 'camera']
    if not cameras:
        raise RuntimeError(f'Drone {config.attacker_id} has no camera. The attacker must be a '
                           'gimbal drone (model gz_x500_gimbal).')
    return world, names, cameras[0].frame


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

    config = fake_config(
        load_config(args.config or default_config_path()).attack, args.realistic,
        attacker_id=args.id, target_id=args.target, fake_rate_hz=args.rate,
        fake_delay=args.delay, fake_angle_noise_deg=args.angle_noise,
        fake_range_noise=args.range_noise, fake_blind_range=args.blind,
        fake_max_range=args.max_range, seed=args.seed)
    try:
        world, names, camera_link = find_scene(config)
    except RuntimeError as e:
        print(e)
        return 1

    import rclpy

    rclpy.init()
    node = rclpy.create_node(f'fake_detection_{config.attacker_id}')
    fake = FakeDetectionNode(node, config, GazeboTruth(world), names, camera_link)
    d = fake.detector
    print(fake.describe(), flush=True)

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
