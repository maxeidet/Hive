"""Run a detector on a drone's camera and publish the target (`mount-backend.sh detect ID`).

Bridges the drone's Gazebo camera to ROS 2 (as `view` does), runs the detector chosen with
--detector (baseline or yolo, see detectors.py) and the k-in-a-row confirmation on the
newest image at --rate Hz. For the best confirmed target (highest score) it publishes, as
agreed with the attack node (vault: detection/Baseline attack work plan):

    /<ns>/detection/target_offset   geometry_msgs/Vector3Stamped: x = horizontal angle from
                                    the image centre (rad, right +), y = vertical (rad, up +)
    /<ns>/detection/target_position geometry_msgs/PointStamped: where the target is relative
                                    to the drone (its camera), world NED, metres: direction
                                    from the pixel and PX4's camera orientation, times the
                                    distance from the box size (a 0.5 m drone)

stamped with the image's time, frame_id = the drone's Gazebo name. Nothing is published
while there is no confirmed target. The angles are corrected for the camera's roll, which
comes from PX4's gimbal status at the image's time (px4_camera.py); until PX4 has reported,
they are sent uncorrected and target_position is not sent at all; the log says so.

It also publishes the image with the boxes drawn on it on /<ns>/detection/debug (watch it
with rqt_image_view, or add an Image display in RViz). Yellow = candidate (number = images
in a row), red = confirmed track, white cross = image centre, with the offset next to the
target. --save DIR also writes the raw and annotated images there every --save-every
seconds, for running through `mount-backend.sh predict --images DIR` afterwards.

Tracks are matched in pixels (fine while the gimbal is still). --horizon ROW can cut off
everything below a fixed image row, e.g. 360 for a level gimbal at 1280x720. LiveDetector
is also used by tests/detection_test.py.
"""
import argparse
import math
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import cv2
import numpy as np

from swarm_backend.config import default_namespace
from swarm_backend.detect import detectors
from swarm_backend.detect.camera import GIMBAL_HFOV, CameraModel, camera_roll
from swarm_backend.detect.orientation import Axes
from swarm_backend.detect.candidates import above_line
from swarm_backend.detect.detectors import Detection, Detector, draw
from swarm_backend.detect.tracking import Confirmer, Track
from swarm_backend.sensor_view import (
    bridge_config, find_drone, find_sensors, gz_topics, start_bridge)

CANDIDATE = (255, 220, 0)
CONFIRMED = (255, 0, 0)
HORIZON = (0, 160, 255)
CENTRE = (255, 255, 255)


@dataclass
class Result:
    """What the detector made of one image."""
    stamp: float  # image time, seconds (Gazebo sim time)
    width: int
    height: int
    boxes: list[Detection]
    tracks: list[Track]  # live tracks after this image
    horizon: float | None
    ms: float  # processing time
    target: Track | None = None  # the confirmed track published for this image
    offset: tuple[float, float] | None = None  # its target_offset (x, y), radians
    axes: Axes | None = None  # camera forward, right, down (world NED) from PX4, if known
    roll: float | None = None  # camera roll used for the offset, radians (None = uncorrected)
    position: tuple[float, float, float] | None = None  # target_position, world NED, metres


def best_target(tracks: list[Track]) -> Track | None:
    """The track to publish: confirmed, detected in this image, highest score."""
    seen = [t for t in tracks if t.confirmed and t.detection is not None]
    return max(seen, key=lambda t: t.detection.score, default=None)


def camera_already_bridged(node, topic: str, wait_s: float = 2.0) -> bool:
    """True if something (e.g. `mount-backend.sh view`) already publishes this ROS topic.
    Waits a moment, since new nodes are discovered with a delay."""
    deadline = time.monotonic() + wait_s
    while time.monotonic() < deadline:
        if node.count_publishers(topic) > 0:
            return True
        time.sleep(0.1)
    return False


def offset_topic(drone_id: int, name: str) -> str:
    """/<namespace>/detection/<name>; PX4's drone 0 has no namespace."""
    ns = default_namespace(drone_id)
    return f'/{ns}/detection/{name}' if ns else f'/detection/{name}'


def image_to_array(msg) -> np.ndarray:
    """sensor_msgs/Image (rgb8, bgr8 or mono8) as an RGB or grey array, without cv_bridge."""
    channels = {'rgb8': 3, 'bgr8': 3, 'mono8': 1}.get(msg.encoding)
    if channels is None:
        raise ValueError(f'Unsupported image encoding {msg.encoding}')
    rows = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    img = rows[:, :msg.width * channels].reshape(msg.height, msg.width, channels)
    if msg.encoding == 'bgr8':
        img = img[:, :, ::-1]
    return img[:, :, 0] if channels == 1 else img


def annotate(img: np.ndarray, result: Result) -> np.ndarray:
    seen = [t for t in result.tracks if t.detection is not None]
    tracked = {id(t.detection) for t in seen}
    out = draw(img, [b for b in result.boxes if id(b) not in tracked], CANDIDATE)
    confirmed = [t for t in seen if t.confirmed]
    out = draw(out, [t.detection for t in confirmed], CONFIRMED,
               labels=[f'T{t.id} {t.detection.width}px' for t in confirmed])
    unconfirmed = [t for t in seen if not t.confirmed]
    out = draw(out, [t.detection for t in unconfirmed], CANDIDATE,
               labels=[str(t.streak) for t in unconfirmed])
    if result.horizon is not None and 0 <= result.horizon < out.shape[0]:
        row = int(result.horizon)
        cv2.line(out, (0, row), (out.shape[1] - 1, row), HORIZON, 1)
    centre = (out.shape[1] // 2, out.shape[0] // 2)
    cv2.drawMarker(out, centre, CENTRE, cv2.MARKER_CROSS, 20, 1)
    if result.target is not None and result.offset is not None:
        d = result.target.detection
        text = (f'x {np.degrees(result.offset[0]):+.1f} deg  '
                f'y {np.degrees(result.offset[1]):+.1f} deg')
        cv2.putText(out, text, (d.x + d.width + 10, d.y + d.height // 2 + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, CONFIRMED, 1, cv2.LINE_AA)
        if result.position is not None:
            n, e, down = result.position
            text = f'{math.hypot(n, e, down):.1f} m: N {n:+.1f} E {e:+.1f} D {down:+.1f}'
            cv2.putText(out, text, (d.x + d.width + 10, d.y + d.height // 2 + 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, CONFIRMED, 1, cv2.LINE_AA)
    return out


class LiveDetector:
    """Detection on one drone's camera, inside an existing ROS node.

    detector: any detector from detectors.py. horizon: callable giving the image row of the
    horizon for the next image (or None to search the whole image). use_px4: read the
    camera's orientation from PX4 (roll for the offset); off, the offset is not
    roll-corrected. on_result is called with every Result.
    """

    def __init__(self, node, drone_id: int, detector: Detector,
                 confirmer: Confirmer | None = None, rate_hz: float = 10.0,
                 horizon: Callable[[], float | None] = lambda: None,
                 on_result: Callable[[Result], None] | None = None,
                 save: Path | None = None, save_every: float = 1.0,
                 use_px4: bool = True, hfov: float = GIMBAL_HFOV):
        from geometry_msgs.msg import PointStamped, Vector3Stamped
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image

        topics = gz_topics()
        drone = find_drone(topics, drone_id)
        cameras = [s for s in find_sensors(topics, drone_id) if s.kind == 'image']
        if drone is None or not cameras:
            raise RuntimeError(f'Drone {drone_id} is not in Gazebo or has no camera. Spawn one '
                               'with ./mount-backend.sh spawn gimbal (or camera).')
        self.camera = next((s for s in cameras if s.name == 'camera'), cameras[0])
        self.drone = drone
        ns = default_namespace(drone_id)
        self.debug_topic = offset_topic(drone_id, 'debug')
        self.offset_topic = offset_topic(drone_id, 'target_offset')
        self.position_topic = offset_topic(drone_id, 'target_position')
        self.detector = detector
        self.px4 = None
        if use_px4:
            from swarm_backend.detect.px4_camera import PX4Camera
            self.px4 = PX4Camera(node, drone_id)
        self.hfov = hfov
        self.confirmer = confirmer or Confirmer()
        self.horizon = horizon
        self.on_result = on_result
        self.save = save
        self.save_every = save_every
        if save:
            save.mkdir(parents=True, exist_ok=True)

        self._Image = Image
        self._bridge = None
        if not camera_already_bridged(node, self.camera.ros_topic):
            # Not when `view` already bridges this camera: a second copy of every image
            # would only slow the simulation down.
            self._bridge = start_bridge(bridge_config([self.camera]), f'detect_bridge_{drone_id}')
        # Reliable, so RViz's Image display (reliable by default) gets it too; a reliable
        # publisher also serves best-effort viewers. Depth 1: only the newest picture matters.
        self._debug_pub = node.create_publisher(Image, self.debug_topic, 1)
        self._Vector3Stamped = Vector3Stamped
        self._offset_pub = node.create_publisher(Vector3Stamped, self.offset_topic, 10)
        self._PointStamped = PointStamped
        self._position_pub = node.create_publisher(PointStamped, self.position_topic, 10)
        self._latest = None
        self._lock = threading.Lock()
        self._mask_key = None
        self._mask = None
        self._last_save = -1e9
        node.create_subscription(Image, self.camera.ros_topic, self._on_image,
                                 qos_profile_sensor_data)
        node.create_timer(1.0 / rate_hz, self._process)

    def close(self) -> None:
        if self._bridge is not None:
            self._bridge.terminate()
            self._bridge.wait()

    def _on_image(self, msg) -> None:
        with self._lock:
            self._latest = msg

    def _sky_mask(self, shape, horizon: float | None):
        if horizon is None:
            return None
        key = (shape[:2], round(horizon))
        if key != self._mask_key:
            self._mask_key, self._mask = key, above_line(shape, horizon)
        return self._mask

    def _process(self) -> None:
        with self._lock:
            msg, self._latest = self._latest, None
        if msg is None:
            return
        img = image_to_array(msg)
        start = time.perf_counter()
        horizon = self.horizon()
        boxes = self.detector(img, self._sky_mask(img.shape, horizon))
        tracks = self.confirmer.update([(b.u, b.v) for b in boxes], boxes)
        target = best_target(tracks)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        axes = None if self.px4 is None else self.px4.axes(stamp)
        roll = None if axes is None else camera_roll(axes[0], axes[1])
        offset = position = None
        if target is not None:
            camera = CameraModel(self.hfov, msg.width, msg.height)
            offset = camera.offset_angles(target.detection.u, target.detection.v, roll or 0.0)
            out_msg = self._Vector3Stamped()
            out_msg.header.stamp = msg.header.stamp  # when the image was taken
            out_msg.header.frame_id = self.drone.name
            out_msg.vector.x, out_msg.vector.y = offset
            self._offset_pub.publish(out_msg)
            if axes is not None:  # the direction needs the camera's orientation
                d = target.detection
                position = camera.position(d.u, d.v, d.size, *axes)
                pos_msg = self._PointStamped()
                pos_msg.header = out_msg.header  # same image, same stamp
                pos_msg.point.x, pos_msg.point.y, pos_msg.point.z = position
                self._position_pub.publish(pos_msg)
        ms = (time.perf_counter() - start) * 1000
        result = Result(stamp, msg.width, msg.height, boxes, tracks, horizon, ms, target, offset,
                        axes, roll, position)

        out = annotate(img, result)
        debug = self._Image()
        debug.header = msg.header
        debug.height, debug.width = out.shape[:2]
        debug.encoding, debug.step = 'rgb8', out.shape[1] * 3
        debug.data = out.tobytes()
        self._debug_pub.publish(debug)

        if self.save and stamp - self._last_save >= self.save_every:
            name = f'{stamp:010.2f}'
            cv2.imwrite(str(self.save / f'{name}.png'), np.ascontiguousarray(img[..., ::-1]))
            cv2.imwrite(str(self.save / f'{name}_boxes.png'), out[..., ::-1])
            self._last_save = stamp
        if self.on_result is not None:
            self.on_result(result)


def describe(track: Track) -> str:
    d = track.detection
    return f'T{track.id} at ({d.u:.0f}, {d.v:.0f}) {d.width}x{d.height}px score {d.score:.0f}'


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--id', type=int, default=2, help='PX4 instance number of the drone')
    parser.add_argument('--rate', type=float, default=10.0, help='images processed per second')
    parser.add_argument('--horizon', type=float, default=None,
                        help='image row of the horizon; only the sky above it is searched')
    parser.add_argument('--k', type=int, default=3, help='images in a row to confirm a track')
    parser.add_argument('--gate', type=float, default=20.0, help='px between images, same track')
    parser.add_argument('--detector', choices=detectors.DETECTORS, default='baseline',
                        help='which detector to run (default %(default)s)')
    parser.add_argument('--no-px4', action='store_true',
                        help="don't read the camera orientation from PX4 (offset not roll-corrected)")
    parser.add_argument('--save', type=Path, default=None, help='folder for raw + annotated PNGs')
    parser.add_argument('--save-every', type=float, default=1.0,
                        help='seconds (sim time) between saves')
    detectors.add_options(parser)
    args = parser.parse_args(argv)
    try:
        detector = detectors.make(args.detector, args)
    except NotImplementedError as e:
        print(e)
        return 1

    import rclpy

    rclpy.init()
    node = rclpy.create_node(f'detect_{args.id}')
    stats = {'last_log': 0.0, 'frames': 0, 'ms': 0.0}

    def log(result: Result) -> None:
        stats['frames'] += 1
        stats['ms'] += result.ms
        now = time.monotonic()
        if now - stats['last_log'] < 1.0:
            return
        confirmed = ', '.join(describe(t) for t in result.tracks
                              if t.confirmed and t.detection is not None) or 'none'
        offset = '' if result.offset is None else (
            f', offset x {math.degrees(result.offset[0]):+.1f} y '
            f'{math.degrees(result.offset[1]):+.1f} deg')
        if result.position is not None:
            n, e, down = result.position
            offset += (f', position N {n:+.1f} E {e:+.1f} D {down:+.1f} m '
                       f'({math.hypot(n, e, down):.1f} m away)')
        camera = ('camera roll unknown (offset not roll-corrected, no position)'
                  if result.roll is None else f'camera roll {math.degrees(result.roll):+.0f} deg')
        print(f'[{result.stamp:.2f}] {len(result.boxes)} candidates, confirmed: {confirmed}'
              f'{offset}; {camera}  ({stats["ms"] / stats["frames"]:.0f} ms/image)', flush=True)
        stats.update(last_log=now, frames=0, ms=0.0)

    try:
        live = LiveDetector(node, args.id, detector, Confirmer(k=args.k, gate=args.gate),
                            args.rate, lambda: args.horizon, log, args.save, args.save_every,
                            use_px4=not args.no_px4)
    except RuntimeError as e:
        print(e)
        return 1
    print(f'Drone {args.id} ({live.drone.name}), {args.detector} detector: '
          f'{live.camera.gz_topic}\n  -> {live.camera.ros_topic}\n'
          f'  target_offset on {live.offset_topic}\n'
          f'  target_position on {live.position_topic}\n  debug image on {live.debug_topic}\n'
          f'  camera orientation from PX4: {"off" if args.no_px4 else "on"}')
    if live._bridge is None:
        print('  using the camera bridge another program (e.g. view) already runs: '
              'closing that stops the images here too')
    if args.save:
        print(f'  saving images to {args.save}')
    print('Running. Ctrl-C to stop.')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        live.close()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
