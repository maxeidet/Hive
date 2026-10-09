"""Run the baseline detector on a drone's camera (`mount-backend.sh detect ID`).

Bridges the drone's Gazebo camera to ROS 2 (as `view` does), runs find_candidates and the
k-in-a-row confirmation on the newest image at --rate Hz, and publishes the image with the
boxes drawn on it on /<ns>/detection/debug (watch it with rqt_image_view, or add an Image
display in RViz). Yellow = candidate (number = images in a row), red = confirmed track.
--save DIR also writes the raw and annotated images there every --save-every seconds, for
tuning offline with `python -m swarm_backend.detect.annotate`.

No camera math yet, so tracks are matched in pixels (fine while the gimbal is still) and
the sky mask is a fixed row (--horizon), e.g. 360 for a level gimbal at 1280x720.
LiveDetector is also used by tests/detection_test.py.
"""
import argparse
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import cv2
import numpy as np

from swarm_backend.config import default_namespace
from swarm_backend.detect.candidates import (
    Box, CandidateConfig, above_line, draw, find_candidates)
from swarm_backend.detect.tracking import Confirmer, Track
from swarm_backend.sensor_view import (
    bridge_config, find_drone, find_sensors, gz_topics, start_bridge)

CANDIDATE = (255, 220, 0)
CONFIRMED = (255, 0, 0)
HORIZON = (0, 160, 255)


@dataclass
class Result:
    """What the detector made of one image."""
    stamp: float  # image time, seconds (Gazebo sim time)
    width: int
    height: int
    boxes: list[Box]
    tracks: list[Track]  # live tracks after this image
    horizon: float | None
    ms: float  # processing time


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
    return out


class LiveDetector:
    """Baseline detection on one drone's camera, inside an existing ROS node.

    horizon: callable giving the image row of the horizon for the next image (or None to
    search the whole image). on_result is called with every Result.
    """

    def __init__(self, node, drone_id: int, config: CandidateConfig = CandidateConfig(),
                 confirmer: Confirmer | None = None, rate_hz: float = 10.0,
                 horizon: Callable[[], float | None] = lambda: None,
                 on_result: Callable[[Result], None] | None = None,
                 save: Path | None = None, save_every: float = 1.0):
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
        self.debug_topic = f'/{ns}/detection/debug' if ns else '/detection/debug'
        self.config = config
        self.confirmer = confirmer or Confirmer()
        self.horizon = horizon
        self.on_result = on_result
        self.save = save
        self.save_every = save_every
        if save:
            save.mkdir(parents=True, exist_ok=True)

        self._Image = Image
        self._bridge = start_bridge(bridge_config([self.camera]), f'detect_bridge_{drone_id}')
        self._debug_pub = node.create_publisher(Image, self.debug_topic, qos_profile_sensor_data)
        self._latest = None
        self._lock = threading.Lock()
        self._mask_key = None
        self._mask = None
        self._last_save = -1e9
        node.create_subscription(Image, self.camera.ros_topic, self._on_image,
                                 qos_profile_sensor_data)
        node.create_timer(1.0 / rate_hz, self._process)

    def close(self) -> None:
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
        boxes = find_candidates(img, self._sky_mask(img.shape, horizon), self.config)
        tracks = self.confirmer.update([(b.u, b.v) for b in boxes], boxes)
        ms = (time.perf_counter() - start) * 1000
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        result = Result(stamp, msg.width, msg.height, boxes, tracks, horizon, ms)

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
    return f'T{track.id} at ({d.u:.0f}, {d.v:.0f}) {d.width}x{d.height}px contrast {d.contrast:.0f}'


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--id', type=int, default=2, help='PX4 instance number of the drone')
    parser.add_argument('--rate', type=float, default=10.0, help='images processed per second')
    parser.add_argument('--horizon', type=float, default=None,
                        help='image row of the horizon; only the sky above it is searched')
    parser.add_argument('--k', type=int, default=3, help='images in a row to confirm a track')
    parser.add_argument('--gate', type=float, default=20.0, help='px between images, same track')
    parser.add_argument('--kernel', type=int, default=CandidateConfig.kernel)
    parser.add_argument('--k-sigma', type=float, default=CandidateConfig.k_sigma)
    parser.add_argument('--min-contrast', type=float, default=CandidateConfig.min_contrast)
    parser.add_argument('--save', type=Path, default=None, help='folder for raw + annotated PNGs')
    parser.add_argument('--save-every', type=float, default=1.0,
                        help='seconds (sim time) between saves')
    args = parser.parse_args(argv)

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
        print(f'[{result.stamp:.2f}] {len(result.boxes)} candidates, confirmed: {confirmed}  '
              f'({stats["ms"] / stats["frames"]:.0f} ms/image)', flush=True)
        stats.update(last_log=now, frames=0, ms=0.0)

    config = CandidateConfig(kernel=args.kernel, k_sigma=args.k_sigma,
                             min_contrast=args.min_contrast)
    try:
        detector = LiveDetector(node, args.id, config, Confirmer(k=args.k, gate=args.gate),
                                args.rate, lambda: args.horizon, log, args.save, args.save_every)
    except RuntimeError as e:
        print(e)
        return 1
    print(f'Drone {args.id} ({detector.drone.name}): {detector.camera.gz_topic}\n'
          f'  -> {detector.camera.ros_topic}\n  debug image on {detector.debug_topic}')
    if args.save:
        print(f'  saving images to {args.save}')
    print('Running. Ctrl-C to stop.')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        detector.close()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
