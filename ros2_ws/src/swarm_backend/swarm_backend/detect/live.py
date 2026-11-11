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
                                    distance from the box size (baseline: a 0.5 m drone, longer side;
                                    others: 0.8 m / sqrt(w*h), see camera.box_scale)

stamped with the image's time, frame_id = the drone's Gazebo name. Nothing is published
while there is no confirmed target. The angles are corrected for the camera's roll, which
comes from PX4's gimbal status at the image's time (px4_camera.py); until PX4 has reported,
they are sent uncorrected and target_position is not sent at all; the log says so.

It also publishes the image with the boxes drawn on it on /<ns>/detection/debug (watch it
with rqt_image_view, or add an Image display in RViz). Yellow = candidate (number = images
in a row), red = confirmed track, white cross = image centre, with the offset next to the
target. --save DIR also writes the raw and annotated images there every --save-every
seconds, for running through `mount-backend.sh predict --images DIR` afterwards.

Detections are matched from image to image by their direction in the world, so the lock
holds while the gimbal or the drone turns (the target jumps in the picture, not in the
world); gate --gate-deg. Until PX4's camera orientation is known they are matched in pixels
(gate --gate), and switching between the two starts the tracks over. --horizon ROW can cut off
everything below a fixed image row, e.g. 360 for a level gimbal at 1280x720. LiveDetector
is also used by tests/detection_test.py.

--tracker bytetrack uses ByteTrack (bytetrack.py) instead of that k-in-a-row matching: a
Kalman filter per track in pixels, moved along when the camera turns (from PX4's
orientation in this image and the last), and low-score detections used to continue
tracks but never to start them. Its score thresholds depend on the detector (--high,
--low, --new; defaults in TRACK_SCORES); for the baseline the detector's own --contrast
is lowered to --low so that it reports the weak candidates too. In the debug image a
track is drawn with its id: red = confirmed, yellow = tentative (images seen), grey =
lost (where the filter predicts it, with the images since it was last seen).

Birds: YOLO (two classes, drone and bird) always reports its bird boxes here (it runs as
with --yolo-birds). They are drawn green ("bird 0.62") and never go to the tracker, the
lock or the topics: only boxes YOLO calls a drone can become the target. A bird that YOLO
calls a drone is drawn and tracked like a drone; nothing here can tell it apart.
"""
import argparse
import math
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

import cv2
import numpy as np

from swarm_backend.config import default_namespace
from swarm_backend.detect import detectors
from swarm_backend.detect.camera import GIMBAL_HFOV, CameraModel, camera_roll
from swarm_backend.detect.orientation import Axes
from swarm_backend.detect.candidates import above_line
from swarm_backend.detect.bytetrack import ByteTracker, Settings, State
from swarm_backend.detect.detectors import Detection, Detector, draw
from swarm_backend.detect.kalman import Noise, rotation_homography
from swarm_backend.detect.tracking import Confirmer, Track, angle_between
from swarm_backend.sensor_view import (
    bridge_config, find_drone, find_sensors, gz_topics, start_bridge)

CANDIDATE = (255, 220, 0)
CONFIRMED = (255, 0, 0)
HORIZON = (0, 160, 255)
CENTRE = (255, 255, 255)
LOST = (170, 170, 170)
BIRD = (0, 220, 0)  # YOLO's bird class: shown, never tracked

# ByteTrack's score thresholds per detector (high, low, new), in the detector's own score:
# the baseline's is contrast in grey levels (it reports nothing under its --contrast, 60 by
# default, so with ByteTrack that is lowered to `low`), YOLO's a confidence 0-1.
# (high, low, new). YOLO's from the threshold replay of 11 Nov 2026 (detect/replay_eval.py,
# vault: detection/Threshold tuning 2026-11-11): high/new 0.3/0.4 instead of 0.5/0.6 start
# tracks at ~62 m instead of ~49 m, with the same switches under --lock.
TRACK_SCORES = {'baseline': (60.0, 30.0, 60.0), 'yolo': (0.3, 0.1, 0.4)}


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
    matching: str = 'pixels'  # how detections were matched to tracks: 'world', 'pixels',
    # or with ByteTrack 'pixels+gimbal' (compensated for the camera turning)
    position: tuple[float, float, float] | None = None  # target_position, world NED, metres
    locked_id: int | None = None  # --lock: the track id locked on (None: no lock, or off)
    lock_event: str | None = None  # --lock: what changed with this image (locked, handover...)
    birds: list[Detection] = field(default_factory=list)  # YOLO's bird boxes: drawn only


def split_birds(found: list[Detection]) -> tuple[list[Detection], list[Detection]]:
    """(boxes that can become a target, YOLO's bird boxes, which are only drawn)."""
    return ([b for b in found if b.label != 'bird'], [b for b in found if b.label == 'bird'])


def best_target(tracks: list[Track]) -> Track | None:
    """The track to publish: confirmed, detected in this image, highest score."""
    seen = [t for t in tracks if t.confirmed and t.detection is not None]
    return max(seen, key=lambda t: t.detection.score, default=None)


LOCK_GATE_DEG = 4.0  # a new track this close to where the locked one should be is it
LOCK_GATE_GROWTH_DEG_S = 5.0  # ... widened by this per second it has not been seen
LOCK_GATE_MAX_DEG = 10.0  # the two hostiles of scenario 2 stay about 40 deg apart
LOCK_TIMEOUT_S = 1.0  # unseen this long, the lock is released (ByteTrack keeps the track 3 s)

Direction = tuple[float, float, float]  # unit vector, world NED


class TargetLock:
    """--lock: stay on one drone, also when the tracker gives it a new id.

    Each image (update) it publishes the locked track while that is confirmed and seen.
    When it is not, a confirmed track within `gate_deg` (+ `growth_deg_s` per second unseen,
    at most `max_deg`) of where the locked one should be is taken as the same drone with a
    new id, and the lock is handed over to it. "Should be": ByteTrack's prediction of the
    lost track (its grey box, compensated for the camera turning), else the direction it
    was last seen in. Otherwise nothing is published, until it has been unseen for
    `timeout` s; then the lock is released and goes to the best target (best_target), even
    though the tracker may still keep the old track. Another drone that scores higher never
    takes the lock while it holds.

    `direction(track, predicted)` gives a track's world direction: from its detection in
    this image, or with predicted=True from the tracker's prediction (None when unknown,
    e.g. before PX4 has reported the camera's orientation: then there is no handover)."""

    def __init__(self, gate_deg: float = LOCK_GATE_DEG,
                 growth_deg_s: float = LOCK_GATE_GROWTH_DEG_S,
                 max_deg: float = LOCK_GATE_MAX_DEG, timeout: float = LOCK_TIMEOUT_S):
        self.gate_deg = gate_deg
        self.growth_deg_s = growth_deg_s
        self.max_deg = max_deg
        self.timeout = timeout
        self.reset()

    def reset(self) -> None:
        self.locked_id: int | None = None
        self.seen_at: float | None = None  # image time the locked drone was last published
        self.where: Direction | None = None  # its direction then
        self.event: str | None = None  # what changed in the last update, for the log

    def gate(self, unseen_s: float) -> float:
        return min(self.max_deg, self.gate_deg + self.growth_deg_s * max(0.0, unseen_s))

    def update(self, tracks: list, stamp: float,
               direction: Callable[[object, bool], Direction | None]):
        """The track to publish for this image (None: nothing)."""
        self.event = None
        seen = [t for t in tracks if t.confirmed and t.detection is not None]
        if self.locked_id is not None:
            locked = next((t for t in tracks if t.id == self.locked_id), None)
            if locked is not None and locked in seen:
                return self._publish(locked, stamp, direction)
            unseen = stamp - self.seen_at
            expected = (direction(locked, True) if locked is not None else None) or self.where
            if expected is not None:
                gate = math.radians(self.gate(unseen))
                close = []
                for t in seen:
                    d = direction(t, False)
                    if d is not None and angle_between(d, expected) <= gate:
                        close.append((angle_between(d, expected), t))
                if close:
                    old = self.locked_id
                    track = min(close, key=lambda c: c[0])[1]
                    self.locked_id = track.id
                    published = self._publish(track, stamp, direction)
                    self.event = f'T{old} -> T{track.id} (handover: the same drone, a new id)'
                    return published
            if unseen <= self.timeout:
                return None  # a short gap: the attack coasts or predicts through it
            self.event = f'T{self.locked_id} released (not seen for {unseen:.1f} s)'
            self.locked_id = None
        best = best_target(seen)
        if best is None:
            return None
        released = self.event
        self.locked_id = best.id
        published = self._publish(best, stamp, direction)
        self.event = (f'{released}; ' if released else '') + f'T{best.id} locked'
        return published

    def _publish(self, track, stamp: float, direction) -> object:
        self.seen_at = stamp
        self.where = direction(track, False) or self.where
        return track


def directions(boxes: list[Detection], camera: CameraModel, axes: Axes) -> list:
    """The world direction (unit vector, NED) to each box's centre."""
    return [camera.direction(b.u, b.v, *axes) for b in boxes]


def world_confirmer(k: int = 3, gate_deg: float = 10.0) -> Confirmer:
    """Confirmation by world direction: the gate is an angle."""
    return Confirmer(k=k, gate=math.radians(gate_deg), distance=angle_between)


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
    out = draw(img, result.birds, BIRD,
               labels=[f'bird {b.score:.2f}' for b in result.birds])
    out = draw(out, [b for b in result.boxes if id(b) not in tracked], CANDIDATE)
    byte = any(hasattr(t, 'state') for t in result.tracks)  # ByteTrack's tracks
    confirmed = [t for t in seen if t.confirmed]
    out = draw(out, [t.detection for t in confirmed], CONFIRMED,
               labels=[f'T{t.id} {t.detection.width}px'
                       + (f' s{t.detection.score:.3g}' if byte else '') for t in confirmed])
    unconfirmed = [t for t in seen if not t.confirmed]
    out = draw(out, [t.detection for t in unconfirmed], CANDIDATE,
               labels=[f'T{t.id} {t.streak}' if byte else str(t.streak) for t in unconfirmed])
    lost = [t for t in result.tracks if getattr(t, 'state', None) == State.LOST]
    out = draw(out, [predicted_box(t) for t in lost], LOST,
               labels=[f'T{t.id} lost {t.misses}' for t in lost])
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


def predicted_box(track) -> Detection:
    """A ByteTrack track's Kalman box (where it should be now) as a Detection, to draw."""
    u, v, w, h = track.box
    return Detection(round(u - w / 2), round(v - h / 2), round(w), round(h), 0.0)


def add_tracker_options(parser: argparse.ArgumentParser) -> None:
    group = parser.add_argument_group('tracker')
    group.add_argument('--tracker', choices=('confirmer', 'bytetrack'), default='confirmer',
                       help='k-in-a-row matching (tracking.py) or ByteTrack (bytetrack.py); '
                            'default %(default)s')
    group.add_argument('--high', type=float,
                       help='ByteTrack: lowest score of a sure detection (default per detector, '
                            'TRACK_SCORES)')
    group.add_argument('--low', type=float,
                       help='ByteTrack: lowest score used at all, to continue tracks')
    group.add_argument('--new', type=float, help='ByteTrack: lowest score that starts a track')
    group.add_argument('--max-lost', type=float, default=Settings.max_lost,
                       help='ByteTrack: how long a lost track is kept, in images at --rate '
                            '(default %(default)s = 3 s at 10 per second), real time')
    group.add_argument('--gate-sigma', type=float, default=Settings.gate,
                       help='ByteTrack: largest distance from the prediction, in standard '
                            'deviations (default %(default)s)')
    group.add_argument('--min-px', type=float, default=Noise.min_px,
                       help="ByteTrack: floor of the Kalman filter's noise, px (default "
                            '%(default)s; covers the camera orientation error)')
    group.add_argument('--recover-low', action=argparse.BooleanOptionalAction,
                       default=Settings.recover_low,
                       help='ByteTrack: lost tracks may come back on a low-score detection '
                            '(default %(default)s)')


def make_tracker(args: argparse.Namespace, argv: list[str] | None = None
                 ) -> ByteTracker | None:
    """The ByteTracker the options ask for, or None for the Confirmer. Call it before
    making the detector: it lowers the detector's own threshold to --low (unless it was
    given), so that the detector also reports the weak candidates ByteTrack can use: the
    baseline's --contrast, YOLO's --conf (its default 0.25 would drop everything between
    low and 0.25 before ByteTrack sees it)."""
    if args.tracker != 'bytetrack':
        return None
    high, low, new = TRACK_SCORES.get(args.detector, TRACK_SCORES['yolo'])
    high = high if args.high is None else args.high
    low = low if args.low is None else args.low
    new = new if args.new is None else args.new
    given = sys.argv[1:] if argv is None else argv
    if args.detector == 'baseline' and not any(a.startswith('--contrast') for a in given):
        args.contrast = min(args.contrast, low)
    if args.detector == 'yolo' and not any(a.startswith('--conf') for a in given):
        args.conf = min(args.conf, low)
    return ByteTracker(Settings(high=high, low=low, new=new, k=args.k, max_lost=args.max_lost,
                                rate_hz=getattr(args, 'rate', Settings.rate_hz),
                                gate=args.gate_sigma, gate_low=args.gate_sigma,
                                recover_low=args.recover_low,
                                noise=Noise(min_px=args.min_px)))


def describe_tracker(tracker: ByteTracker | None) -> str:
    if tracker is None:
        return 'Confirmer (k in a row)'
    s = tracker.settings
    return (f'ByteTrack (high >= {s.high:g}, low >= {s.low:g}, new >= {s.new:g}, k {s.k}, '
            f'gate {s.gate:g} sigma, lost kept {s.max_lost} images)')


class LiveDetector:
    """Detection on one drone's camera, inside an existing ROS node.

    detector: any detector from detectors.py. horizon: callable giving the image row of the
    horizon for the next image (or None to search the whole image). use_px4: read the
    camera's orientation from PX4 (roll for the offset); off, the offset is not
    roll-corrected. on_result is called with every Result. tracker: a ByteTracker to use
    instead of the confirmers (make_tracker).
    """

    def __init__(self, node, drone_id: int, detector: Detector,
                 confirmer: Confirmer | None = None, rate_hz: float = 10.0,
                 horizon: Callable[[], float | None] = lambda: None,
                 on_result: Callable[[Result], None] | None = None,
                 save: Path | None = None, save_every: float = 1.0,
                 use_px4: bool = True, hfov: float = GIMBAL_HFOV,
                 direction_confirmer: Confirmer | None = None,
                 tracker: ByteTracker | None = None,
                 detector_name: str = 'baseline', lock: TargetLock | None = None):
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
        self.detector_name = detector_name  # picks the range rule (camera.box_scale)
        self.px4 = None
        if use_px4:
            from swarm_backend.detect.px4_camera import PX4Camera
            self.px4 = PX4Camera(node, drone_id)
        self.hfov = hfov
        self.confirmer = confirmer or Confirmer()  # in pixels, while the orientation is unknown
        self.world_confirmer = direction_confirmer or world_confirmer(self.confirmer.k)
        self._matching = None
        self.tracker = tracker
        self.lock = lock  # stay on one drone (TargetLock), or None: best score every image
        self._last_axes: Axes | None = None  # the camera's orientation in the last image
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
        found = self.detector(img, self._sky_mask(img.shape, horizon))
        boxes, birds = split_birds(found)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        axes = None if self.px4 is None else self.px4.axes(stamp)
        roll = None if axes is None else camera_roll(axes[0], axes[1])
        camera = CameraModel(self.hfov, msg.width, msg.height)
        if self.tracker is not None:
            tracks, matching = self._byte_track(boxes, camera, axes, stamp)
        else:
            matching = 'pixels' if axes is None else 'world'
            if matching != self._matching:  # positions of the two kinds can't be compared
                self.confirmer.tracks.clear()
                self.world_confirmer.tracks.clear()
                self._matching = matching
                if self.lock is not None:
                    self.lock.reset()  # the other confirmer numbers its tracks itself
            if axes is None:
                tracks = self.confirmer.update([(b.u, b.v) for b in boxes], boxes)
            else:
                tracks = self.world_confirmer.update(directions(boxes, camera, axes), boxes)
        if self.lock is not None:
            def direction(track, predicted: bool):
                if axes is None:
                    return None
                if predicted:
                    if not hasattr(track, 'kalman'):
                        return None  # the Confirmer predicts nothing: use where it was seen
                    u, v = track.box[:2]  # ByteTrack's prediction, also for a lost track
                else:
                    u, v = track.detection.u, track.detection.v
                return camera.direction(u, v, *axes)
            target = self.lock.update(tracks, stamp, direction)
        else:
            target = best_target(tracks)
        offset = position = None
        if target is not None:
            offset = camera.offset_angles(target.detection.u, target.detection.v, roll or 0.0)
            out_msg = self._Vector3Stamped()
            out_msg.header.stamp = msg.header.stamp  # when the image was taken
            out_msg.header.frame_id = self.drone.name
            out_msg.vector.x, out_msg.vector.y = offset
            self._offset_pub.publish(out_msg)
            if axes is not None:  # the direction needs the camera's orientation
                d = target.detection
                px, size = camera.box_scale(self.detector_name, d.width, d.height)
                position = camera.position(d.u, d.v, px, *axes, size=size)
                pos_msg = self._PointStamped()
                pos_msg.header = out_msg.header  # same image, same stamp
                pos_msg.point.x, pos_msg.point.y, pos_msg.point.z = position
                self._position_pub.publish(pos_msg)
        ms = (time.perf_counter() - start) * 1000
        result = Result(stamp, msg.width, msg.height, boxes, tracks, horizon, ms, target, offset,
                        axes, roll, matching, position,
                        None if self.lock is None else self.lock.locked_id,
                        None if self.lock is None else self.lock.event, birds)

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

    def _byte_track(self, boxes: list[Detection], camera: CameraModel, axes: Axes | None,
                    stamp: float | None = None):
        """ByteTrack on this image; the tracks are moved along for the camera's turning since
        the last image when the orientation is known in both, and by the real time since
        then (`stamp`, the image's sim time). Returns all tracks (lost ones too, to draw
        them) and how they were matched."""
        homography = None
        if axes is not None and self._last_axes is not None:
            homography = rotation_homography(camera, self._last_axes, axes)
        self._last_axes = axes
        self.tracker.update(boxes, homography, stamp)
        return list(self.tracker.tracks), 'pixels' if homography is None else 'pixels+gimbal'


def result_record(result: Result) -> dict:
    """One image as a JSON-able dict, for --record (the baseline evaluation scores these
    against Gazebo's truth): every candidate box, and the confirmed tracks seen in it."""
    def box(d):
        return [round(d.u, 1), round(d.v, 1), d.width, d.height]
    return {
        'stamp': result.stamp, 'width': result.width, 'height': result.height,
        'boxes': [box(b) + [round(float(b.score), 3)] for b in result.boxes],
        'confirmed': [box(t.detection) + [t.id] for t in result.tracks
                      if t.confirmed and t.detection is not None],
        'target': None if result.target is None else result.target.id,
        'oriented': result.axes is not None,
        # For replaying the tracker offline (detect/replay_eval.py): every box exactly as
        # the detector gave it (x, y, width, height, score) and PX4's camera orientation.
        'raw': [[d.x, d.y, d.width, d.height, round(float(d.score), 4)] for d in result.boxes],
        'axes': None if result.axes is None else [[round(c, 6) for c in a] for a in result.axes],
        'birds': [[d.x, d.y, d.width, d.height, round(float(d.score), 4)] for d in result.birds],
    }


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
    parser.add_argument('--gate-deg', type=float, default=10.0,
                        help='degrees (world direction) a target may move between images and '
                             'still be the same track (default %(default)s)')
    parser.add_argument('--gate', type=float, default=20.0,
                        help='the same in pixels, used until the camera orientation is known')
    parser.add_argument('--detector', choices=detectors.DETECTORS, default='baseline',
                        help='which detector to run (default %(default)s)')
    parser.add_argument('--no-px4', action='store_true',
                        help="don't read the camera orientation from PX4 (offset not roll-corrected)")
    parser.add_argument('--save', type=Path, default=None, help='folder for raw + annotated PNGs')
    parser.add_argument('--save-every', type=float, default=1.0,
                        help='seconds (sim time) between saves')
    parser.add_argument('--record', type=Path, default=None,
                        help='append one JSON line per processed image to this file (boxes, '
                             'confirmed tracks; used by the baseline evaluation)')
    parser.add_argument('--lock', action='store_true',
                        help='stay on one drone (TargetLock): publish only the locked one, also '
                             'when another scores higher; a new track where it should be takes '
                             'the lock over; unseen for --lock-timeout, the lock goes to the '
                             'best target')
    parser.add_argument('--lock-timeout', type=float, default=LOCK_TIMEOUT_S,
                        help='--lock: seconds unseen before the lock is released '
                             '(default %(default)s)')
    parser.add_argument('--lock-gate-deg', type=float, default=LOCK_GATE_DEG,
                        help='--lock: degrees from where the locked drone should be that a new '
                             'track may be and take the lock over (default %(default)s, '
                             f'+{LOCK_GATE_GROWTH_DEG_S:g} per second unseen, at most '
                             f'{LOCK_GATE_MAX_DEG:g})')
    detectors.add_options(parser)
    add_tracker_options(parser)
    args = parser.parse_args(argv)
    tracker = make_tracker(args, argv)  # before the detector: may lower --contrast
    args.yolo_birds = True  # birds are drawn green and kept out of tracking (_process)
    try:
        detector = detectors.make(args.detector, args)
    except NotImplementedError as e:
        print(e)
        return 1

    import rclpy

    rclpy.init()
    node = rclpy.create_node(f'detect_{args.id}')
    stats = {'last_log': 0.0, 'frames': 0, 'ms': 0.0}
    record = None
    if args.record:
        import json
        args.record.parent.mkdir(parents=True, exist_ok=True)
        record = open(args.record, 'a', buffering=1)

    def log(result: Result) -> None:
        if record is not None:
            record.write(json.dumps(result_record(result)) + '\n')
        stats['frames'] += 1
        stats['ms'] += result.ms
        if result.lock_event:
            print(f'[{result.stamp:.2f}] lock: {result.lock_event}', flush=True)
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
        camera = ('camera roll unknown (offset not roll-corrected, no position, matching in pixels)'
                  if result.roll is None else f'camera roll {math.degrees(result.roll):+.0f} deg')
        locked = '' if not args.lock else (
            f', locked T{result.locked_id}' if result.locked_id is not None else ', locked none')
        birds = f', {len(result.birds)} birds' if result.birds else ''
        print(f'[{result.stamp:.2f}] {len(result.boxes)} candidates{birds}, confirmed: {confirmed}'
              f'{locked}{offset}; {camera}  ({stats["ms"] / stats["frames"]:.0f} ms/image)', flush=True)
        stats.update(last_log=now, frames=0, ms=0.0)

    try:
        live = LiveDetector(node, args.id, detector, Confirmer(k=args.k, gate=args.gate),
                            args.rate, lambda: args.horizon, log, args.save, args.save_every,
                            use_px4=not args.no_px4,
                            direction_confirmer=world_confirmer(args.k, args.gate_deg),
                            tracker=tracker,
                            detector_name=args.detector,
                            lock=TargetLock(args.lock_gate_deg, timeout=args.lock_timeout)
                            if args.lock else None)
    except RuntimeError as e:
        print(e)
        return 1
    print(f'Drone {args.id} ({live.drone.name}), {args.detector} detector: '
          f'{live.camera.gz_topic}\n  -> {live.camera.ros_topic}\n'
          f'  target_offset on {live.offset_topic}\n'
          f'  target_position on {live.position_topic}\n  debug image on {live.debug_topic}\n'
          f'  camera orientation from PX4: {"off" if args.no_px4 else "on"}\n'
          f'  tracker: {describe_tracker(tracker)}\n'
          '  lock on one drone: ' + (
              f'on (handover within {args.lock_gate_deg:g} deg, released after '
              f'{args.lock_timeout:g} s unseen)' if args.lock else 'off (best score in every image)'))
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
