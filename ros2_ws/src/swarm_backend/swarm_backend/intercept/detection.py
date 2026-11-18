"""The two detection topics the attack flies on, and a fake detector that makes them.

The shared contract (hive-vault / project doc "Baseline attack: work plan"):

  /<ns>/detection/target_position  geometry_msgs/PointStamped
      The target relative to the drone (its camera), world NED, metres
      (= distance x direction). Steers the drone.
  /<ns>/detection/target_offset    geometry_msgs/Vector3Stamped
      x = horizontal angle from the image centre (rad, right positive),
      y = vertical angle (rad, up positive), corrected for the camera's roll. z unused (0).
      Steers the gimbal.

Both are only published for a confirmed target (seen k images in a row), the best one if
there are several, and both carry the time the image was taken (simulation time). No
message means no confirmed target. <ns> is '' for PX4 instance 0 and px4_<id> otherwise.

The fake detector below makes the same two messages from where the drones really are
(Gazebo), with the camera's field of view, a range limit, a blind zone, noise and a delay
that can each be turned on. Pure math, no ROS: nodes/fake_detection.py runs it.
"""
import dataclasses
import math
import random
from collections import deque
from dataclasses import dataclass

from swarm_backend.config import AttackConfig, Vector3, default_namespace
from swarm_backend.detect.camera import CameraModel

POSITION_TOPIC = 'detection/target_position'
OFFSET_TOPIC = 'detection/target_offset'
FRAME_ID = 'world_ned'  # header.frame_id of target_position
MIN_RANGE_M = 0.1  # a noisy distance is never shorter than this

# The fake's settings that make it behave like the real baseline detector (work plan,
# Max task 1): 0.25 s late, 1 degree and 20 % noise, nothing seen closer than 5 m.
REALISTIC = {'fake_delay': 0.25, 'fake_angle_noise_deg': 1.0, 'fake_range_noise': 0.2,
             'fake_blind_range': 5.0}


def fake_config(config: AttackConfig, realistic: bool = False, **overrides) -> AttackConfig:
    """The attack config with the fake detector's errors: REALISTIC first, then the
    overrides that are not None."""
    changes = dict(REALISTIC) if realistic else {}
    changes.update({k: v for k, v in overrides.items() if v is not None})
    return dataclasses.replace(config, **changes)


def topic(drone_id: int, name: str) -> str:
    """Full topic name for drone `drone_id`, e.g. /px4_2/detection/target_position."""
    ns = default_namespace(drone_id)
    return f'/{ns}/{name}' if ns else f'/{name}'


@dataclass(frozen=True)
class Measurement:
    """What one image says about the target, as published on the two topics."""
    position: Vector3  # target relative to the camera, world NED, metres
    offset: tuple[float, float]  # (right, up) radians from the image centre, roll corrected


def camera_roll(forward: Vector3, right: Vector3) -> float:
    """How far the image is rolled, radians: positive when its right side points down.

    Measured from the level right direction (90 degrees east of where the camera looks,
    on the horizon). Undefined when looking straight up or down; 0 is returned then."""
    level_right = (-forward[1], forward[0], 0.0)
    norm = math.hypot(*level_right)
    if norm < 1e-9:
        return 0.0
    level_right = tuple(c / norm for c in level_right)
    level_down = _cross(forward, level_right)
    return math.atan2(_dot(right, level_down), _dot(right, level_right))


def level_offset(right_angle: float, up_angle: float, roll: float) -> tuple[float, float]:
    """Image angles (right, up) of a camera rolled by `roll` turned into the angles the
    same point would have in a level image: atan((u - cx) / f) and atan((cy - v) / f) of
    the level image, which is what the gimbal's yaw and pitch need. The image plane is
    turned, so it is exact, not only for small angles."""
    x, y = math.tan(right_angle), math.tan(up_angle)
    c, s = math.cos(roll), math.sin(roll)
    return (math.atan(x * c + y * s), math.atan(-x * s + y * c))


def measure(camera: dict, target: Vector3, model: CameraModel, max_range: float,
            blind_range: float = 0.0, angle_noise: float = 0.0, range_noise: float = 0.0,
            rng: random.Random | None = None) -> Measurement | None:
    """What a detector on `camera` would report about a target at `target`.

    `camera` has the world NED 'position' and the unit vectors 'forward', 'right' and
    'down' of the image (as ground_truth.camera_pose gives them). None when the target is
    behind the camera, outside the image, further than `max_range` or closer than
    `blind_range`. `angle_noise` (radians) and `range_noise` (share of the distance) are
    1-sigma Gaussian errors; they need `rng`."""
    to_target = tuple(t - c for t, c in zip(target, camera['position']))
    depth, right, down = (_dot(to_target, camera[axis]) for axis in ('forward', 'right', 'down'))
    if depth <= 0:
        return None
    if abs(model.f * right / depth) >= model.width / 2 or abs(model.f * down / depth) >= model.height / 2:
        return None
    distance = math.hypot(*to_target)
    if distance > max_range or distance < blind_range:
        return None

    a, b = math.atan2(right, depth), math.atan2(down, depth)  # right, down from the centre
    if angle_noise > 0 or range_noise > 0:
        a += rng.gauss(0.0, angle_noise)
        b += rng.gauss(0.0, angle_noise)
        distance = max(MIN_RANGE_M, distance * (1.0 + rng.gauss(0.0, range_noise)))
    ray = tuple(f + math.tan(a) * r + math.tan(b) * d
                for f, r, d in zip(camera['forward'], camera['right'], camera['down']))
    norm = math.hypot(*ray)
    position = tuple(distance * c / norm for c in ray)
    roll = camera_roll(camera['forward'], camera['right'])
    return Measurement(position, level_offset(a, -b, roll))


class FakeDetector:
    """Turns true poses into the messages a real detector would publish, and when.

    observe() is called with the true camera pose and target position; at most
    `rate_hz` of those become images. A target is confirmed once it was in view
    `confirm_frames` images in a row. Confirmed measurements come out of due() `delay`
    seconds after their image, stamped with the image time."""

    def __init__(self, config: AttackConfig, model: CameraModel = CameraModel()):
        self.model = model
        self.period = 1.0 / config.fake_rate_hz
        self.delay = config.fake_delay
        self.angle_noise = math.radians(config.fake_angle_noise_deg)
        self.range_noise = config.fake_range_noise
        self.blind_range = config.fake_blind_range
        self.max_range = config.fake_max_range or model.range_from_width(config.fake_min_pixels)
        self.confirm_frames = config.fake_confirm_frames
        self.rng = random.Random(config.seed)
        self.streak = 0  # images in a row with the target in view
        self.images = 0
        self.published = 0
        self._last_image: float | None = None
        self._pending: deque[tuple[float, Measurement]] = deque()

    def wants_image(self, stamp: float) -> bool:
        """True when an image taken at `stamp` (seconds) is due at the detector's rate."""
        if self._last_image is not None and stamp < self._last_image:
            self._last_image = None  # the simulation was reset
        return self._last_image is None or stamp - self._last_image >= self.period - 1e-6

    def observe(self, stamp: float, camera: dict | None, target: Vector3 | None) -> None:
        """One image at `stamp`. camera or target None: neither is known, so nothing seen."""
        self._last_image = stamp
        self.images += 1
        found = None
        if camera is not None and target is not None:
            found = measure(camera, target, self.model, self.max_range, self.blind_range,
                            self.angle_noise, self.range_noise, self.rng)
        self.streak = self.streak + 1 if found is not None else 0
        if found is not None and self.streak >= self.confirm_frames:
            self._pending.append((stamp, found))

    def due(self, now: float) -> list[tuple[float, Measurement]]:
        """(image time, measurement) of every confirmed image whose delay is over at `now`."""
        out = []
        while self._pending and self._pending[0][0] + self.delay <= now + 1e-9:
            out.append(self._pending.popleft())
        self.published += len(out)
        return out


def _dot(a, b) -> float:
    return sum(x * y for x, y in zip(a, b))


def _cross(a, b) -> Vector3:
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])
