"""Which way the camera points, from what PX4 reports (task 2 of the baseline attack plan).

PX4's gimbal status (`gimbal_device_attitude_status`) gives the camera's attitude as a
quaternion (w, x, y, z). With the flag DEVICE_FLAGS_YAW_IN_EARTH_FRAME set, which PX4's
simulated gimbal does, it is the camera's full attitude in the world frame (NED, the camera
frame being front-right-down), roll included: rotating the axes (1, 0, 0), (0, 1, 0) and
(0, 0, 1) by it gives the directions the camera looks along, of image right and of image
down. Without the flag it is relative to the drone, and the drone's attitude is applied
first. Measured against Gazebo's truth on 14 Oct: 0.35 degrees median, 1.6 degrees worst.

PX4's message timestamps run at simulation speed but count from a different zero than the
camera's (Gazebo) timestamps, so they are matched through ClockOffset. Pure math, no ROS.
"""
import bisect
import math
from collections.abc import Sequence

Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]  # w, x, y, z, as PX4 sends it
Axes = tuple[Vector3, Vector3, Vector3]  # forward, right, down in the world (NED)


def normalize(q: Quaternion) -> Quaternion:
    norm = math.sqrt(sum(c * c for c in q))
    return tuple(c / norm for c in q)


def multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    """The rotation b followed by a."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def rotate(q: Quaternion, v: Vector3) -> Vector3:
    """v turned by the quaternion q."""
    w, x, y, z = q
    vx, vy, vz = v
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (vx + w * tx + y * tz - z * ty,
            vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx)


def slerp(a: Quaternion, b: Quaternion, t: float) -> Quaternion:
    """The rotation a share t of the way from a to b (shortest way)."""
    dot = sum(x * y for x, y in zip(a, b))
    if dot < 0:  # q and -q are the same rotation: take the short way round
        b, dot = tuple(-c for c in b), -dot
    if dot > 0.9995:
        return normalize(tuple(x + t * (y - x) for x, y in zip(a, b)))
    theta = math.acos(dot)
    sa, sb = math.sin((1 - t) * theta), math.sin(t * theta)
    return normalize(tuple((sa * x + sb * y) / math.sin(theta) for x, y in zip(a, b)))


def camera_axes(gimbal_q: Quaternion, earth_frame: bool,
                drone_q: Quaternion | None = None) -> Axes | None:
    """forward, right and down of the camera in the world (NED) from the gimbal's reported
    attitude; `drone_q` (the drone's attitude) is needed only when it is not in the earth
    frame. None when that is missing."""
    if not earth_frame:
        if drone_q is None:
            return None
        gimbal_q = multiply(drone_q, gimbal_q)
    q = normalize(gimbal_q)
    return rotate(q, (1.0, 0.0, 0.0)), rotate(q, (0.0, 1.0, 0.0)), rotate(q, (0.0, 0.0, 1.0))


class AttitudeHistory:
    """Recent attitudes (time, quaternion), to look one up for the moment an image was taken."""

    def __init__(self, size: int = 200):
        self.size = size
        self._times: list[float] = []
        self._quats: list[Quaternion] = []

    def add(self, t: float, q: Quaternion) -> None:
        if self._times and t <= self._times[-1]:
            if t < self._times[-1] - 1.0:  # far back in time: the simulation was restarted
                self._times.clear()
                self._quats.clear()
            else:
                return
        self._times.append(t)
        self._quats.append(normalize(q))
        if len(self._times) > self.size:
            del self._times[0], self._quats[0]

    def at(self, t: float, tolerance: float = 0.5) -> Quaternion | None:
        """The attitude at time t, interpolated between the two reports around it, or the
        nearest one within `tolerance` seconds when t is outside the history."""
        if not self._times:
            return None
        i = bisect.bisect_left(self._times, t)
        if 0 < i < len(self._times):
            t0, t1 = self._times[i - 1], self._times[i]
            if t1 - t0 <= 2 * tolerance:
                return slerp(self._quats[i - 1], self._quats[i], (t - t0) / (t1 - t0))
        j = min((k for k in (i - 1, i) if 0 <= k < len(self._times)),
                key=lambda k: abs(self._times[k] - t))
        return self._quats[j] if abs(self._times[j] - t) <= tolerance else None


class PositionHistory:
    """Recent positions (time, PX4 local NED), to look one up for any moment: linear
    between the two reports around it, or the nearest within `tolerance` seconds."""

    def __init__(self, size: int = 500):
        self.size = size
        self._times: list[float] = []
        self._points: list[Vector3] = []

    def add(self, t: float, p: Vector3) -> None:
        if self._times and t <= self._times[-1]:
            if t < self._times[-1] - 1.0:  # far back in time: the simulation was restarted
                self._times.clear()
                self._points.clear()
            else:
                return
        self._times.append(t)
        self._points.append(tuple(p))
        if len(self._times) > self.size:
            del self._times[0], self._points[0]

    def at(self, t: float, tolerance: float = 0.5) -> Vector3 | None:
        if not self._times:
            return None
        i = bisect.bisect_left(self._times, t)
        if 0 < i < len(self._times):
            t0, t1 = self._times[i - 1], self._times[i]
            if t1 - t0 <= 2 * tolerance:
                k = (t - t0) / (t1 - t0)
                return tuple(a + k * (b - a) for a, b in zip(self._points[i - 1],
                                                            self._points[i]))
        j = min((k for k in (i - 1, i) if 0 <= k < len(self._times)),
                key=lambda k: abs(self._times[k] - t))
        return self._points[j] if abs(self._times[j] - t) <= tolerance else None


class ClockOffset:
    """PX4 time minus camera (Gazebo) time, learned from pairs (PX4 stamp, Gazebo time when
    the message arrived). A message arrives after it was stamped, so each pair is at most
    the true offset; the largest seen is the best estimate."""

    def __init__(self):
        self.value: float | None = None
        self.samples = 0

    def update(self, px4_stamp: float, gazebo_now: float) -> None:
        sample = px4_stamp - gazebo_now
        if self.value is not None and sample < self.value - 5.0:
            self.value = None  # jumped back by seconds: one of the clocks was restarted
        self.value = sample if self.value is None else max(self.value, sample)
        self.samples += 1

    def to_px4(self, gazebo_t: float) -> float | None:
        return None if self.value is None else gazebo_t + self.value


def axes_error_deg(a: Sequence[Vector3], b: Sequence[Vector3]) -> tuple[float, float]:
    """(angle between the forward directions, difference in roll), degrees, of two cameras."""
    from swarm_backend.detect.camera import camera_roll
    dot = sum(x * y for x, y in zip(a[0], b[0]))
    forward = math.degrees(math.acos(max(-1.0, min(1.0, dot))))
    roll = math.degrees(camera_roll(a[0], a[1]) - camera_roll(b[0], b[1]))
    return forward, abs((roll + 180.0) % 360.0 - 180.0)
