"""Reads which way a drone's camera points from PX4, for the moment an image was taken.

Listens (never commands) to the drone's gimbal status, attitude and position, and to Gazebo's clock
to match PX4's timestamps with the camera's (see orientation.py). Used by the live detector
(roll for target_offset, direction for target_position, its own position at the image's
time for the friendly check) and by snap (to compare with the
truth). Needs ROS and Gazebo's Python bindings, so it runs in the container.
"""
import math
import threading

from px4_msgs.msg import GimbalDeviceAttitudeStatus, VehicleOdometry

from swarm_backend.config import default_namespace
from swarm_backend.detect.orientation import (
    AttitudeHistory, Axes, ClockOffset, PositionHistory, camera_axes)
from swarm_backend.px4.px4_drone import PX4_OUT_QOS
from swarm_backend.px4.topics import PX4Topics

EARTH_FRAME = GimbalDeviceAttitudeStatus.DEVICE_FLAGS_YAW_IN_EARTH_FRAME


class PX4Camera:

    def __init__(self, node, drone_id: int):
        from gz.msgs10.clock_pb2 import Clock
        from gz.transport13 import Node as GzNode

        topics = PX4Topics(default_namespace(drone_id))
        self.gimbal = AttitudeHistory()
        self.drone = AttitudeHistory()
        self.position = PositionHistory()  # PX4 local NED: only differences are used
        self.clock = ClockOffset()
        self.earth_frame: bool | None = None  # from the gimbal's flags, once it has reported
        self._lock = threading.Lock()
        self._gz_now: float | None = None
        self._node = node
        self._subscriptions = [
            node.create_subscription(GimbalDeviceAttitudeStatus, topics.gimbal_attitude,
                                     self._on_gimbal, PX4_OUT_QOS),
            node.create_subscription(VehicleOdometry, topics.vehicle_odometry,
                                     self._on_odometry, PX4_OUT_QOS)]
        self._gz = GzNode()
        if not self._gz.subscribe(Clock, '/clock', self._on_clock):
            raise RuntimeError("Could not subscribe to Gazebo's /clock")

    def close(self) -> None:
        """Stop listening (the node itself stays)."""
        for subscription in self._subscriptions:
            self._node.destroy_subscription(subscription)
        self._subscriptions = []

    def _on_clock(self, msg) -> None:
        with self._lock:
            self._gz_now = msg.sim.sec + msg.sim.nsec * 1e-9

    def _stamp(self, msg) -> float:
        t = msg.timestamp * 1e-6
        if self._gz_now is not None:
            self.clock.update(t, self._gz_now)
        return t

    def _on_gimbal(self, msg) -> None:
        q = tuple(float(v) for v in msg.q)
        if any(v != v for v in q):  # NaN: no attitude yet
            return
        with self._lock:
            self.earth_frame = bool(msg.device_flags & EARTH_FRAME)
            self.gimbal.add(self._stamp(msg), q)

    def _on_odometry(self, msg) -> None:
        q = tuple(float(v) for v in msg.q)
        if any(v != v for v in q):
            return
        p = tuple(float(v) for v in msg.position)
        with self._lock:
            t = self._stamp(msg)
            self.drone.add(t, q)
            if not any(math.isnan(v) for v in p):
                self.position.add(t, p)

    def axes(self, image_stamp: float) -> Axes | None:
        """forward, right and down of the camera in the world (NED) when the image with this
        (Gazebo) timestamp was taken, or None while PX4 has not reported enough yet."""
        with self._lock:
            t = self.clock.to_px4(image_stamp)
            if t is None or self.earth_frame is None:
                return None
            gimbal = self.gimbal.at(t)
            if gimbal is None:
                return None
            drone = None if self.earth_frame else self.drone.at(t)
            return camera_axes(gimbal, self.earth_frame, drone)

    def moved(self, since: float, until: float):
        """How far the drone moved (world NED, metres) between two Gazebo times, from
        PX4's odometry, or None while it does not cover them. Local and world frames only
        differ by the home offset, which cancels."""
        with self._lock:
            t0, t1 = self.clock.to_px4(since), self.clock.to_px4(until)
            if t0 is None:
                return None
            p0, p1 = self.position.at(t0), self.position.at(t1)
            if p0 is None or p1 is None:
                return None
            return tuple(b - a for a, b in zip(p0, p1))

    def drone_attitude(self, image_stamp: float):
        """The drone's attitude (w, x, y, z; body FRD -> world NED) when the image with this
        (Gazebo) timestamp was taken, or None while PX4 has not reported enough yet."""
        with self._lock:
            t = self.clock.to_px4(image_stamp)
            return None if t is None else self.drone.at(t)
