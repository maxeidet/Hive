"""The base station's radar over ROS: the mock that publishes returns, and the
TargetSource that receives them.

Returns are radar_msgs/RadarScan, one RadarReturn per detected object, as the radar
measures them: range (m), azimuth (rad), elevation (rad) and doppler_velocity
(radial velocity, m/s, positive = moving away). Frame `base_radar_ned`: centred on
the radar (config intercept.radar_position, world NED), azimuth clockwise from north,
elevation up from the horizon. Not REP 103's x-forward / y-left: this is how a
ground radar reports bearings. Once hive_interfaces exists this may become its
Detection message (source = RADAR); only this module changes.
"""
import random
from collections.abc import Callable

from radar_msgs.msg import RadarReturn, RadarScan
from rclpy.node import Node

from swarm_backend.config import InterceptConfig
from swarm_backend.intercept.mock import radar_fix, radar_std
from swarm_backend.intercept.target import RadarFix, TargetSource

FRAME_ID = 'base_radar_ned'
TIMER_HZ = 20.0  # how often the mock checks whether the next return is due


class MockRadar:
    """Measures `hostile` every 1 / radar_rate_hz seconds of the hostile drone's own
    (simulation) clock, with the configured noise, and publishes it. `hostile` is
    anything with `.position`, `.velocity` (world NED) and `.stamp` (seconds): a
    PX4Drone or a DroneObserver."""

    def __init__(self, node: Node, hostile, config: InterceptConfig):
        self.hostile = hostile
        self.config = config
        self.period = 1.0 / config.radar_rate_hz
        self.last_fix: RadarFix | None = None  # the last return published
        self.last_true = None  # where the hostile drone really was then
        self._rng = random.Random(config.seed)
        self._next_time: float | None = None
        self._pub = node.create_publisher(RadarScan, config.radar_topic, 10)
        self._timer = node.create_timer(1.0 / TIMER_HZ, self._on_timer)
        self._node = node

    def destroy(self) -> None:
        self._node.destroy_timer(self._timer)
        self._node.destroy_publisher(self._pub)

    def _on_timer(self) -> None:
        now, position, velocity = self.hostile.stamp, self.hostile.position, self.hostile.velocity
        if now is None or position is None or velocity is None:
            return
        if self._next_time is not None and now < self._next_time:
            return
        self._next_time = now + self.period
        fix = radar_fix(position, velocity, self.config, self._rng, now)
        msg = RadarScan()
        msg.header.frame_id = FRAME_ID
        msg.header.stamp.sec = int(now)
        msg.header.stamp.nanosec = int((now % 1.0) * 1e9)
        msg.returns = [RadarReturn(
            range=fix.range, azimuth=fix.azimuth, elevation=fix.elevation,
            doppler_velocity=fix.radial_velocity, amplitude=1.0)]
        self._pub.publish(msg)
        self.last_fix, self.last_true = fix, position


class RadarTarget(TargetSource):
    """Receives the base station's radar returns and stamps them with `clock` (the
    interceptor's clock) on arrival, so the chase can tell how old they are. The
    radar's position and accuracy come from the config, like a real radar's specs."""

    def __init__(self, node: Node, config: InterceptConfig, clock: Callable[[], float | None]):
        self.config = config
        self.clock = clock
        self._std = radar_std(config)
        self._fix: RadarFix | None = None
        self._node = node
        self._sub = node.create_subscription(RadarScan, config.radar_topic, self._on_scan, 10)

    def latest(self) -> RadarFix | None:
        return self._fix

    def destroy(self) -> None:
        self._node.destroy_subscription(self._sub)

    def _on_scan(self, msg: RadarScan) -> None:
        now = self.clock()
        if now is None or not msg.returns:
            return
        # One hostile drone for now: the first return is it.
        r = msg.returns[0]
        std = self._std
        self._fix = RadarFix(
            time=now, source='radar', radar=self.config.radar_position,
            range=r.range, azimuth=r.azimuth, elevation=r.elevation,
            radial_velocity=r.doppler_velocity, range_std=std['range'],
            azimuth_std=std['azimuth'], elevation_std=std['elevation'],
            radial_velocity_std=std['radial_velocity'])
