"""A fast stand-in for PX4 in Gazebo: for closed-loop unit tests and for running the
scenarios without Gazebo (`--sim`, swarm_backend/scenarios). No ROS."""
import math

from swarm_backend.config import DetectConfig
from swarm_backend.detect.camera import CameraModel
from swarm_backend.detect.frame import aimed_camera, make_frame
from swarm_backend.detect.scenario import DetectScenario
from swarm_backend.intercept.scenario import Scenario
from swarm_backend.px4.drone import Drone


class SimDrone(Drone):
    """Flies like a speed- and acceleration-limited position controller with
    feed-forward, which is roughly what PX4 does with our setpoints. A velocity
    setpoint is followed directly, within the same limits. The nose turns to a commanded
    yaw at once, and the gimbal points where it is aimed at once."""

    MAX_SPEED = 12.0
    MAX_ACCEL = 4.0
    GAIN = 1.0

    def __init__(self, drone_id, position, max_speed=MAX_SPEED, max_accel=MAX_ACCEL):
        super().__init__(drone_id, home=(position[0], position[1], 0.0))
        self.position = position
        self.velocity = (0.0, 0.0, 0.0)
        self.max_speed = max_speed
        self.max_accel = max_accel
        self._feed_forward = (0.0, 0.0, 0.0)
        self._velocity_setpoint = None
        self.gimbal_aim = None
        self.yaw = 0.0  # radians from north
        self.attitude = (1.0, 0.0, 0.0, 0.0)

    def _turn(self, yaw):
        if yaw is not None:
            self.yaw = yaw
            self.attitude = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))

    def set_position(self, x, y, z, yaw=None, velocity=None):
        self._turn(yaw)
        self.target = (x, y, z)
        self._feed_forward = velocity or (0.0, 0.0, 0.0)
        self._velocity_setpoint = None

    def set_velocity(self, vx, vy, vz, yaw=None):
        self._turn(yaw)
        self.target = None
        self._velocity_setpoint = (vx, vy, vz)

    def hold(self):
        super().hold()
        self._feed_forward = (0.0, 0.0, 0.0)
        self._velocity_setpoint = None

    def aim_gimbal(self, pitch, yaw):
        self.gimbal_aim = (pitch, yaw)

    def release_gimbal(self):
        self.gimbal_aim = None

    def advance(self, dt):
        if self._velocity_setpoint is not None:
            wanted = list(self._velocity_setpoint)
        elif self.target is not None:
            wanted = [f + self.GAIN * (t - p)
                      for f, t, p in zip(self._feed_forward, self.target, self.position)]
        else:
            return
        wanted = _limit(wanted, self.max_speed)
        change = _limit([w - v for w, v in zip(wanted, self.velocity)], self.max_accel * dt)
        self.velocity = tuple(v + c for v, c in zip(self.velocity, change))
        self.position = tuple(p + v * dt for p, v in zip(self.position, self.velocity))

    def arm(self): ...
    def disarm(self): ...
    def start_offboard(self): ...
    def land(self): ...


class FastSim:
    """The two drones of an intercept scenario, at its start, on a shared clock: a Sim
    for scenario.run_scenario. The hostile drone already flies at its speed, and is
    agile enough to follow its track; the interceptor hovers."""

    def __init__(self, scenario: Scenario, interceptor_max_speed=SimDrone.MAX_SPEED):
        self.hostile = SimDrone(0, scenario.target_start, max_speed=scenario.speed + 5,
                                max_accel=20.0)
        self.hostile.velocity = scenario.velocity
        self.interceptor = SimDrone(1, scenario.interceptor_start,
                                    max_speed=interceptor_max_speed)
        self.time = 0.0

    def now(self):
        return self.time

    def advance(self, dt):
        self.hostile.advance(dt)
        self.interceptor.advance(dt)
        self.time += dt


def _limit(vector, limit):
    norm = math.hypot(*vector)
    return list(vector) if norm <= limit else [v * limit / norm for v in vector]


class DetectFastSim:
    """The world of a detection scenario on a clock, with pinhole cameras on the
    defenders: a DetectSim for detect.scenario.run_scenario. No images."""

    def __init__(self, scenario: DetectScenario, config: DetectConfig):
        self.scenario = scenario
        self.camera = CameraModel(config.camera_hfov, config.image_width, config.image_height)
        self.time = 0.0

    def now(self):
        return self.time

    def advance(self, dt):
        self.time += dt

    def bodies(self):
        return self.scenario.bodies(self.time)

    def frame(self, drone, pitch_deg, yaw_deg):
        pose = aimed_camera(self.scenario.defenders[drone], pitch_deg, yaw_deg)
        return make_frame(self.camera, self.time, drone, pose, self.bodies())
