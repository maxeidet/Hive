"""The MPC as an intercept method for the evaluation harness (intercept/scenario.py).

mpc.py is the calculator: given the interceptor's and the target's position and
velocity it returns one acceleration. MpcIntercept is the pilot around it. Every step
it
  1. reads the detector, which only gives the target's position, noisy and not always,
  2. smooths those positions with a Kalman filter that also estimates the velocity the
     camera never measures (TargetTracker3D),
  3. far out, flies at full speed toward the predicted meeting point (midcourse);
     in the last `terminal_s` seconds before the meeting, asks the MPC for u(1),
  4. sends it as a velocity setpoint, within the drone's limits,
  5. keeps the gimbal on the estimate, and reports LOST when the target has not been
     seen for `track_timeout`.

Why not the MPC all the way: it minimises acceleration, so with a long time to go it
plans a slow start and a fast finish, and the finish can need more than the drone's
top speed (the limit is applied after the optimisation, not in it). It then falls
behind and a target it could have met gets away: at 60 deg and 12 m/s from 700 m it
reaches only 4 m/s in the first 10 s and misses, though flying straight at full speed
would meet the target with 10 s to spare. Full speed to the meeting point is the
fastest way there; the MPC is best at the end, where it fixes the last metres.
terminal_s = None flies the MPC all the way, for comparison.

Between fixes, and in the blind zone close in, it keeps steering on the filter's
prediction instead of a frozen velocity (the baseline's COAST). No ROS: it only uses the
Drone interface and a Detector, so it runs against the fast simulation in tests.
"""
import math
from collections.abc import Callable

import numpy as np

from swarm_backend.config import InterceptConfig, Vector3
from swarm_backend.intercept.interceptor import InterceptAction, Status
from swarm_backend.intercept.mpc import Mpc, MpcWeights, clip_norm, time_to_go, velocity_command
from swarm_backend.intercept.target import PositionFix
from swarm_backend.intercept.target_detector import Detector
from swarm_backend.intercept.tracker import ACCEL_NOISE, INITIAL_SPEED_STD, RESET_DISTANCE2
from swarm_backend.px4.drone import Drone, pitch_yaw_deg, wrap_deg

# What the interceptor can do. The fast simulation's drone accelerates at 4 m/s^2;
# PX4's default horizontal limit (MPC_ACC_HOR) is 3, so in Gazebo the drone simply
# lags a little, and the MPC re-plans from its measured velocity every step.
MAX_ACCEL = 4.0  # m/s^2
MIN_ALTITUDE = 10.0  # metres above the start point: never descend below this
MIN_DIRECTION_M = 1e-3  # closer than this the target has no direction to face
TERMINAL_S = 5.0  # seconds before the meeting where the MPC takes over


class TargetTracker3D:
    """A constant-velocity Kalman filter over (x, y, z, vx, vy, vz), world NED, fed with
    PositionFixes. The same filter as tracker.KalmanTracker, with height estimated
    instead of known: the hostile flies anywhere between 20 and 120 m."""

    def __init__(self, accel_noise: float = ACCEL_NOISE):
        self.accel_noise = accel_noise
        self.state: np.ndarray | None = None
        self.cov: np.ndarray | None = None
        self.time: float | None = None  # of the last fix

    def update(self, fix: PositionFix) -> None:
        z = np.asarray(fix.position, float)
        r = fix.std ** 2 * np.eye(3)
        if self.state is None:
            self._start(fix)
            return
        if fix.time < self.time:
            return  # older than what we have
        state, cov = self._predict(fix.time - self.time)
        h = np.hstack([np.eye(3), np.zeros((3, 3))])  # we measure position only
        innovation = z - h @ state
        s = h @ cov @ h.T + r
        if innovation @ np.linalg.solve(s, innovation) > RESET_DISTANCE2:
            self._start(fix)  # far from where we thought: a new track
            return
        gain = cov @ h.T @ np.linalg.inv(s)
        self.state = state + gain @ innovation
        self.cov = (np.eye(6) - gain @ h) @ cov
        self.time = fix.time

    def estimate(self, t: float) -> tuple[Vector3, Vector3] | None:
        """(position, velocity) predicted to time `t`, or None before the first fix."""
        if self.state is None:
            return None
        s = self._predict(max(0.0, t - self.time))[0]
        return tuple(float(c) for c in s[:3]), tuple(float(c) for c in s[3:])

    def age(self, t: float) -> float:
        """Seconds since the last fix (infinite before the first)."""
        return math.inf if self.time is None else t - self.time

    def _predict(self, dt: float) -> tuple[np.ndarray, np.ndarray]:
        f = np.eye(6)
        f[:3, 3:] = dt * np.eye(3)
        # White-noise acceleration, per axis: the standard constant-velocity model.
        q = self.accel_noise ** 2 * np.array([[dt ** 3 / 3, dt ** 2 / 2], [dt ** 2 / 2, dt]])
        noise = np.kron(q, np.eye(3))  # (x, y, z) block, then (vx, vy, vz)
        return f @ self.state, f @ self.cov @ f.T + noise

    def _start(self, fix: PositionFix) -> None:
        self.state = np.concatenate([np.asarray(fix.position, float), np.zeros(3)])
        self.cov = np.diag([fix.std ** 2] * 3 + [INITIAL_SPEED_STD ** 2] * 3)
        self.time = fix.time


class MpcIntercept(InterceptAction):
    """Planning report, Interception > Guidance Laws: the MPC (intercept/mpc.py) on the
    tracker's estimate of the target. `weights` chooses the variant; the defaults are
    the minimum-effort intercept, MpcWeights(tracking=1, terminal=0, time_to_go=False)
    is Zheng et al.'s. `terminal_s` is when the MPC takes over from the midcourse
    (None: the MPC all the way). For the harness's factory: functools.partial(
    MpcIntercept, weights=..., terminal_s=...).

    For flying on the camera in Gazebo (nodes/attack_node.py --guidance mpc):
    `search` is where the gimbal looks before the target has been seen, (degrees above
    the horizon, degrees from north), as the baseline's SEARCH; once seen and lost, it
    looks where the target was last. `limit` replaces the height check on every
    velocity command, e.g. the baseline's attack.limit_velocity, so both fly within the
    same limits."""

    def __init__(self, drone: Drone, detector: Detector, config: InterceptConfig,
                 clock: Callable[[], float | None], weights: MpcWeights = MpcWeights(),
                 terminal_s: float | None = TERMINAL_S,
                 search: tuple[float, float] | None = None,
                 limit: Callable[[Vector3], Vector3] | None = None):
        super().__init__(drone, detector, config, clock)
        self.mpc = Mpc(weights, max_speed=config.interceptor_max_speed)
        self.terminal_s = terminal_s
        self.search = search
        self.limit = limit
        self.phase: str | None = None  # 'midcourse' or 'terminal', for logging
        self.transitions: list[tuple[float, str]] = []  # (time, what it does now)
        self._last_target: Vector3 | None = None  # last estimated target position
        self.tracker = TargetTracker3D()
        self.running = False
        self.estimate: tuple[Vector3, Vector3] | None = None  # last (position, velocity) used
        self.commanded: Vector3 | None = None  # last velocity setpoint sent
        self._last_fix: PositionFix | None = None
        self._yaw: float | None = None  # radians from north, last commanded
        self._gimbal = config.use_gimbal
        self._status = Status.IDLE

    @property
    def status(self) -> Status:
        return self._status

    def start(self) -> None:
        self.running = True
        self._status = Status.LOST  # until the first fix; the harness only ends a run
        #                             on LOST once the target has been detected
        self._note(self.clock(), 'search')

    def stop(self) -> None:
        """Hover where we are and stop moving the gimbal."""
        if not self.running:
            return
        self.running = False
        self._status = Status.IDLE
        self.drone.hold()
        if self._gimbal:
            self.drone.release_gimbal()

    def step(self) -> None:
        if not self.running or self.drone.position is None or self.drone.velocity is None:
            return
        now = self.clock()
        if now is None:
            return

        # 1-2. New detection -> filter.
        fix = self.detector.latest()
        if fix is not None and fix is not self._last_fix:
            self._last_fix = fix
            self.tracker.update(fix)
        if self.tracker.age(now) > self.config.track_timeout:
            if self._status == Status.TRACKING:
                self.drone.hold()  # lost it: stop rather than fly on a stale guess
                self._note(now, 'lost')
            self._status = Status.LOST
            self.estimate = None
            self._look_for_target()
            return
        self._status = Status.TRACKING
        self.estimate = self.tracker.estimate(now)  # predicted to now, also between fixes
        target, target_velocity = self.estimate
        self._last_target = target

        # 3-4. Midcourse or MPC -> velocity setpoint.
        me = self.drone.position
        speed = self.config.interceptor_max_speed
        t_go = time_to_go(me, target, target_velocity, speed)
        if self.terminal_s is not None and t_go is not None and t_go > self.terminal_s:
            self.phase = 'midcourse'
            meeting = tuple(p + v * t_go for p, v in zip(target, target_velocity))
            wanted = tuple(speed * c for c in _unit(tuple(m - p for m, p in zip(meeting, me))))
            change = clip_norm(tuple(w - v for w, v in zip(wanted, self.drone.velocity)),
                               MAX_ACCEL * self.mpc.weights.dt)
            velocity = tuple(v + c for v, c in zip(self.drone.velocity, change))
        else:
            self.phase = 'terminal'
            u = self.mpc.first_acceleration(me, self.drone.velocity, target, target_velocity)
            velocity = velocity_command(self.drone.velocity, u, self.mpc.weights.dt,
                                        MAX_ACCEL, speed)
        self._note(now, self.phase)
        self.commanded = self.limit(velocity) if self.limit else self._keep_height(velocity)
        dx, dy = target[0] - me[0], target[1] - me[1]
        if math.hypot(dx, dy) > MIN_DIRECTION_M:
            self._yaw = math.atan2(dy, dx)  # face the target, so the gimbal turns little
        self.drone.set_velocity(*self.commanded, yaw=self._yaw)

        # 5. Gimbal on the estimate.
        self._aim_gimbal(me, target)

    def _keep_height(self, velocity: Vector3) -> Vector3:
        """No descent below MIN_ALTITUDE (NED: positive vz is down)."""
        altitude = self.drone.altitude
        if altitude is not None and altitude <= MIN_ALTITUDE and velocity[2] > 0:
            return (velocity[0], velocity[1], 0.0)
        return velocity

    def _note(self, now: float | None, what: str) -> None:
        """Record a change of what the action does (search, midcourse, terminal, lost)."""
        if now is not None and (not self.transitions or self.transitions[-1][1] != what):
            self.transitions.append((now, what))

    def _look_for_target(self) -> None:
        """No estimate: look where the target was last, or else the search direction."""
        me = self.drone.position
        if self._last_target is not None:
            self._aim_gimbal(me, self._last_target)
        elif self.search is not None:
            self._aim_world(*self.search)

    def _aim_gimbal(self, me: Vector3, target: Vector3) -> None:
        dx, dy, dz = (t - m for t, m in zip(target, me))
        self._aim_world(math.degrees(math.atan2(-dz, math.hypot(dx, dy))),
                        math.degrees(math.atan2(dy, dx)))

    def _aim_world(self, pitch: float, heading_deg: float) -> None:
        """Point the gimbal `pitch` degrees above the horizon, `heading_deg` from north."""
        if not self._gimbal:
            return
        heading = (pitch_yaw_deg(self.drone.attitude)[1] if self.drone.attitude is not None
                   else math.degrees(self._yaw or 0.0))
        try:
            self.drone.aim_gimbal(pitch, wrap_deg(heading_deg - heading))
        except NotImplementedError:
            self._gimbal = False


def _unit(vector: Vector3) -> Vector3:
    norm = math.hypot(*vector)
    return (0.0, 0.0, 0.0) if norm == 0.0 else tuple(c / norm for c in vector)
