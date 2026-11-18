"""Model predictive control for the intercept (planning report, Interception >
Guidance Laws, after Zheng et al.).

Every step the controller
  1. predicts the target K steps ahead, assuming it keeps its velocity,
  2. chooses the accelerations u(1)..u(K) that bring the interceptor to those predicted
     positions, without using more acceleration than needed,
  3. returns only u(1); the caller applies it and asks again next step with fresh
     estimates (a receding horizon), so a wrong prediction is corrected every step.

The interceptor is a double integrator per axis, with the acceleration u(k) held
constant over each step of length dt:

    v(k+1) = v(k) + u(k) dt
    p(k+1) = p(k) + v(k) dt + 1/2 u(k) dt^2

Unrolled over the horizon, its positions are linear in the accelerations,

    P = P_free + G U,

where P_free is where it would coast with no acceleration and G (K x K, lower
triangular) says how much u(j) moves p(k). With E = P_target - P_free the cost of the
report,

    J = sum_k ||p_target(k) - p(k)||^2_Q + ||u(k)||^2_R = (E - G U)' Q (E - G U) + U' R U,

is a least-squares problem with the closed-form minimum

    U* = (G' Q G + R)^-1 G' Q E.

The matrix in front of E only depends on dt, K, Q and R, so it can be computed once
and cached; a step is then one matrix product. The three axes are independent and use
the same matrix. Unlike Zheng et al., who keep an offset around the target to fire a
net, the offset is zero: we aim at the target itself.

Two choices decide whether it catches a target faster than itself (v_h > v_i):

  Q   Zheng et al. weight every step of the horizon (`tracking`). That asks to stay
      near the target all along, which a faster target makes impossible, and the best
      compromise is to fly toward where the target is now: pure pursuit again. Weighting
      only the last step (`terminal`) asks instead to *be at the target at the end*
      with the least acceleration on the way.
  K   With a fixed horizon the plan ends at the target's position horizon_s ahead,
      even when the two will meet much sooner, so close in it aims far behind the
      target and misses. With `time_to_go` the horizon ends where they meet.

The time to go can be a minute, 600 steps of 0.1 s, which is too much to solve every
step. So with `time_to_go` the plan has at most `plan_steps` steps, stretched to cover
the whole time to go: coarse while the target is far, where precision does not matter,
and back to dt per step in the last plan_steps x dt seconds, where it does. Only the
first acceleration is flown anyway, for one control step, before planning again.

Final step only with the horizon at the time to go is the minimum-effort intercept,
which is known to be proportional navigation with N = 3 (Zarchan); the defaults use
it. tracking > 0, terminal = 0 and time_to_go = False is the MPC as in Zheng et al.

The limits (acceleration, speed) are not part of the optimisation; they are applied
to the result (velocity_command). Pure math, world NED, no ROS.
"""
import math
from dataclasses import dataclass

import numpy as np

from swarm_backend.config import Vector3


@dataclass(frozen=True)
class MpcWeights:
    """What the controller trades off (see the module docstring for why).

    `tracking` (Q) is the cost per square metre of missing a predicted position, at
    every step; `terminal` is added at the last step only. `effort` (R) is the cost per
    (m/s^2)^2 of acceleration: higher is smoother but slower to react. With
    `time_to_go` the horizon is the time until the meeting, planned in at most
    `plan_steps` steps; without, it is `horizon_s` in steps of dt.
    """
    dt: float = 0.1  # seconds per control step: the harness steps at 10 Hz
    horizon_s: float = 8.0  # fixed horizon (time_to_go = False)
    tracking: float = 0.0
    terminal: float = 1000.0
    effort: float = 1.0
    time_to_go: bool = True
    plan_steps: int = 50  # time_to_go: steps in the plan, at most
    max_time_to_go: float = 300.0  # seconds: never plan further ahead than this

    @property
    def fixed_steps(self) -> int:
        """The fixed horizon, in steps of dt."""
        return max(1, round(self.horizon_s / self.dt))


def prediction_matrix(steps: int, dt: float) -> np.ndarray:
    """G: row k-1 gives the position at step k from the accelerations u(0)..u(K-1).

    u(j) acts for the whole of step j and then keeps adding its velocity change, so at
    step k > j it has moved the position by dt^2 (k - j - 1/2)."""
    k = np.arange(1, steps + 1)[:, None]  # row: the step whose position we want, 1..K
    j = np.arange(steps)[None, :]  # column: the step whose acceleration acts
    return np.where(j < k, dt * dt * (k - j - 0.5), 0.0)


def free_positions(position: Vector3, velocity: Vector3, steps: int,
                   dt: float) -> np.ndarray:
    """(K, 3): where a body at `position` with `velocity` is after 1..K steps, coasting.
    For the interceptor this is P_free; for the target it is the constant-velocity
    prediction."""
    t = dt * np.arange(1, steps + 1)[:, None]
    return np.asarray(position, float)[None, :] + t * np.asarray(velocity, float)[None, :]


def time_to_go(position: Vector3, target: Vector3, target_velocity: Vector3,
               speed: float) -> float | None:
    """Seconds until a target at constant velocity can first be met by flying straight
    at `speed`: the smallest t > 0 with |target + target_velocity t - position| = speed t.
    None if there is none (it gets away). Ignores the time to turn and accelerate."""
    r = np.subtract(target, position)
    w = np.asarray(target_velocity, float)
    a = w @ w - speed * speed  # > 0 when the target is faster than us
    b = 2.0 * (r @ w)
    c = r @ r
    if c == 0.0:
        return 0.0
    if abs(a) < 1e-9:  # same speed: the quadratic is linear
        return -c / b if b < 0 else None
    disc = b * b - 4 * a * c
    if disc < 0:
        return None
    roots = [(-b - math.sqrt(disc)) / (2 * a), (-b + math.sqrt(disc)) / (2 * a)]
    positive = [t for t in roots if t > 0]
    return min(positive) if positive else None


def closest_approach_time(position: Vector3, target: Vector3,
                          target_velocity: Vector3) -> float:
    """Seconds until a target at constant velocity is closest to `position` (0 if it is
    already moving away)."""
    r = np.subtract(target, position)
    w = np.asarray(target_velocity, float)
    speed2 = w @ w
    return 0.0 if speed2 == 0.0 else max(0.0, -(r @ w) / speed2)


@dataclass(frozen=True)
class Plan:
    """The optimal accelerations from here, (K, 3) NED m/s^2, each held for `dt`."""
    accelerations: np.ndarray
    dt: float


class Mpc:
    """The controller. Build it once, then call first_acceleration() every step."""

    def __init__(self, weights: MpcWeights = MpcWeights(), max_speed: float = 12.0):
        """`max_speed` is what the interceptor can fly, for the time to go."""
        self.weights = weights
        self.max_speed = max_speed
        self._first_rows: dict[int, np.ndarray] = {}  # K -> first gain row, for steps of dt

    def horizon(self, position: Vector3, target: Vector3,
                target_velocity: Vector3) -> tuple[int, float]:
        """(K, seconds per step) for this step. With time_to_go the plan ends at the
        meeting (or, if the target gets away, where it is closest)."""
        w = self.weights
        if not w.time_to_go:
            return w.fixed_steps, w.dt
        t = time_to_go(position, target, target_velocity, self.max_speed)
        if t is None:
            t = closest_approach_time(position, target, target_velocity)
        t = min(max(t, w.dt), w.max_time_to_go)
        if t <= w.plan_steps * w.dt:
            return max(1, round(t / w.dt)), w.dt
        return w.plan_steps, t / w.plan_steps

    def gain(self, steps: int, dt: float) -> np.ndarray:
        """(K, K): U* = gain @ E."""
        w = self.weights
        G = prediction_matrix(steps, dt)
        q = np.full(steps, w.tracking)
        q[-1] += w.terminal
        # Solving instead of inverting is the numerically safe way to get the same
        # matrix. G'QG + R is positive definite for R > 0, so it is always solvable.
        return np.linalg.solve(G.T @ (q[:, None] * G) + w.effort * np.eye(steps),
                               G.T * q[None, :])

    def plan(self, position: Vector3, velocity: Vector3, target: Vector3,
             target_velocity: Vector3) -> Plan:
        """The whole optimal plan from here, unlimited. Only its first acceleration is
        flown; the rest shows what the controller intends."""
        k, dt = self.horizon(position, target, target_velocity)
        error = self._error(position, velocity, target, target_velocity, k, dt)
        return Plan(self.gain(k, dt) @ error, dt)

    def first_acceleration(self, position: Vector3, velocity: Vector3, target: Vector3,
                           target_velocity: Vector3) -> Vector3:
        """u(1), the only acceleration that is flown, unlimited."""
        k, dt = self.horizon(position, target, target_velocity)
        if dt == self.weights.dt:  # the gain only depends on K: cache it
            row = self._first_rows.get(k)
            if row is None:
                row = self._first_rows[k] = self.gain(k, dt)[0]
        else:  # stretched steps: K <= plan_steps, cheap to solve every time
            row = self.gain(k, dt)[0]
        u = row @ self._error(position, velocity, target, target_velocity, k, dt)
        return tuple(float(a) for a in u)

    @staticmethod
    def trajectory(position: Vector3, velocity: Vector3, plan: Plan) -> np.ndarray:
        """(K, 3) positions the interceptor flies through with `plan`, for plotting or
        logging."""
        k = len(plan.accelerations)
        return (free_positions(position, velocity, k, plan.dt)
                + prediction_matrix(k, plan.dt) @ plan.accelerations)

    @staticmethod
    def _error(position, velocity, target, target_velocity, steps, dt) -> np.ndarray:
        """E = P_target - P_free, (K, 3)."""
        return (free_positions(target, target_velocity, steps, dt)
                - free_positions(position, velocity, steps, dt))


def clip_norm(vector: Vector3, limit: float) -> Vector3:
    """`vector` scaled down to length `limit` if it is longer, direction kept."""
    norm = math.hypot(*vector)
    if norm <= limit or norm == 0.0:
        return tuple(vector)
    return tuple(c * limit / norm for c in vector)


# PX4's velocity controller accelerates by gain x (setpoint - velocity): MPC_XY_VEL_P_ACC
# (north, east) and MPC_Z_VEL_P_ACC (down), the defaults, read from the SITL drones.
PX4_VELOCITY_GAIN = (1.8, 1.8, 4.0)  # 1/s


def velocity_command(velocity: Vector3, acceleration: Vector3, dt: float, max_accel: float,
                     max_speed: float, gain: Vector3 | None = PX4_VELOCITY_GAIN) -> Vector3:
    """The velocity setpoint that makes the drone accelerate by `acceleration` (u(1)),
    limited to `max_accel`, the setpoint to `max_speed`.

    PX4 only takes velocity setpoints and accelerates by gain x (setpoint - velocity), so
    the setpoint leads the measured velocity by u / gain on each axis. (Sending v + u dt
    instead, as before 11 Nov 2026, gave PX4 a lead of at most 0.4 m/s: it accelerated at
    about 0.7 m/s^2 instead of 4, and the MPC took 5-18 s to reach 4 m/s where pure
    pursuit took 1 s.) gain=None is a drone that reaches each setpoint within the step
    (v + u dt), for a model that follows setpoints directly."""
    a = clip_norm(acceleration, max_accel)
    lead = (dt, dt, dt) if gain is None else tuple(1.0 / g for g in gain)
    return clip_norm(tuple(vi + ai * k for vi, ai, k in zip(velocity, a, lead)), max_speed)
