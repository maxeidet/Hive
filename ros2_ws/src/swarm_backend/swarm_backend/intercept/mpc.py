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

The matrix in front of E only depends on dt, K, Q and R, so it is computed once per K
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
    `time_to_go` the horizon is the time until the meeting, at most `horizon_s`;
    without, it is always `horizon_s`.
    """
    dt: float = 0.1  # seconds per step: the harness steps at 10 Hz
    horizon_s: float = 15.0
    tracking: float = 0.0
    terminal: float = 1000.0
    effort: float = 1.0
    time_to_go: bool = True

    @property
    def max_steps(self) -> int:
        """The longest horizon, in steps."""
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


class Mpc:
    """The controller. Build it once, then call first_acceleration() every step."""

    def __init__(self, weights: MpcWeights = MpcWeights(), max_speed: float = 12.0):
        """`max_speed` is what the interceptor can fly, for the time to go."""
        self.weights = weights
        self.max_speed = max_speed
        self._first_rows: dict[int, np.ndarray] = {}  # K -> first row of the gain

    def horizon_steps(self, position: Vector3, target: Vector3,
                      target_velocity: Vector3) -> int:
        """K for this step: the time to go (or, if the target gets away, the time it is
        closest), in steps, within 1..max_steps; max_steps with a fixed horizon."""
        w = self.weights
        if not w.time_to_go:
            return w.max_steps
        t = time_to_go(position, target, target_velocity, self.max_speed)
        if t is None:
            t = closest_approach_time(position, target, target_velocity)
        return max(1, min(w.max_steps, round(t / w.dt)))

    def gain(self, steps: int) -> np.ndarray:
        """(K, K): U* = gain @ E. Not cached; first_acceleration only needs row 0."""
        w = self.weights
        G = prediction_matrix(steps, w.dt)
        q = np.full(steps, w.tracking)
        q[-1] += w.terminal
        # Solving instead of inverting is the numerically safe way to get the same
        # matrix. G'QG + R is positive definite for R > 0, so it is always solvable.
        return np.linalg.solve(G.T @ (q[:, None] * G) + w.effort * np.eye(steps),
                               G.T * q[None, :])

    def plan(self, position: Vector3, velocity: Vector3, target: Vector3,
             target_velocity: Vector3) -> np.ndarray:
        """(K, 3) accelerations, NED m/s^2: the whole optimal plan from here, unlimited.
        Only row 0 is flown; the rest shows what the controller intends."""
        k = self.horizon_steps(position, target, target_velocity)
        return self.gain(k) @ self._error(position, velocity, target, target_velocity, k)

    def first_acceleration(self, position: Vector3, velocity: Vector3, target: Vector3,
                           target_velocity: Vector3) -> Vector3:
        """u(1), the only acceleration that is flown, unlimited."""
        k = self.horizon_steps(position, target, target_velocity)
        row = self._first_rows.get(k)
        if row is None:
            row = self._first_rows[k] = self.gain(k)[0]
        u = row @ self._error(position, velocity, target, target_velocity, k)
        return tuple(float(a) for a in u)

    def trajectory(self, position: Vector3, velocity: Vector3,
                   accelerations: np.ndarray) -> np.ndarray:
        """(K, 3) positions the interceptor flies through with these accelerations (e.g.
        plan()'s), for plotting or logging."""
        k = len(accelerations)
        return (free_positions(position, velocity, k, self.weights.dt)
                + prediction_matrix(k, self.weights.dt) @ accelerations)

    def _error(self, position, velocity, target, target_velocity, steps) -> np.ndarray:
        """E = P_target - P_free, (K, 3)."""
        dt = self.weights.dt
        return (free_positions(target, target_velocity, steps, dt)
                - free_positions(position, velocity, steps, dt))


def clip_norm(vector: Vector3, limit: float) -> Vector3:
    """`vector` scaled down to length `limit` if it is longer, direction kept."""
    norm = math.hypot(*vector)
    if norm <= limit or norm == 0.0:
        return tuple(vector)
    return tuple(c * limit / norm for c in vector)


def velocity_command(velocity: Vector3, acceleration: Vector3, dt: float, max_accel: float,
                     max_speed: float) -> Vector3:
    """The velocity setpoint that applies `acceleration` for one step: PX4 (and the fast
    simulation) follow velocity setpoints, so u(1) is sent as v + u(1) dt. The
    acceleration is first limited to `max_accel`, the result to `max_speed`."""
    a = clip_norm(acceleration, max_accel)
    return clip_norm(tuple(vi + ai * dt for vi, ai in zip(velocity, a)), max_speed)
