"""Turns target measurements into a position, velocity and uncertainty.

A constant-velocity Kalman filter over the horizontal state (x, y, vx, vy). Radar
returns go in as they are measured (range, azimuth, radial velocity: an extended
Kalman filter), so the filter knows a radar's error is a long thin ellipse across
its line of sight (1 degree at 2 km is 35 m) but only metres along it. Position
fixes from a drone's own sensors go in directly.

The target flies at a known, fixed altitude for now, so height is not estimated
(the radar's elevation, 2 degrees, would be far worse than knowing it). The base
station's fusion (pair 1) replaces this later. Pure math, world NED.
"""
import math

import numpy as np

from swarm_backend.config import Vector3
from swarm_backend.intercept.target import Fix, PositionFix, RadarFix

ACCEL_NOISE = 1.5  # m/s^2: how hard the target is assumed to manoeuvre
INITIAL_SPEED_STD = 15.0  # m/s: velocity we cannot see yet when a track starts
RESET_DISTANCE2 = 100.0  # squared Mahalanobis distance of a fix that restarts the track


class KalmanTracker:

    def __init__(self, target_z: float, accel_noise: float = ACCEL_NOISE):
        """`target_z` is the target's known height (world NED z)."""
        self.target_z = target_z
        self.accel_noise = accel_noise
        self.state: np.ndarray | None = None  # x, y, vx, vy at `time`
        self.cov: np.ndarray | None = None
        self.time: float | None = None  # of the last fix
        self.fixes = 0  # since the last reset

    # --- input --------------------------------------------------------------

    def update(self, fix: Fix) -> None:
        if self.state is not None and fix.time < self.time:
            return  # older than what we have
        if isinstance(fix, RadarFix):
            update, start = self._update_radar, self._start_radar
        elif isinstance(fix, PositionFix):
            update, start = self._update_position, self._start_position
        else:
            raise TypeError(f'Cannot use a {type(fix).__name__}')
        if self.state is None:
            start(fix)
        else:
            state, cov = self._predict(fix.time - self.time)
            result = update(fix, state, cov)
            if result is None:
                start(fix)  # far from where we thought: a new track
            else:
                self.state, self.cov = result
                self.fixes += 1
        self.time = fix.time

    # --- output -------------------------------------------------------------

    def estimate(self, t: float) -> tuple[Vector3, Vector3] | None:
        """(position, velocity) predicted to time `t`, or None before the first fix."""
        if self.state is None:
            return None
        x, y, vx, vy = self._predict(max(0.0, t - self.time))[0]
        return (float(x), float(y), self.target_z), (float(vx), float(vy), 0.0)

    def position_std(self, t: float) -> float:
        """Horizontal position uncertainty at `t` along its worst direction (1 sigma, m)."""
        if self.state is None:
            return math.inf
        cov = self._predict(max(0.0, t - self.time))[1]
        return math.sqrt(max(np.linalg.eigvalsh(cov[:2, :2])))

    def age(self, t: float) -> float:
        """Seconds since the last fix (infinite before the first)."""
        return math.inf if self.time is None else t - self.time

    # --- the filter ---------------------------------------------------------

    def _predict(self, dt: float) -> tuple[np.ndarray, np.ndarray]:
        f = np.eye(4)
        f[0, 2] = f[1, 3] = dt
        # White-noise acceleration: the standard constant-velocity process noise.
        q = self.accel_noise ** 2
        block = q * np.array([[dt ** 3 / 3, dt ** 2 / 2], [dt ** 2 / 2, dt]])
        noise = np.zeros((4, 4))
        noise[np.ix_([0, 2], [0, 2])] = block
        noise[np.ix_([1, 3], [1, 3])] = block
        return f @ self.state, f @ self.cov @ f.T + noise

    def _correct(self, state, cov, innovation, h, r):
        s = h @ cov @ h.T + r
        if innovation @ np.linalg.solve(s, innovation) > RESET_DISTANCE2:
            return None
        gain = cov @ h.T @ np.linalg.inv(s)
        return state + gain @ innovation, (np.eye(4) - gain @ h) @ cov

    def _update_radar(self, fix: RadarFix, state, cov):
        dx, dy = state[0] - fix.radar[0], state[1] - fix.radar[1]
        dz = self.target_z - fix.radar[2]
        horizontal2 = max(dx * dx + dy * dy, 1e-6)
        rng = math.sqrt(horizontal2 + dz * dz)
        vx, vy = state[2], state[3]
        closing = dx * vx + dy * vy
        predicted = np.array([rng, math.atan2(dy, dx), closing / rng])
        h = np.array([
            [dx / rng, dy / rng, 0, 0],
            [-dy / horizontal2, dx / horizontal2, 0, 0],
            [vx / rng - closing * dx / rng ** 3, vy / rng - closing * dy / rng ** 3,
             dx / rng, dy / rng],
        ])
        measured = np.array([fix.range, fix.azimuth, fix.radial_velocity])
        innovation = measured - predicted
        innovation[1] = (innovation[1] + math.pi) % (2 * math.pi) - math.pi
        r = np.diag([fix.range_std ** 2, fix.azimuth_std ** 2, fix.radial_velocity_std ** 2])
        return self._correct(state, cov, innovation, h, r)

    def _update_position(self, fix: PositionFix, state, cov):
        h = np.array([[1.0, 0, 0, 0], [0, 1.0, 0, 0]])
        innovation = np.array(fix.position[:2]) - state[:2]
        return self._correct(state, cov, innovation, h, fix.std ** 2 * np.eye(2))

    def _start_radar(self, fix: RadarFix) -> None:
        dz = self.target_z - fix.radar[2]
        horizontal = math.sqrt(max(fix.range ** 2 - dz ** 2, 0.0))
        along = np.array([math.cos(fix.azimuth), math.sin(fix.azimuth)])
        across = np.array([-along[1], along[0]])
        rotation = np.column_stack([along, across])
        position = np.array(fix.radar[:2]) + horizontal * along
        velocity = fix.radial_velocity * along  # the part across the line of sight is unknown
        cov = np.zeros((4, 4))
        cov[:2, :2] = rotation @ np.diag(
            [fix.range_std ** 2, (horizontal * fix.azimuth_std) ** 2]) @ rotation.T
        cov[2:, 2:] = rotation @ np.diag(
            [fix.radial_velocity_std ** 2, INITIAL_SPEED_STD ** 2]) @ rotation.T
        self._start(np.concatenate([position, velocity]), cov)

    def _start_position(self, fix: PositionFix) -> None:
        cov = np.diag([fix.std ** 2] * 2 + [INITIAL_SPEED_STD ** 2] * 2)
        self._start(np.array([fix.position[0], fix.position[1], 0.0, 0.0]), cov)

    def _start(self, state: np.ndarray, cov: np.ndarray) -> None:
        self.state, self.cov = state, cov
        self.fixes = 1
