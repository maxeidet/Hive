"""Where a tracked box will be in the next image: a constant-velocity Kalman filter on the
box in pixels, one per track, as in ByteTrack.

State (8 numbers): the box centre u, v, its width w and height h, and how fast each of
them changes, du, dv, dw, dh, in px per image. A detection measures u, v, w, h; the
velocities are never measured, the filter works them out from how the box moves.

Each image the tracker calls
  predict()   before matching: moves the box on by its velocity and grows the
              uncertainty (the target may have turned or sped up since),
  update(box) after matching, when a detection was matched to this track: pulls the
              state towards the detection, by how much the filter trusts it compared
              with its own prediction (the Kalman gain).
A track without a matched detection only predicts, so it keeps moving the way it was going
and can be found again a few images later.

The noise is a fraction of the box size, as in ByteTrack (a big box near the camera moves
more px than a small one far away), with a floor in px so that the tiny boxes of far
drones still get a sensible uncertainty. dt = 1 image. Pure numpy, no ROS.
"""
from dataclasses import dataclass

import numpy as np

Box = tuple[float, float, float, float]  # u, v (centre), w, h; px

# state = F @ state: position += velocity (one image later); velocity stays
F = np.eye(8)
F[:4, 4:] = np.eye(4)
# measurement = H @ state: a detection sees the box, not its velocity
H = np.eye(4, 8)


@dataclass(frozen=True)
class Noise:
    """Standard deviations, as a fraction of the box's longer side, with a floor in px."""
    measurement: float = 1 / 20  # how far a detection's box is from the true box
    position: float = 1 / 20  # how much the box can move other than with its velocity, per image
    velocity: float = 1 / 160  # how much the velocity can change, per image
    min_px: float = 1.0  # floor for the three above
    initial_velocity_px: float = 10.0  # velocity std of a new track: not seen yet


class BoxKalman:
    """One track's filter. `mean` is the state, `cov` its uncertainty (covariance)."""

    def __init__(self, box: Box, noise: Noise = Noise()):
        """Starts a track at a detection's box, velocity zero but very uncertain."""
        self.noise = noise
        self.mean = np.array([*box, 0.0, 0.0, 0.0, 0.0])
        position_std = self._std(noise.measurement, box) * 2
        velocity_std = max(noise.initial_velocity_px, self._std(noise.velocity, box) * 10)
        self.cov = np.diag([position_std] * 4 + [velocity_std] * 4) ** 2

    # --- the two steps --------------------------------------------------------------

    def predict(self) -> Box:
        """Moves on one image; returns the predicted box."""
        box = self.box
        q = np.diag([self._std(self.noise.position, box)] * 4
                    + [self._std(self.noise.velocity, box)] * 4) ** 2
        self.mean = F @ self.mean
        self.cov = F @ self.cov @ F.T + q
        return self.box

    def update(self, box: Box) -> Box:
        """Corrects with a matched detection's box; returns the corrected box."""
        r = np.diag([self._std(self.noise.measurement, box)] * 4) ** 2
        s = H @ self.cov @ H.T + r  # uncertainty of the predicted box, as a detection sees it
        gain = self.cov @ H.T @ np.linalg.inv(s)  # 0 = keep the prediction, 1 = take the detection
        innovation = np.asarray(box, dtype=float) - H @ self.mean  # detection - prediction
        self.mean = self.mean + gain @ innovation
        self.cov = (np.eye(8) - gain @ H) @ self.cov
        return self.box

    # --- reading it -----------------------------------------------------------------

    @property
    def box(self) -> Box:
        """The current box (after predict: the prediction), u, v, w, h."""
        u, v, w, h = self.mean[:4]
        return (float(u), float(v), max(1.0, float(w)), max(1.0, float(h)))

    @property
    def velocity(self) -> tuple[float, float]:
        """How fast the centre moves, px per image (du, dv)."""
        return (float(self.mean[4]), float(self.mean[5]))

    @property
    def centre_std(self) -> float:
        """How uncertain the centre is, px (the larger of u's and v's std)."""
        return float(np.sqrt(max(self.cov[0, 0], self.cov[1, 1])))

    def _std(self, fraction: float, box: Box) -> float:
        return max(self.noise.min_px, fraction * max(box[2], box[3]))
