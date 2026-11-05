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
drones still get a sensible uncertainty. dt = 1 image.

When the camera turns (the gimbal, or the drone under it), every target jumps across the
image although it hardly moved in the world. That is not the target's motion, so it must
not end up in the velocity: before predict(), the tracker moves each track from the last
image's pixels to the new image's with compensate(rotation_homography(...)), from the
camera's orientation in both images (PX4's gimbal attitude, orientation.py). This is
BoT-SORT's camera motion compensation, but with the rotation known instead of estimated
from the images. Only the camera's turning is removed; its own flight moves targets
slowly and smoothly, which the velocity follows. Pure numpy, no ROS.
"""
from dataclasses import dataclass

import numpy as np

from swarm_backend.detect.camera import CameraModel
from swarm_backend.detect.orientation import Axes

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
    min_px: float = 3.0  # floor for the three above: covers a noisy detector and the
    # camera orientation error (0.35 degrees ~ 2.5 px); below the real noise, tracks break
    # (track_eval: 3 px noise with a 1 px floor, MOTA 0.76 instead of 0.89)
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

    def compensate(self, homography: np.ndarray) -> bool:
        """Moves the track into the next image's pixels after the camera turned (before
        predict). The centre is mapped exactly; the velocity, size and uncertainty are
        turned and scaled by how the image is stretched around the centre (the
        homography's Jacobian there). False, and nothing changed, if the centre would end
        up behind the camera (the camera turned more than 90 degrees away)."""
        u, v = self.mean[:2]
        x, y, w = homography @ np.array([u, v, 1.0])
        if w <= 1e-9:
            return False
        new_u, new_v = x / w, y / w
        jacobian = (homography[:2, :2] - np.outer([new_u, new_v], homography[2, :2])) / w
        scale = float(np.sqrt(abs(np.linalg.det(jacobian))))
        t = np.zeros((8, 8))
        for at in (0, 4):  # position and velocity are turned the same way
            t[at:at + 2, at:at + 2] = jacobian
            t[at + 2, at + 2] = t[at + 3, at + 3] = scale
        self.mean = t @ self.mean
        self.mean[:2] = new_u, new_v  # exact, not the linearised map
        self.cov = t @ self.cov @ t.T
        return True

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

    def centre_distance(self, u: float, v: float) -> float:
        """How far the centre (u, v) of a detection is from the predicted centre, in standard
        deviations (Mahalanobis distance): the prediction's uncertainty plus a detection's
        own error, so the same px count is far for a sure track and near for an unsure one."""
        box = self.box
        r = np.diag([self._std(self.noise.measurement, box)] * 2) ** 2
        s = self.cov[:2, :2] + r
        d = np.array([u, v]) - self.mean[:2]
        return float(np.sqrt(d @ np.linalg.solve(s, d)))

    @property
    def centre_std(self) -> float:
        """How uncertain the centre is, px (the larger of u's and v's std)."""
        return float(np.sqrt(max(self.cov[0, 0], self.cov[1, 1])))

    def _std(self, fraction: float, box: Box) -> float:
        return max(self.noise.min_px, fraction * max(box[2], box[3]))


def rotation_homography(camera: CameraModel, old: Axes, new: Axes) -> np.ndarray:
    """3x3 matrix taking a pixel of the image taken with orientation `old` to where the same
    world direction shows in the image taken with `new` (only the camera turned: exact for
    any distance). Axes are (forward, right, down), world NED unit vectors.

    pixel -> camera ray (K^-1) -> world (old axes) -> new camera's ray (new axes) -> pixel (K)
    """
    k = np.array([[camera.f, 0.0, camera.cx], [0.0, camera.f, camera.cy], [0.0, 0.0, 1.0]])
    return k @ _to_world(new).T @ _to_world(old) @ np.linalg.inv(k)


def warp(homography: np.ndarray, u: float, v: float) -> tuple[float, float] | None:
    """Where pixel (u, v) goes; None if behind the camera."""
    x, y, w = homography @ np.array([u, v, 1.0])
    return None if w <= 1e-9 else (float(x / w), float(y / w))


def _to_world(axes: Axes) -> np.ndarray:
    """Camera ray (x right, y down, z forward, as K uses) -> world NED."""
    forward, right, down = axes
    return np.column_stack([right, down, forward])
