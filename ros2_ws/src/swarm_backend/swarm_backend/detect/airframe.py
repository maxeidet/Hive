"""Where a drone's own airframe shows in its gimbal camera's picture, so the detector can
ignore it (`detect/live.py --body-mask`).

The gimbal camera of gz_x500_gimbal sees 115 degrees across and hangs just under the body,
so its own arms and propellers show when it looks up and its landing gear and gimbal
mount when it looks down. The baseline detector takes the white parts for sky and a dark
screw hole on them for a drone 10-25 m away; the attack then chased its own feet. A pixel
whose line of sight hits the airframe cannot show anything else (whatever is behind it is
hidden), so masking those pixels loses nothing.

The arms, motors, propellers, legs and skids are segments and discs with a radius each
(the x500's SDF collision and visual poses, model x500_base in the container, a little
larger), in the drone's body frame FRD (x forward, y right, z down), metres from base_link;
what is right above the camera (body, gimbal mount) is a cap: every direction more than
UP_CAP_DEG above the drone's level plane. Every image,
they are sampled into points, projected through the camera (its forward, right and down
directions in the body frame, from PX4's gimbal and vehicle attitudes) and drawn as filled
circles, widened by `margin_px`. The camera sits 14 cm under base_link on the gimbal's
yaw arm, so its position depends on where it points (camera_position). Pure NumPy and
OpenCV, no ROS.
"""
import math
from dataclasses import dataclass

import cv2
import numpy as np

from swarm_backend.detect.camera import CameraModel
from swarm_backend.detect.orientation import Axes, Quaternion, rotate

Point = tuple[float, float, float]

SAMPLE_M = 0.01  # metres between the points a part is sampled into
MIN_DEPTH_M = 0.02  # points closer than this in front of the camera (or behind it) are skipped
GROW_M = 0.015  # every part is this much thicker than in the model: the meshes are rounder
MARGIN_PX = 10  # and the mask this much wider in the picture (attitude error, motion)
# Seen from the camera, everything more than this above the drone's own level plane is
# airframe: the body, the gimbal's mount and dampers, the inner arms (fitted to a Gazebo
# picture looking up 43 degrees, 11 Nov: the airframe ends 40 degrees up). This cap stands
# in for those parts, which are too close to the camera to model point by point.
UP_CAP_DEG = 38.0
CAP_STEP_PX = 4  # the cap is worked out on every 4th pixel, then scaled up

# The gimbal (model `gimbal`, included at z 0.26 in the x500's model frame, where base_link
# is at z 0.24, and turned half a turn): its pitch and roll joints are at the camera, which
# is 14.2 cm under base_link and 1.6 cm from the yaw axis, itself 2.6 cm in front of
# base_link's centre (the SDF's -0.0412 and -0.0255 turned half a turn). So the camera is
# 4.1 cm in front of the centre when it looks forward and 1 cm when it looks back.
GIMBAL_YAW_AXIS = (0.0255, 0.0)
CAMERA_ARM_M = 0.0157
CAMERA_DOWN_M = 0.142


@dataclass(frozen=True)
class Part:
    """A segment from a to b (or a sphere at a, b None), `radius` thick; or, with `disc` > 0,
    a horizontal disc of that radius around a (a spinning propeller)."""
    name: str
    a: Point
    b: Point | None
    radius: float
    disc: float = 0.0


def _flu(x: float, y: float, z: float) -> Point:
    """SDF's x forward, y left, z up to FRD."""
    return (x, -y, -z)


def _x500() -> tuple[Part, ...]:
    parts = []
    for i, (mx, my) in enumerate(((0.174, 0.174), (-0.174, 0.174), (0.174, -0.174),
                                  (-0.174, -0.174))):
        parts += [
            Part(f'arm_{i}', _flu(0.0, 0.0, 0.03), _flu(mx, my, 0.032), 0.015),
            Part(f'motor_{i}', _flu(mx, my, 0.0), _flu(mx, my, 0.07), 0.03),
            Part(f'propeller_{i}', _flu(mx, my, 0.06), None, 0.01, disc=0.145),
        ]
    for side in (1.0, -1.0):  # one leg on each side, rolled out to its skid
        parts += [
            Part(f'leg_{side:+.0f}', _flu(0.0, side * 0.062, -0.025),
                 _flu(0.0, side * 0.134, -0.221), 0.015),
            Part(f'foot_{side:+.0f}', _flu(0.0, side * 0.132, -0.215), None, 0.035),
            Part(f'skid_{side:+.0f}', _flu(-0.13, side * 0.132, -0.2195),
                 _flu(0.13, side * 0.132, -0.2195), 0.018),
        ]
    return tuple(parts)


X500_GIMBAL = _x500()


def body_axes(camera_world: Axes, drone_q: Quaternion) -> Axes:
    """The camera's forward, right and down in the drone's body frame (FRD), from those in
    the world (NED) and the drone's attitude (PX4's vehicle odometry q: body -> world)."""
    w, x, y, z = drone_q
    inverse = (w, -x, -y, -z)
    return tuple(rotate(inverse, axis) for axis in camera_world)


def camera_position(axes: Axes) -> Point:
    """Where the camera is in the body frame (FRD): on the gimbal's yaw arm, which turns
    with the direction it looks along (its forward, seen from above)."""
    fx, fy, _ = axes[0]
    yaw = math.atan2(fy, fx) if math.hypot(fx, fy) > 1e-6 else 0.0
    ax, ay = GIMBAL_YAW_AXIS
    return (ax + CAMERA_ARM_M * math.cos(yaw), ay + CAMERA_ARM_M * math.sin(yaw), CAMERA_DOWN_M)


def sample(parts: tuple[Part, ...], step: float = SAMPLE_M) -> tuple[np.ndarray, np.ndarray]:
    """Every part as points (N x 3, body frame) and the radius of each (N)."""
    points, radii = [], []
    for part in parts:
        if part.disc > 0:
            r = part.disc
            n = int(2 * r / (2 * part.radius + GROW_M)) + 1
            ticks = np.linspace(-r, r, n)
            for dx in ticks:
                for dy in ticks:
                    if dx * dx + dy * dy <= r * r:
                        points.append((part.a[0] + dx, part.a[1] + dy, part.a[2]))
                        radii.append(r / max(1, n - 1) + part.radius)
            continue
        if part.b is None:
            points.append(part.a)
            radii.append(part.radius)
            continue
        length = math.dist(part.a, part.b)
        n = max(1, int(length / step))
        for i in range(n + 1):
            f = i / n
            points.append(tuple(p + f * (q - p) for p, q in zip(part.a, part.b)))
            radii.append(part.radius)
    return np.array(points, float), np.array(radii, float) + GROW_M


_SAMPLED = sample(X500_GIMBAL)


def airframe_mask(shape: tuple[int, ...], camera: CameraModel, axes: Axes,
                  margin_px: int = MARGIN_PX) -> np.ndarray:
    """True where the drone's own airframe is in the picture. `axes`: the camera's forward,
    right and down in the body frame (body_axes)."""
    height, width = shape[:2]
    forward, right, down = (np.array(a, float) for a in axes)
    f = camera.f
    # The cap: a pixel's ray (forward + a right + b down) points up (-z) more than the cap.
    us = np.arange(0, width, CAP_STEP_PX) + CAP_STEP_PX / 2
    vs = np.arange(0, height, CAP_STEP_PX) + CAP_STEP_PX / 2
    a = (us[None, :] - camera.cx) / f
    b = (vs[:, None] - camera.cy) / f
    up = -(forward[2] + a * right[2] + b * down[2]) / np.sqrt(1 + a * a + b * b)
    cap = (up > math.sin(math.radians(UP_CAP_DEG))).astype(np.uint8)
    mask = cv2.resize(cap, (width, height), interpolation=cv2.INTER_NEAREST)
    points, radii = _SAMPLED
    rel = points - np.array(camera_position(axes))
    depth, x, y = rel @ forward, rel @ right, rel @ down
    keep = depth > MIN_DEPTH_M
    u = camera.cx + f * x[keep] / depth[keep]
    v = camera.cy + f * y[keep] / depth[keep]
    r = f * radii[keep] / depth[keep] + margin_px
    limit = 4 * (width + height)
    for ui, vi, ri in zip(u, v, r):
        # A circle that cannot reach the picture is skipped (and OpenCV wants small ints).
        if -ri <= ui <= width + ri and -ri <= vi <= height + ri and ri < limit:
            cv2.circle(mask, (int(round(ui)), int(round(vi))), int(math.ceil(ri)), 1, -1)
    return mask.astype(bool)
