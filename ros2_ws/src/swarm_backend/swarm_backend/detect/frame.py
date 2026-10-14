"""One camera image as the evaluation sees it: the image (if there is one), the
camera's pose, and the truth about what is in it. No ROS.

The pose is the camera's position and the unit vectors it looks along (`forward`), of
image right and of image down, world NED: the same as the truth that save_images.py
writes next to each image (ground_truth.camera_pose). Detectors get the whole Frame,
but only truth-based stand-ins may read `objects`; a real detector (dark object, YOLO)
only looks at `image` and the pose.
"""
import math
from dataclasses import dataclass
from enum import Enum
from typing import Any

from swarm_backend.config import Vector3
from swarm_backend.detect.camera import CameraModel
from swarm_backend.ground_truth import project


class Kind(Enum):
    DRONE = 'drone'
    BIRD = 'bird'


@dataclass(frozen=True)
class TrueObject:
    """An object in the image, from the truth."""
    id: str
    kind: Kind
    pixel: tuple[float, float]  # (u, v) of its centre
    size_px: float  # width in pixels
    position: Vector3 | None = None  # world NED, when known
    distance: float | None = None  # metres from the camera, when known


@dataclass(frozen=True)
class Frame:
    time: float  # seconds, simulation time
    drone: int  # index of the defender whose camera took it
    camera: Vector3  # world NED
    forward: Vector3  # unit vectors, world NED
    right: Vector3
    down: Vector3
    objects: tuple[TrueObject, ...]  # the truth: what is in view
    image: Any = None  # the pixels; None in the fast simulation

    @property
    def pose(self) -> dict:
        """As ground_truth.camera_pose gives it."""
        return {'position': self.camera, 'forward': self.forward, 'right': self.right,
                'down': self.down}


@dataclass(frozen=True)
class Body:
    """Something that can be in view: where it is now and how big it is."""
    id: str
    kind: Kind
    position: Vector3
    size: float  # metres


def aimed_camera(position: Vector3, pitch_deg: float, yaw_deg: float) -> dict:
    """The pose of a camera at `position` pointed as a gimbal is commanded (no roll):
    pitch from the horizon (negative = down), yaw from north."""
    p, y = math.radians(pitch_deg), math.radians(yaw_deg)
    forward = (math.cos(p) * math.cos(y), math.cos(p) * math.sin(y), -math.sin(p))
    right = (-math.sin(y), math.cos(y), 0.0)
    down = (forward[1] * right[2] - forward[2] * right[1],
            forward[2] * right[0] - forward[0] * right[2],
            forward[0] * right[1] - forward[1] * right[0])
    return {'position': position, 'forward': forward, 'right': right, 'down': down}


def make_frame(model: CameraModel, time: float, drone: int, pose: dict, bodies: list[Body],
               image: Any = None) -> Frame:
    """The truth for one image taken from `pose` (as aimed_camera or
    ground_truth.camera_pose give it): every body that projects into it."""
    objects = []
    for body in bodies:
        pixel = project(pose, body.position, model.hfov, model.width, model.height)
        if pixel is None:
            continue
        distance = math.dist(pose['position'], body.position)
        objects.append(TrueObject(id=body.id, kind=body.kind, pixel=pixel,
                                  size_px=model.width_at(distance, body.size),
                                  position=body.position, distance=distance))
    return Frame(time=time, drone=drone, camera=tuple(pose['position']),
                 forward=tuple(pose['forward']), right=tuple(pose['right']),
                 down=tuple(pose['down']), objects=tuple(objects), image=image)
