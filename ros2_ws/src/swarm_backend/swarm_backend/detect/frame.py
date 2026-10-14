"""One camera image as the evaluation sees it: the image (if there is one), the
camera's pose, and the truth about what is in it. No ROS.

Detectors get the whole Frame, but only truth-based stand-ins may read `objects`;
a real detector (dark object, YOLO) only looks at `image` and the pose.
"""
import math
from dataclasses import dataclass
from enum import Enum
from typing import Any

from swarm_backend.config import Vector3
from swarm_backend.detect.camera import CameraModel


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
    pitch_deg: float
    yaw_deg: float  # from north
    objects: tuple[TrueObject, ...]  # the truth: what is in view
    image: Any = None  # the pixels; None in the fast simulation


@dataclass(frozen=True)
class Body:
    """Something that can be in view: where it is now and how big it is."""
    id: str
    kind: Kind
    position: Vector3
    size: float  # metres


def make_frame(model: CameraModel, time: float, drone: int, camera: Vector3,
               pitch_deg: float, yaw_deg: float, bodies: list[Body],
               image: Any = None) -> Frame:
    """The truth for one image: every body that projects into it."""
    objects = []
    for body in bodies:
        pixel = model.project(camera, pitch_deg, yaw_deg, body.position)
        if pixel is None:
            continue
        distance = math.dist(camera, body.position)
        objects.append(TrueObject(id=body.id, kind=body.kind, pixel=pixel,
                                  size_px=model.size_px(distance, body.size),
                                  position=body.position, distance=distance))
    return Frame(time=time, drone=drone, camera=camera, pitch_deg=pitch_deg,
                 yaw_deg=yaw_deg, objects=tuple(objects), image=image)
