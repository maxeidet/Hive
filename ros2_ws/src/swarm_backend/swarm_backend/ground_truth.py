"""Where the drones and a camera really are, from Gazebo's poses (cheating on purpose).

Used to save the truth next to each camera image (save_images.py), so that what the
detector estimates later (direction and distance to the target) can be compared with it.
Everything is in the swarm world frame: NED (x north, y east, z down), metres, origin at
the Gazebo origin. The functions are pure math on Gazebo's pose list (see sensor_view.Pose),
without ROS or Gazebo imports.
"""
import math

from swarm_backend.sensor_view import Pose, drone_transforms

Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]  # x, y, z, w, as Gazebo gives it

FRAME = 'world NED: x north, y east, z down; metres; origin at the Gazebo origin'
# Gazebo's camera looks along +x of the sensor, with y to the left and z up. In PX4's gimbal
# model the sensor is turned half a turn about z in camera_link (<pose>... 0 0 3.14</pose>).
BODY_LINK = 'base_link'
SENSOR_IN_LINK: Quaternion = (0.0, 0.0, math.sin(3.14 / 2), math.cos(3.14 / 2))


def rotate(q: Quaternion, v: Vector3) -> Vector3:
    """v turned by the quaternion q."""
    x, y, z, w = q
    vx, vy, vz = v
    # t = 2 q_vec x v; v' = v + w t + q_vec x t
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (vx + w * tx + y * tz - z * ty,
            vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx)


def multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    """The rotation b followed by a."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def enu_to_ned(v: Vector3) -> Vector3:
    """Gazebo's east-north-up to our north-east-down."""
    return (v[1], v[0], -v[2])


def camera_pose(poses: list[Pose], drone_name: str, link: str) -> dict | None:
    """The camera on `link` of the drone: its position and the directions (unit vectors)
    it looks along (`forward`), of image right and of image down."""
    transforms = drone_transforms(poses, drone_name, [link])
    if len(transforms) < 2:
        return None
    (_, _, drone_xyz, drone_q), (_, _, link_xyz, link_q) = transforms
    offset = rotate(drone_q, link_xyz)
    position = tuple(d + o for d, o in zip(drone_xyz, offset))
    q = multiply(multiply(drone_q, link_q), SENSOR_IN_LINK)
    return {
        'position': enu_to_ned(position),
        'forward': enu_to_ned(rotate(q, (1.0, 0.0, 0.0))),
        'right': enu_to_ned(rotate(q, (0.0, -1.0, 0.0))),
        'down': enu_to_ned(rotate(q, (0.0, 0.0, -1.0))),
    }


def body_position(poses: list[Pose], drone_name: str) -> Vector3 | None:
    """The centre of the drone's body (its base_link; the model's own origin is at its feet)."""
    transforms = drone_transforms(poses, drone_name, [BODY_LINK])
    if not transforms:
        return None
    _, _, drone_xyz, drone_q = transforms[0]
    link_xyz = transforms[1][2] if len(transforms) > 1 else (0.0, 0.0, 0.0)
    return enu_to_ned(tuple(d + o for d, o in zip(drone_xyz, rotate(drone_q, link_xyz))))


def project(camera: dict, point: Vector3, hfov: float, width: int, height: int):
    """Pixel (u, v) where the world point shows in the image of an ideal pinhole camera,
    or None when it is behind the camera or outside the image."""
    to_point = tuple(p - c for p, c in zip(point, camera['position']))
    depth, right, down = (sum(a * b for a, b in zip(to_point, camera[axis]))
                          for axis in ('forward', 'right', 'down'))
    if depth <= 0:
        return None
    f = (width / 2) / math.tan(hfov / 2)
    u, v = width / 2 + f * right / depth, height / 2 + f * down / depth
    return (u, v) if 0 <= u < width and 0 <= v < height else None


def truth(poses: list[Pose], drone_names: dict[int, str], camera_id: int, link: str,
          hfov: float, width: int, height: int) -> dict | None:
    """Everything saved with an image taken by drone `camera_id`: where that drone and its
    camera are, and for every other drone its position, the distance and unit direction
    to it from the camera, and the pixel it shows at (None when out of view)."""
    camera = camera_pose(poses, drone_names[camera_id], link)
    own_position = body_position(poses, drone_names[camera_id])
    if camera is None or own_position is None:
        return None
    others = []
    for drone_id, name in sorted(drone_names.items()):
        position = None if drone_id == camera_id else body_position(poses, name)
        if position is None:
            continue
        to_drone = tuple(p - c for p, c in zip(position, camera['position']))
        distance = math.hypot(*to_drone)
        pixel = project(camera, position, hfov, width, height)
        others.append({
            'id': drone_id,
            'position': _rounded(position),
            'distance': round(distance, 3),
            'direction': _rounded(tuple(c / distance for c in to_drone), 6),
            'pixel': None if pixel is None else [round(pixel[0], 1), round(pixel[1], 1)],
        })
    return {
        'frame': FRAME,
        'camera_drone': {'id': camera_id, 'position': _rounded(own_position)},
        'camera': {'position': _rounded(camera['position']),
                   'forward': _rounded(camera['forward'], 6),
                   'right': _rounded(camera['right'], 6),
                   'down': _rounded(camera['down'], 6),
                   'hfov_rad': hfov, 'width': width, 'height': height},
        'drones': others,
    }


def _rounded(v, digits: int = 3) -> list[float]:
    return [round(c, digits) for c in v]
