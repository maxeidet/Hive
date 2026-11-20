"""Where the drones, birds and a camera really are, from Gazebo's poses (cheating on purpose).

Used to save the truth next to each camera image (save_images.py), so that what the
detector estimates later (direction and distance to the target) can be compared with it,
and as training labels: a box per drone and bird, projected from a box around the model.
Everything is in the swarm world frame: NED (x north, y east, z down), metres, origin at
the Gazebo origin. The functions are pure math on Gazebo's pose list (see sensor_view.Pose),
without ROS or Gazebo imports.
"""
import math

from swarm_backend.sim.sensor_view import Pose, drone_transforms

Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]  # x, y, z, w, as Gazebo gives it

FRAME = 'world NED: x north, y east, z down; metres; origin at the Gazebo origin'
# Gazebo's camera looks along +x of the sensor, with y to the left and z up. In PX4's gimbal
# model the sensor is turned half a turn about z in camera_link (<pose>... 0 0 3.14</pose>).
BODY_LINK = 'base_link'
SENSOR_IN_LINK: Quaternion = (0.0, 0.0, math.sin(3.14 / 2), math.cos(3.14 / 2))
BIRD_PREFIX = 'bird'  # model names of the birds (birds.py)
# Half sizes (m) of the box around each class, along the object's own x (forward), y (left)
# and z (up), centred on the drone's body or the bird model's origin. The x500's are from
# Gazebo's full boxes in box_test (17 Oct: about 0.69 m wide and 0.35 m high, propellers
# included); the birds' are estimates (about 0.45 m across the wings).
HALF_SIZE: dict[str, Vector3] = {'drone': (0.33, 0.33, 0.17), 'bird': (0.10, 0.23, 0.04)}


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


def _pixel(camera: dict, point: Vector3, f: float, width: int, height: int):
    """Pixel (u, v) of a world point for an ideal pinhole camera, also outside the image;
    None when it is behind the camera."""
    to_point = tuple(p - c for p, c in zip(point, camera['position']))
    depth, right, down = (sum(a * b for a, b in zip(to_point, camera[axis]))
                          for axis in ('forward', 'right', 'down'))
    if depth <= 0:
        return None
    return width / 2 + f * right / depth, height / 2 + f * down / depth


def project(camera: dict, point: Vector3, hfov: float, width: int, height: int):
    """Pixel (u, v) where the world point shows in the image of an ideal pinhole camera,
    or None when it is behind the camera or outside the image."""
    f = (width / 2) / math.tan(hfov / 2)
    pixel = _pixel(camera, point, f, width, height)
    if pixel is None:
        return None
    u, v = pixel
    return (u, v) if 0 <= u < width and 0 <= v < height else None


def corners(centre: Vector3, q: Quaternion, half: Vector3) -> list[Vector3]:
    """The 8 corners (world NED) of a box around an object: `centre` in NED, `q` its
    orientation as Gazebo gives it (ENU), `half` its half sizes along its own x, y, z."""
    hx, hy, hz = half
    out = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                offset = enu_to_ned(rotate(q, (sx * hx, sy * hy, sz * hz)))
                out.append(tuple(c + o for c, o in zip(centre, offset)))
    return out


def project_box(camera: dict, points: list[Vector3], hfov: float, width: int,
                height: int) -> dict | None:
    """The 2D box (px) around the projected points, cut to the image, and `in_image`: the
    share of the uncut box inside the image. None when the box is outside the image or a
    point is behind the camera. Trees or other objects in the way are not taken into
    account (Gazebo's bounding-box camera does that; see box_test.py)."""
    f = (width / 2) / math.tan(hfov / 2)
    pixels = [_pixel(camera, p, f, width, height) for p in points]
    if any(p is None for p in pixels):
        return None
    us, vs = [p[0] for p in pixels], [p[1] for p in pixels]
    full = (min(us), min(vs), max(us), max(vs))
    x0, y0 = max(full[0], 0.0), max(full[1], 0.0)
    x1, y1 = min(full[2], float(width)), min(full[3], float(height))
    if x1 <= x0 or y1 <= y0:
        return None
    area = (full[2] - full[0]) * (full[3] - full[1])
    return {'x0': round(x0, 1), 'y0': round(y0, 1), 'x1': round(x1, 1), 'y1': round(y1, 1),
            'in_image': round((x1 - x0) * (y1 - y0) / area, 3) if area > 0 else 1.0}


def body_orientation(poses: list[Pose], drone_name: str) -> Quaternion | None:
    """The drone's orientation (Gazebo ENU quaternion), from its model pose."""
    drone = next((p for p in poses if p[0] == drone_name), None)
    return None if drone is None else drone[3]


def bird_poses(poses: list[Pose]) -> list[Pose]:
    """The birds in Gazebo's pose list: models named bird... (birds.py names them so)."""
    return [p for p in poses if p[0].startswith(BIRD_PREFIX)]


def _target(camera: dict, name: str, kind: str, position: Vector3, q: Quaternion,
            hfov: float, width: int, height: int) -> dict:
    to_it = tuple(p - c for p, c in zip(position, camera['position']))
    distance = math.hypot(*to_it)
    pixel = project(camera, position, hfov, width, height)
    return {
        'name': name, 'class': kind,
        'position': _rounded(position),
        'distance': round(distance, 3),
        'direction': _rounded(tuple(c / distance for c in to_it), 6),
        'pixel': None if pixel is None else [round(pixel[0], 1), round(pixel[1], 1)],
        'box': project_box(camera, corners(position, q, HALF_SIZE[kind]), hfov, width, height),
    }


def truth(poses: list[Pose], drone_names: dict[int, str], camera_id: int, link: str,
          hfov: float, width: int, height: int) -> dict | None:
    """Everything saved with an image taken by drone `camera_id`: where that drone and its
    camera are, and for every other drone its position, the distance and unit direction
    to it from the camera, the pixel it shows at (None when out of view) and its projected
    box. `objects` lists the same for the other drones and for every bird, with a class,
    as labels for training; the model name is the track id."""
    camera = camera_pose(poses, drone_names[camera_id], link)
    own_position = body_position(poses, drone_names[camera_id])
    if camera is None or own_position is None:
        return None
    drones, objects = [], []
    for drone_id, name in sorted(drone_names.items()):
        position = None if drone_id == camera_id else body_position(poses, name)
        if position is None:
            continue
        target = _target(camera, name, 'drone', position, body_orientation(poses, name),
                         hfov, width, height)
        objects.append(target)
        drones.append({'id': drone_id, **{k: target[k] for k in
                                          ('position', 'distance', 'direction', 'pixel', 'box')}})
    for name, _, xyz, q in bird_poses(poses):
        objects.append(_target(camera, name, 'bird', enu_to_ned(xyz), q, hfov, width, height))
    return {
        'frame': FRAME,
        'camera_drone': {'id': camera_id, 'position': _rounded(own_position)},
        'camera': {'position': _rounded(camera['position']),
                   'forward': _rounded(camera['forward'], 6),
                   'right': _rounded(camera['right'], 6),
                   'down': _rounded(camera['down'], 6),
                   'hfov_rad': hfov, 'width': width, 'height': height},
        'drones': drones,
        'objects': objects,
    }


def _rounded(v, digits: int = 3) -> list[float]:
    return [round(c, digits) for c in v]
