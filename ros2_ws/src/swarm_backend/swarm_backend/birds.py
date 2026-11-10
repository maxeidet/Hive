"""Birds in Gazebo: distractors for the detector, and for training images (`mount-backend.sh bird`).

    ./mount-backend.sh bird spawn dove --at 20 0 30          a dove 20 m north, 30 m up
    ./mount-backend.sh bird spawn flock --name flock_1 --at 30 10 25
    ./mount-backend.sh bird spawn dove --near 0 --at 3 0 1   3 m north of drone 0, 1 m higher
    ./mount-backend.sh bird move bird_1 --at 25 5 30
    ./mount-backend.sh bird wander bird_1 --around 20 0 30   fly around randomly, Ctrl-C stops
    ./mount-backend.sh bird remove bird_1

Positions are the swarm world frame (x north, y east, metres from the Gazebo origin) and
the height in metres above the ground. The meshes are in worlds/birds/ (see the README there
for credits and licences); each kind is scaled to a realistic wingspan. A bird is a static
model without collisions: nothing pushes it, gravity doesn't pull it, and a drone flies
through it. Gazebo shows it in a still pose, without the wing animation.

`wander` moves it along the same kind of random path the hostile drone can fly
(intercept/paths.py), facing where it flies, by setting its pose many times a second.
"""
import argparse
import math
import subprocess
import sys
import time
from dataclasses import dataclass

MESHES = '/workspace/worlds/birds'  # the repo, as the start scripts mount it in the container


@dataclass(frozen=True)
class Kind:
    mesh: str  # file in worlds/birds/
    scale: float  # mesh units -> metres
    yaw: float = 0.0  # radians: turns the mesh so it faces +x (its flight direction)
    roll: float = 1.5708  # radians: glTF files are y-up, Gazebo is z-up
    span: float = 0.45  # metres across the wings (the generators use it for the distance)
    flying: bool = True  # in a flying pose (wings spread): used by the data generators


# Scales from each file's size (its extent across the wings) to a realistic wingspan; `span`
# is the width across the wings after scaling, as seen from above. The
# orientations of the 18 Oct birds come from top, front and side views of their meshes:
# eagle faces -z with its wings along x, the gulls face +x with their wings along z (raised
# in a V), soaring_bird is modelled z-up facing +x with its wings along y.
KINDS = {
    'dove': Kind('dove.glb', 0.45 / 27.153, yaw=1.5708),  # faced east at yaw 0 (box test 17 Oct);
    # seen from above its wings are spread, so it counts as flying (it has its feet down)
    'mourning_dove': Kind('mourning_dove.glb', 0.45 / 5.466,
                          flying=False),  # renders as a thin sliver (box test 17 Oct)
    'flock': Kind('flock.glb', 0.4 * 5 / 3.924, span=2.0, flying=False),  # five birds about 4 units across
    # The gulls' wings are raised, so across them they are only a little wider than long:
    # scaled by their length (0.7 and 0.55 m, beak to tail) instead of the wingspan.
    'seagull': Kind('seagull.glb', 0.7 / 82.163, span=0.75),
    'gull': Kind('gull.glb', 0.55 / 82.163, span=0.6),  # same shape as seagull, smaller
    'eagle': Kind('eagle.glb', 1.9 / 260.07, yaw=-1.5708, span=1.9),
    'soaring_bird': Kind('soaring_bird.glb', 0.9 / 241.44, roll=0.0, span=0.9),
}
FLYING = [name for name, kind in KINDS.items() if kind.flying]


def enu(north: float, east: float, up: float) -> tuple[float, float, float]:
    """The swarm world frame (north, east, up) as Gazebo's (x east, y north, z up)."""
    return east, north, up


def yaw_quaternion(heading: float) -> tuple[float, float, float, float]:
    """(x, y, z, w) of a turn about Gazebo's vertical axis by `heading` (radians from
    north, clockwise as in the swarm frame), so the bird faces that way."""
    angle = math.pi / 2 - heading  # Gazebo's yaw is from east, counter-clockwise
    return 0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2)


def bird_sdf(name: str, kind: Kind, label: int | None = None) -> str:
    """A static, collision-free model that shows the bird's mesh. `label`: a class number
    for Gazebo's bounding-box and segmentation cameras (the Label system), or None."""
    s = kind.scale
    plugin = '' if label is None else (
        '\n    <plugin filename="gz-sim-label-system" name="gz::sim::systems::Label">'
        f'<label>{label}</label></plugin>')
    return f'''<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>{plugin}
    <link name="body">
      <visual name="visual">
        <pose>0 0 0 {kind.roll} 0 {kind.yaw}</pose>
        <geometry>
          <mesh>
            <uri>{MESHES}/{kind.mesh}</uri>
            <scale>{s} {s} {s}</scale>
          </mesh>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>'''


def world_name() -> str:
    """The running Gazebo world (from its topics)."""
    topics = subprocess.run(['gz', 'topic', '-l'], capture_output=True, text=True,
                            check=True).stdout.split()
    for topic in topics:
        if topic.startswith('/world/') and topic.endswith('/clock'):
            return topic.split('/')[2]
    for topic in topics:
        if topic.startswith('/world/'):
            return topic.split('/')[2]
    raise RuntimeError('No Gazebo world is running')


class Gazebo:
    """The Gazebo services for putting models in the world and moving them (create,
    set_pose, remove), and for changing the sun (light_config)."""

    def __init__(self, world: str | None = None):
        from gz.transport13 import Node
        self.world = world or world_name()
        self._node = Node()

    def _call(self, service: str, request, request_type, timeout_ms: int = 5000) -> bool:
        from gz.msgs10.boolean_pb2 import Boolean
        ok, reply = self._node.request(f'/world/{self.world}/{service}', request, request_type,
                                       Boolean, timeout_ms)
        return bool(ok and reply.data)

    def spawn(self, name: str, kind: Kind, north: float, east: float, up: float,
              heading: float = 0.0, label: int | None = None) -> bool:
        return self.spawn_sdf(name, bird_sdf(name, kind, label), north, east, up, heading)

    def spawn_sdf(self, name: str, sdf: str, north: float, east: float, up: float,
                  heading: float = 0.0) -> bool:
        """Any model from its SDF text, facing `heading` (radians from north)."""
        from gz.msgs10.entity_factory_pb2 import EntityFactory
        request = EntityFactory()
        request.sdf = sdf
        request.name = name
        _fill_pose(request.pose, north, east, up, heading)
        return self._call('create', request, EntityFactory)

    def move(self, name: str, north: float, east: float, up: float, heading: float = 0.0,
             timeout_ms: int = 1000) -> bool:
        from gz.msgs10.pose_pb2 import Pose
        request = Pose()
        request.name = name
        _fill_pose(request, north, east, up, heading)
        return self._call('set_pose', request, Pose, timeout_ms)

    def set_pose(self, name: str, xyz_enu: tuple[float, float, float],
                 quaternion: tuple[float, float, float, float], timeout_ms: int = 5000,
                 tries: int = 2) -> bool:
        """Put a model anywhere, in Gazebo's own frame (ENU) and orientation (x, y, z, w).
        Gazebo answers on its next simulation step, which takes long when it runs slowly
        (a real-time factor of 0.1 gave 0.3-0.5 s answers and some over 1 s), so the wait is
        generous and a timed-out call is sent once more."""
        from gz.msgs10.pose_pb2 import Pose
        request = Pose()
        request.name = name
        request.position.x, request.position.y, request.position.z = xyz_enu
        (request.orientation.x, request.orientation.y, request.orientation.z,
         request.orientation.w) = quaternion
        return any(self._call('set_pose', request, Pose, timeout_ms) for _ in range(tries))

    def set_sun(self, name: str, direction_enu: tuple[float, float, float],
                diffuse: tuple[float, float, float], specular: float = 0.3) -> bool:
        """Change a directional light (the world's sun) while Gazebo runs."""
        from gz.msgs10.light_pb2 import Light
        request = Light()
        request.name = name
        request.type = Light.DIRECTIONAL
        request.direction.x, request.direction.y, request.direction.z = direction_enu
        request.diffuse.r, request.diffuse.g, request.diffuse.b = diffuse
        request.diffuse.a = 1.0
        request.specular.r = request.specular.g = request.specular.b = specular
        request.specular.a = 1.0
        request.cast_shadows = True
        request.intensity = 1.0
        return self._call('light_config', request, Light)

    def remove(self, name: str) -> bool:
        from gz.msgs10.entity_pb2 import Entity
        request = Entity()
        request.name = name
        request.type = Entity.MODEL
        return self._call('remove', request, Entity)


def drone_position(world: str, drone_id: int, timeout_s: float = 5.0) -> tuple[float, float, float]:
    """Where PX4 drone `drone_id` is in Gazebo: (north, east, height above the ground)."""
    import threading

    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node

    from swarm_backend.save_images import drone_names

    topics = subprocess.run(['gz', 'topic', '-l'], capture_output=True, text=True,
                            check=True).stdout.split()
    name = drone_names(topics)[1].get(drone_id)
    if name is None:
        raise RuntimeError(f'Drone {drone_id} is not in Gazebo')
    found = threading.Event()
    where = {}

    def on_poses(msg):
        for p in msg.pose:
            if p.name == name:
                where['enu'] = (p.position.x, p.position.y, p.position.z)
                found.set()
                return

    node = Node()
    for topic in (f'/world/{world}/dynamic_pose/info', f'/world/{world}/pose/info'):
        node.subscribe(Pose_V, topic, on_poses)
    if not found.wait(timeout_s):
        raise RuntimeError(f'No pose for {name} from Gazebo')
    east, north, up = where['enu']
    return north, east, up


def _fill_pose(pose, north: float, east: float, up: float, heading: float) -> None:
    pose.position.x, pose.position.y, pose.position.z = enu(north, east, up)
    (pose.orientation.x, pose.orientation.y, pose.orientation.z,
     pose.orientation.w) = yaw_quaternion(heading)


def wander(gazebo: Gazebo, name: str, centre: tuple[float, float, float], speed: float,
           size: float, seed: int, rate_hz: float = 20.0) -> None:
    """Fly the bird along a random path around `centre` (north, east, up) until Ctrl-C.
    Timed on the wall clock: a slow Gazebo makes the bird look faster than `speed`."""
    from swarm_backend.intercept.paths import make_path

    north, east, up = centre
    path = make_path('random', (north, east, -up), 0.0, speed, size, seed)
    start = time.monotonic()
    while True:
        (n, e, d), (vn, ve, _) = path.at(time.monotonic() - start)
        heading = math.atan2(ve, vn) if math.hypot(vn, ve) > 1e-6 else 0.0
        gazebo.move(name, n, e, -d, heading, timeout_ms=200)
        time.sleep(1.0 / rate_hz)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest='command', required=True)

    spawn = sub.add_parser('spawn', help='put a bird in the world')
    spawn.add_argument('kind', choices=KINDS)
    spawn.add_argument('--name', default='bird_1', help='its Gazebo name (default %(default)s)')
    spawn.add_argument('--at', nargs=3, type=float, metavar=('NORTH', 'EAST', 'HEIGHT'),
                       default=(20.0, 0.0, 30.0), help='metres (default 20 0 30)')
    spawn.add_argument('--heading', type=float, default=0.0, help='degrees from north')
    spawn.add_argument('--near', type=int, metavar='DRONE',
                       help='--at is then relative to this drone (PX4 instance number)')
    spawn.add_argument('--scale', type=float, help='override the kind\'s mesh scale')
    spawn.add_argument('--roll', type=float,
                       help='degrees about the bird\'s length, to stand it upright if it '
                            'shows on its side (default 90: glTF is y-up)')
    spawn.add_argument('--yaw', type=float,
                       help='degrees, to turn the mesh if it does not face its heading')

    move = sub.add_parser('move', help='put a bird somewhere else')
    move.add_argument('name')
    move.add_argument('--at', nargs=3, type=float, required=True,
                      metavar=('NORTH', 'EAST', 'HEIGHT'))
    move.add_argument('--heading', type=float, default=0.0, help='degrees from north')

    roam = sub.add_parser('wander', help='fly a bird around randomly until Ctrl-C')
    roam.add_argument('name')
    roam.add_argument('--around', nargs=3, type=float, required=True,
                      metavar=('NORTH', 'EAST', 'HEIGHT'))
    roam.add_argument('--speed', type=float, default=6.0, help='m/s (default %(default)s)')
    roam.add_argument('--size', type=float, default=40.0,
                      help='metres: width of the area it roams (default %(default)s)')
    roam.add_argument('--seed', type=int, default=0)
    roam.add_argument('--near', type=int, metavar='DRONE',
                      help='--around is then relative to this drone (PX4 instance number)')

    remove = sub.add_parser('remove', help='take a bird out of the world')
    remove.add_argument('name')
    args = parser.parse_args(argv)

    gazebo = Gazebo()
    if args.command == 'spawn':
        kind = KINDS[args.kind]
        kind = Kind(kind.mesh,
                    kind.scale if args.scale is None else args.scale,
                    kind.yaw if args.yaw is None else math.radians(args.yaw),
                    kind.roll if args.roll is None else math.radians(args.roll))
        at = tuple(args.at)
        if args.near is not None:
            drone = drone_position(gazebo.world, args.near)
            at = tuple(round(d + o, 2) for d, o in zip(drone, at))
        ok = gazebo.spawn(args.name, kind, *at, math.radians(args.heading))
        print(f'{args.name} ({args.kind}) at {at}: ' + ('spawned' if ok else
              'FAILED (is the name already used? `bird remove` it first)'))
        return 0 if ok else 1
    if args.command == 'move':
        ok = gazebo.move(args.name, *args.at, math.radians(args.heading))
        print(f'{args.name} -> {args.at}: ' + ('moved' if ok else 'FAILED (no such bird?)'))
        return 0 if ok else 1
    if args.command == 'wander':
        around = tuple(args.around)
        if args.near is not None:
            drone = drone_position(gazebo.world, args.near)
            around = tuple(round(d + o, 2) for d, o in zip(drone, around))
        print(f'{args.name} wandering around {around} at {args.speed} m/s. Ctrl-C to stop.')
        try:
            wander(gazebo, args.name, around, args.speed, args.size, args.seed)
        except KeyboardInterrupt:
            pass
        return 0
    ok = gazebo.remove(args.name)
    print(f'{args.name}: ' + ('removed' if ok else 'FAILED (no such bird?)'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
