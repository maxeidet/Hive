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


# Scales from each file's size (largest extent across the wings) to a realistic wingspan:
# dove and mourning dove about 0.45 m, the flock's birds about 0.4 m each. Check them by
# eye next to a drone (0.5 m across) and adjust here if needed.
KINDS = {
    'dove': Kind('dove.glb', 0.45 / 27.153),
    'mourning_dove': Kind('mourning_dove.glb', 0.45 / 5.466),
    'flock': Kind('flock.glb', 0.4 * 5 / 3.924),  # five birds about 4 units across
}


def enu(north: float, east: float, up: float) -> tuple[float, float, float]:
    """The swarm world frame (north, east, up) as Gazebo's (x east, y north, z up)."""
    return east, north, up


def yaw_quaternion(heading: float) -> tuple[float, float, float, float]:
    """(x, y, z, w) of a turn about Gazebo's vertical axis by `heading` (radians from
    north, clockwise as in the swarm frame), so the bird faces that way."""
    angle = math.pi / 2 - heading  # Gazebo's yaw is from east, counter-clockwise
    return 0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2)


def bird_sdf(name: str, kind: Kind) -> str:
    """A static, collision-free model that shows the bird's mesh."""
    s = kind.scale
    return f'''<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
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
    """The three Gazebo services this needs: create, set_pose and remove."""

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
              heading: float = 0.0) -> bool:
        from gz.msgs10.entity_factory_pb2 import EntityFactory
        request = EntityFactory()
        request.sdf = bird_sdf(name, kind)
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
