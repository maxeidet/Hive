"""What every scenario shares: where the repo and configs are, the --sim flag, and the
setup / runs / landing frame around the surveillance world."""
import argparse
import shlex
from pathlib import Path

from swarm_backend.config import default_config_path

PKG = Path(__file__).resolve().parents[2]  # ros2_ws/src/swarm_backend (the source tree)
REPO = PKG.parents[2]  # /workspace in the container
SWARM_CONFIG = 'config/swarm.yaml'
SURVEILLANCE_CONFIG = 'scenarios/surveillance.yaml'


def config_path(given: str | None, default: str) -> Path:
    """`given`, else `default` (relative to the package) in the source tree, else the
    installed swarm.yaml."""
    if given:
        return Path(given)
    source = PKG / default
    return source if source.exists() else default_config_path()


def add_backend_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument('--sim', action='store_true',
                        help='the fast simulation instead of Gazebo: no ROS, no PX4, a run '
                             'takes seconds (simpler drones and the fake detector)')
    parser.add_argument('--quiet', action='store_true',
                        help='with --sim: only warnings, errors and the summary')


# What detects, tracks and attacks. This branch tests YOLO + ByteTrack + MPC; `baseline` is
# the report's setup (baseline detector, k-in-a-row Confirmer, pure pursuit).
SETUPS = {
    'yolo-bytetrack-mpc': {'tracker': 'bytetrack', 'detect_args': ('--detector', 'yolo', '--lock'),
                           'guidance': 'mpc'},
    'baseline': {'tracker': 'confirmer', 'detect_args': (), 'guidance': 'pursuit'},
}
DEFAULT_SETUP = 'yolo-bytetrack-mpc'


def add_setup_args(parser: argparse.ArgumentParser) -> None:
    """--setup, and --tracker, --detect-args and --guidance to change one part of it."""
    g = parser.add_argument_group('setup (detector, tracker, attack)')
    g.add_argument('--setup', choices=tuple(SETUPS), default=DEFAULT_SETUP,
                   help='yolo-bytetrack-mpc: YOLO, ByteTrack locked on one target, MPC '
                        'attack; baseline: the baseline detector, the Confirmer and pure '
                        'pursuit (the report\'s). Default %(default)s')
    g.add_argument('--tracker', choices=('confirmer', 'bytetrack'), default=None,
                   help='tracker of the real detectors (detect/live.py --tracker); '
                        'default: the setup\'s')
    g.add_argument('--detect-args', default=None,
                   help='options for every real detector instead of the setup\'s, in quotes, '
                        'e.g. "--detector yolo --lock --high 0.4" ("" for none)')
    g.add_argument('--guidance', choices=('pursuit', 'mpc'), default=None,
                   help='the defenders\' attack; default: the setup\'s')


def resolve_setup(args) -> None:
    """Fill args.tracker, args.detect_args (a tuple) and args.guidance from args.setup
    where they were not given."""
    preset = SETUPS[args.setup]
    if args.tracker is None:
        args.tracker = preset['tracker']
    args.detect_args = (preset['detect_args'] if args.detect_args is None
                        else tuple(shlex.split(args.detect_args)))
    if args.guidance is None:
        args.guidance = preset['guidance']


def setup_tag(args, real_detector: bool = True) -> str:
    """'' for the baseline (whose results keep the report's file names), else a name for
    the default output: the setup's name, or the parts that differ, e.g.
    'bytetrack_detector-yolo_pursuit'. Call after resolve_setup()."""
    parts = {'tracker': args.tracker, 'detect_args': tuple(args.detect_args),
             'guidance': args.guidance}
    if not real_detector:  # the fake detectors have no tracker: only the attack counts
        return '' if args.guidance == 'pursuit' else args.guidance
    for name, preset in SETUPS.items():
        if parts == preset:
            return '' if name == 'baseline' else name
    extra = '_'.join(a.lstrip('-') for a in parts['detect_args']).replace('/', '-')
    return '_'.join(p for p in (args.tracker, extra, args.guidance) if p)


def with_tag(path: Path, tag: str) -> Path:
    """data/x.csv -> data/x_<tag>.csv, data/dir -> data/dir_<tag>; unchanged for ''."""
    return path if not tag else path.with_name(f'{path.stem}_{tag}{path.suffix}')


def setup_options(args) -> dict:
    """The Options fields of the setup (surveillance.Options)."""
    return {'tracker': args.tracker, 'detect_args': tuple(args.detect_args),
            'guidance': args.guidance}


def run_surveillance(node_name: str, config, options, args, body):
    """Make the surveillance world (Gazebo, or the simulation with --sim), set it up, call
    body(world), and always stop and land. Returns (exit code, world or None)."""
    if args.sim:
        from swarm_backend.scenarios.sim_surveillance import SimSurveillance
        try:
            world = SimSurveillance(config, options, quiet=args.quiet)
        except (RuntimeError, ValueError) as e:
            print(e)
            return 1, None
        return _fly(world, body), world

    from swarm_backend.runner import spinning
    from swarm_backend.scenarios.gazebo_surveillance import ros_world
    try:
        rclpy, node, world = ros_world(node_name, config, options)
    except (RuntimeError, ValueError) as e:
        print(e)
        return 1, None
    with spinning(node):
        code = _fly(world, body)
    node.destroy_node()
    rclpy.try_shutdown()
    return code, world


def _fly(world, body) -> int:
    code = 0
    try:
        world.setup()
        body(world)
    except KeyboardInterrupt:
        world.log.info('Ctrl-C')
    except (TimeoutError, RuntimeError) as e:
        world.log.error(str(e))
        code = 1
    finally:
        try:
            world.stop_and_land()
        except (TimeoutError, KeyboardInterrupt) as e:
            world.log.error(f'Did not land cleanly: {e or "Ctrl-C"}')
            code = 1
    return code
