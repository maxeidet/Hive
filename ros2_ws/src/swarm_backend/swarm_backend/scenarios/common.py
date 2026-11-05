"""What every scenario shares: where the repo and configs are, the --sim flag, and the
setup / runs / landing frame around the surveillance world."""
import argparse
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
