"""Intercept baseline scenario: one scenario of the planning report, one interceptor.

The same scenario, run loop and metrics as tests/test_intercept_baseline.py's sweep. In
Gazebo with two PX4 drones: the hostile drone (intercept.hostile_id) flies its straight
track toward the base, and the interceptor (intercept.interceptor_id) runs the baseline
(pure pursuit on the true position plus noise); the scenario is moved so that the
interceptor starts where it is. With --sim the same on two SimDrones (fast_sim.FastSim).

    ./mount-backend.sh scenario intercept_baseline --aspect 0 --speed 4 --seed 0 --csv /tmp/run.csv
    ./mount-backend.sh scenario intercept_baseline --sim --aspect 90 --speed 8

Ctrl-C or any error lands both drones.
"""
import argparse
import csv
import random
import sys
import time
from dataclasses import asdict

from swarm_backend.config import load_config
from swarm_backend.intercept.interceptor import PurePursuitAction
from swarm_backend.intercept.metrics import score_run, summarize
from swarm_backend.intercept.scenario import make_scenario, run_scenario
from swarm_backend.intercept.target_detector import NoisyTruthDetector
from swarm_backend.scenarios import common

CLIMB_TIMEOUT_S = 300.0  # up to 120 m, slower when Gazebo is below real time
TOLERANCE_M = 1.0
POLL_S = 0.01
SEPARATION_TOLERANCE_M = 5.0


class PX4Sim:
    """The scenario.Sim of two PX4 drones in Gazebo, on the interceptor's clock."""

    def __init__(self, hostile, interceptor):
        self.hostile = hostile
        self.interceptor = interceptor

    def now(self) -> float | None:
        return self.interceptor.stamp

    def advance(self, dt: float) -> None:
        start = self.now()
        while start is not None and (self.now() or start) - start < dt:
            time.sleep(POLL_S)
        if start is None:
            time.sleep(dt)


def noisy_truth(sim, scenario, config):
    return NoisyTruthDetector(sim.interceptor, lambda: sim.hostile.position, config,
                              random.Random(scenario.seed))


def fly_sim(scenario, settings):
    """The scenario in the fast simulation: its samples."""
    from swarm_backend.fast_sim import FastSim
    sim = FastSim(scenario, interceptor_max_speed=settings.interceptor_max_speed)
    return run_scenario(scenario, sim, PurePursuitAction, noisy_truth, settings)


def fly_gazebo(config, args):
    """The scenario with two PX4 drones: (scenario as flown, samples), or None."""
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions

    from swarm_backend.px4.px4_drone import PX4Drone
    from swarm_backend.runner import LAND_TIMEOUT_S, spinning, wait_until
    from swarm_backend.swarm.manager import SwarmManager
    from swarm_backend.swarm.swarm import Swarm

    settings = config.intercept
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('intercept_baseline_scenario')
    log = node.get_logger()
    hostile = PX4Drone(node, config.drone(settings.hostile_id), rate_hz=config.rate_hz)
    hunter = PX4Drone(node, config.drone(settings.interceptor_id), rate_hz=config.rate_hz)
    manager = SwarmManager(Swarm([hostile, hunter]), TOLERANCE_M)
    sim = PX4Sim(hostile, hunter)
    scenario = None
    result = None

    with spinning(node):
        try:
            wait_until(manager.connected, 30, 'status and position from both drones')
            scenario = make_scenario(args.aspect, args.speed, args.seed, settings)
            x, y, _ = hunter.position
            sx, sy, _ = scenario.interceptor_start
            scenario = scenario.shifted((x - sx, y - sy, 0.0))
            try:  # fail before takeoff, not in the air
                PurePursuitAction(hunter, noisy_truth(sim, scenario, settings), settings, sim.now)
            except NotImplementedError:
                log.error('The baseline is not implemented yet (PurePursuitAction, '
                          'NoisyTruthDetector)')
                return None
            wait_until(manager.ready, 60, 'pre-flight checks to pass on both drones')
            manager.takeoff(settings.altitude)
            wait_until(manager.all_reached, CLIMB_TIMEOUT_S, f'both drones at {settings.altitude} m')
            hostile.set_position(*scenario.target_start)
            hunter.set_position(*scenario.interceptor_start)
            wait_until(manager.all_reached, CLIMB_TIMEOUT_S, 'the scenario start')
            result = scenario, run_scenario(scenario, sim, PurePursuitAction, noisy_truth,
                                            settings)
        except (TimeoutError, RuntimeError, KeyboardInterrupt) as e:
            log.error(f'Aborting ({e or "Ctrl-C"})')
        finally:
            armed = [d for d in (hostile, hunter) if d.armed]
            if scenario is not None and hostile in armed and hostile.offboard:
                # After a hit they are 1.5 m apart: separate before landing straight down.
                log.info('Separating the drones')
                hunter.hold()
                hostile.set_position(*scenario.target_start)
                try:
                    wait_until(lambda: hostile.reached(SEPARATION_TOLERANCE_M), 60,
                               'the hostile drone to fly back')
                except (TimeoutError, KeyboardInterrupt) as e:
                    log.warning(f'Landing where they are ({e or "Ctrl-C"})')
            if armed:
                log.info('Landing')
                manager.land([d.id for d in armed])
                wait_until(manager.all_landed, LAND_TIMEOUT_S, 'both drones to land and disarm')

    node.destroy_node()
    rclpy.shutdown()
    return result


def write_csv(path: str, samples) -> None:
    with open(path, 'w', newline='') as f:
        rows = [{**asdict(s), 'status': s.status.value} for s in samples]
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog='intercept_baseline',
                                     description=__doc__.splitlines()[0])
    common.add_backend_args(parser)
    parser.add_argument('--config', help='swarm.yaml (default: config/swarm.yaml)')
    parser.add_argument('--aspect', type=float, default=0.0,
                        help='degrees between the target track and its line of sight to '
                             'the interceptor (0 = head-on, 90 = crossing)')
    parser.add_argument('--speed', type=float, default=4.0, help='target speed, m/s')
    parser.add_argument('--seed', type=int, default=0, help='ring position, side and altitude')
    parser.add_argument('--csv', help='write the samples here')
    args = parser.parse_args(argv)
    config = load_config(common.config_path(args.config, common.SWARM_CONFIG))
    settings = config.intercept
    print(f'Scenario: aspect {args.aspect} deg, {args.speed} m/s, seed {args.seed}, '
          f'{"fast simulation" if args.sim else "Gazebo"}', flush=True)

    if args.sim:
        scenario = make_scenario(args.aspect, args.speed, args.seed, settings)
        try:
            samples = fly_sim(scenario, settings)
        except NotImplementedError:
            print('The baseline is not implemented yet (PurePursuitAction, NoisyTruthDetector)')
            return 1
    else:
        result = fly_gazebo(config, args)
        if result is None:
            return 1
        scenario, samples = result
    where = 'fast simulation' if args.sim else 'Gazebo'
    print(summarize([score_run(samples, scenario, settings)],
                    f'Baseline in {where}, seed {args.seed}'))
    if args.csv:
        write_csv(args.csv, samples)
        print(f'Wrote {len(samples)} samples to {args.csv}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
