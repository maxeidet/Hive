"""Baseline attack flight test (work plan tasks 4-6): the defender hovers, the hostile drone
flies past above it, and the attack tries to fly into it. Scored from Gazebo's true positions.

Both drones are real PX4 drones in Gazebo: the defender is `attack.attacker_id` (a gimbal
drone, drone 2 in swarm.yaml) and the hostile drone is `attack.target_id` (drone 0). The
attack knows only the two detection topics. By default this test runs the fake detector
itself; --external-detector leaves that to the camera detector (or `fakedetect`) running
next to it. Each run: the defender hovers at its guard position looking toward the
hostile drone's side, the hostile drone flies the scenario (crossing, headon, circle or
offset, intercept/attack_scenario.py), and the run ends with

  hit            the two within --hit-radius (1.5 m) of each other, from the true positions
  target_passed  the hostile drone reached the end of its line
  timeout        --timeout simulation seconds without a hit

after which the drones move apart and go back to their start points for the next run.
After the last run both fly home at different heights and land. Each run adds one row
to --csv (default data/attack_runs.csv in the repo). Inside the container:

    python3 /workspace/ros2_ws/src/swarm_backend/tests/attack_test.py --pattern crossing
    python3 .../attack_test.py --pattern headon --runs 10 --realistic
    python3 .../attack_test.py --pattern offset     # simplest: same height, 8 m to the side

On the host: ./mount-backend.sh attacktest [same arguments].

A hit means the defender flew into the hostile drone at up to attack.max_speed (5 m/s): in
Gazebo the two can really collide and tumble. The test then still tries to land them; if one has
crashed, restart the simulation.
"""
import argparse
import csv
import datetime
import math
import time
from pathlib import Path

import rclpy
from rclpy.signals import SignalHandlerOptions

from swarm_backend import ground_truth
from swarm_backend.config import default_config_path, load_config
from swarm_backend.intercept.attack import State
from swarm_backend.intercept.attack_scenario import (
    PATTERNS, ClosestApproach, Scenario, hostile_path)
from swarm_backend.intercept.mock import HostileFlight
from swarm_backend.nodes.attack_node import AttackNode
from swarm_backend.nodes.fake_detection import (
    FakeDetectionNode, GazeboTruth, fake_config, find_scene)
from swarm_backend.px4.px4_drone import PX4Drone
from swarm_backend.runner import LAND_TIMEOUT_S, spinning, wait_until
from swarm_backend.swarm.manager import SwarmManager
from swarm_backend.swarm.swarm import Swarm

TOLERANCE_M = 1.5
SETUP_TIMEOUT_S = 900.0  # wall clock: climbs and repositioning, slow when Gazebo is
SETTLE_S = 3.0  # simulation seconds of hovering before each run
APART_M = 8.0  # after a run the hostile drone climbs this much, away from the defender
APART_TIMEOUT_S = 120.0  # wall clock; a drone that crashed in a hit never gets there
HOME_GAP_M = 10.0  # going home, the hostile drone flies this much higher than the defender
POLL_S = 0.05
DEFAULT_CSV = Path('/workspace/data/attack_runs.csv')  # /workspace is the repo on the host
FIELDS = ['time', 'run', 'pattern', 'hostile_speed', 'height', 'ahead', 'half_width', 'detector',
          'gimbal_gain', 'max_speed', 'seed', 'result', 'hit', 'first_detection_s',
          'time_to_hit_s', 'closest_m', 'closest_at_s', 'messages', 'search_s', 'attack_s',
          'coast_s', 'lost_s']


class AttackTest:
    """Everything the test needs, wired to one ROS node."""

    def __init__(self, node, swarm_config, args):
        self.args = args
        self.log = node.get_logger()
        self.config = config = fake_config(
            swarm_config.attack, args.realistic, gimbal_gain=args.gain,
            max_speed=args.max_speed, seed=args.seed)
        self.scenario = Scenario(args.pattern, args.speed, args.height, args.ahead, args.half_width)
        self.detector = 'external' if args.external_detector else (
            'fake-realistic' if args.realistic else 'fake')

        world, self.names, camera_link = find_scene(config)  # RuntimeError: say what to start
        # Everything is timed on Gazebo's simulation clock: steady however slowly Gazebo
        # runs, unlike PX4's time stamps, which can jump when its time sync adjusts.
        self.truth = GazeboTruth(world)
        self.clock = self.truth.clock
        self.attack_node = AttackNode(node, swarm_config, config, clock=self.clock)
        self.attacker = self.attack_node.drone
        self.hostile = PX4Drone(node, swarm_config.drone(config.target_id),
                                rate_hz=swarm_config.rate_hz)
        self.manager = SwarmManager(Swarm([self.hostile, self.attacker]), TOLERANCE_M)
        self.fake = None if args.external_detector else FakeDetectionNode(
            node, config, self.truth, self.names, camera_link)
        self.judge: ClosestApproach | None = None
        self.truth.listeners.append(self._on_truth)
        self.flight: HostileFlight | None = None
        node.create_timer(1.0 / swarm_config.rate_hz, self._on_timer)
        self.results: list[dict] = []

    # --- callbacks ------------------------------------------------------------

    def _on_timer(self) -> None:
        if self.flight is not None:
            self.flight.step()

    def _on_truth(self, stamp: float, poses) -> None:
        judge = self.judge
        if judge is None:
            return
        a = ground_truth.body_position(poses, self.names[self.config.attacker_id])
        h = ground_truth.body_position(poses, self.names[self.config.target_id])
        if a is not None and h is not None:
            judge.update(a, h, stamp)  # simulation time, the attack's clock too

    # --- flying ---------------------------------------------------------------

    @property
    def guard(self):
        x, y, z = self.attacker.home
        return (x, y, z - self.config.guard_altitude)

    def path(self):
        return hostile_path(self.scenario, self.guard, math.radians(self.config.search_heading_deg))

    def wait_sim(self, seconds: float) -> None:
        start = self.clock()
        wait_until(lambda: self.clock() - start >= seconds, SETUP_TIMEOUT_S,
                   f'{seconds} s of simulation time')

    def setup(self) -> None:
        c, s = self.config, self.scenario
        try:
            wait_until(self.manager.connected, 30, 'status and position from both drones')
        except TimeoutError:
            raise TimeoutError(
                f'No data from PX4 for drones {self.manager.not_connected()}. Start them with '
                '`./mount-backend.sh spawn`') from None
        wait_until(self.manager.ready, 60, 'pre-flight checks to pass on both drones')
        wait_until(lambda: self.clock() is not None, 30, "Gazebo's poses")
        self.log.info(f'Taking off: defender {self.attacker.id} to {c.guard_altitude} m, '
                      f'hostile {self.hostile.id} to {c.guard_altitude + s.height_m} m')
        self.manager.takeoff(c.guard_altitude + s.height_m, [self.hostile.id])
        self.manager.takeoff(c.guard_altitude, [self.attacker.id])
        wait_until(self.manager.all_reached, SETUP_TIMEOUT_S, 'both take-off heights')

    def to_start(self) -> None:
        """Hostile drone to the start of its path, defender to its guard position."""
        start, velocity = self.path()[0].at(0.0)
        yaw = math.atan2(velocity[1], velocity[0])
        self.attacker.set_position(*self.guard, yaw=math.radians(self.config.search_heading_deg))
        if abs(self.scenario.height_m) < APART_M:
            # Close to the defender's height, the straight way to the start could go
            # through the defender: climb above it, fly across, then come down.
            transit = self.guard[2] - APART_M
            x, y, z = self.hostile.position
            self.hostile.set_position(x, y, min(z, transit), yaw=yaw)
            wait_until(lambda: self.hostile.reached(TOLERANCE_M), SETUP_TIMEOUT_S,
                       'the hostile drone to climb above the defender')
            self.hostile.set_position(start[0], start[1], min(z, transit), yaw=yaw)
            wait_until(lambda: self.hostile.reached(TOLERANCE_M), SETUP_TIMEOUT_S,
                       'the hostile drone to get above its start point')
        self.hostile.set_position(*start, yaw=yaw)
        wait_until(self.manager.all_reached, SETUP_TIMEOUT_S, 'the start positions')
        self.wait_sim(SETTLE_S)

    def run(self, index: int) -> dict:
        c, s, args = self.config, self.scenario, self.args
        self.to_start()
        path, flight_time = self.path()
        attack = self.attack_node.new_attack()
        messages_before = self.attack_node.inbox.position_count
        judge = ClosestApproach(args.hit_radius)

        self.flight = HostileFlight(self.hostile, path, self.clock)
        self.judge = judge
        self.flight.start()
        attack.start()
        t0 = self.clock()
        what = f'{flight_time:.0f} s line' if flight_time else 'circle'
        self.log.info(f'Run {index + 1}/{args.runs}: {s.pattern} at {s.speed} m/s ({what}), '
                      f'{s.height_m} m above the defender, detector: {self.detector}')
        seen, result = 0, 'timeout'
        try:
            while True:
                t = self.clock() - t0
                for when, state in attack.transitions[seen:]:
                    self.log.info(f'  t={when - t0:5.1f} s {state.value.upper()}, closest so far '
                                  f'{judge.closest:.1f} m')
                seen = len(attack.transitions)
                if judge.hit:
                    result = 'hit'
                    break
                if flight_time is not None and t >= flight_time:
                    result = 'target_passed'
                    break
                if t >= args.timeout:
                    break
                time.sleep(POLL_S)
        finally:
            self.judge = None
            if judge.hit:
                attack.hit()
            else:
                attack.stop()
            self.flight.stop()
            self.flight = None

        row = self.row(index, result, attack, judge, t0, messages_before)
        self.log.info(f'Run {index + 1}: {result.upper()}, closest {judge.closest:.2f} m'
                      + (f', {row["time_to_hit_s"]} s from first detection' if judge.hit else ''))
        self.results.append(row)
        self.write_row(row)
        self.move_apart()
        return row

    def row(self, index, result, attack, judge, t0, messages_before) -> dict:
        c, s = self.config, self.scenario
        first = attack.first_detection
        times = attack.state_time
        return {
            'time': datetime.datetime.now().isoformat(timespec='seconds'), 'run': index + 1,
            'pattern': s.pattern, 'hostile_speed': s.speed, 'height': s.height_m,
            'ahead': s.ahead_m, 'half_width': s.half_width_m, 'detector': self.detector,
            'gimbal_gain': c.gimbal_gain, 'max_speed': c.max_speed, 'seed': c.seed,
            'result': result, 'hit': int(judge.hit),
            'first_detection_s': '' if first is None else round(first - t0, 2),
            'time_to_hit_s': round(judge.hit_time - first, 2) if judge.hit and first else '',
            'closest_m': round(judge.closest, 2),
            'closest_at_s': '' if judge.closest_time is None else round(judge.closest_time - t0, 2),
            'messages': self.attack_node.inbox.position_count - messages_before,
            'search_s': round(times[State.SEARCH], 2), 'attack_s': round(times[State.ATTACK], 2),
            'coast_s': round(times[State.COAST], 2), 'lost_s': round(times[State.LOST], 2),
        }

    def write_row(self, row: dict) -> None:
        path = Path(self.args.csv)
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            new = not path.exists() or path.stat().st_size == 0
            with open(path, 'a', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=FIELDS)
                if new:
                    writer.writeheader()
                writer.writerow(row)
        except OSError as e:
            self.log.error(f'Could not write {path}: {e}')

    def move_apart(self) -> None:
        """Hostile drone up, defender back to its guard position, both away from each other."""
        if self.hostile.offboard and self.hostile.position is not None:
            x, y, z = self.hostile.position
            self.hostile.set_position(x, y, min(z, self.guard[2] - self.scenario.height_m) - APART_M)
        if self.attacker.offboard:
            self.attacker.set_position(*self.guard)
        try:
            wait_until(self.manager.all_reached, APART_TIMEOUT_S, 'the drones to move apart')
        except TimeoutError as e:
            self.log.warning(str(e))

    def stop_and_land(self) -> None:
        self.attack_node.attack.stop()
        if self.flight is not None:
            self.flight.stop()
        armed = [d for d in (self.hostile, self.attacker) if d.armed]
        if not armed:
            return
        # Their way home could cross: go at different heights, then land.
        self.log.info('Returning above the start points')
        heights = {self.attacker.id: self.config.guard_altitude,
                   self.hostile.id: self.config.guard_altitude + HOME_GAP_M}
        try:
            for drone in armed:
                if drone.offboard:
                    x, y, _ = drone.position
                    drone.set_position(x, y, drone.home[2] - heights[drone.id])
            wait_until(lambda: all(d.reached(TOLERANCE_M) for d in armed if d.offboard),
                       SETUP_TIMEOUT_S, 'the return heights')
            for drone in armed:
                if drone.offboard:
                    drone.set_position(drone.home[0], drone.home[1], drone.target[2])
            wait_until(lambda: all(d.reached(2.0) for d in armed if d.offboard),
                       SETUP_TIMEOUT_S, 'the drones to get home')
        except (TimeoutError, KeyboardInterrupt) as e:
            self.log.warning(f'Landing where they are ({e or "Ctrl-C"})')
        self.log.info('Landing')
        self.manager.land([d.id for d in armed])
        # Wall clock: from 30 m at PX4's landing speed, minutes when Gazebo runs slowly.
        wait_until(self.manager.all_landed, max(LAND_TIMEOUT_S, SETUP_TIMEOUT_S),
                   'both drones to land and disarm')
        self.log.info('Landed')

    def summary(self) -> str:
        rows = self.results
        if not rows:
            return 'No runs finished.'
        hits = [r for r in rows if r['hit']]
        lines = [f'{len(hits)}/{len(rows)} hits ({self.scenario.pattern}, {self.scenario.speed} m/s, '
                 f'detector {self.detector}, gimbal gain {self.config.gimbal_gain})']
        if hits:
            times = sorted(r['time_to_hit_s'] for r in hits if r['time_to_hit_s'] != '')
            if times:
                lines.append(f'  time from first detection to hit: median {times[len(times) // 2]} s')
        misses = [r['closest_m'] for r in rows if not r['hit']]
        if misses:
            lines.append(f'  misses: closest {", ".join(f"{m:.1f}" for m in misses)} m')
        lines.append(f'  one row per run in {self.args.csv}')
        return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--config', default=None, help='swarm.yaml (default: the installed one)')
    parser.add_argument('--pattern', choices=PATTERNS, default='crossing')
    parser.add_argument('--speed', type=float, default=1.0, help='hostile speed, m/s')
    parser.add_argument('--height', type=float, default=None,
                        help='metres the hostile drone flies above the defender '
                             '(default 10, 0 for offset)')
    parser.add_argument('--ahead', type=float, default=None,
                        help='metres in front of the defender (default per pattern)')
    parser.add_argument('--half-width', type=float, default=None,
                        help='crossing half-length, circle radius or offset to the side, metres')
    parser.add_argument('--runs', type=int, default=1)
    parser.add_argument('--timeout', type=float, default=150.0,
                        help='simulation seconds per run (the 1 m/s lines take 100-120 s)')
    parser.add_argument('--hit-radius', type=float, default=1.5, help='metres')
    parser.add_argument('--realistic', action='store_true',
                        help='fake detector with 0.25 s delay, 1 deg and 20 %% noise, 5 m blind zone')
    parser.add_argument('--external-detector', action='store_true',
                        help='do not run the fake detector; another node publishes the topics')
    parser.add_argument('--gain', type=float, default=None, help='gimbal gain (attack config)')
    parser.add_argument('--max-speed', type=float, default=None, help='m/s (attack config)')
    parser.add_argument('--seed', type=int, default=None, help='fake detector noise')
    parser.add_argument('--csv', default=str(DEFAULT_CSV), help='append one row per run here')
    args = parser.parse_args()

    swarm_config = load_config(args.config or default_config_path())
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('attack_test')
    try:
        test = AttackTest(node, swarm_config, args)
    except RuntimeError as e:
        print(e)
        node.destroy_node()
        rclpy.try_shutdown()
        return 1
    if test.fake is not None:
        print(test.fake.describe(), flush=True)
    code = 0
    with spinning(node):
        try:
            test.setup()
            for i in range(args.runs):
                test.run(i)
        except KeyboardInterrupt:
            test.log.info('Ctrl-C')
        except TimeoutError as e:
            test.log.error(str(e))
            code = 1
        finally:
            try:
                test.stop_and_land()
            except (TimeoutError, KeyboardInterrupt) as e:
                test.log.error(f'Did not land cleanly: {e or "Ctrl-C"}')
                code = 1
            print(test.summary(), flush=True)
    node.destroy_node()
    rclpy.try_shutdown()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
