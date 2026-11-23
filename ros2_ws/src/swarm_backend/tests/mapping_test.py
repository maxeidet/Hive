"""3D mapping flight test: the map from one drone's depth camera or lidar grows as it flies.

Not a report section yet; it checks the mapping pipeline of hive-vault/mapping/3D mapping
plan.md (swarm_backend/mapping.py: point cloud bridge, TF from Gazebo, octomap_server)
before the DAEP planner flies the drone. Flight:

  start mapping -> wait for the first map -> take off to --alt -> fly --distance m along
  --bearing -> hover --hover sim s -> land

PASS when, after the flight, the map has at least --min-growth times the occupied voxels it
had on the ground, and reaches at least --min-reach-gain m further along the bearing (from
the take-off point). Waits are in simulation time, since Gazebo with a depth camera often
runs at 0.1x real time. Writes runs/mapping_test/<time>_b<bearing>_d<distance>/ (meta.json,
samples.csv, metrics.json, map.bt) in the repo, see hive-vault/Testing.md section 4.

Needs PX4 SITL, Gazebo and the DDS agent running, with the drone spawned as a depth or lidar
model, e.g. `./mount-backend.sh restart -m gz_x500_depth`. Then:

    ./mount-backend.sh maptest                       # drone 0, 3 m up, 6 m north
    ./mount-backend.sh maptest --distance 10 --bearing 90 --alt 4
"""
import argparse
import csv
import json
import math
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rosgraph_msgs.msg import Clock
from visualization_msgs.msg import MarkerArray

from swarm_backend.config import default_config_path, load_config
from swarm_backend.mapping import (
    map_stats, ned_to_enu, point_clouds, reach, ros_executable, saver_args)
from swarm_backend.px4.px4_drone import PX4Drone
from swarm_backend.runner import LAND_TIMEOUT_S, spinning, wait_until
from swarm_backend.sim.sensor_view import find_drone, find_sensors, gz_topics

RUNS = Path('/workspace/runs/mapping_test')  # /workspace = the repo; runs/ is not committed
SAMPLE_PERIOD_S = 0.1  # sim seconds between rows of samples.csv (10 Hz)
FIELDS = ['t', 'drone_x', 'drone_y', 'drone_z', 'drone_vx', 'drone_vy', 'drone_vz',
          'drone_armed', 'drone_offboard', 'phase', 'occupied_voxels']


def git_commit() -> tuple[str, bool]:
    try:
        commit = subprocess.run(['git', '-C', '/workspace', 'rev-parse', 'HEAD'],
                                capture_output=True, text=True, check=True).stdout.strip()
        dirty = bool(subprocess.run(['git', '-C', '/workspace', 'status', '--porcelain'],
                                    capture_output=True, text=True).stdout.strip())
        return commit, dirty
    except (OSError, subprocess.CalledProcessError):
        return '', False


class Run:
    def __init__(self, node: Node, config, args):
        self.node, self.args, self.log = node, args, node.get_logger()
        self.config = config
        self.drone = PX4Drone(node, config.drone(args.drone_id), rate_hz=config.rate_hz)
        self.ns = f'mapping_test_{args.drone_id}'
        self.points: list[tuple[float, float, float]] = []
        self.map_count = 0  # map messages received
        self.phase = 'setup'
        self.rows: list[dict] = []
        self.metrics: dict = {}
        self.mapping: subprocess.Popen | None = None
        self.model = ''
        self._lock = threading.Lock()
        self._last_row: float | None = None
        self._t0: float | None = None
        # Simulation time from Gazebo's clock, which mapping.py bridges to /mapping_<id>/clock.
        # Not PX4's time stamps: they can jump (seen: +105 s) when its time sync adjusts.
        self.clock: float | None = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        node.create_subscription(MarkerArray, f'/{self.ns}/occupied_cells_vis_array',
                                 self._on_map, qos)
        node.create_subscription(Clock, f'/mapping_{args.drone_id}/clock', self._on_clock, 10)
        node.create_timer(0.02, self._record)

    # --- callbacks (executor thread) ---------------------------------------

    def _on_clock(self, msg: Clock) -> None:
        self.clock = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _on_map(self, msg: MarkerArray) -> None:
        points = [(p.x, p.y, p.z) for marker in msg.markers for p in marker.points]
        with self._lock:
            self.points = points
            self.map_count += 1

    def _record(self) -> None:
        d, now = self.drone, self.clock
        if now is None or d.position is None:
            return
        if self._t0 is None:
            self._t0 = now
        if self._last_row is not None and now - self._last_row < SAMPLE_PERIOD_S:
            return
        self._last_row = now
        vel = d.velocity or ('', '', '')
        with self._lock:
            occupied = len(self.points)
        self.rows.append({
            't': round(now - self._t0, 3),
            'drone_x': d.position[0], 'drone_y': d.position[1], 'drone_z': d.position[2],
            'drone_vx': vel[0], 'drone_vy': vel[1], 'drone_vz': vel[2],
            'drone_armed': int(d.armed), 'drone_offboard': int(d.offboard),
            'phase': self.phase, 'occupied_voxels': occupied,
        })

    # --- the flight (main thread) ------------------------------------------

    def wait(self, predicate, sim_timeout: float, what: str) -> None:
        """Like runner.wait_until, but the timeout is in simulation time (Gazebo's clock)."""
        start = None
        while not predicate():
            now = self.clock
            if start is None:
                start = now
            elif now is not None and now - start > sim_timeout:
                raise TimeoutError(f'Timed out after {sim_timeout:.0f} sim s waiting for {what}')
            time.sleep(0.1)

    def sleep_sim(self, seconds: float) -> None:
        start = self.clock
        self.wait(lambda: self.clock - start >= seconds, seconds + 30, 'the hover to end')

    def fresh_map(self) -> list[tuple[float, float, float]]:
        """The map after at least one more update than now (octomap_server publishes it
        after each point cloud it has merged)."""
        with self._lock:
            seen = self.map_count
        self.wait(lambda: self.map_count > seen, 30, 'a map update')
        with self._lock:
            return list(self.points)

    def check_sensor(self) -> None:
        """Fail before take-off if the drone cannot map."""
        topics = gz_topics()
        drone = find_drone(topics, self.args.drone_id)
        if drone is None:
            raise RuntimeError(f'Drone {self.args.drone_id} is not in Gazebo')
        if not point_clouds(find_sensors(topics, self.args.drone_id)):
            raise RuntimeError(f'Drone {self.args.drone_id} ({drone.name}) has no depth camera '
                               'or lidar. Use e.g. ./mount-backend.sh restart -m gz_x500_depth')
        self.model, self.world = drone.model, drone.world

    def fly(self) -> None:
        args, d = self.args, self.drone
        self.check_sensor()
        self.log.info(f'Starting mapping for drone {args.drone_id} on /{self.ns}')
        self.mapping = subprocess.Popen(
            ['python3', '-m', 'swarm_backend.mapping', '--id', str(args.drone_id),
             '--ns', self.ns, '--resolution', str(args.resolution),
             '--max-range', str(args.max_range)],
            stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)

        wait_until(lambda: d.connected and d.position is not None, 180,
                   'PX4 status and position (is the drone spawned?)')
        wait_until(lambda: self.clock is not None, 60, "Gazebo's clock from the mapping bridge")
        self.wait(lambda: d.preflight_ok, 120, 'pre-flight checks')
        self.phase = 'ground'
        self.wait(lambda: self.map_count > 0, 120, 'the first map from octomap_server')
        before = self.fresh_map()
        start = ned_to_enu(d.position)
        self.log.info(f'Map on the ground: {len(before)} occupied voxels')

        self.wall_start, self.sim_start = time.monotonic(), self.clock
        self.phase = 'takeoff'
        d.arm()
        self.wait(lambda: d.armed, 30, 'arming')
        d.start_offboard()
        self.wait(lambda: d.offboard, 30, 'offboard mode')
        d.takeoff(args.alt)
        self.wait(lambda: d.reached(self.config.position_tolerance), 60, 'take-off altitude')

        self.phase = 'move'
        x, y, z = d.target
        b = math.radians(args.bearing)
        self.log.info(f'Flying {args.distance} m at bearing {args.bearing} deg')
        d.set_position(x + args.distance * math.cos(b), y + args.distance * math.sin(b), z)
        self.wait(lambda: d.reached(self.config.position_tolerance),
                  60 + 10 * args.distance, 'the move target')

        self.phase = 'hover'
        self.sleep_sim(args.hover)
        after = self.fresh_map()
        self.sim_rate = (self.clock - self.sim_start) / max(time.monotonic() - self.wall_start, 1e-6)
        self.score(before, after, start)

    def score(self, before, after, start) -> None:
        args = self.args
        b0, b1 = map_stats(before), map_stats(after)
        r0, r1 = reach(before, start, args.bearing), reach(after, start, args.bearing)
        growth = b1.occupied / b0.occupied if b0.occupied else math.inf
        gain = (r1 - r0) if r0 is not None and r1 is not None else None
        min_gain = args.min_reach_gain if args.min_reach_gain is not None else args.distance / 2
        self.metrics = {
            'occupied_before': b0.occupied, 'occupied_after': b1.occupied,
            'growth': round(growth, 2),
            'reach_before_m': r0, 'reach_after_m': r1,
            'reach_gain_m': round(gain, 2) if gain is not None else None,
            'bbox_after_enu': {'min': b1.min, 'max': b1.max},
            'min_growth': args.min_growth, 'min_reach_gain_m': min_gain,
            'passed': growth >= args.min_growth and gain is not None and gain >= min_gain,
        }

    def stop_and_land(self) -> None:
        d = self.drone
        if d.armed:
            self.phase = 'land'
            self.log.info('Landing')
            d.land()
            try:
                wait_until(lambda: not d.armed, LAND_TIMEOUT_S, 'landing and auto-disarm')
            except TimeoutError as e:
                self.log.error(str(e))

    def save_map(self, folder: Path) -> None:
        """Ask octomap_server for the map and write it to map.bt (before mapping stops)."""
        saver = subprocess.Popen(
            [ros_executable('octomap_server', 'octomap_saver_node')]
            + saver_args(self.ns, str(folder / 'map.bt')),
            stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
        try:
            saver.wait(timeout=60)
        except subprocess.TimeoutExpired:
            saver.kill()
            saver.wait()
        if not (folder / 'map.bt').exists():
            self.log.warning('Could not save map.bt (octomap_saver got no map)')

    def write_run(self) -> Path:
        args = self.args
        folder = RUNS / (f'{datetime.now():%Y-%m-%d_%H%M%S}'
                         f'_b{args.bearing:g}_d{args.distance:g}')
        folder.mkdir(parents=True, exist_ok=True)
        commit, dirty = git_commit()
        meta = {
            'test': 'mapping_test', 'scenario': {k: getattr(args, k) for k in (
                'drone_id', 'alt', 'distance', 'bearing', 'hover')},
            'seed': None, 'method': 'swarm_backend.mapping (octomap_server, TF from Gazebo truth)',
            'config': {'resolution': args.resolution, 'max_range': args.max_range,
                       'position_tolerance': self.config.position_tolerance},
            'args': vars(args), 'git_commit': commit, 'dirty': dirty,
            'world': getattr(self, 'world', ''), 'drone_models': {args.drone_id: self.model},
            'started': self.started, 'sim_rate': round(getattr(self, 'sim_rate', 0.0), 3),
        }
        (folder / 'meta.json').write_text(json.dumps(meta, indent=2, default=str))
        (folder / 'metrics.json').write_text(json.dumps(self.metrics, indent=2, default=str))
        if self.rows:
            with open(folder / 'samples.csv', 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=FIELDS)
                writer.writeheader()
                writer.writerows(self.rows)
        if self.map_count:
            self.save_map(folder)
        return folder

    def summary(self) -> str:
        m = self.metrics
        if not m:
            return 'mapping_test: no result (aborted before the flight finished)'
        return '\n'.join([
            f'mapping_test: {"PASSED" if m["passed"] else "FAILED"}',
            f'  occupied voxels  {m["occupied_before"]} on the ground -> {m["occupied_after"]} '
            f'after the flight (x{m["growth"]}, need x{m["min_growth"]})',
            f'  reach at {self.args.bearing:g} deg  {m["reach_before_m"]:.1f} m -> '
            f'{m["reach_after_m"]:.1f} m (+{m["reach_gain_m"]} m, need +{m["min_reach_gain_m"]:g} m)',
            f'  sim rate {getattr(self, "sim_rate", 0.0):.2f}x real time',
        ])


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--config', help='swarm.yaml (default: the installed one)')
    parser.add_argument('--drone-id', type=int, default=0)
    parser.add_argument('--alt', type=float, default=3.0, help='flight altitude, m')
    parser.add_argument('--distance', type=float, default=6.0, help='m to fly')
    parser.add_argument('--bearing', type=float, default=0.0, help='degrees from north')
    parser.add_argument('--hover', type=float, default=10.0, help='sim seconds at the end point')
    parser.add_argument('--resolution', type=float, default=0.1, help='voxel size, m')
    parser.add_argument('--max-range', type=float, default=8.0, help='sensor range for the map, m')
    parser.add_argument('--min-growth', type=float, default=1.5,
                        help='occupied voxels after / before needed to pass')
    parser.add_argument('--min-reach-gain', type=float,
                        help='m the map must reach further along the bearing (default distance/2)')
    args = parser.parse_args()

    config = load_config(args.config or default_config_path())
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('mapping_test')
    run = Run(node, config, args)
    run.started = datetime.now().isoformat(timespec='seconds')
    passed = False

    with spinning(node):
        try:
            run.fly()
            passed = bool(run.metrics.get('passed'))
        except (TimeoutError, RuntimeError, KeyboardInterrupt) as e:
            run.log.error(f'Aborting ({e or "Ctrl-C"})')
        finally:
            run.stop_and_land()
            run.phase = 'done'
            folder = run.write_run()
            if run.mapping is not None:
                run.mapping.terminate()
                run.mapping.wait()
            print(run.summary(), flush=True)
            print(f'Run folder: {folder}', flush=True)

    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if passed else 1)


if __name__ == '__main__':
    main()
