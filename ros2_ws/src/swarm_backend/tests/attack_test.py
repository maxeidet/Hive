"""Surveillance attack test: the whole chain in Gazebo, scored from the true positions.

The defenders patrol the arc, a hostile drone flies in toward the base, the base station
sends one defender, and the defender tries to fly into it.

Settings come from the `surveillance:` and `attack:` sections of the config (default
scenarios/surveillance.yaml): the defenders are surveillance.drone_ids (gimbal drones 1-3),
the hostile drone is attack.target_id (drone 4; `spawn` starts it with the defenders). It
cannot be drone 0: the start scripts run drone 0's PX4 as the container's own process, so
stopping it stops the container. In one process this test runs

  - a patrol for each defender (nodes/patrol_node.py): take off, hold the slot on the arc,
    sweep the camera, attack when the base station allows it
  - the base station (nodes/base_station_node.py)
  - a detector for each defender: the real camera detector (`detect ID`, started here, log
    in /tmp/detect_<id>.log), or with --fake-detector the fake one from Gazebo's poses
  - the referee: the hostile drone's flight, and the hits

Each run: the hostile drone is (re)started on the ground `--start` metres from the base,
`--bearing` degrees off the threat heading, climbs to `--height` metres above the
defenders and flies straight at the base at `--speed`. The run ends with

  hit           a defender within --hit-radius (1.5 m) of it, from the true positions. The
                hostile drone is shot down: its PX4 is stopped and the model moved to a
                graveyard far outside the battle (GRAVEYARD_ENU). The base station gets a
                `kill` event and the defender, which survives, flies back to its slot
  reached_base  it got to the base

After a run that did not end in a hit the hostile drone goes to the graveyard too. The next run
starts when every defender is back on patrol with its sector. After the last run the
defenders land at their slots and the hostile drone is started again at its spawn point.
Each run adds one row to --csv (default data/surveillance_attack_runs.csv in the repo).
Inside the container:

    python3 /workspace/ros2_ws/src/swarm_backend/tests/attack_test.py --config <yaml>
    python3 .../attack_test.py --bearing -30 0 30 --runs 3      # one run through each slot
    python3 .../attack_test.py --fake-detector --realistic      # no camera detector

On the host: ./mount-backend.sh attacktest [same arguments].

A hit means the defender flew into the hostile drone at up to attack.max_speed (5 m/s).
The hostile drone is taken away at once, but the defender can still tumble in Gazebo; if one
has crashed, the base station counts it as lost and the others carry on.
"""
import argparse
import csv
import dataclasses
import datetime
import math
import os
import shlex
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import rclpy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from swarm_backend import ground_truth
from swarm_backend.config import default_config_path, load_config
from swarm_backend.intercept import surveillance_msgs as msgs
from swarm_backend.intercept.attack_scenario import (
    DEFAULT_HEIGHT_M, DEFAULT_START_M, ClosestApproach, Inbound, inbound_path)
from swarm_backend.intercept.base_station import Mode
from swarm_backend.intercept.mock import HostileFlight
from swarm_backend.nodes.base_station_node import BaseStationNode
from swarm_backend.nodes.fake_detection import FakeDetectionNode, GazeboTruth, fake_config
from swarm_backend.nodes.patrol_node import PatrolNode
from swarm_backend.px4.px4_drone import PX4Drone
from swarm_backend.runner import LAND_TIMEOUT_S, spinning, wait_until
from swarm_backend.save_images import drone_names
from swarm_backend.sensor_view import find_sensors
from swarm_backend.swarm.manager import SwarmManager
from swarm_backend.swarm.swarm import Swarm

TOLERANCE_M = 1.5
SLOT_TOLERANCE_M = 2.0
SETUP_TIMEOUT_S = 900.0  # wall clock: climbs and repositioning, slow when Gazebo is
CONNECT_TIMEOUT_S = 180.0  # freshly spawned drones take a while to reach ROS
RESPAWN_TIMEOUT_S = 180.0
SETTLE_S = 3.0  # simulation seconds of hovering before each run
SETTLE_ON_GROUND_S = 2.0  # simulation seconds after moving the hostile model, before its PX4
# Where a shot-down hostile drone goes (Gazebo ENU x, y): on the ground at the far corner of
# the 1000 x 1000 m ground plane, far outside the forest and every camera's range.
GRAVEYARD_ENU = (450.0, -450.0)
POLL_S = 0.05
DEFAULT_CSV = Path('/workspace/data/surveillance_attack_runs.csv')  # /workspace = the repo
# --tracker bytetrack writes to its own file, so its rows are never mixed up with the
# Confirmer's (the tracker column only exists in files started after it was added).
BYTETRACK_CSV = Path('/workspace/data/surveillance_attack_runs_bytetrack.csv')
TRACKERS = ('confirmer', 'bytetrack')  # detect/live.py --tracker
DEFAULT_DETECT_RATE = 20.0
FIELDS = ['time', 'run', 'bearing_deg', 'start_m', 'hostile_speed', 'height', 'detector',
          'gimbal_gain', 'max_speed', 'seed', 'defenders', 'result', 'hit', 'killer',
          'first_sighting_s', 'first_sighting_by', 'engaged_s', 'engaged_by', 'time_to_hit_s',
          'closest_m', 'closest_defender', 'kill_from_base_m', 'return_s', 'tracker']


def detector_args(args) -> list[str]:
    """The tracker options for every real detector (detect/live.py): --tracker and whatever
    --detect-args adds, e.g. "--gate-sigma 5 --no-recover-low"."""
    return (['--tracker', getattr(args, 'tracker', 'confirmer')]
            + shlex.split(getattr(args, 'detect_args', '') or ''))


# --- Gazebo and PX4 processes (this runs in the container) -------------------------

def find_scene(defender_ids, hostile_id) -> tuple[str, dict[int, str], dict[int, str]]:
    """(world, model names by drone id, camera link by defender) from Gazebo, or a
    RuntimeError saying what to start."""
    gz_topics = subprocess.run(
        ['gz', 'topic', '-l'], capture_output=True, text=True, check=True).stdout.split()
    world, names = drone_names(gz_topics)
    missing = [i for i in (*defender_ids, hostile_id) if i not in names]
    if missing:
        raise RuntimeError(
            f'Drone(s) {missing} are not in Gazebo. Start the surveillance scenario with\n'
            '  export SWARM_CONFIG=/workspace/ros2_ws/src/swarm_backend/scenarios/surveillance.yaml\n'
            '  ./mount-backend.sh spawn')
    cameras = {}
    for i in defender_ids:
        found = [s for s in find_sensors(gz_topics, i) if s.kind == 'image' and s.name == 'camera']
        if not found:
            raise RuntimeError(f'Drone {i} has no camera. The defenders must be gimbal drones '
                               '(model gz_x500_gimbal); restart the simulation with the '
                               'surveillance config.')
        cameras[i] = found[0].frame
    return world, names, cameras


def px4_pid(instance: int) -> int | None:
    out = subprocess.run(['pgrep', '-f', f'bin/px4 -i {instance}( |$)'],
                         capture_output=True, text=True).stdout.split()
    return int(out[0]) if out else None


class Px4Process:
    """Stops and starts one PX4 instance the way mount-backend.sh does, with the
    environment and folder of the one that ran first (model, world, HEADLESS)."""

    def __init__(self, instance: int):
        self.instance = instance
        pid = px4_pid(instance)
        if pid is None:
            raise RuntimeError(f'PX4 instance {instance} (the hostile drone) is not running. '
                               'Start it with `./mount-backend.sh spawn` (SWARM_CONFIG = '
                               'scenarios/surveillance.yaml)')
        with open(f'/proc/{pid}/environ', 'rb') as f:
            self.env = dict(item.split('=', 1) for item in
                            f.read().decode(errors='replace').split('\0') if '=' in item)
        self.cwd = os.readlink(f'/proc/{pid}/cwd')

    def stop(self) -> None:
        pid = px4_pid(self.instance)
        if pid is None:
            return
        os.kill(pid, signal.SIGTERM)
        deadline = time.monotonic() + 10.0
        while px4_pid(self.instance) is not None and time.monotonic() < deadline:
            time.sleep(0.2)
        pid = px4_pid(self.instance)
        if pid is not None:
            os.kill(pid, signal.SIGKILL)

    def start(self, model_name: str) -> None:
        """A fresh PX4 attached to the model that is already in Gazebo (PX4_GZ_MODEL_NAME).
        Removing the model and spawning it again does not work: Gazebo keeps the removed
        model's sensors publishing, PX4 then gets every IMU and barometer sample twice, its
        estimator never starts and it cannot arm."""
        env = dict(self.env)
        env.pop('PX4_GZ_MODEL_POSE', None)
        env['PX4_GZ_MODEL_NAME'] = model_name
        log = open(f'/tmp/px4_{self.instance}.log', 'w')
        # -d: no PX4 shell, so it needs no terminal.
        subprocess.Popen(['/usr/local/bin/ros2-entrypoint.sh', 'px4-gazebo', '-i',
                          str(self.instance), '-d'],
                         env=env, cwd=self.cwd, stdin=subprocess.DEVNULL, stdout=log,
                         stderr=subprocess.STDOUT, start_new_session=True)


def set_model_pose(world: str, name: str, east: float, north: float, up: float = 0.3) -> bool:
    """Move a model (upright, facing east) in Gazebo, ENU metres."""
    result = subprocess.run(
        ['gz', 'service', '-s', f'/world/{world}/set_pose', '--reqtype', 'gz.msgs.Pose',
         '--reptype', 'gz.msgs.Boolean', '--timeout', '3000',
         '--req', f'name: "{name}" position {{x: {east} y: {north} z: {up}}} '
                  'orientation {x: 0 y: 0 z: 0 w: 1}'],
        capture_output=True, text=True)
    return 'data: true' in result.stdout


def detector_running(drone_id: int) -> bool:
    return subprocess.run(['pgrep', '-f', f'swarm_backend.detect.live --id {drone_id}( |$)'],
                          capture_output=True).returncode == 0


# --- the test ------------------------------------------------------------------------

class AttackTest:
    """Everything the test needs, wired to one ROS node."""

    def __init__(self, node, swarm_config, args):
        self.node = node
        self.args = args
        self.log = node.get_logger()
        self.surv = s = swarm_config.surveillance
        self.attack_config = fake_config(
            swarm_config.attack, args.realistic, gimbal_gain=args.gain,
            max_speed=args.max_speed, seed=args.seed)
        self.swarm_config = dataclasses.replace(swarm_config, attack=self.attack_config)
        self.hostile_id = self.attack_config.target_id
        self.ids = tuple(s.drone_ids)
        if self.hostile_id in self.ids:
            raise RuntimeError(f'attack.target_id {self.hostile_id} is also a defender '
                               f'(surveillance.drone_ids {list(self.ids)})')
        if self.hostile_id == 0:
            raise RuntimeError('attack.target_id is 0, but the test restarts the hostile '
                               "drone's PX4 every run, and drone 0's PX4 runs the container. "
                               'Use a separate hostile drone (drone 4 in '
                               'scenarios/surveillance.yaml)')
        self.detector = (('fake-realistic' if args.realistic else 'fake')
                         if args.fake_detector else 'real')
        # the fake detector publishes the target directly, with no tracker in between
        self.tracker = '' if args.fake_detector else getattr(args, 'tracker', 'confirmer')
        self.heading = math.radians(s.threat_heading_deg)

        self.world, self.names, cameras = find_scene(self.ids, self.hostile_id)
        self.cameras = cameras  # camera link of each defender
        self.px4 = Px4Process(self.hostile_id)
        # Everything is timed on Gazebo's simulation clock: steady however slowly Gazebo
        # runs, unlike PX4's time stamps, which jump when its time sync adjusts.
        self.truth = GazeboTruth(self.world)
        self.clock = self.truth.clock
        self._hostile_truth = None
        self.judges: dict[int, ClosestApproach] | None = None
        self.truth.listeners.append(self._on_truth)

        self.patrols = {i: PatrolNode(node, self.swarm_config, i, clock=self.clock)
                        for i in self.ids}
        self.defenders = SwarmManager(Swarm([p.drone for p in self.patrols.values()]),
                                      TOLERANCE_M)
        self.hostile = PX4Drone(node, swarm_config.drone(self.hostile_id),
                                rate_hz=swarm_config.rate_hz)
        self.hostile_manager = SwarmManager(Swarm([self.hostile]), TOLERANCE_M)
        self.station: BaseStationNode | None = None  # once Gazebo's clock runs
        self.fakes = [
            FakeDetectionNode(node, dataclasses.replace(self.attack_config, attacker_id=i),
                              self.truth, self.names, cameras[i])
            for i in self.ids] if args.fake_detector else []
        self.detect_procs: list[subprocess.Popen] = []

        self._events = node.create_publisher(String, msgs.EVENT_TOPIC, 10)
        self._sightings: list[tuple[float, int]] = []
        self._lock = threading.Lock()
        node.create_subscription(String, msgs.SIGHTING_TOPIC, self._on_sighting, 10)
        self.flight: HostileFlight | None = None
        node.create_timer(1.0 / swarm_config.rate_hz, self._on_timer)
        self.results: list[dict] = []

    # --- callbacks ------------------------------------------------------------

    def _on_timer(self) -> None:
        if self.flight is not None:
            self.flight.step()

    def _on_truth(self, stamp: float, poses) -> None:
        hostile = ground_truth.body_position(poses, self.names[self.hostile_id])
        self._hostile_truth = hostile
        judges = self.judges
        if judges is None or hostile is None:
            return
        for i, judge in judges.items():
            d = ground_truth.body_position(poses, self.names[i])
            if d is not None:
                judge.update(d, hostile, stamp)  # simulation time, the patrols' clock too

    def _on_sighting(self, msg) -> None:
        now = self.clock()
        if now is not None:
            with self._lock:
                self._sightings.append((now, msgs.decode_sighting(msg.data).drone_id))

    # --- helpers --------------------------------------------------------------

    def wait_sim(self, seconds: float) -> None:
        start = self.clock()
        wait_until(lambda: self.clock() - start >= seconds, SETUP_TIMEOUT_S,
                   f'{seconds} s of simulation time')

    def alive(self) -> list[int]:
        """Defenders the base station still counts."""
        return [i for i in self.ids if self.station.station.drones[i].mode is not Mode.LOST]

    def on_patrol(self, drone_id: int) -> bool:
        """Back on its slot, sweeping the sector the base station gave it."""
        patrol = self.patrols[drone_id]
        order = self.station.orders.get(drone_id)
        return (self.station.station.drones[drone_id].mode is Mode.PATROL
                and patrol.patrol.mode is Mode.PATROL
                and order is not None and order.mode is Mode.PATROL and order.sector is not None
                and patrol.drone.reached(SLOT_TOLERANCE_M))

    def wait_patrol(self) -> None:
        wait_until(lambda: all(self.on_patrol(i) for i in self.alive()), SETUP_TIMEOUT_S,
                   'every defender to be back on patrol')

    def scenario(self, index: int) -> Inbound:
        a = self.args
        return Inbound(a.speed, a.bearing[index % len(a.bearing)], a.start, a.height)

    def path(self, scenario: Inbound):
        any_patrol = self.patrols[self.ids[0]]
        return inbound_path(scenario, self.surv.base_position, any_patrol.patrol.slot_point[2],
                            self.heading)

    # --- the hostile drone ----------------------------------------------------

    def move_hostile(self, east: float, north: float) -> None:
        name = self.names[self.hostile_id]
        if not set_model_pose(self.world, name, east, north):
            self.log.warning(f'Gazebo did not confirm moving {name}')

    def remove_hostile(self) -> None:
        """Shot down (or out of the way for the next run): the model moved to the
        graveyard, far outside the battle, then its PX4 stopped. In that order: stopping PX4
        takes seconds, and a hostile drone falling with its motors off is a target the
        other defenders chase down into the trees."""
        self.move_hostile(*GRAVEYARD_ENU)
        self.px4.stop()

    def respawn_hostile(self, north: float, east: float) -> None:
        """A fresh hostile drone on the ground at (north, east), connected and ready: the
        same Gazebo model, moved there, with a new PX4 attached to it."""
        self.px4.stop()
        time.sleep(1.0)  # its last messages
        self.log.info(f'Starting hostile drone {self.hostile_id} at N {north:+.0f} E {east:+.0f} m')
        self.move_hostile(east, north)
        self.wait_sim(SETTLE_ON_GROUND_S)
        self.hostile.reset((north, east, 0.0))
        self.px4.start(self.names[self.hostile_id])
        log = f'/tmp/px4_{self.hostile_id}.log'
        wait_until(self.hostile_manager.connected, RESPAWN_TIMEOUT_S,
                   f'the hostile drone to connect (see {log})')
        wait_until(self.hostile_manager.ready, RESPAWN_TIMEOUT_S,
                   f"the hostile drone's pre-flight checks (see {log})")

    # --- setup ----------------------------------------------------------------

    def start_detectors(self) -> None:
        if self.args.fake_detector:
            for fake in self.fakes:
                print(fake.describe(), flush=True)
            return
        running = [i for i in self.ids if detector_running(i)]
        if running:
            raise RuntimeError(f'A detector already runs for drone(s) {running}; stop it, this '
                               'test starts its own')
        for i in self.ids:
            log = open(f'/tmp/detect_{i}.log', 'w')
            extra = []
            if getattr(self.args, 'record', False):  # every image, for the baseline evaluation
                path = Path(self.record_path(i))
                path.unlink(missing_ok=True)
                extra = ['--record', str(path)]
            self.detect_procs.append(subprocess.Popen(
                [sys.executable, '-m', 'swarm_backend.detect.live', '--id', str(i),
                 '--rate', str(self.args.detect_rate)] + detector_args(self.args) + extra,
                stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True))
        self.log.info(f'Real detectors ({self.tracker}) started for drones {list(self.ids)}, logs '
                      f'/tmp/detect_<id>.log')

    @staticmethod
    def record_path(drone_id: int) -> str:
        return f'/tmp/detect_{drone_id}.jsonl'

    def set_hostile_param(self, name: str, value: float) -> None:
        """Set a PX4 parameter on the hostile drone (e.g. its speed limit)."""
        result = subprocess.run(['/opt/px4-gazebo/bin/px4-param', '--instance',
                                 str(self.hostile_id), 'set', name, str(value)],
                                capture_output=True, text=True)
        if result.returncode != 0:
            self.log.warning(f'Could not set {name} on drone {self.hostile_id}: '
                             f'{(result.stdout + result.stderr).strip()}')

    def stop_detectors(self) -> None:
        for p in self.detect_procs:
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
        for p in self.detect_procs:
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                p.kill()
        self.detect_procs = []

    def setup(self) -> None:
        s = self.surv
        wait_until(lambda: self.clock() is not None, 30, "Gazebo's poses")
        self.station = BaseStationNode(self.node, self.swarm_config, clock=self.clock)
        try:
            wait_until(self.defenders.connected, CONNECT_TIMEOUT_S,
                       f'status and position from defenders {list(self.ids)}')
        except TimeoutError:
            raise TimeoutError(
                f'No data from PX4 for drones {self.defenders.not_connected()}. Start them with '
                '`./mount-backend.sh spawn` (SWARM_CONFIG = scenarios/surveillance.yaml)') from None
        wait_until(self.defenders.ready, CONNECT_TIMEOUT_S, "the defenders' pre-flight checks")
        self.start_detectors()
        self.log.info(f'Taking off: defenders {list(self.ids)} to {s.altitude} m')
        self.defenders.takeoff(s.altitude)
        wait_until(self.defenders.all_reached, SETUP_TIMEOUT_S, f'{s.altitude} m')
        for i, p in self.patrols.items():
            self.log.info(f'Drone {i} to its slot at {p.patrol.slot_angle:+.0f} deg on the arc')
            p.patrol.start()
        self.wait_patrol()
        self.log.info('All defenders on patrol')

    # --- a run ----------------------------------------------------------------

    def run(self, index: int) -> dict | None:
        args = self.args
        alive = self.alive()
        if not alive:
            self.log.error('No defenders left')
            return None
        scenario = self.scenario(index)
        path, flight_time = self.path(scenario)
        (start, velocity) = path.at(0.0)
        yaw = math.atan2(velocity[1], velocity[0])

        self.respawn_hostile(start[0], start[1])
        altitude = self.hostile.home[2] - start[2]
        self.log.info(f'Hostile taking off to {altitude:.0f} m')
        self.hostile_manager.takeoff(altitude)
        wait_until(self.hostile_manager.all_reached, SETUP_TIMEOUT_S, 'the hostile drone to climb')
        self.hostile.set_position(*start, yaw=yaw)
        wait_until(lambda: self.hostile.reached(TOLERANCE_M), SETUP_TIMEOUT_S,
                   'the hostile drone at its start point')
        self.wait_patrol()
        self.wait_sim(SETTLE_S)

        judges = {i: ClosestApproach(args.hit_radius) for i in alive}
        with self._lock:
            self._sightings = []
        modes = {i: self.patrols[i].patrol.mode for i in alive}
        engaged: tuple[float, int] | None = None
        self.flight = HostileFlight(self.hostile, path, self.clock)
        self.judges = judges
        self.flight.start()
        t0 = self.clock()
        self.log.info(f'Run {index + 1}/{args.runs}: inbound from {scenario.bearing_deg:+.0f} deg, '
                      f'{scenario.start_m:.0f} m out, {scenario.height_m:.0f} m above the '
                      f'defenders, {scenario.speed} m/s ({flight_time:.0f} s), '
                      f'{len(alive)} defenders, detector: {self.detector}')
        result, killer = 'reached_base', None
        try:
            while True:
                t = self.clock() - t0
                for i in alive:
                    mode = self.patrols[i].patrol.mode
                    if mode is not modes[i]:
                        modes[i] = mode
                        closest = judges[i].closest
                        self.log.info(f'  t={t:6.1f} s drone {i} {mode.value.upper()}, '
                                      f'closest so far {closest:.1f} m')
                        if mode is Mode.ENGAGE and engaged is None:
                            engaged = (t, i)
                hits = [(j.hit_time, i) for i, j in judges.items() if j.hit]
                if hits:
                    result, killer = 'hit', min(hits)[1]
                    break
                if t >= flight_time:
                    break
                time.sleep(POLL_S)
        finally:
            self.judges = None
            hostile_at = self._hostile_truth
            self.flight.stop()
            self.flight = None
            self.remove_hostile()
        if killer is not None:
            self._events.publish(String(data=msgs.encode_event(msgs.EventMsg(killer, 'kill'))))
        t_end = self.clock()

        with self._lock:
            sightings = [(t - t0, i) for t, i in self._sightings]
        first = sightings[0] if sightings else None
        closest_id = min(judges, key=lambda i: judges[i].closest)
        bx, by = self.surv.base_position
        hit_time = judges[killer].hit_time if killer is not None else None
        row = {
            'time': datetime.datetime.now().isoformat(timespec='seconds'), 'run': index + 1,
            'bearing_deg': scenario.bearing_deg, 'start_m': scenario.start_m,
            'hostile_speed': scenario.speed, 'height': scenario.height_m,
            'detector': self.detector, 'tracker': self.tracker,
            'gimbal_gain': self.attack_config.gimbal_gain,
            'max_speed': self.attack_config.max_speed, 'seed': self.attack_config.seed,
            'defenders': len(alive), 'result': result, 'hit': int(killer is not None),
            'killer': '' if killer is None else killer,
            'first_sighting_s': '' if first is None else round(first[0], 2),
            'first_sighting_by': '' if first is None else first[1],
            'engaged_s': '' if engaged is None else round(engaged[0], 2),
            'engaged_by': '' if engaged is None else engaged[1],
            'time_to_hit_s': (round(hit_time - t0 - first[0], 2)
                              if hit_time is not None and first is not None else ''),
            'closest_m': round(judges[closest_id].closest, 2), 'closest_defender': closest_id,
            'kill_from_base_m': (round(math.hypot(hostile_at[0] - bx, hostile_at[1] - by), 1)
                                 if killer is not None and hostile_at is not None else ''),
            'return_s': '',
        }
        self.log.info(f'Run {index + 1}: {result.upper()}'
                      + (f' by drone {killer} {row["kill_from_base_m"]} m from the base, '
                         f'{row["time_to_hit_s"]} s after the first sighting' if killer else '')
                      + f'; closest {row["closest_m"]} m (drone {closest_id})')
        try:
            self.wait_patrol()
            if killer is not None:
                row['return_s'] = round(self.clock() - t_end, 2)
                self.log.info(f'Drone {killer} back on patrol after {row["return_s"]} s')
        except TimeoutError as e:
            self.log.warning(str(e))
        self.results.append(row)
        self.write_row(row)
        return row

    def write_row(self, row: dict) -> None:
        path = Path(self.args.csv)
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            new = not path.exists() or path.stat().st_size == 0
            fields = FIELDS
            if not new:  # keep an older file's columns (it may have no tracker column)
                with open(path, newline='') as f:
                    fields = next(csv.reader(f), None) or FIELDS
            with open(path, 'a', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
                if new:
                    writer.writeheader()
                writer.writerow(row)
        except OSError as e:
            self.log.error(f'Could not write {path}: {e}')

    # --- the end --------------------------------------------------------------

    def stop_and_land(self) -> None:
        if self.flight is not None:
            self.flight.stop()
            self.flight = None
        for p in self.patrols.values():
            p.patrol.stop()
        try:
            self.remove_hostile()
        finally:
            self.stop_detectors()
        armed = [p for p in self.patrols.values() if p.drone.armed]
        if armed:
            try:
                self.log.info('Defenders back to their slots')
                for p in armed:
                    if p.drone.offboard:
                        p.drone.set_position(*p.patrol.slot_point)
                wait_until(lambda: all(p.drone.reached(SLOT_TOLERANCE_M)
                                       for p in armed if p.drone.offboard),
                           SETUP_TIMEOUT_S, 'the slots')
            except (TimeoutError, KeyboardInterrupt) as e:
                self.log.warning(f'Landing where they are ({e or "Ctrl-C"})')
            self.log.info('Landing')
            self.defenders.land([p.drone.id for p in armed])
            # Wall clock: from 20 m at PX4's landing speed, minutes when Gazebo runs slowly.
            wait_until(self.defenders.all_landed, max(LAND_TIMEOUT_S, SETUP_TIMEOUT_S),
                       'the defenders to land and disarm')
            self.log.info('Landed')
        # The hostile drone back where the config (and mount-backend.sh) expects it.
        hx, hy, _ = self.swarm_config.drone(self.hostile_id).spawn_ned
        self.log.info(f'Starting drone {self.hostile_id} again at its spawn point')
        self.move_hostile(hy, hx)
        time.sleep(1.0)
        self.hostile.reset((hx, hy, 0.0))
        self.px4.start(self.names[self.hostile_id])

    def summary(self) -> str:
        rows = self.results
        if not rows:
            return 'No runs finished.'
        hits = [r for r in rows if r['hit']]
        lines = [f'{len(hits)}/{len(rows)} hits (inbound at {self.args.speed} m/s, '
                 f'detector {self.detector}'
                 + (f', tracker {self.tracker}' if self.tracker else '')
                 + f', gimbal gain {self.attack_config.gimbal_gain})']
        for r in rows:
            what = (f'hit by drone {r["killer"]} {r["kill_from_base_m"]} m from the base'
                    if r['hit'] else f'{r["result"]}, closest {r["closest_m"]} m')
            lines.append(f'  run {r["run"]} from {r["bearing_deg"]:+.0f} deg: {what}')
        lines.append(f'  one row per run in {self.args.csv}')
        return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--config', default=None,
                        help='swarm.yaml with a surveillance: section (default: the installed one)')
    parser.add_argument('--runs', type=int, default=1)
    parser.add_argument('--bearing', type=float, nargs='+', default=[0.0],
                        help='degrees off the threat heading the hostile comes from, positive '
                             'to the right (slots of 3 defenders: -30 0 30); several are used '
                             'in turn, one per run')
    parser.add_argument('--start', type=float, default=DEFAULT_START_M,
                        help='metres from the base where the hostile drone starts')
    parser.add_argument('--height', type=float, default=DEFAULT_HEIGHT_M,
                        help='metres the hostile drone flies above the defenders')
    parser.add_argument('--speed', type=float, default=1.0, help='hostile speed, m/s')
    parser.add_argument('--hit-radius', type=float, default=1.5, help='metres')
    parser.add_argument('--fake-detector', action='store_true',
                        help="fake detectors from Gazebo's poses instead of the camera detector")
    parser.add_argument('--realistic', action='store_true',
                        help='with --fake-detector: 0.25 s delay, 1 deg and 20 %% noise, '
                             '5 m blind zone')
    parser.add_argument('--detect-rate', type=float, default=DEFAULT_DETECT_RATE,
                        help='images per second for each real detector')
    parser.add_argument('--gain', type=float, default=None, help='gimbal gain (attack config)')
    parser.add_argument('--max-speed', type=float, default=None, help='m/s (attack config)')
    parser.add_argument('--seed', type=int, default=None, help='fake detector noise')
    parser.add_argument('--tracker', choices=TRACKERS, default='confirmer',
                        help='tracker of the real detectors (detect/live.py --tracker); '
                             'default %(default)s')
    parser.add_argument('--detect-args', default='',
                        help='more options for every real detector, in quotes, e.g. '
                             '"--gate-sigma 5 --no-recover-low"')
    parser.add_argument('--csv', default=None,
                        help=f'append one row per run here (default {DEFAULT_CSV}, with '
                             f'--tracker bytetrack {BYTETRACK_CSV})')
    args = parser.parse_args()
    if args.realistic and not args.fake_detector:
        parser.error('--realistic only applies to --fake-detector')
    if args.fake_detector and (args.tracker != 'confirmer' or args.detect_args):
        parser.error('--tracker and --detect-args only apply to the real detector '
                     '(not --fake-detector)')
    if args.csv is None:
        args.csv = str(BYTETRACK_CSV if args.tracker == 'bytetrack' else DEFAULT_CSV)

    swarm_config = load_config(args.config or default_config_path())
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('attack_test')
    try:
        test = AttackTest(node, swarm_config, args)
    except (RuntimeError, ValueError) as e:
        print(e)
        node.destroy_node()
        rclpy.try_shutdown()
        return 1
    code = 0
    with spinning(node):
        try:
            test.setup()
            for i in range(args.runs):
                if test.run(i) is None:
                    break
        except KeyboardInterrupt:
            test.log.info('Ctrl-C')
        except (TimeoutError, RuntimeError) as e:
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
