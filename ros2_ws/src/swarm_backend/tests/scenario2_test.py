"""Scenario 2: a defender with the YOLO detector above the base attacks two incoming drones.

Settings: the attack: and scenario2: sections of the config (default
scenarios/scenario2.yaml). The chain is the same as the surveillance defenders':

  detector   `detect 1 --detector yolo` (detect/live.py) on drone 1's camera, started first,
             log in /tmp/scenario2_detect.log. It publishes the best confirmed target on
             /px4_1/detection/target_position and target_offset.
  attack     the baseline attack (intercept/attack.py, AttackNode as in `attack`): drone 1
             hovers at its guard point right above the base, attack.guard_altitude up, nose
             and gimbal toward attack.search_heading_deg (SEARCH). On target_position it flies
             straight at the target at attack.max_speed and turns the gimbal on
             target_offset (ATTACK), coasts through short gaps (COAST), and when the target
             is lost hovers, then goes back to the guard point (LOST -> SEARCH).
  hostiles   drones 2 and 3, 150 m out, 20 deg left and right of north, climb to
             `hostile_heights` above the guard height and fly in a straight line toward
             the base at `hostile_speed`; they stop `stop_m` short of it and hover.
  referee    a hit is the defender within `hit_radius` of a hostile, from Gazebo's true
             positions. The hostile is taken out (moved far outside the battle, its PX4
             stopped, as in the attack test) and the defender starts over in SEARCH for the
             next one.

With --guidance mpc the defender flies the MPC intercept instead of pure pursuit
(intercept/mpc_action.py, `attack --guidance mpc`), on the same detection topics: a Kalman
filter estimates the target's position and velocity, it flies to the predicted meeting
point and lets the MPC steer the last seconds, and the gimbal is aimed at the estimate.
Lost (no report for intercept.track_timeout, 3 s) it hovers where it is; after a hit it
flies back to the guard point and starts over.

With --birds N, N birds (every kind in birds.py, worlds/birds/) start at random points
around the hostiles' start points and fly in with them, straight over the base and on past
it, at about the hostiles' speed: distractors for the detector. They have no collisions and
are removed at the end. --bird-seed makes their start points repeatable.

With --no-attack the defender only hovers and watches (the detector still runs). Each
target_position is also matched, by direction, to a hostile, so the log says when and at
what distance each was first seen. It runs until Ctrl-C, --duration simulation seconds, or
every hostile is hit; then the drones fly back above their start points and land. A second
Ctrl-C lands them where they are. Inside the container:

    python3 /workspace/ros2_ws/src/swarm_backend/tests/scenario2_test.py --config <yaml>
    python3 .../scenario2_test.py --speed 8 --attack-speed 8 --detector baseline

On the host: scenarios/yolo.sh [same arguments] (bytetrack.sh, mpc.sh and birds.sh build on it).
"""
import argparse
import dataclasses
import json
import math
import signal
import subprocess
import sys
import threading
import time

import rclpy
import yaml
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from attack_test import GRAVEYARD_ENU, Px4Process, set_model_pose  # tests/, next to this file
from swarm_backend import ground_truth
from swarm_backend.config import load_config
from swarm_backend.detect.live import offset_topic
from swarm_backend.intercept.attack import AttackAction, State
from swarm_backend.intercept.attack_scenario import ClosestApproach
from swarm_backend.intercept.inbound import (inbound_line, match_hostile, plan_birds,
                                              straight_at)
from swarm_backend.nodes.attack_node import AttackNode
from swarm_backend.nodes.fake_detection import GazeboTruth
from swarm_backend.px4.px4_drone import PX4Drone
from swarm_backend.runner import LAND_TIMEOUT_S, spinning, wait_until
from swarm_backend.save_images import drone_names
from swarm_backend.sensor_view import find_sensors
from swarm_backend.swarm.manager import SwarmManager
from swarm_backend.swarm.swarm import Swarm

DEFAULT_CONFIG = '/workspace/ros2_ws/src/swarm_backend/scenarios/scenario2.yaml'
DEFAULTS = {
    'defender_id': 1, 'hostile_ids': [2, 3], 'base_position': [0.0, 0.0],
    'hostile_heights': [10.0, 14.0], 'hostile_speed': 5.0, 'stop_m': 20.0, 'stagger_s': 0.0,
    'hit_radius': 1.5, 'detector': 'yolo', 'detect_rate': 10.0,
}
TOLERANCE_M = 1.5
CONNECT_TIMEOUT_S = 180.0  # wall clock; freshly spawned drones take a while to reach ROS
CLIMB_TIMEOUT_S = 1200.0  # wall clock; three drones and a camera make Gazebo slow
RETURN_TIMEOUT_S = 600.0
SETTLE_S = 3.0  # simulation seconds at the guard point before the hostiles leave
REPORT_EVERY_S = 10.0  # simulation seconds between progress lines
DETECTOR_START_S = 8.0  # wall clock: a detector that dies this soon failed to start
MATCH_GATE_DEG = 5.0
DETECT_LOG = '/tmp/scenario2_detect.log'
RECORD_ROOT = '/workspace/data/tune'  # /workspace = the repo: ~/Hive/data/tune (not in git)
POLL_S = 0.05
BIRDS_LOG = '/tmp/scenario2_birds.log'


def settings(path: str, args) -> dict:
    with open(path) as f:
        s = {**DEFAULTS, **((yaml.safe_load(f) or {}).get('scenario2') or {})}
    for key, value in (('hostile_speed', args.speed), ('detector', args.detector),
                       ('stagger_s', args.stagger)):
        if value is not None:
            s[key] = value
    if len(s['hostile_heights']) != len(s['hostile_ids']):
        raise ValueError('scenario2: needs one hostile_heights entry per hostile_ids entry')
    return s


class Scenario2:

    def __init__(self, node: Node, config, s: dict, args):
        self.node = node
        self.log = node.get_logger()
        self.s = s
        self.args = args
        self.def_id = int(s['defender_id'])
        self.hostile_ids = [int(i) for i in s['hostile_ids']]
        self.base = (float(s['base_position'][0]), float(s['base_position'][1]))
        overrides = {'attacker_id': self.def_id}
        if args.attack_speed is not None:
            overrides['max_speed'] = args.attack_speed
        self.attack_config = dataclasses.replace(config.attack, **overrides)

        gz_topics = subprocess.run(['gz', 'topic', '-l'], capture_output=True, text=True,
                                   check=True).stdout.split()
        self.world, self.names = drone_names(gz_topics)
        missing = [i for i in (self.def_id, *self.hostile_ids) if i not in self.names]
        if missing:
            raise RuntimeError(f'Drone(s) {missing} are not in Gazebo. Start them with '
                               'scenarios/yolo.sh (it spawns them)')
        # Everything is timed on Gazebo's simulation clock, which never jumps.
        cameras = [c for c in find_sensors(gz_topics, self.def_id)
                   if c.kind == 'image' and c.name == 'camera']
        self.camera_link = cameras[0].frame if cameras else None
        # --record NAME: what the tuning replay needs (detect/replay_eval.py), in one folder
        self.record_dir = None
        self._truth_file = self._events_file = None
        if args.record:
            from pathlib import Path
            self.record_dir = Path(RECORD_ROOT) / args.record
            if self.record_dir.exists() and any(self.record_dir.iterdir()):
                raise RuntimeError(f'{self.record_dir} is not empty; pick another --record name')
            self.record_dir.mkdir(parents=True, exist_ok=True)
            self._truth_file = open(self.record_dir / 'truth.jsonl', 'w', buffering=1 << 16)
            self._events_file = open(self.record_dir / 'events.jsonl', 'w', buffering=1)
        self.truth = GazeboTruth(self.world)
        self.clock = self.truth.clock
        self.true_pos: dict[int, tuple] = {}
        self._lock = threading.Lock()
        self.judges: dict[int, ClosestApproach] | None = None
        self.truth.listeners.append(self._on_truth)

        # The defender: its PX4Drone, the detection topics and the attack (AttackNode).
        self.guidance = args.guidance
        self.attacker = AttackNode(node, config, self.attack_config, clock=self.clock,
                                   guidance=self.guidance)
        self.defender = self.attacker.drone
        self.guard = (self.base[0], self.base[1],
                      self.defender.home[2] - self.attack_config.guard_altitude)
        self.hostiles = {i: PX4Drone(node, config.drone(i), rate_hz=config.rate_hz)
                         for i in self.hostile_ids}
        self.manager = SwarmManager(Swarm([self.defender, *self.hostiles.values()]),
                                    TOLERANCE_M)
        self.removed: set[int] = set()
        self.hits: dict[int, float] = {}  # hostile: t of the hit
        self.seen_states = 0  # attack transitions already logged

        self.detect_proc: subprocess.Popen | None = None
        self.birds = []  # BirdPlan, with --birds
        self.birds_proc: subprocess.Popen | None = None  # birds.py fly
        self.t0: float | None = None  # simulation time the hostiles left
        self.first_seen: dict[int, tuple[float | None, float]] = {}  # id: (t, true distance)
        self.seen_count = {i: 0 for i in self.hostile_ids}
        self.unmatched = 0
        self.last_reported: int | None = None  # the hostile the last report matched
        self.switches = 0  # reports that jumped to the other hostile while both flew
        self.target_topic = offset_topic(self.def_id, 'target_position')
        node.create_subscription(PointStamped, self.target_topic, self._on_target, 10)

    # --- callbacks ------------------------------------------------------------

    def _on_truth(self, stamp: float, poses) -> None:
        found = {}
        for i in (self.def_id, *self.hostile_ids):
            p = ground_truth.body_position(poses, self.names[i])
            if p is not None:
                found[i] = p
        with self._lock:
            self.true_pos.update(found)
        truth_file = self._truth_file
        if truth_file is not None and self.camera_link is not None:
            cam = ground_truth.camera_pose(poses, self.names[self.def_id], self.camera_link)
            if cam is not None:
                truth_file.write(json.dumps({
                    't': stamp,
                    'cam': [[round(c, 4) for c in cam['position']]]
                           + [[round(c, 6) for c in cam[k]] for k in ('forward', 'right', 'down')],
                    'hostiles': {str(i): [round(c, 3) for c in found[i]]
                                 for i in self.hostile_ids
                                 if i in found and i not in self.removed}}) + '\n')
        judges, d = self.judges, found.get(self.def_id)
        if judges is None or d is None:
            return
        for i, judge in judges.items():
            if i in found and i not in self.removed:
                judge.update(d, found[i], stamp)

    def _on_target(self, msg: PointStamped) -> None:
        with self._lock:
            truth = dict(self.true_pos)
        observer = truth.get(self.def_id)
        if observer is None:
            return
        live = {i: truth[i] for i in self.hostile_ids if i in truth and i not in self.removed}
        match = match_hostile((msg.point.x, msg.point.y, msg.point.z), observer, live,
                              MATCH_GATE_DEG)
        if match is None:
            self.unmatched += 1
            return
        drone_id, distance = match
        last = self.last_reported
        if last is not None and drone_id != last and last not in self.removed:
            self.switches += 1
            self.log.info(f'  {self.when(self.sim_t())}: the detector switched from hostile '
                          f'{last} to hostile {drone_id}')
        self.last_reported = drone_id
        self.seen_count[drone_id] += 1
        if drone_id not in self.first_seen:
            t = self.sim_t()
            self.first_seen[drone_id] = (t, distance)
            self.log.info(f'  {self.when(t)}: {self.s["detector"].upper()} sees hostile '
                          f'{drone_id} for the first time, {distance:.0f} m away')

    # --- helpers --------------------------------------------------------------

    def sim_t(self) -> float | None:
        now = self.clock()
        return None if self.t0 is None or now is None else now - self.t0

    @staticmethod
    def when(t: float | None) -> str:
        return 'before the hostiles left' if t is None or t < 0 else f't={t:6.1f} s'

    def wait_sim(self, seconds: float) -> None:
        start = self.clock()
        wait_until(lambda: self.clock() - start >= seconds, CLIMB_TIMEOUT_S,
                   f'{seconds} s of simulation time')

    def distance(self, a: int, b: tuple) -> float | None:
        with self._lock:
            p = self.true_pos.get(a)
        return None if p is None else math.dist(p[:2], b[:2])

    def new_attack(self) -> None:
        """A fresh attack, searching from the guard point above the base."""
        if self.guidance == 'pursuit':
            self.attacker.attack = AttackAction(self.defender, self.attacker.inbox,
                                                self.attack_config, self.clock,
                                                guard=self.guard)
        else:
            # The MPC searches where the drone is and never flies back by itself.
            self.defender.set_position(*self.guard,
                                       yaw=math.radians(self.attack_config.search_heading_deg))
            self.attacker.new_attack()  # a fresh MpcIntercept and InboxDetector
            self.attacker.detector.start()  # ignore reports from before now
        self.seen_states = 0
        self.attacker.attack.start()

    def attack_state(self) -> str:
        """What the attack does now: search/attack/coast/lost (pursuit), or
        search/midcourse/terminal/lost (MPC)."""
        transitions = self.attacker.attack.transitions
        state = transitions[-1][1] if transitions else 'search'
        return getattr(state, 'value', state)

    def log_attack_states(self) -> None:
        attack = self.attacker.attack
        for _, state in attack.transitions[self.seen_states:]:
            state = getattr(state, 'value', state)
            extra = ''
            message = self.attacker.inbox.position
            if state in (State.ATTACK.value, 'midcourse', 'terminal') and message is not None:
                extra = f', target {math.hypot(*message.value):.0f} m away (estimate)'
            self.log.info(f'  {self.when(self.sim_t())}: defender {state.upper()}{extra}')
        self.seen_states = len(attack.transitions)

    def event(self, kind: str, **data) -> None:
        if self._events_file is not None:
            self._events_file.write(json.dumps({'t': self.clock(), 'event': kind, **data}) + '\n')

    def take_out(self, hostile_id: int) -> None:
        """Shot down: the model moved far outside the battle, then its PX4 stopped (in that
        order, so nobody chases a falling drone)."""
        name = self.names[hostile_id]
        if not set_model_pose(self.world, name, *GRAVEYARD_ENU):
            self.log.warning(f'Gazebo did not confirm moving {name}')
        Px4Process(hostile_id).stop()
        self.removed.add(hostile_id)
        self.event('removed', hostile=hostile_id)

    def start_detector(self) -> None:
        running = subprocess.run(
            ['pgrep', '-f', f'swarm_backend.detect.live --id {self.def_id}( |$)'],
            capture_output=True).returncode == 0
        if running:
            raise RuntimeError(f'A detector already runs for drone {self.def_id}; stop it, '
                               'this scenario starts its own')
        log = open(DETECT_LOG, 'w')
        cmd = [sys.executable, '-m', 'swarm_backend.detect.live', '--id', str(self.def_id),
               '--rate', str(self.s['detect_rate']), '--detector', self.s['detector']]
        if self.record_dir is not None:
            cmd += ['--record', str(self.record_dir / 'detect.jsonl')]
        cmd += self.args.detect_args or []
        self.detect_proc = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=log,
                                            stderr=subprocess.STDOUT, start_new_session=True)
        time.sleep(DETECTOR_START_S)
        self.check_detector()
        self.log.info(f'{self.s["detector"].upper()} detector running on drone {self.def_id}'
                      f"'s camera (log {DETECT_LOG}); it publishes {self.target_topic}")

    def check_detector(self) -> None:
        if self.detect_proc is not None and self.detect_proc.poll() is not None:
            with open(DETECT_LOG) as f:
                tail = ''.join(f.readlines()[-15:])
            raise RuntimeError(f'The detector stopped (exit code {self.detect_proc.returncode}). '
                               f'End of {DETECT_LOG}:\n{tail}')

    def stop_detector(self) -> None:
        p = self.detect_proc
        if p is None or p.poll() is not None:
            return
        p.send_signal(signal.SIGINT)
        try:
            p.wait(timeout=10)
        except subprocess.TimeoutExpired:
            p.kill()

    # --- the flight -----------------------------------------------------------

    def fly(self) -> None:
        s, a = self.s, self.attack_config
        wait_until(lambda: self.clock() is not None, 30, "Gazebo's poses")
        wait_until(self.manager.connected, CONNECT_TIMEOUT_S,
                   f'status and position from drones {self.manager.swarm.ids}')
        wait_until(self.manager.ready, CONNECT_TIMEOUT_S, 'the pre-flight checks')
        self.start_detector()

        guard_z = self.guard[2]
        self.log.info(f'Taking off: defender {self.def_id} to {a.guard_altitude:.0f} m, '
                      'hostiles ' + ', '.join(
                          f'{i} to {a.guard_altitude + h:.0f} m'
                          for i, h in zip(self.hostile_ids, s['hostile_heights'])))
        self.manager.takeoff(a.guard_altitude, [self.def_id])
        lines = {}
        for i, h in zip(self.hostile_ids, s['hostile_heights']):
            drone = self.hostiles[i]
            z = guard_z - h
            self.manager.takeoff(drone.home[2] - z, [i])
            start = (drone.home[0], drone.home[1], z)
            lines[i] = (start, inbound_line(start, self.base, s['stop_m']))
        wait_until(self.manager.all_reached, self.args.climb_timeout, 'the climb')
        if self.args.birds > 0:
            self.plan_birds([start for start, _ in lines.values()])

        for i, (start, end) in lines.items():
            self.hostiles[i].set_position(*start, yaw=math.atan2(end[1] - start[1],
                                                                 end[0] - start[0]))
        if self.args.no_attack:
            self.log.info(f'Defender {self.def_id} to right above the base; --no-attack: it '
                          'only watches')
            self.defender.set_position(*self.guard, yaw=math.radians(a.search_heading_deg))
            self.defender.aim_gimbal(a.search_pitch_deg, 0.0)
        else:
            self.log.info(f'Defender {self.def_id} to its guard point right above the base; '
                          f'it attacks ({self.guidance}) on {self.target_topic} at '
                          f'{a.max_speed} m/s')
            self.new_attack()  # SEARCH: flies to the guard point, gimbal toward the threat
        wait_until(lambda: self.defender.reached(TOLERANCE_M), self.args.climb_timeout,
                   'the defender above the base')
        self.wait_sim(SETTLE_S)

        speed = s['hostile_speed']
        self.judges = {i: ClosestApproach(s['hit_radius']) for i in self.hostile_ids}
        self.t0 = self.clock()
        self.event('start', hostiles=self.hostile_ids, speed=speed)
        if self.birds:
            self.start_birds()
        flight_s = max(math.dist(p, q) for p, q in lines.values()) / speed
        self.log.info(f'Hostiles {self.hostile_ids} flying in at {speed} m/s '
                      f'({flight_s + s["stagger_s"]:.0f} s); Ctrl-C to stop and land')
        arrived_logged = False
        next_report = REPORT_EVERY_S
        while True:
            t = self.clock() - self.t0
            if t >= self.args.duration:
                self.log.info(f'{self.args.duration:.0f} s of simulation time are over')
                return
            self.check_detector()
            if not self.args.no_attack:
                self.log_attack_states()
                for i, judge in self.judges.items():
                    if judge.hit and i not in self.removed:
                        self.hits[i] = judge.hit_time - self.t0
                        self.event('hit', hostile=i, t_hit=judge.hit_time)
                        self.log.info(f'  t={self.hits[i]:6.1f} s HIT: defender {self.def_id} '
                                      f'got hostile {i}, '
                                      f'{self.distance(self.def_id, self.base):.0f} m from '
                                      'the base. Taking it out')
                        attack = self.attacker.attack
                        getattr(attack, 'hit', attack.stop)()  # the MPC has no hit()
                        self.take_out(i)
                        if len(self.removed) == len(self.hostile_ids):
                            self.log.info('Every hostile is down')
                            return
                        self.log.info('  Back to SEARCH for the next one')
                        self.new_attack()
            arrived = []
            for k, (i, (start, end)) in enumerate(lines.items()):
                if i in self.removed:
                    continue
                tk = t - k * s['stagger_s']
                if tk < 0:
                    arrived.append(False)
                    continue
                pos, vel, done = straight_at(start, end, speed, tk)
                yaw = math.atan2(end[1] - start[1], end[0] - start[0])
                self.hostiles[i].set_position(*pos, yaw=yaw, velocity=None if done else vel)
                arrived.append(done)
            if t >= next_report:
                next_report += REPORT_EVERY_S
                self.log.info(f'  t={t:6.1f} s ' + self.status())
            if arrived and all(arrived) and not arrived_logged:
                arrived_logged = True
                self.log.info(f'  t={t:6.1f} s the hostiles left are at their stop points, '
                              f'{s["stop_m"]:.0f} m from the base, and hover there')
            time.sleep(POLL_S)

    def plan_birds(self, hostile_starts: list) -> None:
        import random

        from swarm_backend.birds import KINDS
        rng = random.Random(self.args.bird_seed)
        self.birds = plan_birds(self.args.birds, hostile_starts, self.base, list(KINDS),
                                self.s['hostile_speed'], rng)
        self.log.info(f'{len(self.birds)} birds around the hostiles: ' + ', '.join(
            f'{b.kind} {math.hypot(b.start[0] - self.base[0], b.start[1] - self.base[1]):.0f} m '
            f'out at {b.speed:.1f} m/s' for b in self.birds))
        self.event('birds', birds=[dataclasses.asdict(b) for b in self.birds])

    def start_birds(self) -> None:
        """The birds fly in their own process (birds.py fly): a gz-transport call in this
        one would hold the interpreter lock while Gazebo answers and starve the setpoint
        stream, and PX4 would leave offboard."""
        plan = {'world': self.world, 't0': self.t0,
                'birds': [dataclasses.asdict(b) for b in self.birds]}
        self.birds_proc = subprocess.Popen(
            [sys.executable, '-m', 'swarm_backend.birds', 'fly'], stdin=subprocess.PIPE,
            stdout=open(BIRDS_LOG, 'w'), stderr=subprocess.STDOUT, text=True)
        self.birds_proc.stdin.write(json.dumps(plan))
        self.birds_proc.stdin.close()
        self.log.info(f'Birds flying (log {BIRDS_LOG})')

    def remove_birds(self) -> None:
        proc, self.birds_proc = self.birds_proc, None
        if proc is None or proc.poll() is not None:
            return
        proc.terminate()  # it removes them from Gazebo, then exits
        try:
            proc.wait(60)
        except subprocess.TimeoutExpired:
            proc.kill()
            self.log.warning('The bird process did not stop; birds may be left in Gazebo')

    def status(self) -> str:
        parts = []
        if not self.args.no_attack:
            parts.append(f'defender {self.attack_state()}')
        for i in self.hostile_ids:
            if i in self.removed:
                parts.append(f'hostile {i} down')
                continue
            d = self.distance(i, self.base)
            closest = self.judges[i].closest if self.judges else math.inf
            parts.append(f'hostile {i} {"?" if d is None else f"{d:.0f}"} m from the base, '
                         f'closest to the defender {closest:.0f} m, '
                         f'{self.seen_count[i]} reports')
        return '; '.join(parts)

    def stop_and_land(self) -> None:
        if not self.args.no_attack:
            self.attacker.attack.stop()
        for i, d in self.hostiles.items():
            if i not in self.removed:
                d.hold()
        drones = [self.defender] + [d for i, d in self.hostiles.items() if i not in self.removed]
        armed = [d for d in drones if d.armed]
        if not armed:
            return
        # Straight down above the base would land on the camp and on each other: every
        # drone first flies back above its start point at the height it has now (the
        # hostiles are above the defender, so their paths do not meet), then lands.
        self.log.info('Returning above the start points')
        try:
            for d in armed:
                if d.offboard and d.position is not None:
                    d.set_position(d.home[0], d.home[1], d.position[2])
            wait_until(lambda: all(d.reached(2.0) for d in armed if d.offboard),
                       RETURN_TIMEOUT_S, 'the drones to get back')
        except (TimeoutError, KeyboardInterrupt) as e:
            self.log.warning(f'Landing where they are ({e or "Ctrl-C"})')
        self.log.info('Landing')
        ids = [d.id for d in armed]
        self.manager.land(ids)
        wait_until(lambda: not any(d.armed for d in armed), LAND_TIMEOUT_S,
                   'every drone to land and disarm')
        self.log.info('Landed')

    def close_record(self, summary: str) -> None:
        if self.record_dir is None:
            return
        truth_file, self._truth_file = self._truth_file, None  # Gazebo's thread may still call
        for f in (truth_file, self._events_file):
            if f is not None:
                f.flush()
        with open(self.record_dir / 'meta.json', 'w') as f:
            json.dump({'settings': self.s, 'guidance': self.guidance,
                       'no_attack': self.args.no_attack, 'detect_args': self.args.detect_args,
                       'attack': {k: getattr(self.attack_config, k) for k in
                                  ('guard_altitude', 'max_speed', 'search_pitch_deg',
                                   'search_heading_deg', 'min_altitude')},
                       'summary': summary}, f, indent=1)
        self.log.info(f'Recorded to {self.record_dir} (~/Hive/data/tune/{self.args.record})')

    def summary(self) -> str:
        a = self.attack_config
        mode = ('watch only' if self.args.no_attack
                else f'{self.guidance} attack at {a.max_speed} m/s')
        extra = self.args.detect_args or []
        tracker = (extra[extra.index('--tracker') + 1]
                   if '--tracker' in extra[:-1] else 'confirmer')
        lines = [f'Scenario 2: {self.s["detector"]} detector ({tracker}) on drone '
                 f'{self.def_id} ({mode}), '
                 f'hostiles at {self.s["hostile_speed"]} m/s']
        for i in self.hostile_ids:
            parts = []
            if i in self.first_seen:
                t, d = self.first_seen[i]
                parts.append(f'first seen {self.when(t).replace("t=", "at t=")}, {d:.0f} m away, '
                             f'{self.seen_count[i]} reports')
            else:
                parts.append('never reported')
            if i in self.hits:
                parts.append(f'HIT at t={self.hits[i]:.1f} s')
            elif self.judges:
                parts.append(f'not hit, closest {self.judges[i].closest:.1f} m')
            lines.append(f'  hostile {i}: ' + '; '.join(parts))
        lines.append(f'  reports matching no hostile (within {MATCH_GATE_DEG:.0f} deg): '
                     f'{self.unmatched}')
        lines.append(f'  target switches between the hostiles while both flew: {self.switches}')
        if self.birds:
            lines.append(f'  birds: {len(self.birds)} (' + ', '.join(
                sorted({b.kind for b in self.birds})) + ')')
        if not self.args.no_attack:
            if self.guidance == 'mpc':
                t0 = self.t0 or 0.0
                steps = ', '.join(f'{what} at t={t - t0:.1f} s'
                                  for t, what in self.attacker.attack.transitions)
                lines.append(f'  defender (MPC), last attack: {steps or "nothing"}')
            else:
                lines.append(f'  defender, last attack: {self.attacker.summary()}')
        return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--config', default=DEFAULT_CONFIG, help='default %(default)s')
    parser.add_argument('--speed', type=float, help='hostile speed, m/s (default: config)')
    parser.add_argument('--attack-speed', type=float,
                        help="the defender's pursuit speed, m/s (default: attack.max_speed)")
    parser.add_argument('--stagger', type=float,
                        help='seconds between the two hostiles leaving (default: config)')
    parser.add_argument('--detector', choices=('yolo', 'baseline'),
                        help='camera detector (default: config, yolo)')
    parser.add_argument('--guidance', choices=('pursuit', 'mpc'), default='pursuit',
                        help='how the defender attacks: pure pursuit (the baseline, default) '
                             'or the MPC intercept')
    parser.add_argument('--no-attack', action='store_true',
                        help='the defender only hovers above the base and watches')
    parser.add_argument('--duration', type=float, default=math.inf,
                        help='simulation seconds after the hostiles leave (default: until Ctrl-C)')
    parser.add_argument('--climb-timeout', type=float, default=CLIMB_TIMEOUT_S,
                        help='wall-clock seconds to wait for the climb')
    parser.add_argument('--birds', type=int, default=0, metavar='N',
                        help='N birds fly in with the hostiles (default %(default)s)')
    parser.add_argument('--bird-seed', type=int,
                        help="seed for the birds' start points and speeds (default: random)")
    parser.add_argument('--record', metavar='NAME',
                        help="save the detector's boxes and Gazebo's truth for the threshold "
                             'replay (detect/replay_eval.py) in data/tune/NAME/ in the repo')
    parser.add_argument('--detect-args', nargs=argparse.REMAINDER,
                        help='the rest goes to the detector (detect/live.py), '
                             'e.g. --detect-args --tracker bytetrack --conf 0.4')
    args = parser.parse_args()

    config = load_config(args.config)
    s = settings(args.config, args)

    # Keep Ctrl-C as a KeyboardInterrupt in this thread, so we can still land.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('scenario2_test')
    run = Scenario2(node, config, s, args)

    with spinning(node):
        try:
            run.fly()
        except (TimeoutError, RuntimeError, KeyboardInterrupt) as e:
            run.log.error(f'Stopping ({e or "Ctrl-C"})')
        finally:
            summary = run.summary()
            print(summary, flush=True)
            run.close_record(summary)
            run.remove_birds()
            try:
                run.stop_and_land()
            finally:
                run.stop_detector()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
