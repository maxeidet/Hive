"""The surveillance world in Gazebo (see surveillance.py), in one ROS process:

  - a patrol for each defender (nodes/patrol_node.py): take off, hold the slot on the arc,
    sweep the camera, attack when the base station allows it
  - the base station (nodes/base_station_node.py)
  - a detector for each defender: the real camera detector (`detect ID`, started here, log
    in /tmp/detect_<id>.log), or the fake one from Gazebo's poses
  - the referee: the hostile drone's flight, and the hits

The defenders are surveillance.drone_ids (gimbal drones 1-3), the hostile drone is
attack.target_id (drone 4 in scenarios/surveillance.yaml; `spawn` starts it with the
defenders). It cannot be drone 0: the start scripts run drone 0's PX4 as the container's
own process, so stopping it stops the container.

Every run the hostile drone is restarted on the ground below its start point and climbs.
A hit removes it: its PX4 is stopped and the model moved to a graveyard far outside the
battle (GRAVEYARD_ENU). The base station gets a `kill` event and the defender, which
survives, flies back to its slot. A defender can still tumble in Gazebo after a hit; if one
has crashed, the base station counts it as lost and the others carry on. At the end the
defenders land at their slots and the hostile drone is started again at its spawn point.

Runs in the container (it starts PX4 processes and calls `gz`).
"""
import dataclasses
import json
import math
import os
import random
import signal
import subprocess
import sys
import time
from pathlib import Path

from std_msgs.msg import String

from swarm_backend import ground_truth
from swarm_backend.intercept import surveillance_msgs as msgs
from swarm_backend.nodes.base_station_node import BaseStationNode
from swarm_backend.nodes.fake_detection import FakeDetectionNode, GazeboTruth
from swarm_backend.nodes.hivemind_node import Hivemind
from swarm_backend.nodes.patrol_node import PatrolNode
from swarm_backend.intercept.inbound import plan_birds
from swarm_backend.px4.observer import DroneObserver
from swarm_backend.px4.px4_drone import PX4Drone
from swarm_backend.runner import LAND_TIMEOUT_S, wait_until
from swarm_backend.save_images import drone_names
from swarm_backend.scenarios.surveillance import (
    SETUP_TIMEOUT_S, SLOT_TOLERANCE_M, SurveillanceWorld)
from swarm_backend.scenarios.bird_flight import BirdFlight
from swarm_backend.sensor_view import find_sensors
from swarm_backend.swarm.manager import SwarmManager
from swarm_backend.swarm.swarm import Swarm

TOLERANCE_M = 1.5
CONNECT_TIMEOUT_S = 180.0  # freshly spawned drones take a while to reach ROS
RESPAWN_TIMEOUT_S = 180.0
SETTLE_ON_GROUND_S = 2.0  # simulation seconds after moving the hostile model, before its PX4
UPRIGHT_MAX_TILT_DEG = 10.0  # the hostile must stand this level before its PX4 starts
STILL_M = 0.05  # ... and move less than this over SETTLE_ON_GROUND_S
PLACE_ATTEMPTS = 3
# Where a shot-down hostile drone goes (Gazebo ENU x, y): on the ground at the far corner of
# the 1000 x 1000 m ground plane, far outside the forest and every camera's range.
GRAVEYARD_ENU = (450.0, -450.0)
# A second hostile's spot (fly_many): 20 m away, inside the films' graveyard radius
# (demo/takes.py GRAVEYARD_M, camera_video.Tracker), so they treat it the same.
GRAVEYARD2_ENU = (450.0, -430.0)


@dataclasses.dataclass
class _Hostile:
    """One hostile drone: its Gazebo model, PX4 process, ROS drone and graveyard spot."""
    id: int
    name: str
    drone: object  # PX4Drone
    manager: object  # SwarmManager of it alone
    px4: object  # Px4Process
    graveyard: tuple
    pose: tuple | None = None  # (xyz ENU, quaternion) from Gazebo
POLL_S = 0.05
HOSTILE_SPEED_MARGIN = 3.0  # m/s above the track speed for the hostile's PX4 limit
PX4_MIN_SPEED_LIMIT = 12.0  # PX4's default MPC_XY_VEL_MAX


# --- Gazebo and PX4 processes (this runs in the container) -------------------------

def find_scene(defender_ids, hostile_id) -> tuple[str, dict[int, str], dict[int, str]]:
    """`hostile_id`: one id or a tuple of them."""
    """(world, model names by drone id, camera link by defender) from Gazebo, or a
    RuntimeError saying what to start."""
    gz_topics = subprocess.run(
        ['gz', 'topic', '-l'], capture_output=True, text=True, check=True).stdout.split()
    world, names = drone_names(gz_topics)
    hostile_ids = hostile_id if isinstance(hostile_id, tuple) else (hostile_id,)
    missing = [i for i in (*defender_ids, *hostile_ids) if i not in names]
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


def stop_motors(model_name: str, motors: int = 4) -> bool:
    """Zero motor speed for a model whose PX4 was stopped. Gazebo's motor plugin keeps the
    last speed it was sent, so a drone whose PX4 stops in flight flies on with nobody
    controlling it: it tumbles, and lands upside down where it is moved next (PX4 then
    fails its pre-flight checks with 'Attitude failure (roll)')."""
    result = subprocess.run(
        ['gz', 'topic', '-t', f'/{model_name}/command/motor_speed', '-m', 'gz.msgs.Actuators',
         '-p', f'velocity: [{", ".join(["0"] * motors)}]'],
        capture_output=True, text=True)
    return result.returncode == 0


def tilt_deg(q) -> float:
    """How far a body leans from upright (degrees), from its (x, y, z, w) quaternion."""
    x, y, z, w = q
    up_z = 1.0 - 2.0 * (x * x + y * y)  # world z of the body's up axis
    return math.degrees(math.acos(max(-1.0, min(1.0, up_z))))


def detector_running(drone_id: int) -> bool:
    return subprocess.run(['pgrep', '-f', f'swarm_backend.detect.live --id {drone_id}( |$)'],
                          capture_output=True).returncode == 0


# --- the world -----------------------------------------------------------------------

class GazeboSurveillance(SurveillanceWorld):
    """Everything the scenarios need, wired to one ROS node."""

    backend = 'gazebo'

    def __init__(self, node, swarm_config, options):
        super().__init__(swarm_config, options)
        self.node = node
        self.log = node.get_logger()
        if self.hostile_id == 0:
            raise RuntimeError('attack.target_id is 0, but the scenario restarts the hostile '
                               "drone's PX4 every run, and drone 0's PX4 runs the container. "
                               'Use a separate hostile drone (drone 4 in '
                               'scenarios/surveillance.yaml)')

        self.world, self.names, self.cameras = find_scene(self.ids, self.hostile_ids)
        # Everything is timed on Gazebo's simulation clock: steady however slowly Gazebo
        # runs, unlike PX4's time stamps, which jump when its time sync adjusts.
        self.truth = GazeboTruth(self.world)
        self.truth.listeners.append(self._on_truth)

        self.patrols = {i: PatrolNode(node, self.swarm_config, i, clock=self.clock,
                                      guidance=options.guidance)
                        for i in self.ids}
        self.defenders = SwarmManager(Swarm([p.drone for p in self.patrols.values()]),
                                      TOLERANCE_M)
        self._h: dict[int, _Hostile] = {}
        for k, h in enumerate(self.hostile_ids):
            drone = PX4Drone(node, swarm_config.drone(h), rate_hz=swarm_config.rate_hz)
            self._h[h] = _Hostile(h, self.names[h], drone, SwarmManager(Swarm([drone]), TOLERANCE_M),
                                  Px4Process(h), GRAVEYARD_ENU if k == 0 else GRAVEYARD2_ENU)
        # The single-hostile code's names for the first one (attack.target_id)
        first = self._h[self.hostile_id]
        self.hostile, self.hostile_manager, self.px4 = first.drone, first.manager, first.px4
        self.station: BaseStationNode | None = None  # once Gazebo's clock runs
        fake = options.detector != 'real'
        self.fakes = [
            FakeDetectionNode(node, dataclasses.replace(self.attack_config, attacker_id=i),
                              self.truth, self.names, self.cameras[i])
            for i in self.ids] if fake else []
        # The defenders are friendly: the hivemind broadcasts where they are on
        # /hive/friendlies, and every real detector (detect/live.py) marks a track that
        # points at one of them as a friend, so it is never locked or reported. Only
        # surveillance.drone_ids: the hostile drone (attack.target_id) is never on it.
        # Every other drone in the config (drone 0, which runs the container and stands in
        # the base) is ours too: without it, a defender that sees it reports a target.
        bystanders = [DroneObserver(node, d) for d in swarm_config.drones
                      if d.id not in self.ids and d.id not in self.hostile_ids]
        self.friendly_ids = sorted(list(self.ids) + [b.id for b in bystanders])
        self.hivemind = (None if fake else
                         Hivemind(node, [p.drone for p in self.patrols.values()] + bystanders))
        self.detect_procs: list[subprocess.Popen] = []
        self._offsets = {d: 0 for d in self.ids}

        self._events = node.create_publisher(String, msgs.EVENT_TOPIC, 10)
        node.create_subscription(String, msgs.SIGHTING_TOPIC, self._on_sighting, 10)
        node.create_timer(1.0 / swarm_config.rate_hz, self._on_timer)

    # --- callbacks ------------------------------------------------------------

    def _on_timer(self) -> None:
        if self.flight is not None:
            self.flight.step()
        for f in self.flights:
            f.step()

    def _on_truth(self, stamp: float, poses) -> None:
        for h in self._h.values():
            model = next((p for p in poses if p[0] == h.name), None)
            if model is not None:
                h.pose = (model[2], model[3])
        bodies = {i: ground_truth.body_position(poses, self.names[i]) for i in self.ids}
        if self._multi_judges is not None:
            self.observe_truth_many(stamp, bodies, {
                h.id: ground_truth.body_position(poses, h.name) for h in self._h.values()})
            return
        hostile = ground_truth.body_position(poses, self.names[self.hostile_id])
        self.observe_truth(stamp, bodies, hostile, lambda: {
            i: ground_truth.camera_pose(poses, self.names[i], self.cameras[i]) for i in self.ids})

    @property
    def _hostile_pose(self):
        return self._h[self.hostile_id].pose

    def _hostile(self, hostile_id: int | None) -> _Hostile:
        return self._h[self.hostile_id if hostile_id is None else hostile_id]

    def hostile_drone(self, hostile_id: int):
        return self._h[hostile_id].drone

    def _on_sighting(self, msg) -> None:
        now = self.clock()
        if now is not None:
            self.note_sighting(now, msgs.decode_sighting(msg.data).drone_id)

    # --- the backend ----------------------------------------------------------

    def clock(self) -> float | None:
        return self.truth.clock()

    def wait_until(self, condition, timeout: float, what: str) -> None:
        wait_until(condition, timeout, what)  # wall clock

    def pause(self) -> None:
        time.sleep(POLL_S)

    def move_hostile(self, east: float, north: float, hostile_id: int | None = None) -> None:
        h = self._hostile(hostile_id)
        if not set_model_pose(self.world, h.name, east, north):
            self.log.warning(f'Gazebo did not confirm moving {h.name}')

    def remove_hostile(self, hostile_id: int | None = None) -> None:
        """Shot down (or out of the way for the next run): the model moved to the
        graveyard, far outside the battle, then its PX4 stopped. In that order: stopping PX4
        takes seconds, and a hostile drone falling with its motors off is a target the
        other defenders chase down into the trees."""
        h = self._hostile(hostile_id)
        self.move_hostile(*h.graveyard, hostile_id=h.id)
        self.stop_hostile_px4(h.id)

    def stop_hostile_px4(self, hostile_id: int | None = None) -> None:
        """Its PX4 stopped and its motors with it (stop_motors)."""
        h = self._hostile(hostile_id)
        h.px4.stop()
        if not stop_motors(h.name):
            self.log.warning(f'Could not stop the motors of {h.name}')

    def set_hostile_down(self, east: float, north: float, hostile_id: int | None = None) -> None:
        """The hostile model upright and still on the ground at (east, north), Gazebo ENU,
        its PX4 stopped. Moved again if it does not stand (up to PLACE_ATTEMPTS times)."""
        h = self._hostile(hostile_id)
        for attempt in range(1, PLACE_ATTEMPTS + 1):
            self.move_hostile(east, north, hostile_id=h.id)
            self.wait_sim(SETTLE_ON_GROUND_S / 2)
            before = h.pose
            self.wait_sim(SETTLE_ON_GROUND_S / 2)
            after = h.pose
            if before is None or after is None:
                return  # no poses: nothing to check
            tilt, moved = tilt_deg(after[1]), math.dist(before[0], after[0])
            if tilt <= UPRIGHT_MAX_TILT_DEG and moved <= STILL_M:
                return
            self.log.warning(f'Hostile drone {h.id} not standing after the move (tilt {tilt:.0f} '
                             f'deg, moved {moved:.2f} m), attempt {attempt}/{PLACE_ATTEMPTS}')
            stop_motors(h.name)
        raise RuntimeError(f'The hostile drone {h.id} does not stand still at E {east:+.0f} '
                           f'N {north:+.0f} m (see Gazebo)')

    def respawn_hostile(self, north: float, east: float, hostile_id: int | None = None) -> None:
        """A fresh hostile drone on the ground at (north, east), connected and ready: the
        same Gazebo model, moved there, with a new PX4 attached to it."""
        h = self._hostile(hostile_id)
        self.stop_hostile_px4(h.id)
        time.sleep(1.0)  # its last messages
        self.log.info(f'Starting hostile drone {h.id} at N {north:+.0f} E {east:+.0f} m')
        self.set_hostile_down(east, north, hostile_id=h.id)
        h.drone.reset((north, east, 0.0))
        h.px4.start(h.name)
        log = f'/tmp/px4_{h.id}.log'
        wait_until(h.manager.connected, RESPAWN_TIMEOUT_S,
                   f'the hostile drone {h.id} to connect (see {log})')
        wait_until(h.manager.ready, RESPAWN_TIMEOUT_S,
                   f"the hostile drone {h.id}'s pre-flight checks (see {log})")

    def place_hostile(self, start, goal, speed, hostile_id: int | None = None) -> None:
        h = self._hostile(hostile_id)
        self.respawn_hostile(start[0], start[1], hostile_id=h.id)
        self.set_hostile_param('MPC_XY_VEL_MAX', max(PX4_MIN_SPEED_LIMIT,
                                                     speed + HOSTILE_SPEED_MARGIN), hostile_id=h.id)
        altitude = h.drone.home[2] - start[2]
        self.log.info(f'Hostile {h.id} taking off to {altitude:.0f} m')
        h.manager.takeoff(altitude)
        wait_until(h.manager.all_reached, SETUP_TIMEOUT_S, f'the hostile drone {h.id} to climb')
        yaw = math.atan2(goal[1] - start[1], goal[0] - start[0])
        h.drone.set_position(*start, yaw=yaw)
        wait_until(lambda: h.drone.reached(TOLERANCE_M), SETUP_TIMEOUT_S,
                   f'the hostile drone {h.id} at its start point')

    BIRDS_LOG = '/tmp/surveillance_birds.log'

    def start_extras(self, start, goal, speed) -> None:
        """`birds` birds around the hostile's start, flying over the base (intercept/inbound.py
        plan_birds), named bird_<n> so the pose log and the films find them."""
        if self.birds <= 0:
            return
        from swarm_backend.birds import KINDS
        base = self.swarm_config.surveillance.base_position
        plans = plan_birds(self.birds, [tuple(start)], (base[0], base[1]), list(KINDS), speed,
                           random.Random(self.bird_seed), prefix='bird_')
        self._bird_flight = BirdFlight(self.world, self.BIRDS_LOG)
        self._bird_flight.start(plans, self.clock())
        self.log.info(f'{len(plans)} birds flying in with the hostile (log {self.BIRDS_LOG})')

    def stop_extras(self) -> None:
        flight, self._bird_flight = getattr(self, '_bird_flight', None), None
        if flight is not None and not flight.stop():
            self.log.warning('The bird process did not stop; birds may be left in Gazebo')

    def report_kill(self, drone_id: int) -> None:
        self._events.publish(String(data=msgs.encode_event(msgs.EventMsg(drone_id, 'kill'))))

    def set_hostile_param(self, name: str, value: float, hostile_id: int | None = None) -> None:
        """Set a PX4 parameter on a hostile drone (e.g. its speed limit)."""
        hid = self._hostile(hostile_id).id
        result = subprocess.run(['/opt/px4-gazebo/bin/px4-param', '--instance',
                                 str(hid), 'set', name, str(value)],
                                capture_output=True, text=True)
        if result.returncode != 0:
            self.log.warning(f'Could not set {name} on drone {hid}: '
                             f'{(result.stdout + result.stderr).strip()}')

    # --- detectors --------------------------------------------------------------

    @staticmethod
    def record_path(drone_id: int) -> str:
        return f'/tmp/detect_{drone_id}.jsonl'

    def start_detectors(self) -> None:
        if self.fakes:
            for fake in self.fakes:
                print(fake.describe(), flush=True)
            return
        running = [i for i in self.ids if detector_running(i)]
        if running:
            raise RuntimeError(f'A detector already runs for drone(s) {running}; stop it, this '
                               'scenario starts its own')
        for i in self.ids:
            log = open(f'/tmp/detect_{i}.log', 'w')
            extra = []
            if self.options.record:  # every image, for the baseline evaluation
                path = Path(self.record_path(i))
                path.unlink(missing_ok=True)
                extra = ['--record', str(path)]
            if self.swarm_config.detect.body_mask:  # ignore its own arms, legs, propellers
                extra.append('--body-mask')
            self.detect_procs.append(subprocess.Popen(
                [sys.executable, '-m', 'swarm_backend.detect.live', '--id', str(i),
                 '--rate', str(self.options.detect_rate), '--tracker', self.options.tracker]
                + list(self.options.detect_args) + extra,
                stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True))
        self.log.info(f'Real detectors ({self.options.tracker}'
                      + (f' {" ".join(self.options.detect_args)}' if self.options.detect_args else '')
                      + f') started for drones {list(self.ids)}, logs '
                      f'/tmp/detect_<id>.log; friendly drones {self.friendly_ids} ignored'
                      + (', own airframe ignored' if self.swarm_config.detect.body_mask
                         else ''))

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

    def read_records(self) -> dict:
        if not self.options.record:
            return {}
        out = {}
        for d in self.ids:
            path = Path(self.record_path(d))
            recs = []
            if path.exists():
                with open(path) as f:
                    f.seek(self._offsets[d])
                    for line in f:
                        if line.endswith('\n'):
                            recs.append(json.loads(line))
                    self._offsets[d] = f.tell()
            out[self.index[d]] = recs
        return out

    def after_run(self) -> None:
        if self.options.record:
            time.sleep(1.0)  # the detectors' last lines

    # --- setup and the end ------------------------------------------------------

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
        self.log.info(f'Taking off: defenders {list(self.ids)} to {s.altitude} m')
        self.defenders.takeoff(s.altitude)
        wait_until(self.defenders.all_reached, SETUP_TIMEOUT_S, f'{s.altitude} m')
        # Not on the ground: from there the cameras see the base's trees and tents and the
        # other defenders a few metres away, and reported them as targets inside the base.
        # The flight out to the slots gives the detectors time to start before the release.
        self.start_detectors()
        for i, p in self.patrols.items():
            self.log.info(f'Drone {i} to its slot at {p.patrol.slot_angle:+.0f} deg on the arc')
            p.patrol.start()
        self.wait_patrol()
        self.log.info('All defenders on patrol')

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
        # The hostile drones back where the config (and mount-backend.sh) expects them.
        for h in self._h.values():
            hx, hy, _ = self.swarm_config.drone(h.id).spawn_ned
            self.log.info(f'Starting drone {h.id} again at its spawn point')
            self.stop_hostile_px4(h.id)
            try:
                self.set_hostile_down(hy, hx, hostile_id=h.id)
            except (RuntimeError, TimeoutError) as e:
                self.log.warning(str(e))
            h.drone.reset((hx, hy, 0.0))
            h.px4.start(h.name)


def ros_world(name: str, swarm_config, options):
    """(rclpy, node, world) for a scenario; the caller spins the node."""
    import rclpy
    from rclpy.signals import SignalHandlerOptions

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node(name)
    try:
        return rclpy, node, GazeboSurveillance(node, swarm_config, options)
    except Exception:
        node.destroy_node()
        rclpy.try_shutdown()
        raise
