"""The surveillance world without Gazebo or ROS (see surveillance.py): the same
PatrolAction, AttackAction and base station (intercept/dispatcher.py) as in Gazebo, on
SimDrones (fast_sim.py), with the fake detector (intercept/detection.py) on each
defender's gimbal camera, all stepped on one simulated clock.

What is simpler than in Gazebo: the drones fly like fast_sim.SimDrone (no PX4, no
physics, no crashes), the gimbal and the nose turn at once, take-off is skipped (the
defenders start at their patrol height), messages arrive at once and the detector only
ever sees the hostile drone. The real camera detector needs images, so `detector: real`
runs the fake one with the real one's errors (detection.REALISTIC) at its image rate.
"""
import math
from dataclasses import replace

from swarm_backend.detect.camera import CameraModel
from swarm_backend.detect.frame import Body, Kind, aimed_camera, make_frame
from swarm_backend.detect.scenario import HOSTILE_ID
from swarm_backend.fast_sim import SimDrone
from swarm_backend.intercept.attack import DetectionInbox
from swarm_backend.intercept.detection import REALISTIC, FakeDetector
from swarm_backend.intercept.dispatcher import Dispatcher
from swarm_backend.intercept.mpc_attack import make_attack
from swarm_backend.intercept.patrol import STALE_DETECTION_S, PatrolAction
from swarm_backend.intercept.surveillance_msgs import EventMsg
from swarm_backend.scenarios.surveillance import SurveillanceWorld

STEP_S = 0.05  # 20 Hz: the detectors' image rate
TICK_HZ = 5.0  # the base station's own clock, as nodes/base_station_node.py
HOSTILE_SPEED_MARGIN = 3.0  # m/s above the track speed, as in Gazebo


class SimLog:
    """The ROS logger's info/warning/error, printed with the simulation time."""

    def __init__(self, clock, quiet: bool = False):
        self.clock = clock
        self.quiet = quiet

    def _print(self, level: str, text: str) -> None:
        if not self.quiet or level != 'INFO':
            print(f'[{level}] [sim {self.clock():8.2f}] {text}', flush=True)

    def info(self, text: str) -> None:
        self._print('INFO', text)

    def warning(self, text: str) -> None:
        self._print('WARN', text)

    def error(self, text: str) -> None:
        self._print('ERROR', text)


class SimDefender:
    def __init__(self, drone, inbox, patrol, detector):
        self.drone = drone
        self.inbox = inbox
        self.patrol = patrol
        self.detector = detector


class SimSurveillance(SurveillanceWorld):

    backend = 'sim'

    def __init__(self, swarm_config, options, quiet: bool = False):
        super().__init__(swarm_config, options)
        self.time = 0.0
        self.log = SimLog(self.clock, quiet)
        a = self.attack_config
        if options.detector == 'real':
            # No images here: the fake detector with the real one's errors and rate.
            changes = dict(REALISTIC)
            changes['fake_rate_hz'] = options.detect_rate
            a = replace(a, **changes)
        self.detector_config = a
        self.camera = CameraModel()
        self.target_size = swarm_config.detect.target_size
        speed = max(SimDrone.MAX_SPEED, self.attack_config.max_speed)
        self.patrols = {}
        for slot, i in enumerate(self.ids):
            drone = SimDrone(i, swarm_config.drone(i).spawn_ned, max_speed=speed)
            drone.connected = drone.preflight_ok = drone.armed = drone.offboard = True
            inbox = DetectionInbox()
            patrol = PatrolAction(
                drone, inbox, self.attack_config, self.surv, slot, self.clock,
                make_attack=self._attack_maker(drone, inbox))
            detector = FakeDetector(replace(a, seed=a.seed + i), self.camera)
            self.patrols[i] = SimDefender(drone, inbox, patrol, detector)
        self.station = Dispatcher(self.surv, lambda level, text: getattr(self.log, level)(text))
        self.hostile = SimDrone(self.hostile_id, swarm_config.drone(self.hostile_id).spawn_ned)
        self.hostile.connected = self.hostile.preflight_ok = True
        self.hostile_present = False
        self._records = {self.index[i]: [] for i in self.ids}
        self._next_status = 0.0
        self._next_tick = 0.0

    def _attack_maker(self, drone, inbox):
        def make(guard):
            return make_attack(self.options.guidance, drone, inbox, self.attack_config,
                               self.swarm_config.intercept, self.clock, guard=guard,
                               keep_recent_s=STALE_DETECTION_S)
        return make

    @property
    def detector(self) -> str:
        if self.options.detector == 'real':
            return 'sim: fake detector with the real one\'s errors'
        return f'sim: {self.options.detector}'

    # --- the clock --------------------------------------------------------------

    def clock(self) -> float:
        return self.time

    def wait_until(self, condition, timeout: float, what: str) -> None:
        """Steps the simulation until `condition()`; `timeout` is simulation seconds."""
        deadline = self.time + timeout
        while not condition():
            if self.time >= deadline:
                raise TimeoutError(f'Timed out after {timeout:.0f} s (simulation) waiting for '
                                   f'{what}')
            self.step()

    def pause(self) -> None:
        self.step()

    def step(self) -> None:
        """One STEP_S: setpoints, images, the patrols, messages, then the drones move."""
        now = self.time
        if self.flight is not None:
            self.flight.step()
        for i, d in self.patrols.items():
            self._look(i, d, now)
        for i, d in self.patrols.items():
            d.patrol.step()
            for s in d.patrol.take_sightings():
                self.note_sighting(now, i)
                self._deliver(self.station.on_sighting(s, now))
            for e in d.patrol.take_events():
                self.log.info(f'drone {i}: {e.kind}')
                self._deliver(self.station.on_event(e, now))
        if now >= self._next_status:
            self._next_status = now + 1.0 / self.surv.status_hz
            for i, d in self.patrols.items():
                status = d.patrol.status()
                if status is None or not d.patrol.running:
                    continue
                self._deliver(self.station.on_status(status, now))
                for j, other in self.patrols.items():
                    if j != i:
                        other.patrol.note_friend(i, status.position, now)
        if now >= self._next_tick:
            self._next_tick = now + 1.0 / TICK_HZ
            self._deliver(self.station.tick(now))

        self.time = now + STEP_S
        for d in self.patrols.values():
            d.drone.advance(STEP_S)
            d.drone.stamp = self.time
        if self.hostile_present:
            self.hostile.advance(STEP_S)
        self.hostile.stamp = self.time
        self.observe_truth(self.time, {i: d.drone.position for i, d in self.patrols.items()},
                           self._hostile_position(),
                           lambda: {i: self.camera_pose(d.drone) for i, d in self.patrols.items()})

    def _deliver(self, orders) -> None:
        for order in orders:
            d = self.patrols.get(order.drone_id)
            if d is not None:
                before = d.patrol.mode
                d.patrol.apply(order)
                if d.patrol.mode is not before:
                    self.log.info(f'drone {order.drone_id}: {d.patrol.mode.value.upper()}')

    # --- cameras and detectors ------------------------------------------------

    @staticmethod
    def camera_pose(drone: SimDrone) -> dict:
        """The gimbal camera where it is aimed (pitch from the horizon, yaw from the nose)."""
        pitch, yaw = drone.gimbal_aim or (0.0, 0.0)
        return aimed_camera(drone.position, pitch, math.degrees(drone.yaw) + yaw)

    def _hostile_position(self):
        return self.hostile.position if self.hostile_present else None

    def _look(self, drone_id: int, d: SimDefender, now: float) -> None:
        detector = d.detector
        if detector.wants_image(now):
            camera = self.camera_pose(d.drone)
            hostile = self._hostile_position()
            detector.observe(now, camera, hostile)
            if self.options.record:
                self._records[self.index[drone_id]].append(
                    self._record(now, camera, hostile, detector))
        for stamp, found in detector.due(now):
            if d.patrol.is_friendly(found.position, now):
                continue
            d.inbox.put_position(found.position, stamp, now)
            d.inbox.put_offset(found.offset, stamp, now)

    def _record(self, now: float, camera: dict, hostile, detector: FakeDetector) -> dict:
        """A detector record as detect/live.py --record writes it: the confirmed track is
        a box on the hostile drone's true pixel, while the fake detector confirms it."""
        confirmed = []
        if hostile is not None and detector.streak >= detector.confirm_frames:
            frame = make_frame(self.camera, now, 0, camera,
                               [Body(HOSTILE_ID, Kind.DRONE, hostile, self.target_size)])
            for o in frame.objects:
                confirmed.append([o.pixel[0], o.pixel[1], o.size_px, o.size_px, 1])
        return {'stamp': now, 'width': self.camera.width, 'height': self.camera.height,
                'confirmed': confirmed}

    def read_records(self) -> dict:
        out, self._records = self._records, {self.index[i]: [] for i in self.ids}
        return out

    # --- the hostile drone -----------------------------------------------------

    def place_hostile(self, start, goal, speed) -> None:
        h = self.hostile
        h.position, h.velocity, h.target = tuple(start), (0.0, 0.0, 0.0), tuple(start)
        h.max_speed = speed + HOSTILE_SPEED_MARGIN
        h.armed = h.offboard = True
        h.set_position(*start, yaw=math.atan2(goal[1] - start[1], goal[0] - start[0]))
        self.hostile_present = True

    def remove_hostile(self) -> None:
        self.hostile_present = False
        self.hostile.armed = self.hostile.offboard = False

    def report_kill(self, drone_id: int) -> None:
        self._deliver(self.station.on_event(EventMsg(drone_id, 'kill'), self.time))

    # --- setup and the end ------------------------------------------------------

    def setup(self) -> None:
        s = self.surv
        self._deliver(self.station.start(self.time))
        for i, d in self.patrols.items():  # take-off is skipped: straight to patrol height
            x, y, _ = d.drone.position
            d.drone.position = (x, y, d.drone.home[2] - s.altitude)
            self.log.info(f'Drone {i} to its slot at {d.patrol.slot_angle:+.0f} deg on the arc')
            d.patrol.start()
        self.wait_patrol()
        self.log.info('All defenders on patrol')

    def stop_and_land(self) -> None:
        if self.flight is not None:
            self.flight.stop()
            self.flight = None
        for d in self.patrols.values():
            d.patrol.stop()
        self.remove_hostile()
