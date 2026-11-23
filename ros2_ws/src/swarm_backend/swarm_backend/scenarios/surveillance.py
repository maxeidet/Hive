"""The world the surveillance scenarios fly in (`attack`, `baseline_eval`): the defenders
patrol the arc, the base station decides, each defender has a detector, a hostile drone
flies a straight track, and a referee scores hits from the true positions.

SurveillanceWorld is what the scenarios use; it has two backends with the same interface:

  GazeboSurveillance  (gazebo_surveillance.py) PX4 drones in Gazebo, the patrol and base
                      station ROS nodes, the real camera detectors (or the fake one), the
                      hostile drone restarted every run
  SimSurveillance     (sim_surveillance.py) the same patrol, attack and base-station logic
                      on SimDrones with the fake detector, on a simulated clock. No ROS,
                      no Gazebo: a run takes a fraction of a second

fly() is one run, the same on both: the hostile drone is placed at its start, the
defenders get back on patrol, the hostile flies its track and the run ends with a hit,
at the end of the track, at `timeout` or when `end` says so.

A defender that crashes (upside down, or down on the ground: scenarios/crash.py) sits out
the rest of the run, which is scored as it happened. Between runs, before the next one, it
is respawned below its slot and flies back up to it (Gazebo only; the fast simulation has
no crashes). Each run's crashes are in Flight.crashes, with their cause: 'intercept' when
the drone that hit crashed with the hit (the run is still a hit), else 'other'.
"""
import math
import threading
from dataclasses import dataclass, field, replace

from swarm_backend.config import SwarmConfig, Vector3
from swarm_backend.evaluation.arc_eval import TruthPoint
from swarm_backend.intercept.attack import State
from swarm_backend.evaluation.attack_scenario import ClosestApproach
from swarm_backend.swarm.base_station import Mode
from swarm_backend.intercept.detection import fake_config
from swarm_backend.intercept.interceptor import Status
from swarm_backend.evaluation.mock import HostileFlight
from swarm_backend.evaluation.paths import Path as Track
from swarm_backend.evaluation.paths import polyline_length
from swarm_backend.scenarios.crash import Crash, crash_cause

SLOT_TOLERANCE_M = 2.0
SETUP_TIMEOUT_S = 900.0  # climbs and repositioning, slow when Gazebo is
SETTLE_S = 3.0  # simulation seconds of hovering before each run
TRUTH_PERIOD_S = 0.02  # simulation seconds between recorded truth points
DETECTORS = ('real', 'fake', 'fake-realistic')
TRACKERS = ('confirmer', 'bytetrack')  # detect/live.py --tracker
GUIDANCE = ('pursuit', 'mpc')  # the defenders' attack (intercept/mpc_attack.py)


@dataclass(frozen=True)
class Options:
    detector: str = 'real'  # real (camera; the sim uses the realistic fake), fake, fake-realistic
    hit_radius: float = 1.5  # metres, from the true positions
    gimbal_gain: float | None = None  # attack config overrides; None keeps the config's
    max_speed: float | None = None
    seed: int | None = None  # fake detector noise
    detect_rate: float = 20.0  # images per second for each real detector
    record: bool = False  # keep every detector image (the baseline evaluation scores them)
    tracker: str = 'confirmer'  # real detector's tracker (detect/live.py --tracker)
    detect_args: tuple[str, ...] = ()  # more detect/live.py options, e.g. ('--detector', 'yolo')
    guidance: str = 'pursuit'  # the defenders' attack: pursuit (the baseline) or mpc

    def __post_init__(self):
        if self.detector not in DETECTORS:
            raise ValueError(f'detector must be one of {DETECTORS}')
        if self.tracker not in TRACKERS:
            raise ValueError(f'tracker must be one of {TRACKERS}')
        if self.guidance not in GUIDANCE:
            raise ValueError(f'guidance must be one of {GUIDANCE}')

    @property
    def setup(self) -> str:
        """Detector, tracker and attack in a few words, for the logs and results, e.g.
        'real, bytetrack --detector yolo --lock, mpc'. The fake detectors have no tracker."""
        parts = [self.detector]
        if self.detector == 'real':
            parts.append(' '.join((self.tracker,) + self.detect_args))
        parts.append(self.guidance)
        return ', '.join(parts)


@dataclass
class Flight:
    """What happened in one run. Times are seconds since the hostile drone set off,
    except t0 and t_end (the world's clock)."""
    t0: float
    t_end: float
    track_time: float  # seconds the whole track takes
    judges: dict  # defender id -> ClosestApproach
    killer: int | None = None
    engaged: tuple[float, int] | None = None  # (t, defender) when the first one was sent
    sightings: list = field(default_factory=list)  # (t, defender)
    hostile_at: Vector3 | None = None  # where the hostile drone was at the end
    return_s: float | None = None  # how long the killer took to get back on patrol
    truth: list = field(default_factory=list)  # [TruthPoint]
    records: dict = field(default_factory=dict)  # defender index -> [detector record]
    statuses: list = field(default_factory=list)  # (stamp, {defender index: Status})
    # Defenders that crashed in the run (or on the way back after it), respawned before the
    # next: [{'drone', 't' (s since set off), 'reason', 'cause' ('intercept' or 'other')}]
    crashes: list = field(default_factory=list)

    def hit_time(self, drone_id: int) -> float | None:
        h = self.judges[drone_id].hit_time
        return None if h is None else h - self.t0


class SurveillanceWorld:
    """The backend-independent part. A backend sets `patrols` (id -> anything with
    `.patrol`, a PatrolAction, and `.drone`), `station` (with `.station`, the BaseStation,
    and `.orders`), `hostile` (a Drone) and `log`, and implements the methods below that
    raise NotImplementedError."""

    backend = ''

    def __init__(self, swarm_config: SwarmConfig, options: Options):
        self.options = options
        self.surv = s = swarm_config.surveillance
        self.attack_config = self.make_attack_config(swarm_config.attack, options)
        self.swarm_config = replace(swarm_config, attack=self.attack_config)
        self.hostile_id = self.attack_config.target_id
        self.ids = tuple(s.drone_ids)
        if self.hostile_id in self.ids:
            raise RuntimeError(f'attack.target_id {self.hostile_id} is also a defender '
                               f'(surveillance.drone_ids {list(self.ids)})')
        self.index = {d: i for i, d in enumerate(self.ids)}  # slot order, left to right
        self.heading = math.radians(s.threat_heading_deg)
        self.patrols: dict = {}
        self.station = None
        self.hostile = None
        self.log = None
        self.flight: HostileFlight | None = None
        self.judges: dict[int, ClosestApproach] | None = None
        self.truth_log: list[TruthPoint] | None = None
        self._last_truth = -math.inf
        self._previous_truth = None  # (stamp, bodies, hostile, cameras) of the last poses
        self._hostile_truth: Vector3 | None = None
        self._sightings: list[tuple[float, int]] = []
        self._reported_crashes: set[Crash] = set()  # in a Flight already
        self._lock = threading.Lock()

    @staticmethod
    def make_attack_config(attack, options: Options):
        return fake_config(attack, options.detector == 'fake-realistic',
                           gimbal_gain=options.gimbal_gain, max_speed=options.max_speed,
                           seed=options.seed)

    @property
    def detector(self) -> str:
        """What detects, for the results."""
        return self.options.detector

    # --- the backend ------------------------------------------------------------

    def clock(self) -> float | None:
        """Simulation time, seconds."""
        raise NotImplementedError

    def wait_until(self, condition, timeout: float, what: str) -> None:
        raise NotImplementedError

    def pause(self) -> None:
        """Let a moment pass (one step of the run's loop)."""
        raise NotImplementedError

    def setup(self) -> None:
        """The defenders in the air and on patrol."""
        raise NotImplementedError

    def place_hostile(self, start: Vector3, goal: Vector3, speed: float) -> None:
        """The hostile drone hovering at `start`, facing `goal`, ready to fly at `speed`."""
        raise NotImplementedError

    def remove_hostile(self) -> None:
        """Shot down, or out of the way: no defender can see it any more."""
        raise NotImplementedError

    def report_kill(self, drone_id: int) -> None:
        """Tell the base station that `drone_id` destroyed its target."""
        raise NotImplementedError

    def read_records(self) -> dict:
        """The detector records (Options.record) since the last call, by defender index."""
        return {}

    def after_run(self) -> None:
        """Called at the end of a run, before the records are read."""

    def stop_and_land(self) -> None:
        raise NotImplementedError

    def crashed(self) -> dict[int, Crash]:
        """Defenders that have crashed and are not respawned yet. None in a backend
        without crashes (the fast simulation)."""
        return {}

    def respawn_defenders(self, drone_ids: list[int]) -> None:
        """Start crashed defenders again, back in the air on their slots and patrolling."""

    # --- shared -------------------------------------------------------------------

    def wait_sim(self, seconds: float) -> None:
        start = self.clock()
        self.wait_until(lambda: self.clock() - start >= seconds, SETUP_TIMEOUT_S,
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
        """Every defender back on patrol, except the crashed ones (they never get there)."""
        self.wait_until(lambda: all(self.on_patrol(i) for i in self.alive()
                                    if i not in self.crashed()),
                        SETUP_TIMEOUT_S, 'every defender to be back on patrol')

    def note_crashes(self, flight: Flight) -> None:
        """Between runs: record the run's crashes in `flight`, then respawn those drones
        and wait until every defender is on patrol again."""
        crashes = self.crashed()
        if not crashes:
            return
        killer = flight.killer
        hit_time = None if killer is None else flight.judges[killer].hit_time
        for d, crash in sorted(crashes.items()):
            if crash in self._reported_crashes:  # a respawn that failed before: tried again
                continue
            self._reported_crashes.add(crash)
            cause = crash_cause(crash, killer, hit_time)
            flight.crashes.append({'drone': d, 't': round(crash.time - flight.t0, 2),
                                   'reason': crash.reason, 'cause': cause})
            self.log.warning(f'Drone {d} crashed at t={crash.time - flight.t0:.1f} s '
                             f'({crash.reason}, cause: {cause}): respawning it')
        self.respawn_defenders(sorted(crashes))
        self.wait_patrol()

    def status(self, drone_id: int) -> Status:
        """What the defender's attack is doing, as the evaluation harness counts it."""
        patrol = self.patrols[drone_id].patrol
        if patrol.mode is Mode.ENGAGE and patrol.attack is not None:
            state = patrol.attack.state
            if state in (State.ATTACK, State.COAST):
                return Status.TRACKING
            return Status.LOST if state is State.LOST else Status.IDLE
        return Status.LOST if patrol.mode is Mode.RETURN else Status.IDLE

    def note_sighting(self, now: float, drone_id: int) -> None:
        with self._lock:
            self._sightings.append((now, drone_id))

    def observe_truth(self, stamp: float, bodies: dict, hostile: Vector3 | None,
                      cameras=None) -> None:
        """The true positions at `stamp`: `bodies` by defender id, the hostile drone (None
        when it is gone) and `cameras()`, which gives the camera poses by defender id
        (only called when they are recorded)."""
        self._hostile_truth = hostile
        judges = self.judges
        new_hit = False
        if judges is not None and hostile is not None:
            for i, judge in judges.items():
                if bodies.get(i) is not None:
                    was_hit = judge.hit
                    judge.update(bodies[i], hostile, stamp)
                    new_hit = new_hit or (judge.hit and not was_hit)
        previous, self._previous_truth = self._previous_truth, (stamp, bodies, hostile, cameras)
        log = self.truth_log
        if log is None or cameras is None:
            return
        if new_hit:
            # The referee decided on the segment from the previous poses to these: record
            # both, so the scoring (arc_eval.intercept_samples) sees the same contact.
            if previous is not None and previous[3] is not None \
                    and previous[0] > self._last_truth and previous[2] is not None:
                self._log_truth(log, *previous)
        elif stamp - self._last_truth < TRUTH_PERIOD_S:
            return
        self._log_truth(log, stamp, bodies, hostile, cameras)

    def _log_truth(self, log: list, stamp: float, bodies: dict, hostile, cameras) -> None:
        self._last_truth = stamp
        poses = cameras()
        log.append(TruthPoint(
            stamp, {self.index[i]: p for i, p in bodies.items() if p is not None},
            {self.index[i]: p for i, p in poses.items() if p is not None}, hostile))

    def fly(self, start: Vector3, goal: Vector3, speed: float, timeout: float = math.inf,
            end=None, path: list[Vector3] | None = None) -> Flight:
        """One run: the hostile drone flies from `start` to `goal` (world NED) at `speed`,
        straight, or along `path` (an open polyline from start to goal, paths.inbound).
        `end(t, hostile position)` ends it early."""
        points = [tuple(start), tuple(goal)] if path is None else [tuple(p) for p in path]
        self.read_records()  # skip what came before this run
        self.place_hostile(start, points[1], speed)  # facing along its first leg
        self.wait_patrol()
        self.wait_sim(SETTLE_S)

        track = Track(points, speed)  # closed, but the run ends at the goal
        alive = self.alive()
        judges = {d: ClosestApproach(self.options.hit_radius) for d in alive}
        with self._lock:
            self._sightings = []
        modes = {d: self.patrols[d].patrol.mode for d in alive}
        statuses = []
        self.truth_log, self._last_truth = [], -math.inf
        flight = Flight(t0=0.0, t_end=0.0, track_time=polyline_length(points) / speed,
                        judges=judges, statuses=statuses)
        self.flight = HostileFlight(self.hostile, track, self.clock)
        self.judges = judges
        self.flight.start()
        t0 = flight.t0 = self.clock()
        try:
            while True:
                now = self.clock()
                t = now - t0
                statuses.append((now, {self.index[d]: self.status(d) for d in alive}))
                for d in alive:
                    mode = self.patrols[d].patrol.mode
                    if mode is not modes[d]:
                        modes[d] = mode
                        self.log.info(f'  t={t:6.1f} s drone {d} {mode.value.upper()}, '
                                      f'closest so far {judges[d].closest:.1f} m')
                        if mode is Mode.ENGAGE and flight.engaged is None:
                            flight.engaged = (t, d)
                hits = [(j.hit_time, d) for d, j in judges.items() if j.hit]
                if hits:
                    flight.killer = min(hits)[1]
                    break
                if t >= flight.track_time or t >= timeout or (
                        end is not None and end(t, self._hostile_truth)):
                    break
                self.pause()
        finally:
            flight.t_end = self.clock()
            self.judges = None
            flight.hostile_at = self._hostile_truth
            self.flight.stop()
            self.flight = None
            self.remove_hostile()
            flight.truth, self.truth_log = self.truth_log, None
        if flight.killer is not None:
            self.report_kill(flight.killer)
        killed_at = self.clock()
        with self._lock:
            flight.sightings = [(t - t0, i) for t, i in self._sightings]
        try:
            self.wait_patrol()
            if flight.killer is not None and flight.killer not in self.crashed():
                flight.return_s = self.clock() - killed_at
                self.log.info(f'Drone {flight.killer} back on patrol after '
                              f'{flight.return_s:.1f} s')
        except TimeoutError as e:
            self.log.warning(str(e))
        self.after_run()
        flight.records = self.read_records()
        try:  # between runs: after the records, so the respawn is in none of them
            self.note_crashes(flight)
        except (RuntimeError, TimeoutError) as e:
            # Carry on with the drones that fly; the next run tries the respawn again.
            self.log.warning(f'Respawn failed: {e}')
        return flight
