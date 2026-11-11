"""The MPC intercept (mpc_action.MpcIntercept) behind the baseline attack's face, so
everything that flies an AttackAction can fly the MPC instead: the attack node
(`--guidance mpc`), the surveillance patrols and the scenarios (`--guidance mpc`, or
`--setup yolo-bytetrack-mpc`).

Both read the same DetectionInbox (the target_position topic). Here an InboxDetector
turns each message into a world position, the MPC's Kalman filter tracks it, and the MPC
flies; this class only translates what the rest of the code asks an attack:

  state          SEARCH until the first fix, ATTACK while tracking, LOST once the track
                 timed out (intercept.track_timeout); never COAST or HIT (the referee
                 scores hits, as for the baseline)
  target_world, target_range, closing_speed, commanded, engaged_since, running
                 for the operator UI (attack.attack_status)

The MPC flies at attack.max_speed (swarm.yaml may keep it low for testing) and within the
baseline's limits (attack.limit_velocity: minimum height, climb and descent speed), so
the two can be compared on the same runs.
"""
import dataclasses
import math
from collections.abc import Callable

from swarm_backend.config import AttackConfig, InterceptConfig, Vector3
from swarm_backend.intercept.attack import AttackAction, DetectionInbox, State, limit_velocity
from swarm_backend.intercept.inbox_detector import InboxDetector
from swarm_backend.intercept.interceptor import Status
from swarm_backend.intercept.mpc_action import MpcIntercept
from swarm_backend.px4.drone import Drone

GUIDANCE = ('pursuit', 'mpc')


class MpcAttack:

    def __init__(self, drone: Drone, inbox: DetectionInbox, config: AttackConfig,
                 intercept: InterceptConfig, clock: Callable[[], float | None],
                 guard: Vector3 | None = None, keep_recent_s: float = 0.0):
        """`guard` is where the drone came from (the patrol's slot); the MPC itself hovers
        where it is when it loses the target, the patrol or node flies it back. Detections
        that arrived more than `keep_recent_s` before start() are ignored (a patrol keeps
        the one that made it engage; the attack node drops what came during take-off)."""
        self.keep_recent_s = keep_recent_s
        self.drone = drone
        self.inbox = inbox
        self.config = config
        self.clock = clock
        self._guard = guard
        self.detector = InboxDetector(drone, inbox, clock)
        self.mpc = MpcIntercept(
            drone, self.detector,
            dataclasses.replace(intercept, interceptor_max_speed=config.max_speed), clock,
            search=(config.search_pitch_deg, config.search_heading_deg),
            limit=lambda v: limit_velocity(v, drone.altitude, config))
        self.engaged_since: float | None = None
        self.first_detection: float | None = None
        self._lost_since = -math.inf

    @property
    def guard(self) -> Vector3:
        if self._guard is not None:
            return self._guard
        x, y, z = self.drone.home
        return (x, y, z - self.config.guard_altitude)

    @property
    def running(self) -> bool:
        return self.mpc.running

    @property
    def transitions(self) -> list[tuple[float, str]]:
        """(time, 'search' | 'midcourse' | 'terminal' | 'lost')"""
        return self.mpc.transitions

    def start(self) -> None:
        self.detector.start()
        if self.detector.started is not None:
            self.detector.started -= self.keep_recent_s
        self.mpc.start()

    def stop(self) -> None:
        self.mpc.stop()

    def step(self) -> None:
        now = self.clock()
        if now is None:
            return
        was = self.state
        self.detector.update(now)  # newest message -> world position
        self.mpc.step()
        if self.first_detection is None and self.mpc.tracker.time is not None:
            self.first_detection = now
        state = self.state
        if state is State.ATTACK and self.engaged_since is None:
            self.engaged_since = now
        if state is State.LOST and was is not State.LOST:
            self._lost_since = now

    @property
    def state(self) -> State:
        if self.mpc.status is Status.TRACKING:
            return State.ATTACK
        return State.SEARCH if self.mpc.tracker.time is None else State.LOST

    # --- for the operator UI (attack.attack_status) -------------------------------

    @property
    def _last_message(self) -> float:
        return self.detector._seen_at

    @property
    def commanded(self) -> Vector3 | None:
        return self.mpc.commanded

    @property
    def target_world(self) -> Vector3 | None:
        if self.mpc.estimate is not None:
            return self.mpc.estimate[0]
        return self.mpc._last_target

    @property
    def target_range(self) -> float | None:
        target, me = self.target_world, self.drone.position
        return None if target is None or me is None else math.dist(target, me)

    @property
    def closing_speed(self) -> float | None:
        """m/s at which the distance to the estimate shrinks (negative: it grows)."""
        me, mine = self.drone.position, self.drone.velocity
        if self.mpc.estimate is None or me is None or mine is None:
            return None
        target, theirs = self.mpc.estimate
        d = [t - m for t, m in zip(target, me)]
        r = math.hypot(*d)
        if r == 0.0:
            return None
        return -sum(di * (vt - vm) for di, vt, vm in zip(d, theirs, mine)) / r


def make_attack(guidance: str, drone: Drone, inbox: DetectionInbox, config: AttackConfig,
                intercept: InterceptConfig, clock: Callable[[], float | None],
                guard: Vector3 | None = None, keep_recent_s: float = 0.0
                ) -> AttackAction | MpcAttack:
    """A fresh attack: 'pursuit' (the baseline, AttackAction) or 'mpc' (MpcAttack)."""
    if guidance == 'pursuit':
        return AttackAction(drone, inbox, config, clock, guard=guard)
    if guidance == 'mpc':
        return MpcAttack(drone, inbox, config, intercept, clock, guard=guard,
                         keep_recent_s=keep_recent_s)
    raise ValueError(f'guidance must be one of {GUIDANCE}, not {guidance!r}')
