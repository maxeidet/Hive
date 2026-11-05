"""The baseline attack: fly into the hostile drone with pure pursuit, on what the camera sees.

The drone reads the two detection topics (intercept/detection.py) and nothing else about
the target. States, as in the work plan:

  SEARCH  hover at the guard position, gimbal fixed toward where the hostile comes from,
          tilted up a little                          first message -> ATTACK
  ATTACK  velocity = max_speed x direction of target_position; the gimbal turns by
          gimbal_gain x target_offset on every message  no message for coast_after -> COAST
  COAST   keep the last velocity, gimbal held still     message -> ATTACK;
                                                        none for lost_after more -> LOST
  LOST    hover, gimbal toward the last known direction message -> ATTACK;
                                                        after lost_hold -> SEARCH
  HIT     set by whoever scores the run (hit()); the action stops

Every velocity command goes through limit_velocity: never faster than PX4 flies, the
direction kept, and never down below min_altitude, also while coasting.

Coasting covers the last metres: the real detector loses the target closer than about
5 m, where it is too big for it. Call step() at the setpoint rate. No ROS here: it only
uses the Drone interface and a DetectionInbox, so it runs against simulated drones in tests.
"""
import math
import threading
from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum

from swarm_backend.config import AttackConfig, Vector3
from swarm_backend.px4.drone import Drone, pitch_yaw_deg, wrap_deg

MAX_GIMBAL_PITCH = 90.0  # degrees either way from the horizon
MIN_DIRECTION_M = 1e-3  # a target_position shorter than this has no direction
SLOW_ZONE_M = 3.0  # descent slows down over this many metres above min_altitude


class State(Enum):
    SEARCH = 'search'
    ATTACK = 'attack'
    COAST = 'coast'
    LOST = 'lost'
    HIT = 'hit'


@dataclass(frozen=True)
class Message:
    value: tuple  # target_position (x, y, z) or target_offset (right, up)
    stamp: float  # when the image was taken (the detector's clock)
    received: float  # when it arrived, on the attack's own clock


class DetectionInbox:
    """The newest message of each detection topic. ROS callbacks fill it (put_*), the
    attack reads it; the two may run on different threads."""

    def __init__(self):
        self._lock = threading.Lock()
        self._position: Message | None = None
        self._offset: Message | None = None
        self.position_count = 0  # target_position messages received

    def put_position(self, position: Vector3, stamp: float, received: float) -> None:
        with self._lock:
            self._position = Message(tuple(position), stamp, received)
            self.position_count += 1

    def put_offset(self, offset: tuple[float, float], stamp: float, received: float) -> None:
        with self._lock:
            self._offset = Message(tuple(offset), stamp, received)

    @property
    def position(self) -> Message | None:
        with self._lock:
            return self._position

    @property
    def offset(self) -> Message | None:
        with self._lock:
            return self._offset


def pursuit_velocity(target: Vector3, speed: float) -> Vector3 | None:
    """Pure pursuit: `speed` straight at the target (relative position). None if it has
    no direction."""
    distance = math.hypot(*target)
    if distance < MIN_DIRECTION_M:
        return None
    return tuple(speed * c / distance for c in target)


def limit_velocity(velocity: Vector3, altitude: float | None, config: AttackConfig) -> Vector3:
    """`velocity` (NED) scaled down, direction kept, until it is within PX4's limits:
    max_speed horizontally, max_climb_speed up, max_descent_speed down. Below
    min_altitude + SLOW_ZONE_M the descent is cut back, to nothing at min_altitude."""
    vx, vy, vz = velocity
    down_limit = config.max_descent_speed
    if altitude is not None:
        room = altitude - config.min_altitude
        down_limit = max(0.0, min(down_limit, down_limit * room / SLOW_ZONE_M))
    factor = 1.0
    horizontal = math.hypot(vx, vy)
    if horizontal > config.max_speed:
        factor = config.max_speed / horizontal
    if vz < 0 and -vz > config.max_climb_speed:
        factor = min(factor, config.max_climb_speed / -vz)
    if vz > 0 and vz > down_limit:
        if down_limit == 0.0:
            return (vx * factor, vy * factor, 0.0)  # level off rather than stop
        factor = min(factor, down_limit / vz)
    return (vx * factor, vy * factor, vz * factor)


def look_angles(target: Vector3) -> tuple[float, float]:
    """(pitch above the horizon, heading from north), degrees, toward a relative position."""
    horizontal = math.hypot(target[0], target[1])
    return (math.degrees(math.atan2(-target[2], horizontal)),
            math.degrees(math.atan2(target[1], target[0])))


class AttackAction:

    def __init__(self, drone: Drone, inbox: DetectionInbox, config: AttackConfig,
                 clock: Callable[[], float | None], guard: Vector3 | None = None):
        """`clock` gives seconds on the clock the inbox's `received` times use (None while
        it is not known yet). `guard` is where the drone hovers while searching, a world
        position; the default is above its start point."""
        self._guard = guard
        self.drone = drone
        self.inbox = inbox
        self.config = config
        self.clock = clock
        self.state = State.SEARCH
        self.running = False
        self.velocity: Vector3 | None = None  # last pursuit velocity (before the limits)
        self.commanded: Vector3 | None = None  # what was last sent, after the limits
        self.first_detection: float | None = None  # clock time of the first message
        self.transitions: list[tuple[float, State]] = []
        self.state_time: dict[State, float] = {s: 0.0 for s in State}  # seconds in each
        # Where the gimbal should look, in the world: degrees above the horizon and from
        # north. Sent relative to the nose on every step, so turning the drone keeps it.
        self.gimbal_pitch = config.search_pitch_deg
        self.gimbal_heading = config.search_heading_deg
        self._yaw = math.radians(config.search_heading_deg)  # last commanded heading
        self._last_position: Message | None = None  # last target_position acted on
        self._last_offset: Message | None = None
        self._last_message = -math.inf  # clock time the last target_position arrived
        self._lost_since = -math.inf
        self._started: float | None = None
        self._last_step: float | None = None
        self._gimbal = True

    @property
    def guard(self) -> Vector3:
        """Where the drone hovers while searching: above its start point."""
        if self._guard is not None:
            return self._guard
        x, y, z = self.drone.home
        return (x, y, z - self.config.guard_altitude)

    def start(self) -> None:
        self.running = True
        self._started = self.clock()
        self._last_step = self._started
        self._enter(State.SEARCH, self._started)

    def stop(self) -> None:
        """Hover where we are and stop moving the gimbal."""
        if not self.running:
            return
        self.running = False
        self.drone.hold()
        if self._gimbal:
            self.drone.release_gimbal()

    def hit(self) -> None:
        """The run is scored as a hit: stop attacking."""
        now = self.clock()
        if self.running and now is not None:
            self._count_time(now)
            self._enter(State.HIT, now)
        self.stop()

    def step(self) -> None:
        if not self.running or self.drone.position is None:
            return
        now = self.clock()
        if now is None:
            return
        if self._started is None:
            self._started = self._last_step = now
        self._count_time(now)

        position = self.inbox.position
        fresh = (position is not None and position is not self._last_position
                 and position.received >= self._started)
        if fresh:
            self._last_position = position
            self._last_message = position.received
            if self.first_detection is None:
                self.first_detection = now
            if self.state != State.ATTACK:
                self._enter(State.ATTACK, now)
            self._pursue(position.value)
        elif self.state == State.ATTACK and now - self._last_message > self.config.coast_after:
            self._enter(State.COAST, now)
        elif self.state == State.COAST and (
                now - self._last_message > self.config.coast_after + self.config.lost_after):
            self._enter(State.LOST, now)
        elif self.state == State.LOST and now - self._lost_since > self.config.lost_hold:
            self._enter(State.SEARCH, now)
        if self.state in (State.ATTACK, State.COAST) and self.velocity is not None:
            self._send_velocity()  # every tick, so the height limit holds while coasting

        offset = self.inbox.offset
        if offset is not None and offset is not self._last_offset:
            self._last_offset = offset
            if self.state == State.ATTACK and offset.received >= self._started:
                self._turn_gimbal(offset.value)
        self._aim_gimbal()

    # --- states -------------------------------------------------------------

    def _enter(self, state: State, now: float) -> None:
        self.state = state
        self.transitions.append((now, state))
        if state == State.SEARCH:
            self.velocity = None
            self._yaw = math.radians(self.config.search_heading_deg)
            self.drone.set_position(*self.guard, yaw=self._yaw)
            self.gimbal_pitch = self.config.search_pitch_deg
            self.gimbal_heading = self.config.search_heading_deg
        elif state == State.LOST:
            self._lost_since = now
            self.velocity = None
            self.drone.hold()
            if self._last_position is not None:
                self.gimbal_pitch, self.gimbal_heading = look_angles(self._last_position.value)
        # ATTACK: _pursue sends the velocity. COAST: PX4 keeps flying the last one.

    def _pursue(self, target: Vector3) -> None:
        velocity = pursuit_velocity(target, self.config.max_speed)
        if velocity is None:
            return  # keep the last command
        if math.hypot(target[0], target[1]) > MIN_DIRECTION_M:
            self._yaw = math.atan2(target[1], target[0])  # face the target, gimbal yaw small
        self.velocity = velocity

    def _send_velocity(self) -> None:
        self.commanded = limit_velocity(self.velocity, self.drone.altitude, self.config)
        self.drone.set_velocity(*self.commanded, yaw=self._yaw)

    # --- gimbal -------------------------------------------------------------

    def _turn_gimbal(self, offset: tuple[float, float]) -> None:
        right, up = offset
        gain = self.config.gimbal_gain
        self.gimbal_heading = wrap_deg(self.gimbal_heading + gain * math.degrees(right))
        self.gimbal_pitch = max(-MAX_GIMBAL_PITCH, min(MAX_GIMBAL_PITCH,
                                self.gimbal_pitch + gain * math.degrees(up)))

    def _aim_gimbal(self) -> None:
        if not self._gimbal:
            return
        heading = (math.degrees(self._yaw) if self.drone.attitude is None
                   else pitch_yaw_deg(self.drone.attitude)[1])
        try:
            self.drone.aim_gimbal(self.gimbal_pitch, wrap_deg(self.gimbal_heading - heading))
        except NotImplementedError:
            self._gimbal = False

    def _count_time(self, now: float) -> None:
        if self._last_step is not None and now > self._last_step:
            self.state_time[self.state] += now - self._last_step
        self._last_step = now
