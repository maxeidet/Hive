"""One defender in the surveillance plan: hold a point on the arc, sweep the camera across
the sector the base station gave it, report what it sees, and attack when told to.

  PATROL  hold the slot, sweep the sector. A confirmed target is reported; if the base
          station allowed it (`may_start`), the drone turns on it at once (decision 6)
  ENGAGE  the attack, searching from the slot: the baseline (intercept/attack.py), or
          whatever `make_attack` gives, e.g. the MPC (intercept/mpc_attack.py)
  RETURN  fly back to the slot, then PATROL

Call step() at the setpoint rate, apply() with each order from the base station, and
collect what to send with take_sightings() and take_events(). No ROS here: it only uses
the Drone interface, like AttackAction.

Two guards against false targets: a detection counts only when it arrived within
STALE_DETECTION_S (the detector runs during take-off, and an old message in the inbox
must not start an attack), and is_friendly() tells the ROS node to drop detections that
are where another defender is (from their status messages), so a defender never attacks
a neighbour on the arc or one flying back to its slot.
"""
import math
from collections.abc import Callable

from swarm_backend.config import AttackConfig, SurveillanceConfig, Vector3
from swarm_backend.intercept.attack import AttackAction, DetectionInbox, State
from swarm_backend.intercept.base_station import Mode, Order
from swarm_backend.intercept.surveillance_msgs import EventMsg, SightingMsg, StatusMsg
from swarm_backend.px4.drone import Drone, pitch_yaw_deg, wrap_deg
from swarm_backend.swarm.arc import aim_bearing_deg, arc_points, slot_angles

RETURN_TOLERANCE_M = 1.5
STALE_DETECTION_S = 1.0  # no target_position this long and the run of detections is over
ENGAGE_TIMEOUT_S = 3.0  # an attack that has not seen its target by then gives up
FRIENDLY_GATE_M = 8.0  # a detection this close to another defender is that defender
FRIENDLY_MAX_AGE_S = 3.0  # positions older than this are not used (status comes at 2 Hz)


class PatrolAction:

    def __init__(self, drone: Drone, inbox: DetectionInbox, attack_config: AttackConfig,
                 config: SurveillanceConfig, slot: int, clock: Callable[[], float | None],
                 make_attack: Callable[[Vector3], AttackAction] | None = None):
        """`slot` counts from the left of the arc (0 .. len(config.drone_ids) - 1).
        `make_attack(guard)` gives a fresh attack for each engagement (default the
        baseline's AttackAction; intercept/mpc_attack.make_attack for the MPC)."""
        self.make_attack = make_attack or (
            lambda guard: AttackAction(drone, inbox, attack_config, clock, guard=guard))
        self.drone = drone
        self.inbox = inbox
        self.attack_config = attack_config
        self.config = config
        self.clock = clock
        n = len(config.drone_ids)
        self.slot_angle = slot_angles(n, config.arc_deg)[slot]
        self.heading = math.radians(config.threat_heading_deg)
        bx, by = config.base_position
        self.slot_point = arc_points((bx, by, drone.home[2] - config.altitude), self.heading,
                                     config.arc_radius, n, config.arc_deg)[slot]
        self.mode = Mode.PATROL
        self.sector: tuple[float, float] | None = None
        self.may_start = True
        self.attack: AttackAction | None = None
        self.running = False
        self._phi = self.slot_angle
        self._direction = 1.0
        self._last_step: float | None = None
        self._last_position = None
        self._last_detection = -math.inf
        self._hits = 0
        self._engage_started = 0.0
        self._contact_sent = False
        self._lost_sent = False
        self._sightings: list[SightingMsg] = []
        self._events: list[EventMsg] = []
        self._friends: dict[int, tuple[Vector3, float]] = {}  # id -> (position, when)

    # --- commands -----------------------------------------------------------

    def start(self) -> None:
        self.running = True
        self.mode = Mode.PATROL
        self._last_position = self.inbox.position  # whatever came before is not a sighting
        self._fly_to_slot()

    def stop(self) -> None:
        if self.attack is not None:
            self.attack.stop()
            self.attack = None
        self.running = False

    def apply(self, order: Order) -> None:
        """An order from the base station. Sending the same one again does nothing."""
        if order.drone_id != self.drone.id:
            return
        if order.mode is Mode.PATROL:
            self.sector, self.may_start = order.sector, order.may_start
        elif order.mode is Mode.ENGAGE:
            if self.mode is not Mode.ENGAGE:
                self._begin_engage()
        elif order.mode is Mode.RETURN and self.mode is not Mode.RETURN:
            self._begin_return()

    def take_sightings(self) -> list[SightingMsg]:
        out, self._sightings = self._sightings, []
        return out

    def take_events(self) -> list[EventMsg]:
        out, self._events = self._events, []
        return out

    def status(self) -> StatusMsg | None:
        if self.drone.position is None:
            return None
        bx, by = self.config.base_position
        return StatusMsg(self.drone.id, self.mode,
                         math.hypot(self.drone.position[0] - bx, self.drone.position[1] - by),
                         tuple(self.drone.position))

    def note_friend(self, drone_id: int, position: Vector3, now: float) -> None:
        """Where another defender is (from its status)."""
        if drone_id != self.drone.id:
            self._friends[drone_id] = (tuple(position), now)

    def is_friendly(self, relative: Vector3, now: float) -> bool:
        """Is a detection at `relative` (from this drone, world NED) another defender?"""
        if self.drone.position is None:
            return False
        world = tuple(p + r for p, r in zip(self.drone.position, relative))
        return any(now - when <= FRIENDLY_MAX_AGE_S and math.dist(world, pos) <= FRIENDLY_GATE_M
                   for pos, when in self._friends.values())

    # --- the loop -----------------------------------------------------------

    def step(self) -> None:
        now = self.clock()
        if not self.running or now is None or self.drone.position is None:
            return
        dt = 0.0 if self._last_step is None else max(0.0, now - self._last_step)
        self._last_step = now
        position = self.inbox.position
        fresh = (position is not None and position is not self._last_position
                 and now - position.received <= STALE_DETECTION_S)
        if fresh:
            self._last_position = position
            self._last_detection = now

        if self.mode is Mode.PATROL:
            self._patrol(now, dt, position if fresh else None)
        elif self.mode is Mode.ENGAGE:
            self._engage(now, position if fresh else None)
        elif self.mode is Mode.RETURN:
            self._sweep(dt)
            self._aim()
            if self.drone.reached(RETURN_TOLERANCE_M):
                self.mode = Mode.PATROL
                self._phi, self._hits = self.slot_angle, 0
                self._events.append(EventMsg(self.drone.id, 'returned'))

    # --- modes --------------------------------------------------------------

    def _patrol(self, now: float, dt: float, fresh) -> None:
        self._sweep(dt)
        self._aim()
        if fresh is None:
            if now - self._last_detection > STALE_DETECTION_S:
                self._hits = 0
            return
        self._hits += 1
        world = tuple(p + r for p, r in zip(self.drone.position, fresh.value))
        self._sightings.append(SightingMsg(
            self.drone.id, world, self._hits >= self.config.confirmed_hits))
        if self.may_start:
            self._begin_engage()

    def _engage(self, now: float, fresh) -> None:
        self.attack.step()
        if fresh is not None and not self._contact_sent and (
                math.hypot(*fresh.value) <= self.config.contact_m):
            self._contact_sent = True
            self._events.append(EventMsg(self.drone.id, 'contact'))
        gave_up = self.attack.state is State.LOST or (
            self.attack.state is State.SEARCH and now - self._engage_started > ENGAGE_TIMEOUT_S)
        if gave_up and not self._lost_sent:
            self._lost_sent = True
            self._events.append(EventMsg(self.drone.id, 'target_lost'))

    def _begin_engage(self) -> None:
        self.mode = Mode.ENGAGE
        self._contact_sent = self._lost_sent = False
        self._engage_started = self.clock() or 0.0
        self.attack = self.make_attack(self.slot_point)
        self.attack.start()

    def _begin_return(self) -> None:
        if self.attack is not None:
            self.attack.stop()
            self.attack = None
        self.mode = Mode.RETURN
        self._fly_to_slot()

    # --- the camera -----------------------------------------------------------

    def _fly_to_slot(self) -> None:
        self.drone.set_position(*self.slot_point, yaw=self.heading)

    def _sweep(self, dt: float) -> None:
        """Move the aim point across the sector, seen from the base; hold the slot's own
        direction while there is no sector (just back, or no order yet)."""
        if self.sector is None:
            self._phi = self.slot_angle
            return
        lo, hi = self.sector
        if hi <= lo:
            self._phi = lo
            return
        self._phi = min(hi, max(lo, self._phi))
        self._phi += self._direction * self.config.sweep_deg_s * dt
        if self._phi >= hi:
            self._phi, self._direction = hi, -1.0
        elif self._phi <= lo:
            self._phi, self._direction = lo, 1.0

    def _aim(self) -> None:
        c = self.config
        bearing = aim_bearing_deg(self.drone.position, c.base_position, self.heading, self._phi,
                                  c.arc_radius + c.aim_beyond_arc)
        nose = (math.degrees(self.heading) if self.drone.attitude is None
                else pitch_yaw_deg(self.drone.attitude)[1])
        try:
            self.drone.aim_gimbal(self.attack_config.search_pitch_deg, wrap_deg(bearing - nose))
        except NotImplementedError:
            pass
