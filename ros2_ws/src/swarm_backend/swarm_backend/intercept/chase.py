"""The chase action: fly to a moving target, then shadow it.

Call step() at the setpoint rate (e.g. from a ROS timer). Each step reads new fixes
from the target sources, updates the tracker, picks a phase and sends the drone one
position setpoint (with velocity feed-forward) and, with a gimbal, where to look.
No ROS here: it only uses the Drone interface, so it runs against FakeDrones in tests.
"""
import math
from collections.abc import Callable, Sequence

from swarm_backend.config import InterceptConfig, Vector3
from swarm_backend.intercept import guidance
from swarm_backend.intercept.guidance import Phase
from swarm_backend.intercept.target import Fix, TargetSource
from swarm_backend.intercept.tracker import KalmanTracker
from swarm_backend.px4.drone import Drone, pitch_yaw_deg

LEAD_RANGE_FACTOR = 3.0  # aim at the predicted meeting point only beyond this x standoff


class ChaseAction:

    def __init__(self, drone: Drone, sources: Sequence[TargetSource], config: InterceptConfig,
                 clock: Callable[[], float | None]):
        """`sources` in order of preference; `clock` returns seconds on the same clock
        the sources stamp their fixes with (None while it is not known yet)."""
        self.drone = drone
        self.sources = list(sources)
        self.config = config
        self.clock = clock
        # The hostile drone flies at the same fixed altitude as we do (for now).
        self.tracker = KalmanTracker(target_z=self.altitude_z)
        self.phase = Phase.LOST
        self.estimate: tuple[Vector3, Vector3] | None = None  # target (position, velocity)
        self.estimate_std = math.inf  # metres, 1 sigma, worst direction
        self.standoff = config.standoff  # what we keep now; larger while uncertain
        self.source: str | None = None  # which source the last fix came from
        self.running = False
        self._last_fix: Fix | None = None
        self._gimbal = config.use_gimbal

    def start(self) -> None:
        self.running = True
        self.phase = Phase.LOST

    def stop(self) -> None:
        """Hover where we are and stop moving the gimbal."""
        if not self.running:
            return
        self.running = False
        self.drone.hold()
        if self._gimbal:
            self.drone.release_gimbal()

    def step(self) -> None:
        if not self.running or self.drone.position is None:
            return
        now = self.clock()
        if now is None:
            return
        self._read_fixes(now)

        self.estimate = None
        if self.tracker.age(now) <= self.config.track_timeout:
            self.estimate = self.tracker.estimate(now)
            self.estimate_std = self.tracker.position_std(now)
        me = self.drone.position
        target_range = None if self.estimate is None else (
            guidance.horizontal_distance(me, self.estimate[0]))

        # Never closer than the estimate can be trusted: with the radar alone at 2 km
        # (35 m across) that is far more than the standoff.
        margin = self.config.standoff_sigmas * self.estimate_std
        self.standoff = max(self.config.standoff, margin)
        min_separation = max(self.config.min_separation, margin)

        previous = self.phase
        self.phase = guidance.next_phase(
            previous, target_range, self.standoff, self.config.capture_margin)
        if self.phase == Phase.LOST:
            if previous != Phase.LOST:
                self.drone.hold()
            return

        target, velocity = self.estimate
        if self.phase == Phase.INTERCEPT:
            # Far out, head for the spot behind where the target will be when we get
            # there. Close in, work round the target itself at standoff, so the path to
            # "behind" never cuts across it.
            if target_range > LEAD_RANGE_FACTOR * self.standoff:
                target = guidance.lead_point(target, velocity, me, self.config.max_speed)
            aim = guidance.shadow_point(target, velocity, me, self.standoff)
        else:
            aim = guidance.shadow_point(target, velocity, me, self.standoff)
        target = self.estimate[0]
        aim = guidance.keep_clear(aim, target, min_separation)
        # Move with the target and close in on top of that, braking in time.
        feed_forward = (velocity[0], velocity[1], 0.0)
        x, y, _ = guidance.approach_point(aim, me, self.config.max_speed)
        yaw = guidance.bearing(me, target)
        self.drone.set_position(x, y, self.altitude_z, yaw=yaw, velocity=feed_forward)

        if self._gimbal:
            # The drone turns toward the target, but slowly; aim from where it faces now.
            heading = yaw if self.drone.attitude is None else (
                math.radians(pitch_yaw_deg(self.drone.attitude)[1]))
            try:
                self.drone.aim_gimbal(*guidance.gimbal_angles(me, heading, target))
            except NotImplementedError:
                self._gimbal = False

    @property
    def altitude_z(self) -> float:
        return self.drone.home[2] - self.config.altitude

    def _read_fixes(self, now: float) -> None:
        for source in self.sources:
            fix = source.latest()
            if fix is None or now - fix.time > self.config.track_timeout:
                continue  # nothing fresh here; try the next source
            if fix is not self._last_fix:
                self._last_fix = fix
                self.source = fix.source
                self.tracker.update(fix)
            return
