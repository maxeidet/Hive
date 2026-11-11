"""The camera detector's topics as a Detector, so MpcIntercept (or any InterceptAction)
flies on the same messages as the baseline attack.

The detector (or the fake one) publishes target_position: the target relative to the
drone, world NED, metres (= distance x direction, worked out from where the target is
in the image, the gimbal's and the drone's orientation, and the target's width in
pixels). The attack node puts each message in a DetectionInbox (intercept/attack.py).
This turns the newest one into a world position:

    target (world) = our position when the image was taken + target_position

The image is older than the message (about 0.25 s with the real detector), and we fly
up to 12 m/s, so "our position" is taken back to when the image was taken:
position - velocity x age. Using where we are now would put the target 3 m off.

The distance comes from the target's width in pixels and is the weak part (about 20 %
off), the direction is good to a degree or two. A PositionFix has one error for all
axes, so `range_std_frac` x distance is used for all of them: cautious across the line
of sight, right along it. No ROS: the node fills the inbox.
"""
import math
from collections.abc import Callable

from swarm_backend.config import Vector3
from swarm_backend.intercept.attack import DetectionInbox, Message
from swarm_backend.intercept.target import PositionFix
from swarm_backend.intercept.target_detector import Detector
from swarm_backend.px4.drone import Drone

MAX_IMAGE_AGE = 1.0  # seconds; an older (or future) image stamp is a clock mismatch: ignore it
MIN_STD = 0.5  # metres, so the filter never trusts a fix completely
SEEN_FOR = 0.5  # seconds a message counts as "the target is seen" (the baseline's coast_after)


class InboxDetector(Detector):

    def __init__(self, drone: Drone, inbox: DetectionInbox, clock: Callable[[], float | None],
                 range_std_frac: float = 0.2):
        """`clock` is the one the inbox's `received` times use."""
        self.drone = drone
        self.inbox = inbox
        self.clock = clock
        self.range_std_frac = range_std_frac
        self.started: float | None = None  # messages from before this are ignored
        self._message: Message | None = None  # the last one turned into a fix
        self._fix: PositionFix | None = None
        self._seen_at = -math.inf

    def start(self) -> None:
        """Ignore what arrived before now, e.g. before take-off."""
        self.started = self.clock()

    def update(self, now: float) -> None:
        message = self.inbox.position
        if (message is None or message is self._message or self.drone.position is None
                or (self.started is not None and message.received < self.started)):
            return
        self._message = message
        self._seen_at = message.received
        self._fix = to_fix(message, self.drone.position, self.drone.velocity or (0.0, 0.0, 0.0),
                           now, self.range_std_frac)

    def latest(self) -> PositionFix | None:
        return self._fix

    @property
    def in_view(self) -> bool:
        """The detector only reports confirmed targets, so in view = recently reported."""
        return self.detected

    @property
    def detected(self) -> bool:
        now = self.clock()
        return now is not None and now - self._seen_at <= SEEN_FOR


def to_fix(message: Message, position: Vector3, velocity: Vector3, now: float,
           range_std_frac: float) -> PositionFix:
    """A target_position message as a world position, taken at the image's time.

    `position` and `velocity` are ours at `now`; the message's `received` is on the same
    clock as `now`, its `stamp` on the camera's (simulation time). The age of the image is
    received - stamp, if that is plausible, plus how long the message waited for us."""
    age = message.received - message.stamp
    if not 0.0 <= age <= MAX_IMAGE_AGE:
        age = 0.0  # clocks do not agree: better no correction than a wrong one
    taken = message.received - age
    back = now - taken
    me_then = tuple(p - v * back for p, v in zip(position, velocity))
    relative = message.value
    world = tuple(m + r for m, r in zip(me_then, relative))
    std = max(MIN_STD, range_std_frac * math.hypot(*relative))
    return PositionFix(time=taken, source='camera', position=world, std=std)
