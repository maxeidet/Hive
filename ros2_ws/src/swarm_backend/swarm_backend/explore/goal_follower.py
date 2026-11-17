"""Flies one drone to DAEP's goals: the part of daep_bridge.py that needs no ROS.

DAEP works in the Gazebo world frame, ENU (x east, y north, z up, yaw counter-clockwise
from east). The drone (swarm_backend's Drone) works in NED (x north, y east, z down, yaw
clockwise from north). Both have their origin at the Gazebo world origin.

Tests: python3 -m pytest tests/test_goal_follower.py
"""
import math

Point = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]

NEW, ACTIVE, DONE, IGNORED = 'new', 'active', 'done', 'ignored'


def enu_to_ned(p: Point) -> Point:
    east, north, up = p
    return (north, east, -up)


def ned_to_enu(p: Point) -> Point:
    north, east, down = p
    return (east, north, -down)


def wrap(angle: float) -> float:
    """Angle in (-pi, pi]."""
    return -((-angle + math.pi) % (2.0 * math.pi) - math.pi)


def swap_yaw(yaw: float) -> float:
    """ENU yaw (from east, counter-clockwise) <-> NED yaw (from north, clockwise)."""
    return wrap(math.pi / 2.0 - yaw)


def roll_pitch_yaw(q: Quaternion) -> tuple[float, float, float]:
    """ZYX Euler angles (radians) of a quaternion w, x, y, z."""
    w, x, y, z = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def enu_quaternion(q_ned: Quaternion) -> Quaternion:
    """PX4's attitude (w, x, y, z; body forward-right-down in NED) as a ROS quaternion
    (x, y, z, w; body forward-left-up in ENU)."""
    roll, pitch, yaw = roll_pitch_yaw(q_ned)
    roll, pitch, yaw = roll, -pitch, swap_yaw(yaw)
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def carrot(start: Point, goal: Point, position: Point, lead: float) -> Point:
    """The point on the line start -> goal that is `lead` metres ahead of where `position`
    is along it, at most the goal. Flying to this point instead of the goal keeps the drone
    on the line DAEP checked for collisions, and slow: PX4 flies faster the further away
    its setpoint is."""
    length = math.dist(start, goal)
    if length <= lead:
        return goal
    along = sum((p - s) * (g - s) for p, s, g in zip(position, start, goal)) / length
    t = min(max(along, 0.0) + lead, length) / length
    return tuple(s + (g - s) * t for s, g in zip(start, goal))


class GoalFollower:
    """Flies `drone` to one DAEP goal at a time.

    on_goal() takes every Goal message (DAEP repeats the same one at 10 Hz until it is
    reached); step() is called at a fixed rate, commands the drone and returns True once,
    when the goal is reached: the position within `position_tolerance` metres and the yaw
    within `yaw_tolerance` radians.

    A goal with initial_motion is one turn on the spot, in steps of `turn_step` radians.
    Each step waits for the drone to get there, so the turn takes as long as it needs when
    the simulation runs slower than real time.
    """

    def __init__(self, drone, position_tolerance: float = 0.3,
                 yaw_tolerance: float = math.radians(10.0), lead: float = 1.0,
                 turn_step: float = math.radians(30.0)):
        self.drone = drone
        self.position_tolerance = position_tolerance
        self.yaw_tolerance = yaw_tolerance
        self.lead = lead
        self.turn_step = turn_step
        self.uuid: str | None = None  # the goal being flown
        self.done_uuid: str | None = None  # the last goal reached
        self._start: Point | None = None  # NED
        self._goal: Point | None = None  # NED
        self._yaw = 0.0  # NED, the yaw setpoint
        self._yaw_steps_left = 0

    def on_goal(self, uuid: str, enu: Point, enu_yaw: float, initial_motion: bool = False) -> str:
        """NEW when this starts a goal, ACTIVE when it is the one being flown, DONE when it
        was already reached, IGNORED when the drone has no position yet."""
        if uuid == self.done_uuid:
            return DONE
        if uuid == self.uuid:
            return ACTIVE
        d = self.drone
        if d.position is None or d.attitude is None:
            return IGNORED
        self.uuid = uuid
        self._start = d.position
        if initial_motion:
            # DAEP's own position for this goal is meant for a TurtleBot leaving its dock.
            self._goal = d.position
            self._yaw_steps_left = round(2.0 * math.pi / self.turn_step)
            self._yaw = wrap(roll_pitch_yaw(d.attitude)[2] + self.turn_step)
        else:
            self._goal = enu_to_ned(enu)
            self._yaw_steps_left = 1
            self._yaw = swap_yaw(enu_yaw)
        return NEW

    def step(self) -> bool:
        if self.uuid is None:
            return False
        d = self.drone
        if (math.dist(d.position, self._goal) < self.position_tolerance
                and abs(wrap(roll_pitch_yaw(d.attitude)[2] - self._yaw)) < self.yaw_tolerance):
            self._yaw_steps_left -= 1
            if self._yaw_steps_left == 0:
                self.done_uuid, self.uuid = self.uuid, None
                return True
            self._yaw = wrap(self._yaw + self.turn_step)
        d.set_position(*carrot(self._start, self._goal, d.position, self.lead), yaw=self._yaw)
        return False
