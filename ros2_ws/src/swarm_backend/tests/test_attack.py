"""The baseline attack (intercept/attack.py), without ROS: its states on hand-made
messages, and a closed loop against the fake detector with a simulated drone."""
import math
from dataclasses import replace

import pytest

from swarm_backend.config import AttackConfig
from swarm_backend.intercept.attack import (
    AttackAction, DetectionInbox, State, limit_velocity, look_angles, pursuit_velocity,
    state_label)
from swarm_backend.intercept.detection import FakeDetector
from swarm_backend.px4.drone import Drone

STEP = 0.1  # attack and setpoint rate, 10 Hz
HIT_RADIUS = 1.5


class SimDrone(Drone):
    """A point that flies position or velocity setpoints with limited speed and
    acceleration (roughly PX4 offboard: 12 m/s across, 3 up, 1.5 down), and a gimbal that
    points at once."""

    MAX_SPEED = 12.0
    MAX_UP, MAX_DOWN = 3.0, 1.5
    MAX_ACCEL = 4.0
    GAIN = 1.0

    def __init__(self, drone_id, position, home=(0.0, 0.0, 0.0)):
        super().__init__(drone_id, home)
        self.position = position
        self.velocity = (0.0, 0.0, 0.0)
        self.yaw = 0.0
        self.mode = 'position'
        self.command = position
        self.gimbal_aim = None
        self.target = position

    def set_position(self, x, y, z, yaw=None, velocity=None):
        self.mode, self.command, self.target = 'position', (x, y, z), (x, y, z)
        if yaw is not None:
            self.yaw = yaw

    def set_velocity(self, vx, vy, vz, yaw=None):
        self.mode, self.command, self.target = 'velocity', (vx, vy, vz), None
        if yaw is not None:
            self.yaw = yaw

    def aim_gimbal(self, pitch, yaw):
        self.gimbal_aim = (pitch, yaw)

    def release_gimbal(self):
        self.gimbal_aim = None

    def camera(self):
        """The gimbal camera: where the action asked it to look (pitch, yaw from the nose)."""
        pitch, yaw = self.gimbal_aim
        p, h = math.radians(pitch), self.yaw + math.radians(yaw)
        forward = (math.cos(p) * math.cos(h), math.cos(p) * math.sin(h), -math.sin(p))
        right = (-math.sin(h), math.cos(h), 0.0)
        down = (forward[1] * right[2] - forward[2] * right[1],
                forward[2] * right[0] - forward[0] * right[2],
                forward[0] * right[1] - forward[1] * right[0])
        return {'position': self.position, 'forward': forward, 'right': right, 'down': down}

    def advance(self, dt):
        if self.mode == 'velocity':
            wanted = self.command
        else:
            wanted = [self.GAIN * (c - p) for c, p in zip(self.command, self.position)]
        wanted = _limit(wanted, self.MAX_SPEED)
        wanted[2] = max(-self.MAX_UP, min(self.MAX_DOWN, wanted[2]))  # PX4 limits each axis
        change = _limit([w - v for w, v in zip(wanted, self.velocity)], self.MAX_ACCEL * dt)
        self.velocity = tuple(v + c for v, c in zip(self.velocity, change))
        self.position = tuple(p + v * dt for p, v in zip(self.position, self.velocity))

    def arm(self): ...
    def disarm(self): ...
    def start_offboard(self): ...
    def land(self): ...


def _limit(vector, limit):
    norm = math.hypot(*vector)
    return list(vector) if norm <= limit else [v * limit / norm for v in vector]


def segment_distance(a0, a1, b0, b1):
    """Closest distance between two points moving in a straight line from *0 to *1."""
    d0 = [p - q for p, q in zip(a0, b0)]
    dv = [(p1 - p0) - (q1 - q0) for p0, p1, q0, q1 in zip(a0, a1, b0, b1)]
    vv = sum(v * v for v in dv)
    s = 0.0 if vv == 0 else max(0.0, min(1.0, -sum(d * v for d, v in zip(d0, dv)) / vv))
    return math.hypot(*[d + s * v for d, v in zip(d0, dv)])


def make_action(config=AttackConfig()):
    clock = [0.0]
    drone = SimDrone(2, (0.0, 0.0, -20.0))
    inbox = DetectionInbox()
    action = AttackAction(drone, inbox, config, lambda: clock[0])
    return action, drone, inbox, clock


def run_for(action, clock, seconds, inbox=None, message=None):
    """Step the action for `seconds`, optionally putting `message` (position) each step."""
    for _ in range(round(seconds / STEP)):
        clock[0] = round(clock[0] + STEP, 6)
        if message is not None:
            inbox.put_position(message, clock[0], clock[0])
        action.step()


# --- pieces -------------------------------------------------------------------

def test_pursuit_flies_at_full_speed_straight_at_the_target():
    assert pursuit_velocity((30.0, 40.0, 0.0), 12.0) == pytest.approx((7.2, 9.6, 0.0))
    assert pursuit_velocity((0.0, 0.0, 0.0), 12.0) is None


def test_look_angles_are_pitch_up_and_heading_from_north():
    assert look_angles((10.0, 10.0, -math.sqrt(200))) == pytest.approx((45.0, 45.0))


def test_limits_keep_the_direction_and_px4s_speeds():
    config = AttackConfig()  # 12 m/s across, 3 up, 1.5 down
    assert limit_velocity((0.0, 12.0, 0.0), 20.0, config) == pytest.approx((0.0, 12.0, 0.0))
    # Steeply up: the climb limit slows the whole vector, so it still points at the target.
    v = limit_velocity((8.0, 0.0, -6.0), 20.0, config)
    assert v == pytest.approx((4.0, 0.0, -3.0))
    # Down: 1.5 m/s at most.
    assert limit_velocity((0.0, 0.0, 6.0), 20.0, config) == pytest.approx((0.0, 0.0, 1.5))


def test_never_descends_below_the_minimum_height():
    config = replace(AttackConfig(), min_altitude=5.0)
    assert limit_velocity((6.0, 0.0, 6.0), 6.5, config)[2] == pytest.approx(0.75)  # slowing
    assert limit_velocity((6.0, 0.0, 6.0), 5.0, config) == pytest.approx((6.0, 0.0, 0.0))  # level
    assert limit_velocity((6.0, 0.0, 6.0), 3.0, config) == pytest.approx((6.0, 0.0, 0.0))
    assert limit_velocity((6.0, 0.0, -2.0), 3.0, config) == pytest.approx((6.0, 0.0, -2.0))  # up is fine


def test_coasting_keeps_the_height_limit():
    config = replace(AttackConfig(), min_altitude=5.0, guard_altitude=6.0)
    action, drone, inbox, clock = make_action(config)
    drone.position = (0.0, 0.0, -6.0)
    action.start()
    run_for(action, clock, 0.3, inbox, (10.0, 0.0, 10.0))  # target below: dive at it
    run_for(action, clock, 0.6)
    assert action.state == State.COAST
    drone.position = (0.0, 0.0, -5.0)  # reached the minimum height while coasting
    run_for(action, clock, 0.1)
    assert drone.command[2] == pytest.approx(0.0)


# --- states -------------------------------------------------------------------

def test_search_hovers_at_the_guard_position_with_the_gimbal_up():
    config = replace(AttackConfig(), guard_altitude=25.0, search_heading_deg=90.0,
                     search_pitch_deg=10.0)
    action, drone, _, clock = make_action(config)
    action.start()
    action.step()
    assert action.state == State.SEARCH
    assert drone.command == (0.0, 0.0, -25.0)
    assert drone.yaw == pytest.approx(math.pi / 2)
    assert drone.gimbal_aim == pytest.approx((10.0, 0.0))  # facing 90 deg already


def test_first_message_starts_the_attack_toward_the_target():
    action, drone, inbox, clock = make_action()
    action.start()
    run_for(action, clock, 0.5)
    inbox.put_position((0.0, 30.0, -40.0), 0.6, 0.6)
    run_for(action, clock, 0.1)
    assert action.state == State.ATTACK
    assert drone.mode == 'velocity'
    assert action.velocity == pytest.approx((0.0, 7.2, -9.6))  # 12 m/s toward it
    # Sent within PX4's 3 m/s climb limit, still pointing at the target.
    assert drone.command == pytest.approx((0.0, 2.25, -3.0))
    assert drone.yaw == pytest.approx(math.pi / 2)  # facing it
    assert action.first_detection == pytest.approx(0.6)


def test_messages_from_before_the_start_are_ignored():
    action, drone, inbox, clock = make_action()
    clock[0] = 5.0
    inbox.put_position((10.0, 0.0, 0.0), 4.0, 4.0)
    action.start()
    run_for(action, clock, 1.0)
    assert action.state == State.SEARCH


def test_attack_coasts_then_is_lost_then_searches_again():
    action, drone, inbox, clock = make_action()
    action.start()
    run_for(action, clock, 1.0, inbox, (20.0, 0.0, 0.0))  # messages until t = 1.0
    assert action.state == State.ATTACK
    run_for(action, clock, 0.5)  # 0.5 s without: still attacking
    assert action.state == State.ATTACK
    run_for(action, clock, 0.1)
    assert action.state == State.COAST
    assert drone.mode == 'velocity' and drone.command == pytest.approx((12.0, 0.0, 0.0))
    run_for(action, clock, 1.5)  # 2.1 s without a message
    assert action.state == State.LOST
    assert drone.mode == 'position'  # hovering
    assert action.gimbal_pitch == pytest.approx(0.0)  # toward where the target was
    assert action.gimbal_heading == pytest.approx(0.0)
    run_for(action, clock, 3.0)
    assert action.state == State.LOST
    run_for(action, clock, 0.1)
    assert action.state == State.SEARCH
    assert drone.command == pytest.approx(action.guard)
    states = [s for _, s in action.transitions]
    assert states == [State.SEARCH, State.ATTACK, State.COAST, State.LOST, State.SEARCH]


def test_coast_returns_to_attack_when_the_target_is_seen_again():
    action, drone, inbox, clock = make_action()
    action.start()
    run_for(action, clock, 0.5, inbox, (20.0, 0.0, 0.0))
    run_for(action, clock, 0.8)
    assert action.state == State.COAST
    run_for(action, clock, 0.1, inbox, (0.0, 20.0, 0.0))
    assert action.state == State.ATTACK
    assert drone.command == pytest.approx((0.0, 12.0, 0.0))


def test_gimbal_turns_by_the_gain_times_the_offset_only_while_attacking():
    action, drone, inbox, clock = make_action(replace(AttackConfig(), search_pitch_deg=10.0))
    action.start()
    inbox.put_offset((math.radians(4.0), math.radians(-2.0)), 0.1, 0.1)
    run_for(action, clock, 0.1)
    assert (action.gimbal_heading, action.gimbal_pitch) == pytest.approx((0.0, 10.0))  # SEARCH
    inbox.put_position((20.0, 0.0, 0.0), 0.2, 0.2)
    inbox.put_offset((math.radians(4.0), math.radians(-2.0)), 0.2, 0.2)
    run_for(action, clock, 0.1)
    assert action.gimbal_heading == pytest.approx(2.0)
    assert action.gimbal_pitch == pytest.approx(9.0)
    run_for(action, clock, 0.2)  # same message again: no further turn
    assert action.gimbal_heading == pytest.approx(2.0)


def test_gimbal_yaw_is_sent_relative_to_the_nose():
    action, drone, inbox, clock = make_action()
    action.start()
    inbox.put_position((0.0, 20.0, 0.0), 0.1, 0.1)  # due east: the drone turns to 90 deg
    run_for(action, clock, 0.1)
    action.gimbal_heading = 80.0
    run_for(action, clock, 0.1)
    assert drone.gimbal_aim[1] == pytest.approx(-10.0)


def test_hit_stops_the_attack_and_counts_the_time_in_each_state():
    action, drone, inbox, clock = make_action()
    action.start()
    run_for(action, clock, 1.0)
    run_for(action, clock, 2.0, inbox, (20.0, 0.0, 0.0))
    action.hit()
    assert action.state == State.HIT and not action.running
    assert drone.mode == 'position'  # holding
    # The step that reads the first message still counts as searching.
    assert action.state_time[State.SEARCH] == pytest.approx(1.1)
    assert action.state_time[State.ATTACK] == pytest.approx(1.9)



def test_state_label_is_idle_unless_running_but_keeps_a_hit():
    action, drone, inbox, clock = make_action()
    assert state_label(action) == 'idle'  # before start: taking off
    action.start()
    assert state_label(action) == 'search'
    run_for(action, clock, 1.0, inbox, (20.0, 0.0, 0.0))
    assert state_label(action) == 'attack'
    action.hit()
    assert state_label(action) == 'hit'
    action, *_ = make_action()
    action.start()
    action.stop()
    assert state_label(action) == 'idle'  # stopped without a hit: landing

# --- closed loop --------------------------------------------------------------

def fly(config, hostile_start, hostile_velocity, seconds=60.0, dt=0.02):
    """The attack against a hostile flying a straight line, seen through the fake
    detector. Returns (closest distance, time of the hit or None, action)."""
    clock = [0.0]
    drone = SimDrone(2, (0.0, 0.0, -20.0))
    inbox = DetectionInbox()
    action = AttackAction(drone, inbox, config, lambda: clock[0])
    detector = FakeDetector(config)
    hostile = hostile_start
    action.start()
    action.step()
    closest, hit_time = math.inf, None
    next_step = STEP
    for _ in range(round(seconds / dt)):
        t = clock[0] = round(clock[0] + dt, 6)
        before = (drone.position, hostile)
        drone.advance(dt)
        hostile = tuple(p + v * dt for p, v in zip(hostile, hostile_velocity))
        if detector.wants_image(t):
            detector.observe(t, drone.camera(), hostile)
        for stamp, found in detector.due(t):
            inbox.put_position(found.position, stamp, t)
            inbox.put_offset(found.offset, stamp, t)
        if t >= next_step - 1e-9:
            action.step()
            next_step += STEP
        closest = min(closest, segment_distance(before[0], drone.position, before[1], hostile))
        if closest < HIT_RADIUS:
            hit_time = t
            action.hit()
            break
    return closest, hit_time, action


def test_perfect_detector_hits_a_slow_target_coming_head_on():
    # The hostile flies at 4 m/s straight toward the defender, 10 m above it.
    closest, hit_time, action = fly(AttackConfig(), (80.0, 0.0, -30.0), (-4.0, 0.0, 0.0))
    assert hit_time is not None, f'missed, closest {closest:.1f} m'
    states = [s for _, s in action.transitions]
    assert states[:2] == [State.SEARCH, State.ATTACK] and states[-1] == State.HIT


def test_head_on_from_far_above_falls_short_on_the_climb_limit():
    # 15 m above, PX4's 3 m/s climb is too slow to get up to it in time.
    closest, hit_time, _ = fly(AttackConfig(), (80.0, 0.0, -35.0), (-4.0, 0.0, 0.0))
    assert hit_time is None and closest < 4.0


def test_crossing_target_is_reached_but_the_last_metres_are_hard():
    # 4 m/s across, 35 m in front of the defender and 10 m above it. With the work plan's
    # values (4 Hz, gimbal gain 0.5, 12 m/s) this simulated drone's first pass misses by
    # about 3 m: in the last second the target slides out of view and the drone coasts
    # past (it may come round and hit later). With a gain of 1.0 the camera keeps up.
    closest, hit_time, action = fly(AttackConfig(), (35.0, -60.0, -30.0), (0.0, 4.0, 0.0),
                                    seconds=20.0)
    assert closest < 5.0
    assert State.ATTACK in [s for _, s in action.transitions]
    faster_gimbal = replace(AttackConfig(), gimbal_gain=1.0)
    closest, hit_time, _ = fly(faster_gimbal, (35.0, -60.0, -30.0), (0.0, 4.0, 0.0))
    assert hit_time is not None, f'missed, closest {closest:.1f} m'


def test_pure_pursuit_cannot_catch_a_target_flying_away_faster():
    # 17 m/s straight away from the defender: the baseline's known weakness.
    config = AttackConfig()
    closest, hit_time, _ = fly(config, (30.0, 0.0, -30.0), (17.0, 0.0, 0.0), seconds=20.0)
    assert hit_time is None and closest > 20.0
