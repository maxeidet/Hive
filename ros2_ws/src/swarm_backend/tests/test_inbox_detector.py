"""InboxDetector (intercept/inbox_detector.py): target_position messages as world
positions, and the MPC flying on them, through the attack's fake detector, on the same
scenarios as the baseline in test_attack.py. No ROS."""
import dataclasses
import math

from swarm_backend.config import AttackConfig, InterceptConfig
from swarm_backend.intercept.attack import DetectionInbox, Message, limit_velocity
from swarm_backend.intercept.detection import FakeDetector
from swarm_backend.intercept.inbox_detector import InboxDetector, to_fix
from swarm_backend.intercept.mpc_action import MpcIntercept

from test_attack import HIT_RADIUS, STEP, SimDrone, fly, segment_distance

# `fakedetect --realistic`: like the real detector.
REALISTIC = dataclasses.replace(AttackConfig(), fake_delay=0.25, fake_angle_noise_deg=1.0,
                                fake_range_noise=0.2, fake_blind_range=5.0)


# --- relative -> world ---------------------------------------------------------

def test_world_position_is_ours_when_the_image_was_taken_plus_the_relative_one():
    # Image at 10.0 s, message at 10.25 s, read at 10.3 s. We fly 12 m/s north, so
    # 0.3 s ago we were 3.6 m further south than now.
    message = Message(value=(150.0, 80.0, -20.0), stamp=10.0, received=10.25)
    fix = to_fix(message, position=(1000.0, 0.0, -50.0), velocity=(12.0, 0.0, 0.0),
                 now=10.3, range_std_frac=0.2)
    assert fix.time == 10.0
    assert all(abs(a - b) < 1e-9 for a, b in zip(fix.position, (1146.4, 80.0, -70.0)))
    assert abs(fix.std - 0.2 * math.hypot(150.0, 80.0, -20.0)) < 1e-9


def test_a_stamp_on_another_clock_is_not_used_for_the_correction():
    message = Message(value=(10.0, 0.0, 0.0), stamp=5000.0, received=10.0)  # "from the future"
    fix = to_fix(message, (0.0, 0.0, -50.0), (12.0, 0.0, 0.0), now=10.0, range_std_frac=0.2)
    assert fix.time == 10.0 and fix.position == (10.0, 0.0, -50.0)


def test_only_new_messages_after_start_count():
    clock = [0.0]
    drone = SimDrone(2, (0.0, 0.0, -20.0))
    inbox = DetectionInbox()
    detector = InboxDetector(drone, inbox, lambda: clock[0])
    inbox.put_position((5.0, 0.0, 0.0), stamp=0.0, received=0.0)  # before start: ignored
    clock[0] = 1.0
    detector.start()
    detector.update(1.0)
    assert detector.latest() is None and not detector.detected
    inbox.put_position((5.0, 0.0, 0.0), stamp=1.0, received=1.0)
    detector.update(1.0)
    first = detector.latest()
    assert first is not None and detector.detected and detector.in_view
    detector.update(1.1)
    assert detector.latest() is first  # the same message is not used twice
    clock[0] = 2.0
    assert not detector.detected  # nothing new for more than SEEN_FOR


# --- closed loop, against pure pursuit ---------------------------------------------

def fly_mpc(config, hostile_start, hostile_velocity, seconds=40.0, dt=0.02):
    """test_attack.fly with the MPC: the same drone, fake detector and inbox, the
    MpcIntercept the attack node makes with --guidance mpc. (closest, hit time or None)."""
    clock = [0.0]
    drone = SimDrone(2, (0.0, 0.0, -20.0))
    inbox = DetectionInbox()
    detector = InboxDetector(drone, inbox, lambda: clock[0])
    intercept = dataclasses.replace(InterceptConfig(), interceptor_max_speed=config.max_speed)
    action = MpcIntercept(drone, detector, intercept, lambda: clock[0],
                          search=(config.search_pitch_deg, config.search_heading_deg),
                          limit=lambda v: limit_velocity(v, drone.altitude, config))
    fake = FakeDetector(config)
    hostile = hostile_start
    detector.start()
    action.start()
    action.step()
    closest, next_step = math.inf, STEP
    for _ in range(round(seconds / dt)):
        t = clock[0] = round(clock[0] + dt, 6)
        before = (drone.position, hostile)
        drone.advance(dt)
        hostile = tuple(p + v * dt for p, v in zip(hostile, hostile_velocity))
        if fake.wants_image(t):
            fake.observe(t, drone.camera(), hostile)
        for stamp, found in fake.due(t):
            inbox.put_position(found.position, stamp, t)
            inbox.put_offset(found.offset, stamp, t)
        if t >= next_step - 1e-9:
            detector.update(t)
            action.step()
            next_step += STEP
        closest = min(closest, segment_distance(before[0], drone.position, before[1], hostile))
        if closest < HIT_RADIUS:
            return closest, t
    return closest, None


def test_mpc_hits_a_slow_head_on_target_with_realistic_detections_where_pursuit_misses():
    start, velocity = (80.0, 0.0, -30.0), (-4.0, 0.0, 0.0)
    assert fly(REALISTIC, start, velocity, seconds=40.0)[1] is None
    closest, hit = fly_mpc(REALISTIC, start, velocity)
    assert hit is not None, f'missed by {closest:.1f} m'


def test_mpc_climbs_to_a_target_15_m_up_that_pursuit_falls_short_of():
    """test_attack: pure pursuit falls short on the climb limit; the MPC starts climbing
    toward where the target will be."""
    start, velocity = (80.0, 0.0, -35.0), (-4.0, 0.0, 0.0)
    assert fly(AttackConfig(), start, velocity, seconds=40.0)[1] is None
    assert fly_mpc(AttackConfig(), start, velocity)[1] is not None


def test_mpc_meets_a_crossing_target_much_earlier_than_pursuit():
    for config in (AttackConfig(), REALISTIC):
        for start, velocity in (((35.0, -60.0, -30.0), (0.0, 4.0, 0.0)),
                                ((40.0, -80.0, -30.0), (0.0, 8.0, 0.0))):
            pursuit = fly(config, start, velocity, seconds=40.0)[1]
            mpc = fly_mpc(config, start, velocity)[1]
            assert pursuit is not None and mpc is not None and mpc < pursuit - 2.0, (
                config.fake_delay, velocity, pursuit, mpc)
