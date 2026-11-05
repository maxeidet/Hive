import math
import random
import statistics

import pytest

from swarm_backend.config import InterceptConfig, parse_config
from swarm_backend.intercept import guidance
from swarm_backend.intercept.chase import ChaseAction
from swarm_backend.intercept.guidance import Phase
from swarm_backend.intercept.mock import HostileFlight, radar_fix
from swarm_backend.intercept.paths import PATTERNS, make_path
from swarm_backend.intercept.target import PositionFix, TargetSource
from swarm_backend.intercept.tracker import KalmanTracker

from swarm_backend.fast_sim import SimDrone

ALT = -50.0
DT = 0.1


# --- paths ------------------------------------------------------------------

@pytest.mark.parametrize('pattern', PATTERNS)
def test_paths_start_at_anchor_fly_at_speed_and_keep_altitude(pattern):
    anchor = (10.0, -20.0, ALT)
    path = make_path(pattern, anchor, heading=math.radians(30), speed=4.0, size=80.0, seed=3)
    position, velocity = path.at(0.0)
    assert math.dist(position, anchor) < 1e-9
    assert math.degrees(math.atan2(velocity[1], velocity[0])) == pytest.approx(30.0, abs=1e-6)
    previous = position
    for i in range(1, 2000):
        position, velocity = path.at(i * DT)
        assert position[2] == ALT
        assert math.hypot(*velocity) == pytest.approx(4.0)
        assert math.dist(position, previous) <= 4.0 * DT + 1e-9  # continuous
        previous = position


def test_random_path_depends_only_on_seed():
    a = make_path('random', (0, 0, ALT), 0.0, 4.0, 80.0, seed=1).points
    assert a == make_path('random', (0, 0, ALT), 0.0, 4.0, 80.0, seed=1).points
    assert a != make_path('random', (0, 0, ALT), 0.0, 4.0, 80.0, seed=2).points


def test_unknown_pattern():
    with pytest.raises(ValueError):
        make_path('spiral', (0, 0, ALT), 0.0, 4.0, 80.0)


# --- tracker ----------------------------------------------------------------

def radar_at(distance):
    """The default config with the radar `distance` metres south of the origin."""
    return InterceptConfig(radar_position=(-distance, 0.0, 0.0))


def test_radar_errors_have_the_configured_sigmas():
    config, rng = radar_at(2000.0), random.Random(0)
    truth, velocity = (0.0, 0.0, ALT), (3.0, 4.0, 0.0)  # radial (north) part is 3 m/s
    fixes = [radar_fix(truth, velocity, config, rng, 0.0) for _ in range(4000)]
    true_range = math.dist(truth, config.radar_position)
    assert statistics.stdev(f.range for f in fixes) == pytest.approx(3.5, rel=0.1)
    assert statistics.mean(f.range for f in fixes) == pytest.approx(true_range, abs=0.3)
    assert statistics.stdev(math.degrees(f.azimuth) for f in fixes) == pytest.approx(1.0, rel=0.1)
    assert statistics.mean(f.radial_velocity for f in fixes) == pytest.approx(3.0, abs=0.05)
    # 1 degree at 2 km: about 35 m across the line of sight (east), little along it.
    assert statistics.stdev(f.position[1] for f in fixes) == pytest.approx(35, rel=0.1)
    assert statistics.stdev(f.position[0] for f in fixes) < 5


@pytest.mark.parametrize('distance', [200.0, 2000.0])
def test_tracker_beats_the_raw_radar_and_knows_its_error(distance):
    config, rng = radar_at(distance), random.Random(0)
    tracker = KalmanTracker(target_z=ALT)
    raw_errors, errors, stds, speed_errors = [], [], [], []
    for i in range(240):  # 2 minutes of 2 Hz returns, flying east (across the radar's view)
        t = i * 0.5
        truth = (0.0, 4.0 * t, ALT)
        fix = radar_fix(truth, (0.0, 4.0, 0.0), config, rng, t)
        tracker.update(fix)
        if i > 40:
            position, velocity = tracker.estimate(t)
            raw_errors.append(guidance.horizontal_distance(fix.position, truth))
            errors.append(guidance.horizontal_distance(position, truth))
            stds.append(tracker.position_std(t))
            speed_errors.append(math.hypot(velocity[0], velocity[1] - 4.0))
    assert statistics.mean(errors) < 0.6 * statistics.mean(raw_errors)
    # The uncertainty it reports matches the error it makes (within a factor of 2).
    assert 0.5 < statistics.mean(errors) / statistics.mean(stds) < 2.0
    # Speed across the line of sight only comes from successive positions: worse far out.
    assert statistics.mean(speed_errors) < (1.5 if distance <= 200 else 2.5)


def test_tracker_predicts_between_fixes_and_restarts_on_a_jump():
    tracker = KalmanTracker(target_z=ALT)
    assert tracker.estimate(0.0) is None and tracker.age(0.0) == math.inf
    for i in range(60):
        tracker.update(PositionFix(time=float(i), source='depth', position=(2.0 * i, 0, ALT),
                                   std=0.2))
    position, velocity = tracker.estimate(59.5)
    assert position == pytest.approx((119.0, 0.0, ALT), abs=0.2)
    assert velocity == pytest.approx((2.0, 0.0, 0.0), abs=0.05)
    assert tracker.age(59.5) == 0.5 and tracker.position_std(59.5) < 1.0
    tracker.update(PositionFix(time=60.0, source='depth', position=(500.0, 500.0, ALT), std=0.2))
    assert tracker.estimate(60.0) == ((500.0, 500.0, ALT), (0.0, 0.0, 0.0))
    assert tracker.fixes == 1


# --- guidance ---------------------------------------------------------------

def test_lead_point_is_where_a_constant_velocity_target_can_be_met():
    me, target, velocity = (0.0, 0.0, ALT), (100.0, 0.0, ALT), (0.0, 5.0, 0.0)
    lead = guidance.lead_point(target, velocity, me, max_speed=10.0)
    t_target = math.dist(target, lead) / 5.0
    t_me = math.dist(me, lead) / 10.0
    assert t_target == pytest.approx(t_me, rel=0.01)


def test_shadow_point_works_round_to_behind_the_target():
    target, north = (0.0, 0.0, ALT), (4.0, 0.0, 0.0)
    # Already behind: straight behind.
    assert guidance.shadow_point(target, north, (-50, 0, ALT), 10.0) == pytest.approx((-10, 0, ALT))
    # Off to the side (east): at most 45 degrees round toward behind, at standoff.
    s = 10 / math.sqrt(2)
    assert guidance.shadow_point(target, north, (0, 50, ALT), 10.0) == pytest.approx((-s, s, ALT))
    # A target that is not moving is held on our side.
    still = guidance.shadow_point(target, (0, 0, 0), (0, 50, ALT), 10.0)
    assert still == pytest.approx((0, 10, ALT))


def test_approach_point_slows_down_near_the_aim():
    me = (0.0, 0.0, ALT)
    far = guidance.approach_point((200.0, 0.0, ALT), me, max_speed=10.0)
    assert far[0] == pytest.approx(10.0 / guidance.POSITION_GAIN)
    near = guidance.approach_point((2.0, 0.0, ALT), me, max_speed=10.0)
    assert near == pytest.approx((2.0, 0.0, ALT))


def test_keep_clear_pushes_points_out_to_min_separation():
    target = (0.0, 0.0, ALT)
    assert guidance.keep_clear((2.0, 0.0, ALT), target, 5.0) == (5.0, 0.0, ALT)
    assert guidance.keep_clear((8.0, 0.0, ALT), target, 5.0) == (8.0, 0.0, ALT)


def test_phases_with_hysteresis():
    assert guidance.next_phase(Phase.INTERCEPT, None, 10, 3) == Phase.LOST
    assert guidance.next_phase(Phase.LOST, 50, 10, 3) == Phase.INTERCEPT
    assert guidance.next_phase(Phase.INTERCEPT, 12, 10, 3) == Phase.SHADOW
    assert guidance.next_phase(Phase.INTERCEPT, 15, 10, 3) == Phase.INTERCEPT
    assert guidance.next_phase(Phase.SHADOW, 15, 10, 3) == Phase.SHADOW
    assert guidance.next_phase(Phase.SHADOW, 21, 10, 3) == Phase.INTERCEPT


def test_gimbal_angles():
    me = (0.0, 0.0, ALT)
    assert guidance.gimbal_angles(me, 0.0, (10.0, 0.0, ALT)) == pytest.approx((0.0, 0.0))
    assert guidance.gimbal_angles(me, 0.0, (0.0, 10.0, ALT)) == pytest.approx((0.0, 90.0))
    # Target 10 m away and 10 m lower, while we face east: look 45 down and to the left.
    assert guidance.gimbal_angles(me, math.pi / 2, (10.0, 0.0, ALT + 10)) == pytest.approx(
        (-45.0, -90.0))


def test_pointing_error():
    me, target = (0.0, 0.0, ALT), (10.0, 0.0, ALT)
    assert guidance.pointing_error_deg(0.0, 0.0, me, target) == pytest.approx(0.0, abs=1e-6)
    assert guidance.pointing_error_deg(-30.0, 0.0, me, target) == pytest.approx(30.0)
    assert guidance.pointing_error_deg(0.0, 90.0, me, target) == pytest.approx(90.0)


# --- config -----------------------------------------------------------------

def test_intercept_config_defaults_and_overrides():
    drones = [{'id': 0}, {'id': 1}]
    assert parse_config({'drones': drones}).intercept == InterceptConfig()
    config = parse_config({'drones': drones, 'intercept': {'standoff': 15, 'use_gimbal': False}})
    assert config.intercept.standoff == 15.0 and config.intercept.use_gimbal is False
    with pytest.raises(ValueError):
        parse_config({'drones': drones, 'intercept': {'stand_off': 15}})
    with pytest.raises(ValueError):
        parse_config({'drones': drones, 'intercept': {'interceptor_id': 1, 'hostile_id': 1}})


# --- closed loop --------------------------------------------------------------

class LatestFix(TargetSource):
    def __init__(self):
        self.fix = None

    def latest(self):
        return self.fix


def run_chase(pattern, config, seconds=180.0, seed=1):
    """Fly a SimDrone chase against the mock radar; returns per-step samples."""
    clock = [0.0]
    hostile = SimDrone(0, (0.0, 0.0, ALT))
    hunter = SimDrone(2, (-60.0, -40.0, ALT))
    path = make_path(pattern, hostile.position, heading=math.radians(45),
                     speed=config.hostile_speed, size=config.pattern_size, seed=seed)
    flight = HostileFlight(hostile, path, lambda: clock[0])
    radar = LatestFix()
    chase = ChaseAction(hunter, [radar], config, lambda: clock[0])
    rng = random.Random(seed)
    flight.start()
    chase.start()
    samples = []
    for i in range(int(seconds / DT)):
        clock[0] = i * DT
        if i % round(1 / (config.radar_rate_hz * DT)) == 0:
            radar.fix = radar_fix(hostile.position, hostile.velocity, config, rng, clock[0])
        flight.step()
        chase.step()
        hostile.advance(DT)
        hunter.advance(DT)
        samples.append({
            't': clock[0], 'phase': chase.phase,
            'range': guidance.horizontal_distance(hunter.position, hostile.position),
            'standoff': chase.standoff})
    return samples, chase, hunter


@pytest.mark.parametrize('distance', [200.0, 2000.0])
@pytest.mark.parametrize('pattern', PATTERNS)
def test_chase_catches_up_and_shadows_at_standoff(pattern, distance):
    config = radar_at(distance)
    samples, chase, hunter = run_chase(pattern, config)
    if distance <= 200:
        # With a far radar (about 15 m sigma after filtering) a target that turns back
        # can still pass close: radar alone cannot keep a same-altitude chase safe.
        closest = min(samples, key=lambda s: s['range'])
        assert closest['range'] > 4.0, f'{closest["range"]:.1f} m at t={closest["t"]:.1f}'
    capture = next(s['t'] for s in samples if s['phase'] == Phase.SHADOW)
    assert capture < 40
    assert chase.phase == Phase.SHADOW
    assert hunter.target[2] == ALT and hunter.gimbal_aim is not None
    late = [s for s in samples if s['t'] > 90]
    mean_error = statistics.mean(abs(s['range'] - s['standoff']) for s in late)
    mean_standoff = statistics.mean(s['standoff'] for s in late)
    if distance <= 200:  # radar error of a few metres: the configured standoff
        assert mean_standoff < config.standoff + 1
        assert mean_error < 3.5, f'mean standoff error {mean_error:.1f} m'
    else:  # tens of metres: kept further away, in proportion
        assert mean_standoff > config.standoff + 5
        assert mean_error < 0.6 * mean_standoff, f'mean standoff error {mean_error:.1f} m'


def test_chase_holds_when_the_target_is_lost():
    config = InterceptConfig()
    clock = [0.0]
    hunter = SimDrone(2, (0.0, 0.0, ALT))
    radar = LatestFix()
    chase = ChaseAction(hunter, [radar], config, lambda: clock[0])
    chase.start()
    radar.fix = PositionFix(time=0.0, source='depth', position=(100.0, 0.0, ALT), std=1.0)
    chase.step()
    assert chase.phase == Phase.INTERCEPT and hunter.target[0] > 5 and hunter.target[2] == ALT
    clock[0] = config.track_timeout + 1
    chase.step()
    assert chase.phase == Phase.LOST and hunter.target == hunter.position
