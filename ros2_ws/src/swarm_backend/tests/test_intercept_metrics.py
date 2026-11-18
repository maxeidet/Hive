"""The intercept evaluation harness: scenarios, the run loop and the metrics of the
planning report (Interception > Evaluation). No guidance law is needed: the closed
loop runs with small test doubles."""
import math

import pytest

from swarm_backend.config import InterceptConfig, parse_config
from swarm_backend.intercept.interceptor import InterceptAction, Status
from swarm_backend.intercept.metrics import (LockOnTracker, Outcome, Sample, closest_approach,
                                             first_within, score_run,
                                             segment_closest_distance, summarize)
from swarm_backend.intercept.scenario import Scenario, make_scenario, run_scenario, scenario_grid
from swarm_backend.intercept.target import PositionFix
from swarm_backend.intercept.target_detector import Detector

from swarm_backend.fast_sim import FastSim

CONFIG = InterceptConfig()
DT = 0.1


# --- closest approach -------------------------------------------------------------

def test_closest_approach_of_crossing_and_parallel_tracks():
    distance, f = closest_approach((-1, 0, 0), (1, 0, 0), (0, -1, 0), (0, 1, 0))
    assert distance == pytest.approx(0.0) and f == pytest.approx(0.5)
    distance, f = closest_approach((0, 0, 0), (1, 0, 0), (0, 3, 0), (1, 3, 0))
    assert distance == pytest.approx(3.0) and f == 0.0
    assert segment_closest_distance((0, 0, 0), (0, 0, 0), (5, 0, 0), (5, 0, 0)) == 5.0


def test_head_on_hit_between_samples_is_found():
    # 12 + 17 m/s head-on: 2.9 m per 0.1 s step, 1 m to the side.
    p0, p1 = (0.0, 0.0, 0.0), (1.2, 0.0, 0.0)
    q0, q1 = (1.5, 1.0, 0.0), (-0.2, 1.0, 0.0)
    assert math.dist(p0, q0) > CONFIG.hit_radius and math.dist(p1, q1) > CONFIG.hit_radius
    assert segment_closest_distance(p0, p1, q0, q1) == pytest.approx(1.0)
    entry = first_within(p0, p1, q0, q1, CONFIG.hit_radius)
    assert 0 < entry < 0.5  # on the way in, before the closest point
    assert first_within(p0, p1, q0, q1, 0.9) is None


# --- lock-on ----------------------------------------------------------------------

def feed(tracker, images, start=0.0, distance=10.0):
    """`images`: (in_view, detected) per image, DT apart. Returns the lock per image."""
    return [tracker.update(start + i * DT, v, d, distance) for i, (v, d) in enumerate(images)]


def test_lock_on_needs_k_of_n_detections():
    tracker = LockOnTracker(k=3, n=5, distance=20.0, duration=3.0)
    locks = feed(tracker, [(True, d) for d in (True, False, True, False, True, False, False)])
    assert locks == [False, False, False, False, True, False, False]


def test_lock_on_must_last_without_a_break():
    tracker = LockOnTracker(k=1, n=1, distance=20.0, duration=3.0)
    feed(tracker, [(True, True)] * 21)  # 2 s
    feed(tracker, [(False, True)], start=2.1)  # out of view: the run breaks
    assert not tracker.held and tracker.longest == pytest.approx(2.0)
    feed(tracker, [(True, True)] * 32, start=2.2)  # 3.1 s
    assert tracker.held and tracker.held_from == pytest.approx(2.2)
    assert tracker.longest == pytest.approx(3.1)


def test_no_lock_on_beyond_lock_distance():
    tracker = LockOnTracker(k=1, n=1, distance=20.0, duration=3.0)
    assert not any(feed(tracker, [(True, True)] * 50, distance=20.5))
    assert tracker.longest == 0.0


# --- scoring ----------------------------------------------------------------------

def scenario(**kwargs) -> Scenario:
    fields = dict(aspect_deg=0.0, speed=10.0, seed=0, base=(0.0, 0.0, 0.0),
                  interceptor_start=(0.0, 0.0, -50.0), target_start=(0.0, 0.0, -50.0),
                  target_goal=(0.0, 0.0, -50.0))
    return Scenario(**{**fields, **kwargs})


def samples_of(interceptor, target, seconds, detected=lambda t: True, status=Status.TRACKING):
    """Samples every DT of positions given as functions of t."""
    return [Sample(t=i * DT, interceptor=interceptor(i * DT), target=target(i * DT),
                   in_view=detected(i * DT), detected=detected(i * DT), status=status)
            for i in range(round(seconds / DT) + 1)]


def toward_base(start, speed):
    """A target flying north to south toward the base at the origin, 50 m up."""
    return lambda t: (start - speed * t, 0.0, -50.0)


def test_hit_far_from_base_is_a_success():
    still = lambda t: (500.0, 1.0, -50.0)  # noqa: E731
    run = score_run(samples_of(still, toward_base(600.0, 10.0), 30), scenario(), CONFIG)
    assert run.outcome == Outcome.HIT and run.success
    assert run.time_to_hit == pytest.approx(10.0 - math.sqrt(1.5 ** 2 - 1) / 10, abs=1e-6)
    assert run.base_distance_at_hit == pytest.approx(500.0 + math.sqrt(1.5 ** 2 - 1), abs=1e-6)
    assert run.closest_to_base == run.base_distance_at_hit
    assert run.closest_miss <= CONFIG.hit_radius


def test_hit_inside_goal_radius_is_not_a_success():
    still = lambda t: (50.0, 0.0, -50.0)  # noqa: E731
    run = score_run(samples_of(still, toward_base(150.0, 10.0), 30), scenario(), CONFIG)
    assert run.outcome == Outcome.HIT and not run.success
    assert run.base_distance_at_hit < CONFIG.goal_radius


def test_time_to_hit_counts_from_first_detection():
    still = lambda t: (500.0, 0.0, -50.0)  # noqa: E731
    samples = samples_of(still, toward_base(600.0, 10.0), 30, detected=lambda t: t >= 2.0)
    run = score_run(samples, scenario(), CONFIG)
    assert run.time_to_hit == pytest.approx(10.0 - 0.15 - 2.0, abs=1e-6)


def test_target_reaching_base():
    aside = lambda t: (300.0, 300.0, -50.0)  # noqa: E731
    run = score_run(samples_of(aside, toward_base(600.0, 10.0), 60), scenario(), CONFIG)
    assert run.outcome == Outcome.REACHED_BASE and not run.success
    assert run.time_to_hit is None and run.base_distance_at_hit is None
    assert run.closest_to_base == pytest.approx(0.0, abs=1e-6)
    assert run.closest_miss == pytest.approx(300.0)


@pytest.mark.parametrize('status, outcome', [(Status.LOST, Outcome.LOST),
                                             (Status.TRACKING, Outcome.TIMEOUT)])
def test_lost_and_timeout(status, outcome):
    aside = lambda t: (300.0, 300.0, -50.0)  # noqa: E731
    run = score_run(samples_of(aside, toward_base(600.0, 10.0), 10, status=status),
                    scenario(), CONFIG)
    assert run.outcome == outcome and run.closest_to_base == pytest.approx(500.0)


def test_lock_on_only_counts_outside_the_goal_radius():
    # Shadows the target 10 m behind it all the way in: locked from the k-th image.
    target = toward_base(160.0, 10.0)
    behind = lambda t: (target(t)[0] + 10.0, 0.0, -50.0)  # noqa: E731
    run = score_run(samples_of(behind, target, 16), scenario(), CONFIG)
    assert run.lock_on and run.time_to_lock == pytest.approx((CONFIG.lock_k - 1) * DT)
    assert run.longest_lock == pytest.approx(5.9 - run.time_to_lock)  # until 100 m
    late = toward_base(130.0, 10.0)
    behind_late = lambda t: (late(t)[0] + 10.0, 0.0, -50.0)  # noqa: E731
    assert not score_run(samples_of(behind_late, late, 13), scenario(), CONFIG).lock_on


def test_summary_has_a_row_per_aspect_and_speed():
    still = lambda t: (500.0, 0.0, -50.0)  # noqa: E731
    runs = [score_run(samples_of(still, toward_base(600.0, 10.0), 30),
                      scenario(aspect_deg=a, speed=s, seed=i), CONFIG)
            for a in (0.0, 90.0) for s in (4.0, 17.0) for i in range(3)]
    text = summarize(runs)
    assert '12 runs' in text and 'hit=3' in text
    assert len([line for line in text.splitlines() if 'hit=3' in line]) == 4


# --- scenarios --------------------------------------------------------------------

@pytest.mark.parametrize('aspect', [0.0, 30.0, 60.0, 90.0])
@pytest.mark.parametrize('seed', [0, 1, 2])
def test_scenario_geometry(aspect, seed):
    s = make_scenario(aspect, 8.0, seed, CONFIG)
    me, target = s.interceptor_start, s.target_start
    assert math.hypot(me[0], me[1]) == pytest.approx(CONFIG.perimeter_radius)
    assert me[2] == -CONFIG.altitude
    assert math.hypot(target[0] - me[0], target[1] - me[1]) == pytest.approx(CONFIG.detect_range)
    assert CONFIG.hostile_altitude_min <= -target[2] <= CONFIG.hostile_altitude_max
    assert s.target_goal == (0.0, 0.0, target[2])  # level flight to above the base
    los = math.atan2(me[1] - target[1], me[0] - target[0])
    heading = math.atan2(s.velocity[1], s.velocity[0])
    angle = abs(math.degrees((los - heading + math.pi) % (2 * math.pi) - math.pi))
    assert angle == pytest.approx(aspect, abs=1e-6)
    assert math.hypot(*s.velocity) == pytest.approx(8.0)
    assert s.arrival_time == pytest.approx(math.hypot(target[0], target[1]) / 8.0)
    assert make_scenario(aspect, 8.0, seed, CONFIG) == s


def test_scenario_grid_and_shift():
    grid = scenario_grid(CONFIG)
    assert len(grid) == (len(CONFIG.eval_aspects) * len(CONFIG.eval_speeds)
                         * CONFIG.eval_runs)
    s = grid[0].shifted((10.0, -5.0, 0.0))
    assert s.base == (10.0, -5.0, 0.0)
    assert s.target_start[0] == grid[0].target_start[0] + 10.0


def test_eval_settings_from_yaml():
    config = parse_config({'drones': [{'id': 0}],
                           'intercept': {'eval_speeds': [4, 17], 'lock_k': 2}})
    assert config.intercept.eval_speeds == (4.0, 17.0) and config.intercept.lock_k == 2


# --- the run loop, with test doubles ----------------------------------------------

class TruthDetector(Detector):
    """Sees the true position every step, wherever it is."""

    def __init__(self, sim, scenario, config):
        self.sim = sim
        self.fix = None

    def update(self, now):
        self.fix = PositionFix(time=now, source='truth', position=self.sim.hostile.position,
                               std=0.0)

    def latest(self):
        return self.fix

    in_view = True
    detected = True


class Halfway(InterceptAction):
    """Not a guidance law: jumps halfway to the latest fix every step."""

    def start(self):
        self._status = Status.TRACKING

    def stop(self):
        self._status = Status.IDLE

    def step(self):
        fix = self.detector.latest()
        if fix is not None:
            self.drone.position = tuple((a + b) / 2 for a, b in zip(self.drone.position,
                                                                    fix.position))

    @property
    def status(self):
        return self._status


class Hover(Halfway):
    def step(self):
        pass


def test_run_scenario_ends_at_a_hit():
    s = make_scenario(0.0, 4.0, 0, CONFIG)
    samples = run_scenario(s, FastSim(s), Halfway, TruthDetector, CONFIG)
    run = score_run(samples, s, CONFIG)
    assert run.outcome == Outcome.HIT and run.success
    assert 0 < run.time_to_hit < 1.0
    assert run.closest_miss <= CONFIG.hit_radius
    assert samples[-1].t == pytest.approx(run.duration)
    assert samples[0].t == 0.0 and samples[1].t == pytest.approx(DT)


def test_run_scenario_ends_when_the_target_reaches_base():
    s = make_scenario(90.0, 17.0, 0, CONFIG)
    samples = run_scenario(s, FastSim(s), Hover, TruthDetector, CONFIG)
    run = score_run(samples, s, CONFIG)
    assert run.outcome == Outcome.REACHED_BASE
    assert run.closest_to_base < 17.0 * DT * 2
    assert samples[-1].t == pytest.approx(s.arrival_time, abs=DT)
