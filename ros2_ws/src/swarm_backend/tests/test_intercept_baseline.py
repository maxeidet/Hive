"""The intercept baseline on every scenario of the planning report (Interception >
Evaluation): pure pursuit with the stand-in detector, eval_runs seeds per aspect and
speed, in the fast simulation. `pytest -s` prints the report's metrics table.

Skipped until PurePursuitAction and NoisyTruthDetector are implemented. The asserts
are the report's expectations for the baseline, so they are its acceptance test.
"""
import math
import random

import pytest

from swarm_backend.config import InterceptConfig
from swarm_backend.intercept.interceptor import PurePursuitAction
from swarm_backend.intercept.metrics import Outcome, score_run, summarize
from swarm_backend.intercept.scenario import run_scenario, scenario_grid
from swarm_backend.intercept.target_detector import NoisyTruthDetector

from sim import FastSim

CONFIG = InterceptConfig()


def noisy_truth(sim, scenario, config):
    return NoisyTruthDetector(sim.interceptor, lambda: sim.hostile.position, config,
                              random.Random(scenario.seed))


def run(scenario):
    sim = FastSim(scenario, interceptor_max_speed=CONFIG.interceptor_max_speed)
    samples = run_scenario(scenario, sim, PurePursuitAction, noisy_truth, CONFIG)
    return score_run(samples, scenario, CONFIG)


@pytest.fixture(scope='module')
def results():
    grid = scenario_grid(CONFIG)
    try:
        first = run(grid[0])
    except NotImplementedError:
        pytest.skip('baseline guidance not implemented yet (PurePursuitAction, '
                    'NoisyTruthDetector)')
    results = [first] + [run(s) for s in grid[1:]]
    print('\n' + summarize(results, 'Baseline: pure pursuit, true position + noise'))
    return results


def cell(results, aspect, speed):
    return [r for r in results if r.aspect_deg == aspect and r.speed == speed]


def rate(runs, predicate):
    return sum(map(predicate, runs)) / len(runs)


def test_every_run_is_scored(results):
    assert len(results) == len(scenario_grid(CONFIG))
    for r in results:
        assert math.isfinite(r.closest_miss) and math.isfinite(r.closest_to_base)
        assert (r.outcome == Outcome.HIT) == (r.time_to_hit is not None)


def test_head_on_slower_targets_are_hit(results):
    # The report aims for a share of hits like Zheng et al.'s 64.7 % at 4 m/s.
    slow = [s for s in CONFIG.eval_speeds if s < CONFIG.interceptor_max_speed]
    for speed in slow:
        runs = cell(results, 0.0, speed)
        assert rate(runs, lambda r: r.success) >= 0.6, f'head-on at {speed} m/s'


def test_crossing_targets_faster_than_the_interceptor_are_not_hit(results):
    # Pure pursuit ends up behind a crossing target and cannot catch a faster one.
    fast = [s for s in CONFIG.eval_speeds if s > CONFIG.interceptor_max_speed]
    for speed in fast:
        runs = cell(results, 90.0, speed)
        assert rate(runs, lambda r: r.outcome == Outcome.HIT) == 0.0, f'crossing at {speed} m/s'


def test_same_seed_same_result(results):
    scenario = scenario_grid(CONFIG)[0]
    assert run(scenario) == results[0]
