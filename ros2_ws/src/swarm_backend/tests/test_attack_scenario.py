"""The attack test's scenario geometry and hit scoring (intercept/attack_scenario.py)."""
import math

import pytest

from swarm_backend.intercept.attack_scenario import (
    ClosestApproach, Scenario, hostile_path, segment_distance)

GUARD = (5.0, 0.0, -20.0)
NORTH = 0.0


def test_crossing_runs_across_the_view_above_the_guard():
    path, seconds = hostile_path(Scenario('crossing', speed=4.0, height=10.0), GUARD, NORTH)
    start, velocity = path.at(0.0)
    assert start == pytest.approx((40.0, -60.0, -30.0))  # 35 m ahead, 60 m left, 10 m up
    assert velocity == pytest.approx((0.0, 4.0, 0.0))  # flying east, to the right
    assert seconds == pytest.approx(30.0)  # 120 m at 4 m/s
    assert path.at(seconds - 1e-6)[0] == pytest.approx((40.0, 60.0, -30.0), abs=1e-3)


def test_headon_flies_over_the_guard_toward_it():
    path, seconds = hostile_path(Scenario('headon', speed=5.0), GUARD, math.radians(90))
    start, velocity = path.at(0.0)
    assert start == pytest.approx((5.0, 80.0, -30.0))  # 80 m east (the search heading)
    assert velocity == pytest.approx((0.0, -5.0, 0.0))
    assert seconds == pytest.approx(100.0 / 5.0)


def test_offset_comes_head_on_at_the_same_height_beside_the_guard():
    path, seconds = hostile_path(Scenario('offset', speed=4.0), GUARD, NORTH)
    start, velocity = path.at(0.0)
    assert start == pytest.approx((65.0, 8.0, -20.0))  # 60 m ahead, 8 m right, same height
    assert velocity == pytest.approx((-4.0, 0.0, 0.0))  # straight back toward the guard
    assert seconds == pytest.approx(80.0 / 4.0)  # to 20 m behind the guard
    assert path.at(seconds - 1e-6)[0] == pytest.approx((-15.0, 8.0, -20.0), abs=1e-3)


def test_offset_height_and_side_can_be_changed():
    path, _ = hostile_path(Scenario('offset', height=2.0, half_width=-5.0), GUARD, NORTH)
    assert path.at(0.0)[0] == pytest.approx((65.0, -5.0, -22.0))


def test_circle_is_centred_ahead_and_loops():
    path, seconds = hostile_path(Scenario('circle', half_width=20.0), GUARD, NORTH)
    assert seconds is None
    centre = (GUARD[0] + 30.0, GUARD[1])
    for t in range(0, 40, 3):
        x, y, z = path.at(float(t))[0]
        # make_path turns the first leg half a degree, so the centre is off by ~0.2 m
        assert math.hypot(x - centre[0], y - centre[1]) == pytest.approx(20.0, abs=0.3)
        assert z == pytest.approx(-30.0)


def test_unknown_pattern_is_rejected():
    with pytest.raises(ValueError):
        Scenario('zigzag')


def test_segment_distance_catches_a_pass_between_samples():
    # Head-on at 16 m/s closing, sampled 0.1 s apart: 1.6 m per sample, passing 0.5 m apart.
    assert segment_distance((0, 0, 0), (1.2, 0, 0), (0.8, 0.5, 0), (0.4, 0.5, 0)) == pytest.approx(0.5)
    assert segment_distance((0, 0, 0), (0, 0, 0), (3, 4, 0), (3, 4, 0)) == pytest.approx(5.0)


def test_closest_approach_records_the_first_hit_time():
    judge = ClosestApproach(radius=1.5)
    for t, x in enumerate([10.0, 6.0, 2.0, -2.0, -6.0]):  # defender passes the target at 0
        judge.update((x, 0.0, 0.0), (0.0, 1.0, 0.0), float(t))
    assert judge.hit and judge.hit_time == 3.0  # the pass between t=2 and t=3
    assert judge.closest == pytest.approx(1.0)
    miss = ClosestApproach(radius=1.5)
    miss.update((10.0, 3.0, 0.0), (0.0, 0.0, 0.0), 0.0)
    miss.update((-10.0, 3.0, 0.0), (0.0, 0.0, 0.0), 1.0)
    assert not miss.hit and miss.closest == pytest.approx(3.0)
