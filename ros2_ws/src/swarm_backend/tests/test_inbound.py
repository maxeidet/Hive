"""Scenario 2's geometry: straight-in flights and matching detections (intercept/inbound.py)."""
import math

import pytest

import random

from swarm_backend.intercept.inbound import (inbound_line, match_hostile, plan_birds,
                                              straight_at)

BASE = (0.0, 0.0)


def test_inbound_line_stops_short_of_the_base_on_its_own_line():
    start = (150.0 * math.cos(math.radians(20)), 150.0 * math.sin(math.radians(20)), -40.0)
    end = inbound_line(start, BASE, 20.0)
    assert math.hypot(end[0], end[1]) == pytest.approx(20.0)
    assert math.atan2(end[1], end[0]) == pytest.approx(math.radians(20))  # same bearing
    assert end[2] == -40.0  # level flight


def test_the_two_scenario_lines_end_apart():
    ends = [inbound_line((150 * math.cos(math.radians(b)), 150 * math.sin(math.radians(b)), -40),
                         BASE, 20.0) for b in (-20, 20)]
    assert math.dist(*ends) == pytest.approx(2 * 20 * math.sin(math.radians(20)))


def test_inbound_line_refuses_a_start_inside_the_stop_distance():
    with pytest.raises(ValueError):
        inbound_line((5.0, 0.0, -30.0), BASE, 10.0)


def test_straight_at_flies_at_constant_speed_then_stays_at_the_end():
    start, end = (100.0, 0.0, -40.0), (20.0, 0.0, -40.0)
    pos, vel, done = straight_at(start, end, 5.0, 4.0)
    assert pos == pytest.approx((80.0, 0.0, -40.0))
    assert vel == pytest.approx((-5.0, 0.0, 0.0))
    assert not done
    pos, vel, done = straight_at(start, end, 5.0, 100.0)
    assert pos == pytest.approx(end)
    assert vel == (0.0, 0.0, 0.0)
    assert done


def test_match_hostile_picks_the_one_in_the_detections_direction():
    observer = (0.0, 0.0, -30.0)
    hostiles = {2: (100.0, -30.0, -40.0), 3: (100.0, 30.0, -44.0)}
    # A detection toward hostile 3, with the distance 20 % short (box-size estimate).
    to3 = [0.8 * (h - o) for h, o in zip(hostiles[3], observer)]
    drone_id, distance = match_hostile(tuple(to3), observer, hostiles)
    assert drone_id == 3
    assert distance == pytest.approx(math.dist(hostiles[3], observer))


def test_match_hostile_none_outside_the_gate():
    observer = (0.0, 0.0, -30.0)
    assert match_hostile((0.0, 0.0, 1.0), observer, {2: (100.0, 0.0, -40.0)}) is None


KINDS = ['dove', 'mourning_dove', 'flock', 'seagull', 'gull', 'eagle', 'soaring_bird']
STARTS = [(141.0, -51.3, -40.0), (141.0, 51.3, -44.0)]


def test_birds_start_around_the_hostiles_and_fly_over_the_base():
    birds = plan_birds(10, STARTS, BASE, KINDS, 5.0, random.Random(1))
    assert len(birds) == 10 and len({b.name for b in birds}) == 10
    for b in birds:
        near = min(math.dist(b.start[:2], s[:2]) for s in STARTS)
        assert 5.0 <= near <= 25.0 + 1e-9
        assert -52.0 <= b.start[2] <= -5.0 and b.end[2] == b.start[2]  # level
        assert 3.5 - 1e-9 <= b.speed <= 6.5 + 1e-9
        # the base lies on its line, 60 m before the end
        d_start, d_end = math.hypot(*b.start[:2]), math.hypot(*b.end[:2])
        assert math.dist(b.start[:2], b.end[:2]) == pytest.approx(d_start + 60.0)
        assert d_end == pytest.approx(60.0)


def test_every_bird_kind_is_used_before_any_twice():
    birds = plan_birds(10, STARTS, BASE, KINDS, 5.0, random.Random(2))
    assert set(b.kind for b in birds[:7]) == set(KINDS)


def test_birds_stay_above_the_ground_and_follow_the_seed():
    low = [(100.0, 0.0, -6.0)]
    birds = plan_birds(20, low, BASE, KINDS, 5.0, random.Random(3))
    assert all(b.start[2] <= -5.0 for b in birds)
    assert birds == plan_birds(20, low, BASE, KINDS, 5.0, random.Random(3))
    assert plan_birds(0, low, BASE, KINDS, 5.0, random.Random(3)) == []


def test_plan_birds_prefix():
    import random as _random
    named = plan_birds(3, [(100.0, 0.0, -40.0)], (0.0, 0.0), ['dove'], 4.0, _random.Random(0),
                       prefix='bird_')
    assert [b.name for b in named] == ['bird_0', 'bird_1', 'bird_2']
    default = plan_birds(1, [(100.0, 0.0, -40.0)], (0.0, 0.0), ['dove'], 4.0, _random.Random(0))
    assert default[0].name.startswith('scn_bird_')
