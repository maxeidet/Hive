"""The attack test's scenario geometry and hit scoring (evaluation/attack_scenario.py)."""
import math

import pytest

from swarm_backend.evaluation.attack_scenario import (
    ClosestApproach, Inbound, inbound_path, segment_distance)

BASE = (0.0, 0.0)
DEFENDER_Z = -20.0  # defenders patrol 20 m up
NORTH = 0.0


def test_inbound_starts_out_on_the_threat_heading_and_flies_at_the_base():
    path, seconds = inbound_path(Inbound(speed=2.0), BASE, DEFENDER_Z, NORTH)
    start, velocity = path.at(0.0)
    assert start == pytest.approx((150.0, 0.0, -30.0))  # 150 m north, 10 m above the defenders
    assert velocity == pytest.approx((-2.0, 0.0, 0.0))  # straight back south
    assert seconds == pytest.approx(75.0)
    assert path.at(seconds - 1e-6)[0] == pytest.approx((0.0, 0.0, -30.0), abs=1e-3)


def test_inbound_bearing_is_off_the_threat_heading_positive_to_the_right():
    # Threat from the east; +30 deg is to the right of it, seen from the base: south-east.
    path, _ = inbound_path(Inbound(bearing_deg=30.0, start_m=100.0, height_m=5.0),
                           (10.0, 20.0), DEFENDER_Z, math.radians(90))
    start, velocity = path.at(0.0)
    assert start == pytest.approx((10.0 + 100 * math.cos(math.radians(120)),
                                   20.0 + 100 * math.sin(math.radians(120)), -25.0))
    # Toward the base, at 1 m/s
    assert velocity == pytest.approx((-math.cos(math.radians(120)),
                                      -math.sin(math.radians(120)), 0.0))


def test_inbound_rejects_a_bad_speed_or_start():
    with pytest.raises(ValueError):
        Inbound(speed=0.0)
    with pytest.raises(ValueError):
        Inbound(start_m=-1.0)


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
