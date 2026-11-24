"""Several hostiles in one run: surveillance2.yaml, SurveillanceWorld.fly_many."""
from swarm_backend.config import load_config
from swarm_backend.scenarios import common
from swarm_backend.scenarios.sim_surveillance import SimSurveillance
from swarm_backend.scenarios.surveillance import Options

TWO = load_config(common.PKG / 'scenarios' / 'surveillance2.yaml')
ONE = load_config(common.PKG / common.SURVEILLANCE_CONFIG)


def test_hostile_ids():
    assert SimSurveillance(TWO, Options(detector='fake'), quiet=True).hostile_ids == (4, 5)
    assert SimSurveillance(ONE, Options(detector='fake'), quiet=True).hostile_ids == (4,)


import math  # noqa: E402

import pytest  # noqa: E402

from swarm_backend.scenarios.surveillance import Leg  # noqa: E402
from tests.test_scenarios import head_on, world  # noqa: E402


def two_world():
    w = SimSurveillance(TWO, Options(detector='fake'), quiet=True)
    w.setup()
    return w


def leg(w, hostile_id, bearing_deg, speed=4.0, out_m=300.0):
    """From `out_m` out at `bearing_deg` off the threat heading, at the defenders' height,
    straight at the base (world NED)."""
    b = w.heading + math.radians(bearing_deg)
    alt = w.patrols[w.ids[0]].patrol.slot_point[2]
    bx, by = w.surv.base_position
    return Leg(hostile_id, (bx + out_m * math.cos(b), by + out_m * math.sin(b), alt),
               (bx, by, alt), speed)


def test_two_hostiles_both_hit():
    w = two_world()
    flight = w.fly_many([leg(w, 4, -30.0), leg(w, 5, 30.0)])
    assert [r.hostile_id for r in flight.results] == [4, 5]
    assert all(r.killer is not None and r.hit_time is not None for r in flight.results), \
        [(r.hostile_id, r.closest) for r in flight.results]


def test_two_defenders_sent():
    w = two_world()
    flight = w.fly_many([leg(w, 4, -30.0), leg(w, 5, 30.0)])
    killers = [r.killer for r in flight.results]
    assert None not in killers and killers[0] != killers[1]


def test_one_of_two_hit():
    # The second hostile at the arc's edge (+55 deg: missed by 63 m even alone, 24 Nov): the
    # run goes on after the first hit and ends at the second's track end, the second unhit.
    w = two_world()
    flight = w.fly_many([leg(w, 4, -30.0), leg(w, 5, 55.0)], timeout=200.0)
    first, second = flight.results
    assert first.killer is not None
    assert second.killer is None and second.closest > 0.0
    assert flight.t_end - flight.t0 > first.hit_time


def test_fly_unchanged():
    # Recorded from the code before fly_many (24 Nov): head-on, perfect fake detector.
    w = world()
    scenario, start, goal = head_on(w)
    flight = w.fly(start, goal, scenario.speed)
    assert flight.killer == 2
    assert flight.hit_time(2) == pytest.approx(33.1, abs=0.05)
    assert flight.engaged == (pytest.approx(25.7, abs=0.05), 2)
