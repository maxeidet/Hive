from swarm_backend.intercept.base_station import BaseConfig
from swarm_backend.intercept.surveillance_sim import Hostile, SimConfig, run

BASE = BaseConfig(leash_m=200.0, close_radius_m=100.0)


def simulate(hostiles, **config):
    return run(hostiles, SimConfig(**config), BASE)


def test_one_hostile_is_seen_and_hit_before_it_reaches_the_base():
    result = simulate([Hostile('A', -10.0, 150.0)])
    a = result.outcomes['A']
    assert a.destroyed_s is not None and a.reached_base_s is None
    assert a.destroyed_from_base_m > 50.0
    assert result.drones_lost == 1


def test_two_hostiles_at_once_are_both_hit_by_different_drones():
    result = simulate([Hostile('A', -10.0, 150.0), Hostile('B', 25.0, 138.0)])
    assert all(o.destroyed_s is not None for o in result.outcomes.values())
    assert result.drones_lost == 2


def test_the_reserve_drone_only_goes_for_a_target_inside_the_close_radius():
    hostiles = [Hostile('A', -10.0, 150.0), Hostile('B', 25.0, 138.0), Hostile('C', -35.0, 144.0)]
    result = simulate(hostiles)
    engaged = [text for _, text in result.events if 'engages' in text]
    assert len(engaged) == 3
    c = result.outcomes['C']
    assert c.first_seen_from_base_m <= BASE.close_radius_m
    assert c.destroyed_s is not None


def test_every_attack_missing_lets_them_through():
    result = simulate([Hostile('A', -10.0, 150.0)], miss_probability=1.0)
    assert result.outcomes['A'].reached_base_s is not None
    assert result.drones_lost == 0


def test_a_hostile_outside_every_camera_is_never_seen():
    result = simulate([Hostile('A', 80.0, 150.0)])  # far off the 90 degree arc
    assert result.outcomes['A'].first_seen_s is None
    assert result.outcomes['A'].reached_base_s is not None


def test_same_seed_same_result():
    hostiles = [Hostile('A', -10.0, 150.0), Hostile('B', 25.0, 140.0)]
    a = simulate(hostiles, miss_probability=0.5, seed=3)
    b = simulate(hostiles, miss_probability=0.5, seed=3)
    assert a.events == b.events
