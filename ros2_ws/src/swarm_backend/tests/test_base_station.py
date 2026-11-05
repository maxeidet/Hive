from swarm_backend.intercept.base_station import BaseConfig, BaseStation, Mode, Order, Sighting

NEAR, FAR = 400.0, 900.0  # inside / outside close_radius_m, both inside the leash


def station(**config):
    s = BaseStation([1, 2, 3], BaseConfig(**config))
    s.tick(0.0)
    return s


def see(drone, target='A', t=1.0, rng=FAR, confirmed=False):
    return Sighting(drone, target, t, rng, confirmed)


def engage_orders(orders):
    return [o for o in orders if o.mode is Mode.ENGAGE]


def mode(s, drone):
    return s.drones[drone].mode


def test_start_gives_every_drone_a_30_degree_sector():
    s = BaseStation([1, 2, 3])
    orders = s.tick(0.0)
    assert {o.drone_id: o.sector for o in orders} == {
        1: (-45, -15), 2: (-15, 15), 3: (15, 45)}
    assert all(o.may_start for o in orders)
    assert s.tick(1.0) == []


def test_the_drone_that_sees_the_target_goes_and_the_others_widen_to_45():
    s = station()
    orders = s.on_sightings([see(2)], 1.0)
    assert engage_orders(orders) == [Order(2, Mode.ENGAGE, 'A')]
    patrol = {o.drone_id: o for o in orders if o.mode is Mode.PATROL}
    assert patrol[1].sector == (-45, 0) and patrol[3].sector == (0, 45)


def test_first_report_wins_and_ties_go_to_the_lowest_id():
    s = station()
    orders = s.on_sightings([see(3, t=1.0), see(1, t=1.2), see(2, t=1.0)], 1.5)
    assert [o.drone_id for o in engage_orders(orders)] == [2]
    assert mode(s, 1) is Mode.PATROL and mode(s, 3) is Mode.PATROL


def test_a_second_sighting_of_a_served_target_does_nothing():
    s = station()
    s.on_sightings([see(2)], 1.0)
    assert engage_orders(s.on_sightings([see(1, t=2.0)], 2.0)) == []


def test_the_last_patrol_drone_is_held_back_until_the_target_is_confirmed_and_close():
    s = station()
    s.on_sightings([see(1, 'A')], 1.0)
    s.on_sightings([see(2, 'B')], 2.0)  # two attackers, one left on patrol
    assert mode(s, 3) is Mode.PATROL
    assert not s.drones[3].target
    far = s.on_sightings([see(3, 'C', rng=FAR, confirmed=True)], 3.0)
    assert engage_orders(far) == []
    unconfirmed = s.on_sightings([see(3, 'D', rng=NEAR, confirmed=False)], 4.0)
    assert engage_orders(unconfirmed) == []
    close = s.on_sightings([see(3, 'C', rng=NEAR, confirmed=True)], 5.0)
    assert [o.drone_id for o in engage_orders(close)] == [3]


def test_the_held_back_drone_is_told_not_to_start_by_itself():
    s = station()
    s.on_sightings([see(1)], 1.0)
    orders = s.on_sightings([see(2, 'B')], 2.0)
    last = [o for o in orders if o.mode is Mode.PATROL]
    assert [(o.drone_id, o.may_start) for o in last] == [(3, False)]


def test_each_target_gets_one_drone_and_the_closest_is_served_first():
    s = station(reserve=2)  # only one drone may go
    s.on_sightings([see(1, 'far', rng=900), see(2, 'near', rng=300)], 1.0)
    assert s.drones[2].target == 'near'
    assert mode(s, 1) is Mode.PATROL


def test_a_target_beyond_the_leash_is_not_attacked():
    s = station()
    assert engage_orders(s.on_sightings([see(1, rng=1500)], 1.0)) == []


def test_waiting_target_is_served_when_it_comes_close():
    s = station(reserve=3)
    s.on_sightings([see(1, 'A', rng=FAR)], 1.0)
    assert mode(s, 1) is Mode.PATROL
    orders = s.on_sightings([see(1, 'A', t=5.0, rng=NEAR, confirmed=True)], 5.0)
    assert [o.drone_id for o in engage_orders(orders)] == [1]


def test_attacker_past_the_leash_turns_back_and_the_target_is_released():
    s = station()
    s.on_sightings([see(2)], 1.0)
    assert s.on_status(2, 900.0, 2.0) == []
    orders = s.on_status(2, 1100.0, 3.0)
    assert Order(2, Mode.RETURN) in orders
    assert mode(s, 2) is Mode.RETURN and 'A' not in s.targets


def test_attacker_that_loses_its_target_returns():
    s = station()
    s.on_sightings([see(2)], 1.0)
    orders = s.on_target_lost(2, 2.0)
    assert Order(2, Mode.RETURN) in orders
    assert mode(s, 2) is Mode.RETURN


def test_miss_with_a_surviving_attacker_sends_it_back():
    s = station()
    s.on_sightings([see(2)], 1.0)
    orders = s.on_miss(2, drone_alive=True, target_inbound=False, now=2.0)
    assert Order(2, Mode.RETURN) in orders
    assert 'A' not in s.targets


def test_miss_with_a_lost_attacker_sends_a_new_one_if_the_target_is_inbound():
    s = station()
    s.on_sightings([see(2, rng=FAR), see(1, rng=FAR, t=1.5)], 1.0)
    orders = s.on_miss(2, drone_alive=False, target_inbound=True, now=2.0)
    assert mode(s, 2) is Mode.LOST
    assert [o.drone_id for o in engage_orders(orders)] == [1]


def test_miss_with_a_lost_attacker_and_a_gone_target_sends_nobody():
    s = station()
    s.on_sightings([see(2)], 1.0)
    orders = s.on_miss(2, drone_alive=False, target_inbound=False, now=2.0)
    assert engage_orders(orders) == []
    assert mode(s, 2) is Mode.LOST
    patrol = {o.drone_id: o.sector for o in orders if o.mode is Mode.PATROL}
    assert patrol == {} or set(patrol) <= {1, 3}


def test_a_hit_removes_the_target_and_loses_the_attacker():
    s = station()
    s.on_sightings([see(2)], 1.0)
    s.on_hit(2, 2.0)
    assert mode(s, 2) is Mode.LOST and 'A' not in s.targets


def test_a_kill_removes_the_target_and_sends_the_attacker_back():
    s = station()
    s.on_sightings([see(2)], 1.0)
    orders = s.on_kill(2, 2.0)
    assert Order(2, Mode.RETURN) in orders
    assert mode(s, 2) is Mode.RETURN and 'A' not in s.targets
    s.on_returned(2, 3.0)
    assert mode(s, 2) is Mode.PATROL


def test_a_kill_by_a_drone_on_patrol_changes_nothing():
    s = station()
    assert s.on_kill(1, 2.0) == []
    assert mode(s, 1) is Mode.PATROL


def test_returned_drone_gets_its_sector_back_after_the_delay():
    s = station(resweep_delay_s=10.0)
    s.on_sightings([see(2)], 1.0)
    s.on_target_lost(2, 2.0)
    s.on_returned(2, 30.0)
    assert s.tick(35.0) == []  # still waiting, the two others keep 45 degrees
    orders = s.tick(41.0)
    assert {o.drone_id: o.sector for o in orders} == {
        1: (-45, -15), 2: (-15, 15), 3: (15, 45)}


def test_drop_target_recalls_its_attacker():
    s = station()
    s.on_sightings([see(2)], 1.0)
    orders = s.drop_target('A', 2.0)
    assert Order(2, Mode.RETURN) in orders


def test_a_drone_that_returns_is_told_its_sector_again_even_if_it_was_never_recalled():
    s = station()
    orders = s.on_returned(2, 5.0)
    assert 2 in [o.drone_id for o in orders if o.mode is Mode.PATROL]
