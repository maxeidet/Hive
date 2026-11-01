"""A defender on the arc (intercept/patrol.py), on a simulated drone and hand-made detections."""
import math

import pytest
from test_attack import SimDrone

from swarm_backend.config import AttackConfig, SurveillanceConfig
from swarm_backend.intercept.attack import DetectionInbox
from swarm_backend.intercept.base_station import Mode, Order
from swarm_backend.intercept.patrol import PatrolAction

STEP = 0.1
CONFIG = SurveillanceConfig()


def make(slot=1, **changes):
    clock = [0.0]
    drone = SimDrone(2, (0.0, 5.0, 0.0))
    inbox = DetectionInbox()
    config = SurveillanceConfig(**changes) if changes else CONFIG
    patrol = PatrolAction(drone, inbox, AttackConfig(), config, slot, lambda: clock[0])
    patrol.start()
    return patrol, drone, inbox, clock


def run(patrol, drone, clock, seconds, inbox=None, message=None):
    for _ in range(round(seconds / STEP)):
        clock[0] = round(clock[0] + STEP, 6)
        if message is not None:
            inbox.put_position(message, clock[0], clock[0])
        drone.advance(STEP)
        patrol.step()


def test_the_slot_is_on_the_arc_at_the_patrol_height():
    patrol, *_ = make(slot=1)
    assert patrol.slot_point == pytest.approx((60.0, 0.0, -20.0))  # middle, due north
    left, *_ = make(slot=0)
    assert math.hypot(*left.slot_point[:2]) == pytest.approx(60.0)
    assert left.slot_point[1] < 0  # left of north is west


def test_start_sends_the_drone_to_its_slot_facing_the_threat():
    patrol, drone, *_ = make(slot=2)
    assert drone.command == pytest.approx(patrol.slot_point)
    assert drone.yaw == 0.0


def test_the_aim_point_sweeps_across_the_sector_and_back():
    patrol, drone, inbox, clock = make()
    patrol.apply(Order(2, Mode.PATROL, sector=(-15.0, 15.0), may_start=False))
    seen = set()
    for _ in range(40):
        run(patrol, drone, clock, 0.1)
        seen.add(round(patrol._phi))
    assert min(seen) <= -14 and max(seen) >= 14
    assert drone.gimbal_aim is not None


def test_a_confirmed_detection_is_reported_with_its_world_position():
    patrol, drone, inbox, clock = make()
    patrol.apply(Order(2, Mode.PATROL, sector=(-15.0, 15.0), may_start=False))
    drone.position = (60.0, 0.0, -20.0)
    run(patrol, drone, clock, 1.0, inbox, (20.0, 5.0, 0.0))
    sightings = patrol.take_sightings()
    assert len(sightings) >= 10
    assert sightings[0].position == pytest.approx((80.0, 5.0, -20.0))
    assert not sightings[0].confirmed and sightings[-1].confirmed
    assert patrol.mode is Mode.PATROL  # not allowed to start by itself
    assert patrol.take_sightings() == []


def test_a_drone_that_may_start_turns_on_the_target_at_once():
    patrol, drone, inbox, clock = make()
    patrol.apply(Order(2, Mode.PATROL, sector=(-15.0, 15.0), may_start=True))
    run(patrol, drone, clock, 0.3, inbox, (20.0, 5.0, 0.0))
    assert patrol.mode is Mode.ENGAGE
    assert drone.mode == 'velocity'


def test_an_engage_order_starts_the_attack_and_repeats_do_nothing():
    patrol, drone, inbox, clock = make()
    order = Order(2, Mode.ENGAGE, 'T1')
    patrol.apply(order)
    attack = patrol.attack
    patrol.apply(order)
    assert patrol.mode is Mode.ENGAGE and patrol.attack is attack


def test_contact_is_reported_once():
    patrol, drone, inbox, clock = make()
    patrol.apply(Order(2, Mode.ENGAGE, 'T1'))
    run(patrol, drone, clock, 1.0, inbox, (2.0, 0.0, 0.0))
    assert [e.kind for e in patrol.take_events()] == ['contact']


def test_a_target_never_seen_again_is_reported_lost():
    patrol, drone, inbox, clock = make()
    patrol.apply(Order(2, Mode.ENGAGE, 'T1'))
    run(patrol, drone, clock, 4.0)
    assert [e.kind for e in patrol.take_events()] == ['target_lost']


def test_return_flies_back_to_the_slot_and_reports_it():
    patrol, drone, inbox, clock = make()
    drone.position = (90.0, 10.0, -20.0)
    patrol.apply(Order(2, Mode.ENGAGE, 'T1'))
    patrol.apply(Order(2, Mode.RETURN))
    assert patrol.mode is Mode.RETURN
    assert drone.command == pytest.approx(patrol.slot_point)
    run(patrol, drone, clock, 20.0)
    assert patrol.mode is Mode.PATROL
    assert [e.kind for e in patrol.take_events()] == ['returned']


def test_status_gives_the_distance_from_the_base():
    patrol, drone, *_ = make()
    drone.position = (30.0, 40.0, -20.0)
    status = patrol.status()
    assert status.distance_m == pytest.approx(50.0) and status.mode is Mode.PATROL


def test_orders_for_other_drones_are_ignored():
    patrol, *_ = make()
    patrol.apply(Order(3, Mode.ENGAGE, 'T1'))
    assert patrol.mode is Mode.PATROL
