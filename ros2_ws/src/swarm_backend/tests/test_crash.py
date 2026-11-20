"""When a defender has crashed, and why (scenarios/crash.py), and how the base station takes
a respawned one back (swarm/base_station.py, swarm/dispatcher.py)."""
import math

import pytest

from swarm_backend.config import SurveillanceConfig
from swarm_backend.swarm.base_station import Mode, Order
from swarm_backend.swarm.dispatcher import Dispatcher
from swarm_backend.swarm.messages import (
    EventMsg, SightingMsg, StatusMsg, decode_event, encode_event)
from swarm_backend.scenarios.crash import (
    CRASH_HEIGHT_M, INTERCEPT_CRASH_S, Crash, CrashWatch, crash_cause, crash_reason, misplaced,
    tilt_deg)

LEVEL = (0.0, 0.0, 0.0, 1.0)  # x, y, z, w
FLYING = (10.0, 5.0, -40.0)  # NED, 40 m up


def rolled(deg):
    """Rolled `deg` about the x axis."""
    h = math.radians(deg) / 2
    return (math.sin(h), 0.0, 0.0, math.cos(h))


@pytest.mark.parametrize('deg', [0, 30, 59, 61, 90, 180])
def test_tilt(deg):
    assert tilt_deg(rolled(deg)) == pytest.approx(deg, abs=1e-6)


def test_crash_reason():
    assert crash_reason(FLYING, LEVEL) is None
    assert crash_reason(FLYING, rolled(45)) is None  # a hard turn, not a crash
    assert crash_reason(FLYING, rolled(61)) is None  # the attack's hardest bank seen in Gazebo
    assert crash_reason(FLYING, rolled(90)) is None  # on its side in the air: not clear yet
    assert crash_reason(FLYING, rolled(130)) == 'tilted 130 deg'  # upside down, even in the air
    assert crash_reason(FLYING, rolled(180)) == 'tilted 180 deg'
    assert crash_reason((0.0, 0.0, -1.0), LEVEL) == 'down (1.0 m up)'  # upright on the ground
    assert crash_reason((0.0, 0.0, -CRASH_HEIGHT_M - 0.1), LEVEL) is None
    assert crash_reason(None, None) is None  # no pose yet


def test_watch_only_counts_watched_drones_and_keeps_the_first_time():
    w = CrashWatch()
    w.update(1.0, 1, (0.0, 0.0, 0.0), LEVEL)  # on the ground before take-off: not watched
    assert w.crashes() == {}
    w.watch([1, 2])
    w.update(2.0, 1, FLYING, LEVEL)
    w.update(2.5, 2, FLYING, rolled(61))  # a hard bank: not a crash
    w.update(3.0, 2, FLYING, rolled(150))
    w.update(4.0, 2, (0.0, 0.0, -0.2), rolled(175))  # still tumbling: the first time stays
    w.update(4.0, 3, FLYING, rolled(180))  # not a watched defender
    assert w.crashes() == {2: Crash(2, 3.0, 'tilted 150 deg')}


def test_watch_clear_and_stop():
    w = CrashWatch()
    w.watch([1, 2])
    w.update(1.0, 1, (0.0, 0.0, 0.0), LEVEL)
    w.clear([1])  # respawned and flying again
    assert w.crashes() == {}
    w.update(2.0, 1, (0.0, 0.0, 0.0), LEVEL)
    assert 1 in w.crashes()  # a new crash counts
    w.stop()  # landing
    w.update(3.0, 2, (0.0, 0.0, 0.0), LEVEL)
    assert 2 not in w.crashes()


def test_crash_cause():
    hit = 100.0
    assert crash_cause(Crash(2, hit + 0.5, ''), killer=2, hit_time=hit) == 'intercept'
    assert crash_cause(Crash(2, hit - 0.05, ''), killer=2, hit_time=hit) == 'intercept'
    assert crash_cause(Crash(2, hit + INTERCEPT_CRASH_S + 1, ''), killer=2, hit_time=hit) == 'other'
    assert crash_cause(Crash(1, hit + 0.5, ''), killer=2, hit_time=hit) == 'other'  # not the hitter
    assert crash_cause(Crash(2, hit, ''), killer=None, hit_time=None) == 'other'  # no hit


def test_misplaced_finds_a_px4_started_away_from_its_spawn_point():
    # Seen in Gazebo: drone 2 respawned below its slot (N 150), the next scenario took its
    # home from the config's spawn point (N -4) again, and put it 154 m north.
    truth = {1: (-4.0, -2.0, -0.2), 2: (-4.0, 1.0, -0.2), 3: (-4.0, 4.0, -0.2)}
    px4 = {1: (-4.1, -2.0, 0.0), 2: (150.0, 1.0, 0.0), 3: (-4.0, 4.2, 0.0)}
    assert misplaced(px4, truth) == {2: pytest.approx(154.0)}
    assert misplaced({1: None, 2: (150.0, 1.0, 0.0)}, {1: (0.0, 0.0, 0.0)}) == {}  # unknown


# --- the base station takes a respawned defender back --------------------------------

CONFIG = SurveillanceConfig()  # drones 1, 2, 3; status timeout 5 s


def test_respawned_is_a_message():
    assert decode_event(encode_event(EventMsg(2, 'respawned'))) == EventMsg(2, 'respawned')


def test_a_lost_defender_is_back_on_patrol_after_a_respawn():
    d = Dispatcher(CONFIG)
    d.start(0.0)
    for i in CONFIG.drone_ids:
        d.on_status(StatusMsg(i, Mode.PATROL, CONFIG.arc_radius), 0.5)
    for i in (1, 2):
        d.on_status(StatusMsg(i, Mode.PATROL, CONFIG.arc_radius), 6.0)
    d.tick(9.0)  # drone 3 silent (crashed, its PX4 restarting): lost
    assert d.station.drones[3].mode is Mode.LOST
    assert d.on_event(EventMsg(3, 'returned'), 20.0) == []  # still ignored from a lost drone
    orders = d.on_event(EventMsg(3, 'respawned'), 30.0)
    assert d.station.drones[3].mode is Mode.PATROL
    assert orders == [  # the arc shared by three again, drone 3 sweeping at once
        Order(1, Mode.PATROL, None, (-45.0, -15.0), True),
        Order(2, Mode.PATROL, None, (-15.0, 15.0), True),
        Order(3, Mode.PATROL, None, (15.0, 45.0), True)]


def test_a_respawned_attacker_drops_its_target():
    d = Dispatcher(CONFIG)
    d.start(0.0)
    for i in CONFIG.drone_ids:
        d.on_status(StatusMsg(i, Mode.PATROL, CONFIG.arc_radius), 0.5)
    d.on_sighting(SightingMsg(2, (CONFIG.arc_radius + 40.0, 0.0, -30.0), True), 1.0)
    assert d.station.drones[2].mode is Mode.ENGAGE
    d.on_event(EventMsg(2, 'respawned'), 2.0)
    assert d.station.drones[2].mode is Mode.PATROL and d.station.drones[2].target is None
    assert d.station.targets == {} and d.associator._tracks == {}
