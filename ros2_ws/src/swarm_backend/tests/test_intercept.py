import math

import pytest

from swarm_backend.config import InterceptConfig, parse_config
from swarm_backend.intercept import guidance
from swarm_backend.evaluation.paths import PATTERNS, make_path

ALT = -50.0
DT = 0.1


# --- paths ------------------------------------------------------------------

@pytest.mark.parametrize('pattern', PATTERNS)
def test_paths_start_at_anchor_fly_at_speed_and_keep_altitude(pattern):
    anchor = (10.0, -20.0, ALT)
    path = make_path(pattern, anchor, heading=math.radians(30), speed=4.0, size=80.0, seed=3)
    position, velocity = path.at(0.0)
    assert math.dist(position, anchor) < 1e-9
    assert math.degrees(math.atan2(velocity[1], velocity[0])) == pytest.approx(30.0, abs=1e-6)
    previous = position
    for i in range(1, 2000):
        position, velocity = path.at(i * DT)
        assert position[2] == ALT
        assert math.hypot(*velocity) == pytest.approx(4.0)
        assert math.dist(position, previous) <= 4.0 * DT + 1e-9  # continuous
        previous = position


def test_random_path_depends_only_on_seed():
    a = make_path('random', (0, 0, ALT), 0.0, 4.0, 80.0, seed=1).points
    assert a == make_path('random', (0, 0, ALT), 0.0, 4.0, 80.0, seed=1).points
    assert a != make_path('random', (0, 0, ALT), 0.0, 4.0, 80.0, seed=2).points


def test_unknown_pattern():
    with pytest.raises(ValueError):
        make_path('spiral', (0, 0, ALT), 0.0, 4.0, 80.0)


# --- gimbal -----------------------------------------------------------------

def test_gimbal_angles():
    me = (0.0, 0.0, ALT)
    assert guidance.gimbal_angles(me, 0.0, (10.0, 0.0, ALT)) == pytest.approx((0.0, 0.0))
    assert guidance.gimbal_angles(me, 0.0, (0.0, 10.0, ALT)) == pytest.approx((0.0, 90.0))
    # Target 10 m away and 10 m lower, while we face east: look 45 down and to the left.
    assert guidance.gimbal_angles(me, math.pi / 2, (10.0, 0.0, ALT + 10)) == pytest.approx(
        (-45.0, -90.0))


# --- config -----------------------------------------------------------------

def test_intercept_config_defaults_and_overrides():
    drones = [{'id': 0}, {'id': 1}]
    assert parse_config({'drones': drones}).intercept == InterceptConfig()
    config = parse_config({'drones': drones,
                           'intercept': {'track_timeout': 5, 'use_gimbal': False}})
    assert config.intercept.track_timeout == 5.0 and config.intercept.use_gimbal is False
    with pytest.raises(ValueError):
        parse_config({'drones': drones, 'intercept': {'standoff': 15}})  # the old chase's
    with pytest.raises(ValueError):
        parse_config({'drones': drones, 'intercept': {'interceptor_id': 1, 'hostile_id': 1}})
