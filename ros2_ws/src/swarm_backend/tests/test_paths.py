"""The hostile drone's way in (paths.inbound): straight, weave, zigzag."""
import math

import pytest

from swarm_backend.intercept.paths import inbound, polyline_length

START, GOAL = (226.0, 130.0, -30.0), (0.0, 0.0, -30.0)


def _side(p):
    """Signed distance of p from the straight START-GOAL line."""
    dx, dy = GOAL[0] - START[0], GOAL[1] - START[1]
    return ((p[0] - START[0]) * dy - (p[1] - START[1]) * dx) / math.hypot(dx, dy)


def test_straight_is_the_line():
    assert inbound(START, GOAL) == [START, GOAL]


@pytest.mark.parametrize('kind', ['weave', 'zigzag'])
def test_curved_paths_start_and_end_on_the_line_and_swing_by_the_amplitude(kind):
    points = inbound(START, GOAL, kind, amplitude=15.0, wavelength=60.0)
    assert points[0] == pytest.approx(START) and points[-1] == pytest.approx(GOAL)
    assert all(p[2] == -30.0 for p in points)  # level, at the start's altitude
    sides = [_side(p) for p in points]
    assert max(sides) == pytest.approx(15.0, abs=0.2)
    assert min(sides) == pytest.approx(-15.0, abs=0.2)
    assert polyline_length(points) > math.dist(START, GOAL) * 1.05  # really not straight


def test_bad_paths_are_refused():
    with pytest.raises(ValueError):
        inbound(START, GOAL, 'spiral')
    with pytest.raises(ValueError):
        inbound(START, START, 'weave')
