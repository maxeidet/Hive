import math

import pytest

from swarm_backend.swarm.formation import (
    Formation, assign_targets, centroid, circle_radius_for_spacing, translate)

CENTER = (10.0, -3.0, -8.0)


def min_pair_distance(points):
    return min(math.dist(a, b) for i, a in enumerate(points) for b in points[i + 1:])


@pytest.mark.parametrize('n', [1, 2, 3, 4, 5, 9])
@pytest.mark.parametrize('shape', ['grid', 'line'])
def test_shapes_are_centred_spaced_and_level(n, shape):
    points = getattr(Formation(n), shape)(CENTER, 4.0)
    assert len(points) == n
    assert all(p[2] == CENTER[2] for p in points)
    if shape == 'line' or n in (1, 2, 4, 9):  # full grids are symmetric
        assert centroid(points) == pytest.approx(CENTER)
    if n > 1:
        assert min_pair_distance(points) == pytest.approx(4.0)


def test_grid_of_four_is_a_square():
    assert sorted(Formation(4).grid((0, 0, -10), 5)) == [
        (-2.5, -2.5, -10), (-2.5, 2.5, -10), (2.5, -2.5, -10), (2.5, 2.5, -10)]


def test_line_default_runs_east_west():
    points = Formation(3).line((0, 0, -5), 2)
    assert [p[0] for p in points] == pytest.approx([0, 0, 0])
    assert [p[1] for p in points] == pytest.approx([-2, 0, 2])


def test_circle_spacing():
    n, spacing = 6, 4.0
    points = Formation(n).circle(CENTER, circle_radius_for_spacing(n, spacing))
    assert centroid(points) == pytest.approx(CENTER)
    assert min_pair_distance(points) == pytest.approx(spacing)
    assert points[0][0] > CENTER[0]  # first drone due north


def test_assign_targets_avoids_swapping():
    positions = [(0, 0, 0), (0, 10, 0)]
    targets = [(0, 11, 0), (0, 1, 0)]
    assert assign_targets(positions, targets) == [(0, 1, 0), (0, 11, 0)]


def test_assign_targets_greedy_for_large_swarms():
    positions = [(float(i), 0.0, 0.0) for i in range(12)]
    targets = translate(positions, (0.0, 0.0, -5.0))[::-1]
    assert assign_targets(positions, targets) == translate(positions, (0.0, 0.0, -5.0))


def test_assign_targets_length_mismatch():
    with pytest.raises(ValueError):
        assign_targets([(0, 0, 0)], [])
