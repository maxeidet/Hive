import math

import pytest

from swarm_backend.swarm.arc import arc_points, slot_angles, sectors


def test_three_slots_sit_in_the_middle_of_their_slices():
    assert slot_angles(3) == pytest.approx([-30, 0, 30])


def test_one_slot_is_straight_ahead():
    assert slot_angles(1) == [0.0]
    with pytest.raises(ValueError):
        slot_angles(0)


def test_arc_points_are_on_the_circle_and_the_middle_one_faces_the_threat():
    center = (5.0, -2.0, -20.0)
    points = arc_points(center, math.pi / 2, 80.0, 3)  # threat to the east
    assert all(math.dist(p, center) == pytest.approx(80.0) for p in points)
    assert all(p[2] == center[2] for p in points)
    assert points[1] == pytest.approx((5.0, 78.0, -20.0))
    assert points[0][0] > points[1][0] > points[2][0]  # -30 deg is to the left of east: north


def test_sectors_split_the_arc_evenly():
    assert sectors([0, 1, 2]) == {0: (-45, -15), 1: (-15, 15), 2: (15, 45)}


def test_losing_the_middle_drone_widens_the_two_others_to_45_degrees():
    assert sectors([0, 2]) == {0: (-45, 0), 2: (0, 45)}


def test_no_drones_no_sectors():
    assert sectors([]) == {}


def test_aim_bearing_points_at_the_sector_direction_from_the_base():
    from swarm_backend.swarm.arc import aim_bearing_deg
    base = (0.0, 0.0)
    # A drone at the middle slot (due north) aiming straight ahead sees due north.
    assert aim_bearing_deg((60.0, 0.0, -20.0), base, 0.0, 0.0, 90.0) == pytest.approx(0.0)
    # Aiming at +15 degrees from the base, from the side, is not a 15 degree turn.
    bearing = aim_bearing_deg((60.0, 0.0, -20.0), base, 0.0, 15.0, 90.0)
    assert bearing == pytest.approx(40.86, abs=0.01)
