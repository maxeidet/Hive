import math

import pytest

from swarm_backend.ui.model import SearchArea, drone_view, update_search_area


def yaw_q(deg: float) -> tuple[float, float, float, float]:
    half = math.radians(deg) / 2
    return (math.cos(half), 0.0, 0.0, math.sin(half))


AREA = SearchArea(heading_deg=0.0, width_deg=90.0, radius_m=100.0)


def test_update_changes_only_given_fields_and_counts_revisions():
    area = update_search_area(AREA, {'heading_deg': 45})
    assert (area.heading_deg, area.width_deg, area.radius_m) == (45.0, 90.0, 100.0)
    assert area.revision == AREA.revision + 1
    assert area.centre == AREA.centre


def test_heading_wraps_into_0_360():
    assert update_search_area(AREA, {'heading_deg': -30}).heading_deg == 330.0
    assert update_search_area(AREA, {'heading_deg': 400}).heading_deg == 40.0


@pytest.mark.parametrize('data', [
    {'width_deg': 5}, {'width_deg': 200}, {'radius_m': 1}, {'radius_m': 1e6},
    {'heading_deg': 'north'}, {'heading_deg': float('nan')}, {'centre': [1, 2]}, {'speed': 3},
])
def test_rejects_bad_requests(data):
    with pytest.raises(ValueError):
        update_search_area(AREA, data)


def test_contains_is_the_arc():
    area = SearchArea(heading_deg=90.0, width_deg=90.0, radius_m=100.0)  # facing east
    assert area.contains(0.0, 50.0)
    assert area.contains(40.0, 50.0)  # 39 deg off the heading, inside ±45
    assert not area.contains(60.0, 50.0)  # 50 deg off
    assert not area.contains(0.0, 150.0)  # too far
    assert not area.contains(0.0, -50.0)  # behind the base


def test_contains_across_north():
    area = SearchArea(heading_deg=350.0, width_deg=40.0, radius_m=100.0)
    assert area.contains(50.0, 5.0)  # bearing about 6 deg


def test_drone_view_before_px4_has_sent_anything():
    view = drone_view(1, None, None, None, None, False, None, None, None)
    assert view['north'] is None and view['gimbal_heading_deg'] is None and view['age_s'] is None


def test_drone_view_position_heading_and_gimbal_relative_to_nose():
    view = drone_view(2, (10.0, -5.0, -20.0), (3.0, 4.0, -1.0), yaw_q(90.0), yaw_q(-30.0),
                      False, True, 'OFFBOARD', 0.123)
    assert (view['north'], view['east'], view['altitude']) == (10.0, -5.0, 20.0)
    assert view['speed'] == 5.0
    assert view['heading_deg'] == pytest.approx(90.0)
    assert view['gimbal_heading_deg'] == pytest.approx(60.0)  # nose 90 - 30
    assert view['age_s'] == 0.12


def test_drone_view_gimbal_in_earth_frame_ignores_the_nose():
    view = drone_view(2, None, None, yaw_q(90.0), yaw_q(-30.0), True, True, None, 0.0)
    assert view['gimbal_heading_deg'] == pytest.approx(330.0)


def test_drone_view_relative_gimbal_needs_the_drone_heading():
    view = drone_view(2, None, None, None, yaw_q(10.0), False, None, None, None)
    assert view['gimbal_heading_deg'] is None
    assert view['gimbal_pitch_deg'] == pytest.approx(0.0)
