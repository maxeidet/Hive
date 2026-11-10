import math

import pytest

from swarm_backend.ui.model import (SearchArea, attack_view, drone_status, drone_view,
                                   load_map, update_search_area)


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


@pytest.mark.parametrize('attack_state, armed, nav_state, age_s, expected', [
    (None, None, None, None, 'no link'),
    ('attack', True, 'OFFBOARD', 5.0, 'no link'),  # stale beats everything
    ('search', True, 'OFFBOARD', 0.1, 'searching'),
    ('attack', True, 'OFFBOARD', 0.1, 'intercept'),
    ('coast', True, 'OFFBOARD', 0.1, 'intercept'),
    ('lost', True, 'OFFBOARD', 0.1, 'target lost'),
    ('hit', False, 'AUTO_LOITER', 0.1, 'hit'),
    ('idle', True, 'AUTO_TAKEOFF', 0.1, 'taking off'),  # idle falls back to PX4
    (None, False, 'AUTO_LOITER', 0.1, 'landed'),
    (None, True, 'AUTO_RTL', 0.1, 'returning'),
    (None, True, 'AUTO_LAND', 0.1, 'landing'),
    (None, True, 'OFFBOARD', 0.1, 'flying'),
])
def test_drone_status(attack_state, armed, nav_state, age_s, expected):
    assert drone_status(attack_state, armed, nav_state, age_s, stale_s=2.0) == expected


def test_attack_view_turns_ned_into_what_the_map_draws():
    view = attack_view('{"state": "attack", "target": [10, 5, -30], "velocity": [3, 4, -1], '
                       '"range_m": 12.0}')
    assert view['target'] == {'north': 10, 'east': 5, 'altitude': 30}
    assert view['velocity'] == {'north': 3, 'east': 4, 'up': 1}
    assert view['range_m'] == 12.0


@pytest.mark.parametrize('raw', ['', 'not json', '[1, 2]', 'null'])
def test_attack_view_ignores_garbage(raw):
    assert attack_view(raw) is None


def test_attack_view_without_a_target():
    view = attack_view('{"state": "search", "target": null, "velocity": null}')
    assert view['target'] is None and view['velocity'] is None


def test_load_map_needs_both_files_and_a_valid_extent(tmp_path):
    assert load_map(tmp_path) is None
    (tmp_path / 'map.png').write_bytes(b'png')
    assert load_map(tmp_path) is None  # no map.json
    (tmp_path / 'map.json').write_text('{"north_min": 10, "north_max": -10, '
                                       '"east_min": -10, "east_max": 10}')
    assert load_map(tmp_path) is None  # north upside down
    (tmp_path / 'map.json').write_text('{"world": "w", "north_min": -10, "north_max": 10, '
                                       '"east_min": -20, "east_max": 20}')
    info = load_map(tmp_path)
    assert info['world'] == 'w' and info['east_min'] == -20.0 and 'version' in info
    (tmp_path / 'map.json').write_text('not json')
    assert load_map(tmp_path) is None
