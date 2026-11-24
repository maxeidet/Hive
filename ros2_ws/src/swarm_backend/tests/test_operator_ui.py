import math

from pathlib import Path

import pytest

from swarm_backend.ui.model import (EventLog, SearchArea, attack_view, drone_home, sdf_origin,
                                   drone_status, drone_view, latlon_offset_m, load_map,
                                   update_search_area, world_origin)


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


# PX4's local frame origins as the drones on gish reported them (hive_base, drones 0 and 2).
WORLD = (47.397971057728974, 8.546163739800146)  # hive_base.sdf spherical_coordinates


def ref_at(north, east, origin=WORLD):
    """The (lat, lon) north/east metres from origin, the inverse of latlon_offset_m."""
    lat = origin[0] + math.degrees(north / 6371000.0)
    lon = origin[1] + math.degrees(east / (6371000.0 * math.cos(math.radians(origin[0]))))
    return lat, lon


def test_latlon_offset_matches_px4_on_gish():
    # Drone 2 spawns at N -4, E +1; its PX4 reported this ref, drone 0 (at the origin) the other.
    north, east = latlon_offset_m(47.39793503195908, 8.546176965634066,
                                  47.39797151760928, 8.546163919449194)
    assert north == pytest.approx(-4.0, abs=0.1)
    assert east == pytest.approx(1.0, abs=0.1)


def test_a_moved_hostile_is_placed_where_its_px4_started():
    spawns = {0: (0.0, 0.0, 0.0), 2: (-4.0, 1.0, 0.0), 4: (150.0, 0.0, 0.0)}
    refs = {0: ref_at(0, 0), 2: ref_at(-4, 1), 4: ref_at(210.4, -181.3)}  # 4 was moved
    origin = world_origin(refs, spawns, skip=(4,))
    assert latlon_offset_m(*origin, *WORLD) == pytest.approx((0.0, 0.0), abs=1e-6)
    home = drone_home(spawns[4], refs[4], origin)
    assert home[:2] == pytest.approx((210.4, -181.3), abs=1e-3)
    assert drone_home(spawns[2], refs[2], origin) == pytest.approx((-4.0, 1.0, 0.0), abs=1e-3)


def test_one_moved_drone_does_not_shift_the_origin():
    spawns = {i: (float(i), 0.0, 0.0) for i in range(4)}
    refs = {i: ref_at(i, 0) for i in range(4)}
    refs[3] = ref_at(500, 500)
    origin = world_origin(refs, spawns)
    assert latlon_offset_m(*origin, *WORLD) == pytest.approx((0.0, 0.0), abs=1e-6)


def test_spawn_point_until_px4_reports_its_origin():
    assert drone_home((150.0, 0.0, 0.0), None, WORLD) == (150.0, 0.0, 0.0)
    assert drone_home((150.0, 0.0, 0.0), ref_at(1, 1), None) == (150.0, 0.0, 0.0)
    assert world_origin({4: ref_at(0, 0)}, {4: (0.0, 0.0, 0.0)}, skip=(4,)) is None


# --- event log ---------------------------------------------------------------------------

def view(drone_id=2, status='searching', armed=True, detection=None, attack=None, hostile=False,
         age_s=0.1):
    return {'id': drone_id, 'status': status, 'armed': armed, 'detection': detection,
            'attack': attack, 'hostile': hostile, 'age_s': age_s}


def texts(log):
    return [e.text for e in log.events]


def test_events_link_up_lost_and_back():
    log = EventLog()
    log.update(0.0, [view(), view(0, 'flying', hostile=True)])
    assert texts(log) == ['Drone 2 link up', 'Drone 0 link up']
    log.update(1.0, [view(status='no link', age_s=3.0)])
    log.update(2.0, [view()])
    assert texts(log)[2:] == ['Drone 2 link lost', 'Drone 2 link back']
    assert log.events[2].level == 'bad' and log.events[2].kind == 'fleet'


def test_nothing_new_nothing_logged():
    log = EventLog()
    for t in range(5):
        log.update(float(t), [view()])
    assert texts(log) == ['Drone 2 link up']


def test_events_of_an_attack_from_detection_to_hit():
    log = EventLog()
    target = {'north': 50.0, 'east': 0.0, 'altitude': 30.0}
    log.update(0.0, [view()])
    log.update(10.0, [view(detection=target)])
    log.update(10.2, [view(status='intercept', detection=target, attack={'range_m': 48.4})])
    log.update(14.0, [view(status='target lost', attack={'seen_s_ago': 1.5})])
    log.update(15.0, [view(status='intercept', detection=target, attack={'range_m': 9.0})])
    log.update(17.5, [view(status='hit', detection=target)])
    log.update(21.0, [view()])
    assert texts(log)[1:] == [
        'Drone 2 sees the target', 'Drone 2 searching → intercept',
        'Drone 2 lost sight of the target',
        'Drone 2 lost the target', 'Drone 2 sees the target', 'Drone 2 target lost → intercept',
        'Drone 2 hit the target', 'Drone 2 lost sight of the target', 'Drone 2 back to searching']
    hit = log.events[7]
    assert hit.kind == 'attack' and hit.level == 'good'
    assert hit.detail == '7.5 s after it first saw it'  # counted from the first sighting at 10 s
    assert log.events[2].detail == 'range 48 m'


def test_flight_events():
    log = EventLog()
    log.update(0.0, [view(status='landed', armed=False)])
    log.update(1.0, [view(status='taking off')])
    log.update(9.0, [view(status='landing')])
    log.update(12.0, [view(status='landed', armed=False)])
    assert texts(log)[1:] == ['Drone 2 armed', 'Drone 2 taking off', 'Drone 2 landing',
                              'Drone 2 landed and disarmed']


def test_after_returns_newer_events_and_the_log_is_bounded():
    log = EventLog(size=3)
    for i in range(5):
        log.add(float(i), 'operator', f'event {i}')
    assert [e['text'] for e in log.after(0)] == ['event 2', 'event 3', 'event 4']
    assert [e['id'] for e in log.after(4)] == [5]
    assert log.after(5) == []


@pytest.mark.parametrize('path,expected', [
    ('/api/state', 0), ('/api/state?events_after=12', 12), ('/api/state?events_after=x', 0),
    ('/api/state?events_after=-3', 0)])
def test_events_after_query(path, expected):
    pytest.importorskip('yaml')  # operator_ui imports the config loader
    from swarm_backend.nodes.operator_ui import events_after
    assert events_after(path) == expected


# The world's own origin (<spherical_coordinates>), so positions do not depend on the
# spawn points in the config the UI was started with.

HIVE_BASE_SDF = Path(__file__).resolve().parents[4] / 'worlds' / 'hive_base.sdf'


def sdf(inner: str) -> str:
    return f'<sdf version="1.9"><world name="w">{inner}</world></sdf>'


def test_sdf_origin_of_hive_base():
    assert sdf_origin(HIVE_BASE_SDF.read_text()) == pytest.approx((47.397971057728974, 8.546163739800146))


def test_sdf_origin_missing_turned_or_broken():
    assert sdf_origin(sdf('<gravity>0 0 -9.8</gravity>')) is None
    turned = ('<spherical_coordinates><latitude_deg>47</latitude_deg><longitude_deg>8</longitude_deg>'
              '<heading_deg>90</heading_deg></spherical_coordinates>')
    assert sdf_origin(sdf(turned)) is None
    assert sdf_origin('<sdf><world') is None
    plain = '<spherical_coordinates><latitude_deg>47</latitude_deg><longitude_deg>8</longitude_deg></spherical_coordinates>'
    assert sdf_origin(sdf(plain)) == (47.0, 8.0)


def test_drones_placed_right_whatever_spawns_the_config_has():
    """The bug: defenders started below their slots (at_slots), UI started with
    config/swarm.yaml's spawn points. The estimate from those is off by 150 m; the
    world's origin puts every drone where Gazebo has it."""
    true_spawn = {1: (129.904, -75.0, 0.0), 2: (150.0, 0.0, 0.0), 3: (129.904, 75.0, 0.0)}
    refs = {i: ref_at(n, e) for i, (n, e, _) in true_spawn.items()}
    wrong_config = {1: (0.0, 5.0, 0.0), 2: (5.0, 0.0, 0.0), 3: (0.0, 0.0, 0.0)}
    estimate = world_origin(refs, wrong_config)
    off = drone_home(wrong_config[2], refs[2], estimate)
    assert abs(off[0] - 150.0) > 100  # the old way: far off
    for i, spawn in true_spawn.items():
        assert drone_home(wrong_config[i], refs[i], WORLD) == pytest.approx(spawn, abs=1e-3)
