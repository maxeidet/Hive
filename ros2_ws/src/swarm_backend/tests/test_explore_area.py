"""Unit tests for swarm_backend/explore/area.py (no ROS needed)."""
import json

import pytest

from swarm_backend.explore.area import (
    DONE, EXPLORING, FAILED, FLYING_TO_AREA, IDLE, LANDING, STOPPING, TAKING_OFF, Box, Progress,
    Status, box_around, last_coverage, parse_command)


def test_circle_becomes_the_square_around_it_in_enu():
    box = box_around({'circle': {'centre': [-14.0, 13.5], 'radius_m': 7.0}})
    assert box.center == (13.5, -14.0)  # x east, y north
    assert box.half == (7.0, 7.0)
    assert box.launch_args() == ['center:=13.5,-14', 'radius:=7,7', 'z_min:=0.5', 'z_max:=5',
                                 'alt:=4']


def test_polygon_becomes_its_bounding_box():
    # North 0..10, east -4..8: a triangle.
    box = box_around({'points': [[0, -4], [10, 2], [0, 8]]}, alt=23, z_min=2, z_max=24)
    assert box.center == (2.0, 5.0)
    assert box.half == (6.0, 5.0)
    assert box.to_dict() == {'min': [-4.0, 0.0, 2.0], 'max': [8.0, 10.0, 24.0], 'alt': 23.0}


@pytest.mark.parametrize('area, heights', [
    ({}, {}),
    ({'points': [[0, 0], [5, 5]]}, {}),  # two corners
    ({'circle': {'centre': [0, 0]}}, {}),  # no radius
    ({'circle': {'centre': [0, 0], 'radius_m': 1}}, {}),  # too small for DAEP
    ({'circle': {'centre': [0, 0], 'radius_m': 6}}, {'alt': 8}),  # alt above z_max
    ({'circle': {'centre': [0, 0], 'radius_m': 6}}, {'z_min': -1}),
])
def test_bad_areas_and_heights_are_refused(area, heights):
    with pytest.raises(ValueError):
        box_around(area, **heights)


def test_commands():
    start = parse_command(json.dumps({'command': 'start', 'area': {
        'circle': {'centre': [0, 0], 'radius_m': 6}}, 'alt': 3, 'z_max': None}))
    assert start.kind == 'start' and start.box.alt == 3 and start.box.z_max == 5.0
    assert parse_command('{"command": "stop"}').kind == 'stop'
    for bad in ('not json', '[]', '{"command": "fly"}', '{"command": "start"}'):
        with pytest.raises(ValueError):
            parse_command(bad)


# Lines as the launch prints them (colours removed), in the order a run prints them.
RUN = [
    '[daep_bridge-8] [INFO] [1.0] [daep.px4_bridge]: Taking off to 4 m',
    '[daep_bridge-8] [INFO] [2.0] [daep.px4_bridge]: Flying to the area at (13.5, -14)',
    '[daep_bridge-8] [INFO] [3.0] [daep.px4_bridge]: In the air: publishing the pose and taking goals from DAEP',
    '[rpl_exploration-7] [WARN] [4.0] [daep.rpl_exploration]: Coverage 95.1 % reached the threshold (95.0 %), exploration complete!',
    '[daep_bridge-8] [INFO] [4.1] [daep.px4_bridge]: Landing',
    '[save_map-9] [INFO] [4.5] [daep.save_map]: Map saved: /workspace/ros2_ws/data/map.bt (48 kB)',
    '[INFO] [save_map-9]: process has finished cleanly [pid 1]',
    '[daep_bridge-8] [INFO] [9.0] [daep.px4_bridge.drone0]: disarmed',
]


def test_progress_follows_a_run():
    p = Progress(Status(state='starting'))
    states = []
    for line in RUN:
        p.line(line)
        states.append(p.status.state)
    assert states[:4] == [TAKING_OFF, FLYING_TO_AREA, EXPLORING, LANDING]
    assert p.status.map_path == '/workspace/ros2_ws/data/map.bt'
    assert p.finished_flying
    assert not Progress(Status(state=LANDING)).finished_flying


def test_a_node_dying_fails_the_run_unless_we_stop_it():
    died = '[ERROR] [pig.py-2]: process has died [pid 1149, exit code 1, cmd ...]'
    p = Progress(Status(state=EXPLORING))
    p.line(died)
    assert p.status.state == FAILED and 'pig.py' in p.status.detail
    p.line(RUN[2])  # nothing moves a failed run on
    assert p.status.state == FAILED
    stopping = Progress(Status(state=STOPPING))
    stopping.line(died)
    stopping.line(RUN[-1])
    assert stopping.status.state == STOPPING and stopping.landed
    for finished in (IDLE, DONE):
        q = Progress(Status(state=finished))
        q.line(RUN[0])
        assert q.status.state == finished


def test_status_json_and_coverage():
    box = Box((1.0, 2.0), (6.0, 6.0), 0.5, 5.0, 4.0)
    data = json.loads(Status(EXPLORING, box=box, coverage_pct=80.3, run=2).to_json())
    assert data == {'state': EXPLORING, 'detail': '', 'run': 2, 'coverage_pct': 80.3,
                    'map': None, 'box': {'min': [-5.0, -4.0, 0.5], 'max': [7.0, 8.0, 5.0],
                                         'alt': 4.0}}
    csv = 'Time, Coverage (m3), Coverage (%), Free\n0, 0, 0, 0\n15.2, 280.4, 77.887, 280\n'
    assert last_coverage(csv) == 77.9
    assert last_coverage('Time, Coverage (m3), Coverage (%)\n') is None
