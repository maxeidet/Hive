import json
import sys
from pathlib import Path

import pytest

from swarm_backend.ui.replay import (Recorder, ReplayUi, frame_times, load_recording,
                                     sim_query)

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / 'recording'))
import compose  # noqa: E402  (recording/compose.py)


def fake_state(n_events_after, events):
    return {'drones': [{'id': 1}], 'map': {'big': True}, 'uptime_s': 99.0,
            'search_area': {}, 'events': [e for e in events if e['id'] > n_events_after]}


def test_recorder_writes_new_events_only_and_drops_the_map(tmp_path):
    events = []
    rec = Recorder(tmp_path / 'run' / 'ui.jsonl', lambda after: fake_state(after, events))
    assert not rec.record(None, 1.0)  # no sim time yet: nothing written
    events.append({'id': 1, 't': 50.0, 'text': 'Drone 1 link up'})
    assert rec.record(10.0, 1000.0)
    assert rec.record(10.1, 1000.1)
    rec.close()
    lines = [json.loads(line) for line in (tmp_path / 'run' / 'ui.jsonl').read_text().splitlines()]
    assert [line['sim'] for line in lines] == [10.0, 10.1]
    assert 'map' not in lines[0]['state']
    assert [e['id'] for e in lines[0]['state']['events']] == [1]
    assert lines[1]['state']['events'] == []


def write(path, lines):
    path.write_text(''.join(json.dumps(line) + '\n' for line in lines))


def line(sim, drone_north, events=(), wall=None):
    return {'sim': sim, 'wall': wall, 'state': {'drones': [{'id': 1, 'north': drone_north}],
                                                 'events': list(events), 'uptime_s': 5.0}}


def test_replay_answers_the_last_state_at_or_before_t(tmp_path):
    write(tmp_path / 'ui.jsonl', [line(10.0, 0.0), line(10.2, 1.0), line(10.4, 2.0)])
    ui = ReplayUi(load_recording(tmp_path / 'ui.jsonl'))
    assert ui.state(sim=10.3)['drones'][0]['north'] == 1.0
    assert ui.state(sim=10.4)['drones'][0]['north'] == 2.0
    assert ui.state(sim=5.0)['drones'][0]['north'] == 0.0  # before: the first state
    assert ui.state(sim=99.0)['drones'][0]['north'] == 2.0  # after: the last
    assert ui.state(sim=10.3)['uptime_s'] == 0.3  # T+ on sim time from the start


def test_replay_events_appear_at_their_sim_time_with_t_on_sim_time(tmp_path):
    e1 = {'id': 1, 't': 400.0, 'text': 'a'}
    e2 = {'id': 2, 't': 401.0, 'text': 'b'}
    write(tmp_path / 'ui.jsonl', [line(10.0, 0, [e1]), line(12.5, 0, [e2])])
    ui = ReplayUi(load_recording(tmp_path / 'ui.jsonl'))
    assert [e['id'] for e in ui.state(sim=12.0)['events']] == [1]
    events = ui.state(sim=13.0)['events']
    assert [(e['id'], e['t']) for e in events] == [(1, 0.0), (2, 2.5)]
    assert [e['id'] for e in ui.state(events_after=1, sim=13.0)['events']] == [2]
    assert [row[0] for row in ui.event_table()] == [10.0, 12.5]


def test_load_skips_a_torn_line_and_keeps_only_the_last_gazebo_run(tmp_path):
    path = tmp_path / 'ui.jsonl'
    write(path, [line(300.0, 0), line(300.2, 0), line(0.5, 7), line(0.7, 8)])
    with open(path, 'a') as fh:
        fh.write('{"sim": 0.9, "sta')
    lines = load_recording(path)
    assert [x['sim'] for x in lines] == [0.5, 0.7]


def test_wall_clock_follows_sim_between_samples(tmp_path):
    write(tmp_path / 'ui.jsonl', [line(10.0, 0, wall=1000.0), line(11.0, 0, wall=1003.0)])
    ui = ReplayUi(load_recording(tmp_path / 'ui.jsonl'))
    assert ui.wall_at(10.5) == pytest.approx(1000.5)
    assert ui.wall_at(11.0) == pytest.approx(1003.0)


def test_sim_query():
    assert sim_query('/api/state?events_after=3&t=312.480') == pytest.approx(312.48)
    assert sim_query('/api/state?events_after=3') is None
    assert sim_query('/api/state?t=abc') is None


def test_frame_times_match_the_gazebo_sim_time_rule():
    times = frame_times(312.0, 2.0, 25)
    assert len(times) == 50
    assert times[0] == 312.0 and times[1] == pytest.approx(312.04)
    assert times[-1] == pytest.approx(312.0 + 49 / 25)
    with pytest.raises(ValueError):
        frame_times(0, 0, 25)


def test_compose_trims_the_earlier_video():
    assert compose.trims(0.0, 60, 50) == (0.0, 0.0, 50)
    assert compose.trims(1.5, 60, 50) == (1.5, 0.0, 50)  # Gazebo starts later: cut the UI
    assert compose.trims(-2.0, 60, 50) == (0.0, 2.0, 48)
    with pytest.raises(ValueError):
        compose.trims(70, 60, 50)


def test_compose_layouts_build():
    for layout in ('pip', 'side', 'stack'):
        assert '[out]' in compose.filter_graph(layout, 25, 760, 'br', 720, 1280)
    with pytest.raises(ValueError):
        compose.filter_graph('grid', 25, 760, 'br', 720, 1280)
