import json
import sys
from pathlib import Path

import pytest

from swarm_backend.ui.replay import Recorder, ReplayUi, load_recording, main

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


def test_at_is_the_last_state_at_or_before_t(tmp_path):
    write(tmp_path / 'ui.jsonl', [line(10.0, 0.0), line(10.2, 1.0), line(10.4, 2.0)])
    ui = ReplayUi(load_recording(tmp_path / 'ui.jsonl'))
    north = lambda sim: ui.at(sim)['state']['drones'][0]['north']  # noqa: E731
    assert north(10.3) == 1.0
    assert north(10.4) == 2.0
    assert north(5.0) == 0.0  # before: the first state
    assert north(99.0) == 2.0  # after: the last


def test_events_are_on_sim_time_from_the_start(tmp_path, capsys):
    e1 = {'id': 1, 't': 400.0, 'text': 'a'}
    e2 = {'id': 2, 't': 401.0, 'text': 'b', 'detail': 'why'}
    write(tmp_path / 'ui.jsonl', [line(10.0, 0, [e1]), line(12.5, 0, [e2])])
    ui = ReplayUi(load_recording(tmp_path / 'ui.jsonl'))
    assert [(sim, t, e['id']) for sim, t, e in ui.event_table()] == [(10.0, 0.0, 1), (12.5, 2.5, 2)]
    assert main([str(tmp_path)]) == 0  # the folder: its ui.jsonl
    out = capsys.readouterr().out
    assert '12.5' in out and 'b  (why)' in out


def test_load_skips_a_torn_line_and_keeps_only_the_last_gazebo_run(tmp_path):
    path = tmp_path / 'ui.jsonl'
    write(path, [line(300.0, 0), line(300.2, 0), line(0.5, 7), line(0.7, 8)])
    with open(path, 'a') as fh:
        fh.write('{"sim": 0.9, "sta')
    lines = load_recording(path)
    assert [x['sim'] for x in lines] == [0.5, 0.7]


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
