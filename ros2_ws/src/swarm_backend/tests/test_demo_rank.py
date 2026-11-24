import json

import pytest

from swarm_backend.demo.fake_run import make_run
from swarm_backend.demo.shoot import expand, load_shoot, write_plan
from swarm_backend.demo.story import StoryError, pick_take
from swarm_backend.demo.takes import find_takes, rank, shoot_takes


def make_shoot(tmp_path, runs):
    """runs: {run id: make_run kwargs}; a shoot.json listing them in order."""
    folder = tmp_path / 'shoot'
    for rid, kw in runs.items():
        make_run(folder / rid, videos=False, **kw)
    (folder / 'shoot.json').write_text(json.dumps(
        {'yaml': '', 'runs': [{'id': rid, 'status': 'done'} for rid in runs]}))
    return folder


def test_shoot_takes_names(tmp_path):
    folder = make_shoot(tmp_path, {'a-1': {'takes': 2}, 'a-2': {'takes': 2}})
    takes = shoot_takes(folder)
    assert [t.name for t in takes] == ['a-1/1', 'a-1/2', 'a-2/1', 'a-2/2']
    assert takes[2].folder == folder / 'a-2'
    assert [t.name for t in shoot_takes(folder / 'a-1')] == ['1', '2']


def test_rank_order(tmp_path):
    folder = make_shoot(tmp_path, {'a-1': {'takes': 2, 'hit': [False, True]}})
    ranked = rank(shoot_takes(folder))
    assert ranked[0][0].name == 'a-1/2'
    assert any('hit' in r for r in ranked[0][2])


def test_best_names(tmp_path):
    folder = make_shoot(tmp_path, {'a-1': {'takes': 2, 'hit': [False, True]}})
    takes = shoot_takes(folder)
    assert pick_take('best', takes).name == 'a-1/2'
    assert pick_take('a-1/1', takes).name == 'a-1/1'
    with pytest.raises(StoryError, match='only 1'):
        pick_take('best2', takes)


def test_in_frame_frac(tmp_path):
    make_run(tmp_path, takes=1)
    f = find_takes(tmp_path)[0].facts['in_frame_frac']
    assert 0.5 <= f <= 1.0
    make_run(tmp_path / 'novideo', takes=1, videos=False)
    assert find_takes(tmp_path / 'novideo')[0].facts['in_frame_frac'] is None
