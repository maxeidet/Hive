"""recording/stories/examiner.yaml cuts the examiner shoot (recording/shoots/examiner.yaml)."""
import json
from pathlib import Path

import yaml

from swarm_backend.demo.fake_run import T0, TAKE_S, _clip, make_run
from swarm_backend.demo.story import load_story, resolve
from swarm_backend.demo.takes import shoot_takes

REPO = Path(__file__).resolve().parents[4]
STORY = REPO / 'recording' / 'stories' / 'examiner.yaml'
SHOOT = REPO / 'recording' / 'shoots' / 'examiner.yaml'


def fake_examiner_shoot(tmp_path):
    folder, runs = tmp_path / 'examiner', []
    rid = 'overview-1'
    make_run(folder / rid, takes=1, videos=False)
    info = json.loads((folder / rid / 'run.json').read_text())
    info['command'] = ['idle', '60']
    (folder / rid / 'run.json').write_text(json.dumps(info))
    _clip(folder / rid, 'cam_base_orbit', T0, T0 + TAKE_S,
          {'drone': 0, 'hfov': 1.2, 'model': 'rec_cam_base_orbit', 'angle': 'base_orbit'})
    runs.append({'id': rid, 'status': 'done'})
    for entry, kw in (('head-on', {}), ('zigzag', {}), ('birds', {'birds_late': True}),
                      ('two-hostiles', {'hostiles': 2})):
        for side, setup in (('baseline', 'baseline'), ('ours', 'yolo-bytetrack-mpc')):
            rid = f'{entry}-1-{side}'
            make_run(folder / rid, videos=True, **kw)
            info = json.loads((folder / rid / 'run.json').read_text())
            info['setup'], info['options']['pair'] = setup, f'{entry}-1'
            (folder / rid / 'run.json').write_text(json.dumps(info))
            runs.append({'id': rid, 'status': 'done'})
    (folder / 'shoot.json').write_text(json.dumps({'yaml': str(SHOOT), 'runs': runs}))
    return folder


def test_examiner_shoot_file():
    entries = yaml.safe_load(SHOOT.read_text())
    assert [e['name'] for e in entries] == ['overview', 'head-on', 'zigzag', 'birds', 'two-hostiles']
    for e in entries[1:]:
        assert e['compare'] == ['baseline', 'yolo-bytetrack-mpc'] and str(e['seeds']) == '0'
    assert '--birds 8' in entries[3]['run'] and '--hostiles 2' in entries[4]['run']
    assert entries[4]['config'].endswith('scenarios/surveillance2.yaml')


def test_examiner_story_resolves(tmp_path):
    folder = fake_examiner_shoot(tmp_path)
    scenes, problems = resolve(load_story(STORY), shoot_takes(folder))
    assert not problems
    assert {r.take.pair for r in scenes if r.take is not None and r.take.pair} >= {
        'head-on-1', 'zigzag-1', 'birds-1', 'two-hostiles-1'}
