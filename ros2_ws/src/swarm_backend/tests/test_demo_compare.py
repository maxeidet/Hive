import json
import subprocess

import pytest

from swarm_backend.demo.fake_run import make_run
from swarm_backend.demo.final import build
from swarm_backend.demo.story import load_story, resolve
from swarm_backend.demo.takes import shoot_takes


def pair_shoot(tmp_path, base_hit=True, ours_hit=True):
    folder = tmp_path / 'shoot'
    runs = []
    for rid, setup, hit in (('h-1-baseline', 'baseline', base_hit), ('h-1-ours', 'yolo-bytetrack-mpc', ours_hit)):
        make_run(folder / rid, takes=1, hit=[hit], videos=True)
        info = json.loads((folder / rid / 'run.json').read_text())
        info['setup'] = setup
        info['options']['pair'] = 'h-1'
        (folder / rid / 'run.json').write_text(json.dumps(info))
        runs.append({'id': rid, 'status': 'done'})
    (folder / 'shoot.json').write_text(json.dumps({'yaml': '', 'runs': runs}))
    return folder


def story(tmp_path, text):
    p = tmp_path / 'c.yaml'
    p.write_text(text)
    return p


SCENE = '''scenes:
- caption: Head-on
  take: pair:h-1
  from: release + 8
  to: release + 12
  layout: compare
  show: [chase]
'''


def test_compare_synced_on_release(tmp_path):
    folder = pair_shoot(tmp_path)
    takes = shoot_takes(folder)
    scenes, problems = resolve(load_story(story(tmp_path, SCENE)), takes)
    assert problems == []
    ours, base = scenes
    assert (ours.side, base.side) == ('ours', 'baseline')
    assert ours.start - ours.take.moments['release'] == pytest.approx(
        base.start - base.take.moments['release'])
    assert ours.take.folder.name == 'h-1-ours' and base.take.folder.name == 'h-1-baseline'


def test_compare_side_without_hit(tmp_path, capsys):
    folder = pair_shoot(tmp_path, base_hit=False)
    out = build(folder, story(tmp_path, SCENE.replace('release + 12', 'hit + 1')), preview=True)
    assert out.exists()
    assert 'no hit' in capsys.readouterr().out


def probe_len(path):
    return float(subprocess.run(['ffprobe', '-v', 'error', '-show_entries', 'format=duration',
                                 '-of', 'csv=p=0', str(path)], capture_output=True, text=True).stdout)


def test_compare_preview_builds(tmp_path):
    folder = pair_shoot(tmp_path)
    out = build(folder, story(tmp_path, 'end_card: compare\n' + SCENE), preview=True)
    assert probe_len(out) == pytest.approx(4 + 3, abs=0.15)


def test_best_pair(tmp_path):
    folder = pair_shoot(tmp_path)
    scenes, problems = resolve(load_story(story(tmp_path, SCENE.replace('pair:h-1', 'best_pair'))),
                               shoot_takes(folder))
    assert problems == [] and scenes[0].take.pair == 'h-1'


def test_compare_story_builds(tmp_path):
    from pathlib import Path
    folder = pair_shoot(tmp_path)
    story_file = Path(__file__).resolve().parents[4] / 'recording' / 'stories' / 'compare.yaml'
    out = build(folder, story_file, preview=True, skip_missing=True)
    assert out.exists()
