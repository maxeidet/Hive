import json
import subprocess

import pytest

from swarm_backend.demo.fake_run import make_run
from swarm_backend.demo.final import build, end_card_lines, needed_cuts
from swarm_backend.demo.story import Resolved, Scene
from swarm_backend.demo.takes import Take


def probe(path):
    out = subprocess.run(['ffprobe', '-v', 'error', '-show_entries',
                          'stream=width,height,codec_name,pix_fmt,r_frame_rate:format=duration',
                          '-of', 'json', str(path)], capture_output=True, text=True, check=True).stdout
    d = json.loads(out)
    return {**d['streams'][0], 'duration': float(d['format']['duration'])}


def story(tmp_path, text, name='t'):
    p = tmp_path / f'{name}.yaml'
    p.write_text(text)
    return p


def test_needed_cuts_union():
    sc = Scene(1, 'x', None, None, 'hit', 'hit', 'full', ['chase'])
    r = [Resolved(sc, None, 10.0, 20.0, {'chase': 'cam_attacker_watch'}),
         Resolved(sc, None, 15.0, 30.0, {'chase': 'cam_attacker_watch'})]
    assert needed_cuts(r) == {(None, 'cam_attacker_watch'): (9.0, 31.0)}


THREE = '''title: Test film
title_seconds: 2
scenes:
- caption: Map
  from: release
  to: release + 4
  layout: full
  show: [map]
- caption: Slow
  from: hit - 1
  to: hit
  layout: side
  show: [chase, yolo]
  speed: 0.5
- caption: Grid
  from: first_sighting
  to: first_sighting + 3
  layout: grid
  show: [chase, yolo, map, wide]
'''


def test_build_preview_lengths(tmp_path):
    run = tmp_path / 'run'
    make_run(run, takes=1)
    out = build(run, story(tmp_path, THREE), preview=True)
    info = probe(out)
    assert out.name == 'preview_t.mp4'
    assert (info['width'], info['height'], info['r_frame_rate']) == (960, 540, '25/1')
    assert info['duration'] == pytest.approx(2 + 4 + 2 + 3, abs=0.15)
    clips = sorted(p.name for p in (run / 'scenes' / 't' / 'preview').glob('*.mp4'))
    assert [c[:3] for c in clips] == ['01_', '02_', '03_', '04_']


def test_full_output_format(tmp_path):
    run = tmp_path / 'run'
    make_run(run, takes=1, videos=False)
    out = build(run, story(tmp_path, 'scenes:\n- caption: M\n  from: release\n  to: release + 1\n  layout: full\n  show: [map]\n'))
    info = probe(out)
    assert (info['width'], info['height'], info['codec_name'], info['pix_fmt']) == (1920, 1080, 'h264', 'yuv420p')
    data = out.read_bytes()
    assert 0 <= data.find(b'moov') < data.find(b'mdat')  # faststart


HIT_SCENE = '''scenes:
- caption: Before
  from: release
  to: release + 1
  layout: full
  show: [map]
- caption: The hit
  from: hit - 1
  to: hit
  layout: full
  show: [map]
'''


def test_skip_missing_builds_the_rest(tmp_path, capsys):
    run = tmp_path / 'run'
    make_run(run, takes=1, hit=[False], videos=False)
    p = story(tmp_path, HIT_SCENE)
    with pytest.raises(SystemExit) as e:
        build(run, p)
    assert e.value.code == 1
    assert "scene 2 'The hit'" in capsys.readouterr().out
    out = build(run, p, skip_missing=True)
    assert probe(out)['duration'] == pytest.approx(1.0, abs=0.15)


def test_missing_recording_is_reported(tmp_path, capsys):
    run = tmp_path / 'run'
    make_run(run, takes=1, videos=False)
    p = story(tmp_path, 'scenes:\n- caption: Wide\n  from: release\n  to: release + 1\n  layout: full\n  show: [wide]\n')
    with pytest.raises(SystemExit):
        build(run, p)
    assert 'cam_arc_side.mkv' in capsys.readouterr().out


def test_screen_too_late_is_reported(tmp_path, capsys):
    run = tmp_path / 'run'
    make_run(run, takes=1, videos=False, screen=True)
    p = story(tmp_path, 'scenes:\n- caption: UI\n  from: takeoff - 2\n  to: takeoff + 2\n  layout: full\n  show: [ui]\n')
    with pytest.raises(SystemExit):
        build(run, p)
    out = capsys.readouterr().out
    assert 'scene 1' in out and 'starts' in out and 'Traceback' not in out


def test_cache_reused(tmp_path):
    run = tmp_path / 'run'
    make_run(run, takes=1, videos=False)
    p = story(tmp_path, 'scenes:\n- caption: M\n  from: release\n  to: release + 1\n  layout: full\n  show: [map]\n')
    build(run, p, preview=True)
    cached = {f: f.stat().st_mtime_ns for f in (run / 'cache').rglob('*.mp4')}
    assert cached
    build(run, p, preview=True)
    assert {f: f.stat().st_mtime_ns for f in (run / 'cache').rglob('*.mp4')} == cached


def test_end_card_lines():
    takes = [Take(1, {}, 3, 'x', {'hit': True, 'sighting_to_hit_s': 3.0, 'sighting_range_m': 70.0}),
             Take(2, {}, 2, 'x', {'hit': True, 'sighting_to_hit_s': 3.2, 'sighting_range_m': 78.0})]
    lines = end_card_lines(takes)
    assert '2 of 2 runs ended in a hit' in lines
    assert '3.1 s from first sighting to hit (median)' in lines
    assert 'first seen at 74 m (median)' in lines


def test_caption_with_quote_and_percent(tmp_path):
    run = tmp_path / 'run'
    make_run(run, takes=1, videos=False)
    out = build(run, story(tmp_path, "scenes:\n- caption: \"The defender's view: 100% sure, really\"\n  from: release\n  to: release + 1\n  layout: full\n  show: [map]\n"), preview=True)
    assert out.exists()
