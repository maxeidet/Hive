import subprocess

import pytest

from swarm_backend.demo.fake_run import make_run
from swarm_backend.demo.final import build
from swarm_backend.demo.insights import insights_for
from swarm_backend.demo.takes import find_takes


def take(tmp_path, **kw):
    make_run(tmp_path, takes=1, videos=False, **kw)
    return find_takes(tmp_path)[0]


def test_labels_in_range(tmp_path):
    t = take(tmp_path)
    m = t.moments
    labels = insights_for(t, m['first_sighting'] - 1, m['hit'] + 1, 1.0)
    by = {lab.name: lab for lab in labels}
    assert {'seen_at', 'assigned', 'time_to_hit', 'closest'} <= set(by)
    assert by['seen_at'].at == pytest.approx(1.0, abs=0.05)
    assert by['time_to_hit'].at == pytest.approx(m['hit'] - m['first_sighting'] + 1, abs=0.05)
    assert by['seen_at'].text.startswith('seen at ')


def test_speed_scales_times(tmp_path):
    t = take(tmp_path)
    m = t.moments
    a = {lab.name: lab.at for lab in insights_for(t, m['first_sighting'] - 1, m['hit'] + 1, 1.0)}
    b = {lab.name: lab.at for lab in insights_for(t, m['first_sighting'] - 1, m['hit'] + 1, 0.5)}
    assert b['seen_at'] == pytest.approx(2 * a['seen_at'])


def test_insights_stack(tmp_path):
    t = take(tmp_path)
    m = t.moments
    labels = {lab.name: lab for lab in insights_for(t, m['first_sighting'] - 1, m['hit'] + 1, 1.0)}
    assert labels['seen_at'].row != labels['assigned'].row


def test_missing_fact_left_out(tmp_path):
    t = take(tmp_path)
    t.facts['sighting_range_m'] = None
    m = t.moments
    assert 'seen_at' not in {lab.name for lab in insights_for(t, m['first_sighting'] - 1, m['hit'] + 1, 1.0)}


def grab(path, at):
    return subprocess.run(['ffmpeg', '-v', 'error', '-ss', f'{at}', '-i', str(path), '-frames:v', '1',
                           '-f', 'rawvideo', '-pix_fmt', 'gray', '-'], capture_output=True).stdout


def test_scene_with_insights_builds(tmp_path):
    run = tmp_path / 'run'
    make_run(run, takes=1, videos=False)
    s = 'scenes:\n- caption: X\n  from: first_sighting - 1\n  to: first_sighting + 3\n  layout: full\n  show: [map]\n'
    (tmp_path / 'a.yaml').write_text(s + '  insights: true\n')
    (tmp_path / 'b.yaml').write_text(s)
    a = build(run, tmp_path / 'a.yaml', preview=True)
    b = build(run, tmp_path / 'b.yaml', preview=True)
    assert grab(a, 1.8) != grab(b, 1.8)
