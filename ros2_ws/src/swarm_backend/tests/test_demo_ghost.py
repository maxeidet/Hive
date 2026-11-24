import numpy as np
import pytest

from swarm_backend.demo.fake_run import make_run
from swarm_backend.demo.final import build
from swarm_backend.demo.map_video import (FRIEND, GHOST, map_state, project, render_ghost)
from swarm_backend.ui.camera_video import pose_at

from test_demo_compare import pair_shoot, probe_len, story  # noqa: E402


def near(img, u, v, colour, r=6):
    patch = img[max(0, v - r):v + r, max(0, u - r):u + r].reshape(-1, 3).astype(int)
    return bool((np.abs(patch - np.array(colour)).sum(axis=1) < 40).any())


def test_ghost_has_both(tmp_path):
    t_ours = make_run(tmp_path / 'ours', takes=1, videos=False, attackers=(3,))
    make_run(tmp_path / 'base', takes=1, videos=False, attackers=(1,))
    ours, base = map_state(tmp_path / 'ours'), map_state(tmp_path / 'base')
    sim = t_ours['sighting'][0] + 3.0
    img = render_ghost(ours, base, sim, sim)
    p3 = pose_at(ours.sims, ours.rows, 'x500_gimbal_3', sim)[0]
    p1 = pose_at(base.sims, base.rows, 'x500_gimbal_1', sim)[0]
    assert near(img, *project(ours, p3[0], p3[1]), FRIEND)
    assert near(img, *project(base, p1[0], p1[1]), GHOST)


def test_ghost_past_one_end(tmp_path):
    make_run(tmp_path / 'ours', takes=1, videos=False)
    make_run(tmp_path / 'base', takes=1, videos=False)
    ours, base = map_state(tmp_path / 'ours'), map_state(tmp_path / 'base')
    end = base.sims[-1]
    a = render_ghost(ours, base, 120.0, end + 1.0)
    b = render_ghost(ours, base, 125.0, end + 5.0)
    ghost_a = (np.abs(a.astype(int) - np.array(GHOST)).sum(axis=2) < 30)
    ghost_b = (np.abs(b.astype(int) - np.array(GHOST)).sum(axis=2) < 30)
    assert ghost_a.sum() > 0
    assert (ghost_a & ghost_b).sum() / ghost_a.sum() > 0.8      # the baseline holds still
    assert not np.array_equal(a, b)                              # ours keeps moving


def test_ghost_scene_builds(tmp_path):
    folder = pair_shoot(tmp_path)
    out = build(folder, story(tmp_path, '''scenes:
- caption: Ghost
  take: pair:h-1
  from: release + 5
  to: release + 8
  layout: ghost
  show: [map]
'''), preview=True)
    assert probe_len(out) == pytest.approx(3.0, abs=0.15)
