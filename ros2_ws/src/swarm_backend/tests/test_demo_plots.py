import json

import numpy as np
import pytest

from swarm_backend.demo.fake_run import make_run
from swarm_backend.demo.final import build
from swarm_backend.demo.plots import PLOTS, plot_data, write_plot
from swarm_backend.demo.takes import find_takes


def test_range_plot_values(tmp_path):
    make_run(tmp_path, takes=1, videos=False)
    take = find_takes(tmp_path)[0]
    sims, values, title, unit = plot_data(tmp_path, take, 'range')
    assert min(values) == pytest.approx(take.facts['closest_pass_m'], abs=0.2)
    assert unit == 'm' and set(PLOTS) >= {'range', 'base_distance', 'est_error'}


def test_plot_frames(tmp_path):
    make_run(tmp_path, takes=1, videos=False)
    take = find_takes(tmp_path)[0]
    out = write_plot(tmp_path, take, 'range', take.moments['first_sighting'], take.moments['hit'],
                     tmp_path / 'p.mp4', (480, 270), dot_positions := [])
    n = int(round((take.moments['hit'] - take.moments['first_sighting']) * 25))
    assert out.exists() and len(dot_positions) == n
    assert dot_positions[-1][0] > dot_positions[0][0]


def test_plot_scene_builds(tmp_path):
    make_run(tmp_path / 'run', takes=1, videos=False)
    (tmp_path / 's.yaml').write_text('scenes:\n- caption: P\n  from: first_sighting\n  to: hit\n  layout: side\n  show: [map, "plot:range"]\n')
    assert build(tmp_path / 'run', tmp_path / 's.yaml', preview=True).exists()
