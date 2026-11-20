"""Every scenario with a fast simulation, run end to end with --sim: it sets up, flies,
scores and writes its results. No ROS or Gazebo, so a broken scenario shows up here
before anyone spends Gazebo time on it. The same code flies them in Gazebo; only the
world (swarm_backend/scenarios/sim_surveillance.py vs gazebo_surveillance.py) differs.
"""
import csv
import json

import pytest

from swarm_backend.config import load_config
from swarm_backend.intercept.attack_scenario import Inbound, inbound_path
from swarm_backend.intercept.metrics import Outcome
from swarm_backend.scenarios import SCENARIOS, attack, baseline_eval, common
from swarm_backend.scenarios.__main__ import main as scenarios_main
from swarm_backend.scenarios.sim_surveillance import SimSurveillance
from swarm_backend.scenarios.surveillance import Options

SURVEILLANCE = load_config(common.PKG / common.SURVEILLANCE_CONFIG)
SWARM = load_config(common.PKG / common.SWARM_CONFIG)


@pytest.mark.parametrize('name', [n for n, (_, sim) in SCENARIOS.items() if sim])
def test_every_scenario_has_help(name, capsys):
    with pytest.raises(SystemExit) as e:
        scenarios_main([name, '--help'])
    assert e.value.code == 0
    assert '--sim' in capsys.readouterr().out


def test_unknown_scenario():
    assert scenarios_main(['no_such_scenario']) == 1


# --- the surveillance world ----------------------------------------------------------

def world(detector='fake', record=False, config=SURVEILLANCE):
    w = SimSurveillance(config, Options(detector=detector, record=record), quiet=True)
    w.setup()
    return w


def test_defenders_get_on_patrol():
    w = world()
    assert all(w.on_patrol(i) for i in w.ids)
    for i in w.ids:
        assert w.patrols[i].drone.distance_to(w.patrols[i].patrol.slot_point) < 2.0


def head_on(w, speed=2.0):
    # From 100 m beyond the arc, straight at the middle defender and on to the base.
    scenario = Inbound(speed=speed, start_m=w.surv.arc_radius + 100.0)
    path, _ = inbound_path(scenario, w.surv.base_position,
                           w.patrols[w.ids[0]].patrol.slot_point[2], w.heading)
    return scenario, path.points[0], path.points[1]


def test_head_on_hostile_is_seen_and_hit():
    # A slow hostile drone at the middle defender, perfect fake detector.
    w = world()
    scenario, start, goal = head_on(w)
    flight = w.fly(start, goal, scenario.speed)
    assert flight.sightings, 'nobody saw the hostile drone'
    assert flight.engaged is not None, 'the base station sent nobody'
    assert flight.killer is not None, (
        f'no hit; closest {min(j.closest for j in flight.judges.values()):.1f} m')
    assert flight.return_s is not None  # and the killer got back on patrol
    assert all(w.on_patrol(i) for i in w.alive())


def test_runs_follow_each_other():
    # The hostile drone is removed after a run and placed again for the next.
    w = world()
    scenario, start, goal = head_on(w)
    first = w.fly(start, goal, scenario.speed)
    second = w.fly(start, goal, scenario.speed)
    assert (first.killer is None) == (second.killer is None)


def test_records_for_the_evaluation():
    w = world(detector='real', record=True)
    scenario, start, goal = head_on(w)
    flight = w.fly(start, goal, scenario.speed)
    assert set(flight.records) == set(w.index.values())
    images = [r for recs in flight.records.values() for r in recs]
    assert images and {'stamp', 'width', 'height', 'confirmed'} <= set(images[0])
    assert any(r['confirmed'] for r in images)
    assert len(flight.truth) > 10 and flight.statuses


# --- attack ---------------------------------------------------------------------------

def test_attack_scenario_writes_its_rows(tmp_path, capsys):
    out = tmp_path / 'attack.csv'
    start = str(SURVEILLANCE.surveillance.arc_radius + 100.0)
    assert attack.main(['--sim', '--quiet', '--runs', '3', '--bearing', '-30', '0', '30',
                        '--start', start, '--speed', '3', '--fake-detector',
                        '--csv', str(out)]) == 0
    with open(out) as f:
        rows = list(csv.DictReader(f))
    assert [r['run'] for r in rows] == ['1', '2', '3']
    assert set(rows[0]) == set(attack.FIELDS)
    assert all(r['result'] in ('hit', 'reached_base') for r in rows)
    assert 'hits' in capsys.readouterr().out


# --- baseline_eval ----------------------------------------------------------------------

def test_baseline_eval_intercept_and_resume(tmp_path):
    args = ['intercept', '--sim', '--quiet', '--aspects', '0', '--speeds', '4',
            '--seeds', '0-1', '--out', str(tmp_path)]
    assert baseline_eval.main(args) == 0
    rows = baseline_eval.load_runs(tmp_path / 'intercept.jsonl')
    assert [(r['aspect_deg'], r['speed'], r['seed']) for r in rows] == [(0.0, 4.0, 0), (0.0, 4.0, 1)]
    assert all(r['backend'] == 'sim' for r in rows)
    for r in rows:
        Outcome(r['outcome'])
    assert (tmp_path / 'summary.txt').read_text().strip()
    # Saved runs are skipped.
    assert baseline_eval.main(args) == 0
    assert len(baseline_eval.load_runs(tmp_path / 'intercept.jsonl')) == 2


def test_baseline_eval_detect(tmp_path):
    assert baseline_eval.main(['detect', '--sim', '--quiet', '--seeds', '0',
                               '--out', str(tmp_path)]) == 0
    rows = baseline_eval.load_runs(tmp_path / 'detect.jsonl')
    assert len(rows) == 1 and rows[0]['seed'] == 0
    assert rows[0]['engaged_by'] is None  # in detect mode nobody is sent
    json.dumps(rows[0])


def test_baseline_eval_summary_without_runs(tmp_path, capsys):
    assert baseline_eval.main(['summary', '--out', str(tmp_path)]) == 0
    assert 'No saved runs' in capsys.readouterr().out
