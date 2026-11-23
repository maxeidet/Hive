"""scenarios/at_slots.py: defenders start right below their slots on the arc."""
import math
from pathlib import Path

import pytest
import yaml

from swarm_backend.config import load_config, parse_config
from swarm_backend.evaluation.arc_eval import slots
from swarm_backend.scenarios.at_slots import at_slots, main

SURVEILLANCE = Path(__file__).resolve().parent.parent / 'scenarios' / 'surveillance.yaml'


def raw():
    return yaml.safe_load(SURVEILLANCE.read_text())


def test_defenders_start_below_their_slots():
    config = parse_config(at_slots(raw()))
    for drone_id, (north, east, _) in zip(config.surveillance.drone_ids, slots(config.surveillance)):
        n, e, d = config.drone(drone_id).spawn_ned
        assert math.isclose(n, north, abs_tol=1e-3) and math.isclose(e, east, abs_tol=1e-3)
        assert d == 0.0


def test_middle_defender_is_straight_out_on_the_threat_heading():
    config = parse_config(at_slots(raw()))
    s = config.surveillance
    middle = s.drone_ids[len(s.drone_ids) // 2]
    north, east, _ = config.drone(middle).spawn_ned
    assert math.isclose(north, s.arc_radius, abs_tol=1e-3) and math.isclose(east, 0.0, abs_tol=1e-3)


def test_everything_else_is_kept():
    before, after = raw(), at_slots(raw())
    defenders = set(before['surveillance']['drone_ids'])
    for b, a in zip(before['drones'], after['drones']):
        if b['id'] not in defenders:
            assert a == b  # the hostile drone and drone 0 stay where they were
        else:
            assert {k: v for k, v in a.items() if k != 'spawn_pose'} == \
                   {k: v for k, v in b.items() if k != 'spawn_pose'}
    assert {k: v for k, v in after.items() if k != 'drones'} == \
           {k: v for k, v in before.items() if k != 'drones'}
    assert before == raw()  # the input is not changed


def test_missing_defender_is_an_error():
    data = raw()
    data['drones'] = [d for d in data['drones'] if d['id'] != data['surveillance']['drone_ids'][0]]
    with pytest.raises(ValueError):
        at_slots(data)


def test_main_writes_a_loadable_config(tmp_path):
    out = tmp_path / 'sub' / 'at_slots.yaml'
    assert main(['--config', str(SURVEILLANCE), '--out', str(out)]) == 0
    config = load_config(out)
    assert config.surveillance.drone_ids == load_config(SURVEILLANCE).surveillance.drone_ids
