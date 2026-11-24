import json
import time

from swarm_backend.demo.prescreen import WANTS, choose, prescreen, sim_args, sim_facts
from swarm_backend.demo.shoot import Entry

ROW = json.loads('{"aspect_deg": 0.0, "speed": 4.0, "seed": 0, "outcome": "hit", "success": true, '
                 '"time_to_hit": 4.48, "closest_miss": 1.28, "killer": 3, "detected_by": 3, '
                 '"detect_distance": 50.8, "setup": "baseline"}')


def test_sim_facts_names():
    f = sim_facts(ROW)
    assert f == {'hit': True, 'attacker': 3, 'sighting_range_m': 50.8, 'sighting_to_hit_s': 4.48,
                 'closest_pass_m': 1.28, 'setup': 'baseline'}


def test_wants():
    hit, miss = {'hit': True, 'sighting_range_m': 70}, {'hit': False, 'sighting_range_m': 50}
    assert WANTS['ours_hits'](hit, miss) and WANTS['baseline_misses'](hit, miss)
    assert not WANTS['ours_hits'](miss, hit) and not WANTS['baseline_misses'](miss, hit)
    assert WANTS['ours_sees_earlier'](hit, miss)


def test_sim_args_from_trial():
    e = Entry('z', ['scenarios/trial.sh', '3', '--path', 'zigzag', '--seeds', '4'])
    assert sim_args(e) == ['--aspects', '0', '--speeds', '4', '--path', 'zigzag']


def test_screen_nothing_matches():
    facts = {s: ({'hit': False}, {'hit': True}) for s in (1, 2, 3)}
    seeds, report = choose(facts, ['ours_hits', 'baseline_misses'], keep=2, name='z')
    assert len(seeds) == 2 and '0 of 3' in report


def test_prescreen_runs_the_sim(tmp_path):
    e = Entry('z', ['scenarios/trial.sh', '1'], repeat=1, compare=['baseline', 'yolo-bytetrack-mpc'],
              seeds='random', want=['ours_hits'])
    t = time.time()
    seeds, report = prescreen(e, 3, tmp_path)
    assert len(seeds) == 1 and 'z:' in report
    assert (tmp_path / 'z' / 'baseline' / 'intercept.jsonl').exists()
    assert (tmp_path / 'z' / 'yolo-bytetrack-mpc' / 'intercept.jsonl').exists()
    print(f'prescreen of 3 candidates took {time.time() - t:.1f} s')
