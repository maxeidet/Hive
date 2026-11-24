import json

from swarm_backend.demo.evidence import evidence, evidence_lines


def rows(folder, outcomes):
    folder.mkdir(parents=True)
    with open(folder / 'intercept.jsonl', 'w') as f:
        for i, (hit, t, d) in enumerate(outcomes):
            f.write(json.dumps({'seed': i, 'outcome': 'hit' if hit else 'reached_base',
                                'time_to_hit': t, 'detect_distance': d}) + '\n')


def test_evidence_counts(tmp_path):
    rows(tmp_path / 'baseline_eval', [(True, 6.0, 50), (False, None, 40), (False, None, None)])
    rows(tmp_path / 'baseline_eval_yolo-bytetrack-mpc', [(True, 3.0, 80), (True, 4.0, 90), (False, None, 70)])
    ev = evidence(tmp_path / 'baseline_eval')
    assert ev['baseline'] == {'runs': 3, 'hits': 1, 'hit_rate': 1 / 3, 'median_time_to_hit_s': 6.0,
                              'median_detect_m': 45.0}
    assert ev['ours']['hits'] == 2 and ev['ours']['median_time_to_hit_s'] == 3.5
    lines = evidence_lines(ev)
    assert lines[0] == 'Gazebo evaluation: baseline 1/3 hits, ours 2/3 hits'
    assert 'median time to hit: baseline 6.0 s, ours 3.5 s' in lines


def test_missing_setup(tmp_path):
    rows(tmp_path / 'baseline_eval', [(True, 6.0, 50)])
    ev = evidence(tmp_path / 'baseline_eval')
    assert ev['ours'] is None
    assert 'ours: no evaluation runs' in evidence_lines(ev)[0]
