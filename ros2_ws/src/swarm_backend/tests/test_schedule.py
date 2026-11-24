"""scenarios/schedule.py: the schedule file, the commands it turns into, the summary."""
import importlib.util
import json
from pathlib import Path

import pytest

SPEC = importlib.util.spec_from_file_location(
    'schedule', Path(__file__).resolve().parents[1] / 'scenarios' / 'schedule.py')
schedule = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(schedule)


def write(tmp_path, text):
    f = tmp_path / 's.yaml'
    f.write_text(text)
    return f


def test_the_example_schedule_loads():
    name, steps = schedule.load(schedule.SCHEDULES / 'paths_altitudes.yaml')
    assert name == 'paths_altitudes'
    assert [s['label'] for s in steps] == ['straight', 'weave_5m', 'weave_15m', 'zigzag_15m']


def test_defaults_runs_per_altitude_and_labels(tmp_path):
    _, steps = schedule.load(write(tmp_path, """
defaults: {runs: 2, altitudes: [-10, 10]}
steps:
  - path: straight
    runs: {0: 4, 10: 1}
  - path: zigzag
    amplitude: 7.5
"""))
    assert steps[0]['groups'] == {0.0: 4, 10.0: 1}
    assert steps[1]['groups'] == {-10.0: 2, 10.0: 2}
    assert steps[1]['label'] == 'zigzag_7.5m'


def test_one_command_per_run_count_with_the_same_seeds(tmp_path):
    _, steps = schedule.load(write(tmp_path, """
steps:
  - path: weave
    amplitude: 5
    runs: {-10: 2, 0: 5, 10: 2}
    seed_start: 10
    args: [--spawn-at-slots]
"""))
    calls = schedule.commands(steps[0], Path('/out'), ['--sim'])
    assert len(calls) == 2
    two, five = calls
    assert two[two.index('--seeds') + 1] == '10-11'
    assert two[two.index('--altitude-offsets') + 1:two.index('--sector')] == ['-10', '10']
    assert five[five.index('--seeds') + 1] == '10-14'
    assert five[five.index('--out') + 1] == '/out/weave_5m'
    assert '--path-amplitude' in five and five[-2:] == ['--spawn-at-slots', '--sim']


@pytest.mark.parametrize('text, error', [
    ('steps: []', 'list of steps'),
    ('steps: [{path: loop}]', 'path must be'),
    ('steps: [{path: straight, speeed: 3}]', 'unknown keys'),
    ('steps: [{path: straight, runs: 0}]', 'at least 1 run'),
    ('steps: [{path: straight}, {path: straight}]', 'give one a label'),
])
def test_bad_schedules_say_what_is_wrong(tmp_path, text, error):
    with pytest.raises(SystemExit, match=error):
        schedule.load(write(tmp_path, text))


def test_summary_per_step_and_altitude(tmp_path):
    _, steps = schedule.load(write(tmp_path, 'steps: [{path: straight}, {path: zigzag}]'))
    (tmp_path / 'straight').mkdir()
    rows = [dict(outcome='hit', success=True, time_to_hit=12.0, closest_miss=0.5,
                 hostile_altitude=40.0, detect_distance=80.0),
            dict(outcome='reached base', success=False, time_to_hit=None, closest_miss=9.0,
                 hostile_altitude=40.0, detect_distance=None)]
    (tmp_path / 'straight' / 'intercept.jsonl').write_text(
        '\n'.join(json.dumps(r) for r in rows) + '\n')
    text = schedule.summary(tmp_path, steps, failed={'zigzag_15m'})
    line = next(l for l in text.splitlines() if l.startswith('straight'))
    assert '1/2' in line and '12.0 s' in line and 'hit 1, reached base 1' in line
    assert 'failed' in next(l for l in text.splitlines() if l.startswith('zigzag_15m'))
