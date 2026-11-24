"""--birds N: birds hook into the single-hostile run (surveillance.py start_extras/stop_extras)."""
import pytest

from swarm_backend.scenarios import baseline_eval
from swarm_backend.scenarios.sim_surveillance import SimSurveillance
from swarm_backend.scenarios.surveillance import Options
from tests.test_scenarios import SURVEILLANCE, head_on


class Recording(SimSurveillance):
    def __init__(self, *a, fail_at_pause=None, **k):
        super().__init__(*a, **k)
        self.calls, self.pauses, self.fail_at_pause = [], 0, fail_at_pause

    def start_extras(self, start, goal, speed):
        import math
        at = self.hostile.position
        self.calls.append(('start', tuple(start), tuple(goal), speed, math.dist(at, start) < 1.0))

    def stop_extras(self):
        self.calls.append(('stop',))

    def pause(self):
        self.pauses += 1
        if self.fail_at_pause is not None and self.pauses == self.fail_at_pause:
            raise RuntimeError('boom')
        super().pause()


def recording_world(**k):
    w = Recording(SURVEILLANCE, Options(detector='fake'), quiet=True, **k)
    w.setup()
    return w


def test_hooks_called_around_the_flight():
    w = recording_world()
    scenario, start, goal = head_on(w)
    w.fly(start, goal, scenario.speed)
    assert [c[0] for c in w.calls] == ['start', 'stop']
    _, s, g, speed, before_release = w.calls[0]
    assert s == tuple(start) and g == tuple(goal) and speed == scenario.speed
    assert before_release  # the hostile still at its start: it had not set off yet


def test_birds_stopped_when_the_run_fails():
    w = recording_world()
    scenario, start, goal = head_on(w)
    w.fail_at_pause = w.pauses + 3
    with pytest.raises(RuntimeError, match='boom'):
        w.fly(start, goal, scenario.speed)
    assert w.calls[-1] == ('stop',)


def test_birds_option_in_rows(tmp_path):
    common = ['intercept', '--sim', '--quiet', '--aspects', '0', '--speeds', '4', '--seeds', '0']
    assert baseline_eval.main(common + ['--birds', '8', '--out', str(tmp_path / 'b')]) == 0
    row = baseline_eval.load_runs(tmp_path / 'b' / 'intercept.jsonl')[0]
    assert row['birds'] == 8 and row['bird_seed'] == 0
    assert baseline_eval.main(common + ['--out', str(tmp_path / 'n')]) == 0
    row = baseline_eval.load_runs(tmp_path / 'n' / 'intercept.jsonl')[0]
    assert 'birds' not in row and 'bird_seed' not in row
