import io
import json
import random

from swarm_backend.intercept.inbound import plan_birds
from swarm_backend.scenarios import bird_flight


class FakeProc:
    started = []

    def __init__(self, args, stdin=None, stdout=None, stderr=None, text=None):
        self.args, self.stdin, self.terminated = args, io.StringIO(), 0
        self.stdin.close = lambda: None
        self.returncode = None
        FakeProc.started.append(self)

    def poll(self):
        return self.returncode

    def terminate(self):
        self.terminated += 1
        self.returncode = 0

    def wait(self, timeout=None):
        return self.returncode

    def kill(self):
        self.returncode = -9


def plans():
    return plan_birds(3, [(200.0, 0.0, -40.0)], (0.0, 0.0), ['dove', 'eagle'], 4.0,
                      random.Random(1), prefix='bird_')


def test_start_writes_the_plan(monkeypatch, tmp_path):
    FakeProc.started = []
    monkeypatch.setattr(bird_flight.subprocess, 'Popen', FakeProc)
    flight = bird_flight.BirdFlight('hive_base', str(tmp_path / 'birds.log'))
    flight.start(plans(), 12.5)
    proc = FakeProc.started[0]
    assert proc.args[-3:] == ['-m', 'swarm_backend.birds', 'fly']
    plan = json.loads(proc.stdin.getvalue())
    assert plan['world'] == 'hive_base' and plan['t0'] == 12.5 and len(plan['birds']) == 3
    assert set(plan['birds'][0]) >= {'name', 'kind', 'start', 'end', 'speed'}
    assert flight.running


def test_stop_twice_is_harmless(monkeypatch, tmp_path):
    FakeProc.started = []
    monkeypatch.setattr(bird_flight.subprocess, 'Popen', FakeProc)
    flight = bird_flight.BirdFlight('hive_base', str(tmp_path / 'birds.log'))
    flight.stop()
    flight.start(plans(), 0.0)
    flight.stop()
    flight.stop()
    assert FakeProc.started[0].terminated == 1 and not flight.running
