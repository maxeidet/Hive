"""baseline_eval --one-run / --restart-on-crash: the scenario ends with exit code 75 for
scenarios/baseline_eval.sh to restart Gazebo, and goes on with the unsaved runs next time.
Run in the fast simulation with the restart decision faked, since a real restart needs
Gazebo."""
import json

import pytest

from swarm_backend.scenarios import baseline_eval, common

ARGS = ['intercept', '--aspects', '0', '--speeds', '4', '--altitude-offsets', '0',
        '--sector', '2', '--quiet']


def flown(out):
    path = out / 'intercept.jsonl'
    return [json.loads(line)['seed'] for line in path.read_text().splitlines()] if path.exists() else []


def test_restart_flags_are_refused_in_the_fast_simulation(tmp_path):
    for flag in ('--one-run', '--restart-on-crash'):
        with pytest.raises(SystemExit):
            baseline_eval.main([*ARGS, '--seeds', '0', '--sim', flag, '--out', str(tmp_path)])


def test_one_run_per_call_then_the_rest(tmp_path, monkeypatch):
    """--one-run, as Gazebo would get it: each call flies one more run and asks for a
    restart while runs are left; the last call ends normally."""
    real = common.run_surveillance

    def as_if_gazebo(name, config, options, args, body):
        args.one_run = True  # --one-run is refused with --sim; the logic is the same
        return real(name, config, options, args, body)

    monkeypatch.setattr(common, 'run_surveillance', as_if_gazebo)
    argv = [*ARGS, '--seeds', '0-2', '--sim', '--out', str(tmp_path)]
    assert baseline_eval.main(argv) == common.EXIT_RESTART
    assert flown(tmp_path) == [0]
    assert baseline_eval.main(argv) == common.EXIT_RESTART
    assert flown(tmp_path) == [0, 1]
    assert baseline_eval.main(argv) == common.EXIT_RESTART  # after the last run too: a fresh world
    assert flown(tmp_path) == [0, 1, 2]
    assert baseline_eval.main(argv) == 0  # nothing left: the summary, a normal end
    assert flown(tmp_path) == [0, 1, 2]
    assert (tmp_path / 'summary.txt').exists()


def test_a_crash_on_the_last_run_still_restarts(tmp_path, monkeypatch):
    """The bug: a crash in the last run left the crashed drone in the world (and the
    landing waited for it). Now it ends for a restart like any other crash."""
    real = common.run_surveillance

    def crash_in_every_run(name, config, options, args, body):
        def crashing(world):
            original = world.fly

            def fly(*a, **kw):
                flight = original(*a, **kw)
                world.restart_needed = True  # as note_crashes does with --restart-on-crash
                return flight
            world.fly = fly
            body(world)
        return real(name, config, options, args, crashing)

    monkeypatch.setattr(common, 'run_surveillance', crash_in_every_run)
    argv = [*ARGS, '--seeds', '0', '--sim', '--out', str(tmp_path)]
    assert baseline_eval.main(argv) == common.EXIT_RESTART
    assert flown(tmp_path) == [0]
