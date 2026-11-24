import pytest

from swarm_backend.demo.shoot import ShootError, expand, load_shoot, plan_lines


def write(tmp_path, text):
    p = tmp_path / 'demo.yaml'
    p.write_text(text)
    return p


def test_expand_repeat_and_ids(tmp_path):
    runs = expand(load_shoot(write(tmp_path, '- name: head-on\n  run: scenarios/trial.sh 1\n  repeat: 2\n')))
    assert [r.id for r in runs] == ['head-on-1', 'head-on-2']
    assert runs[0].command == ['scenarios/trial.sh', '1']


def test_compare_pairs(tmp_path):
    runs = expand(load_shoot(write(tmp_path, '- name: h\n  run: scenarios/trial.sh 1\n  compare: [baseline, yolo-bytetrack-mpc]\n  seeds: 7\n')))
    assert [r.id for r in runs] == ['h-1-baseline', 'h-1-ours']
    assert runs[0].pair == runs[1].pair == 'h-1'
    assert [r.setup for r in runs] == ['baseline', 'yolo-bytetrack-mpc']
    for r in runs:
        assert r.command[-4:-2] == ['--seeds', '7'] or '--seeds' in r.command
        assert r.command[r.command.index('--setup') + 1] == r.setup


def test_random_seeds_repeatable(tmp_path):
    entries = load_shoot(write(tmp_path, '- name: random\n  run: scenarios/trial.sh 1\n  seeds: random\n  repeat: 2\n'))
    a, b = expand(entries, rng_seed=3), expand(entries, rng_seed=3)
    assert [r.seeds for r in a] == [r.seeds for r in b]
    c = expand(entries, prescreened={'random': [12, 40]})
    assert [r.seeds for r in c] == [[12], [40]]


def test_unknown_key_and_setup(tmp_path):
    with pytest.raises(ShootError, match='repat'):
        load_shoot(write(tmp_path, '- name: a\n  run: x\n  repat: 2\n'))
    with pytest.raises(ShootError, match='best'):
        load_shoot(write(tmp_path, '- name: a\n  run: x\n  compare: [best]\n'))


def test_plan_restarts_sim_on_config_change(tmp_path):
    runs = expand(load_shoot(write(tmp_path, '- name: a\n  run: x\n- name: b\n  run: y\n  config: other.yaml\n- name: c\n  run: z\n  config: other.yaml\n')))
    keep = [line.split('\t')[1] for line in plan_lines(runs)]
    assert keep == ['0', '1', '0']


def test_mark_and_status(tmp_path, capsys):
    from swarm_backend.demo.shoot import main, mark, write_plan
    y = write(tmp_path, '- name: a\n  run: x\n  repeat: 2\n')
    write_plan(tmp_path / 's', y, expand(load_shoot(y)))
    mark(tmp_path / 's', 'a-1', 'done')
    main(['status', str(tmp_path / 's')])
    out = capsys.readouterr().out
    assert 'done   a-1' in out and 'todo   a-2' in out
    with pytest.raises(ShootError):
        write_plan(tmp_path / 's', write(tmp_path, '- name: b\n  run: y\n'), [])
