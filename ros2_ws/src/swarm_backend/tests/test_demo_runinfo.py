from pathlib import Path

from swarm_backend.demo.runinfo import read_run_info, write_run_info

PKG = Path(__file__).resolve().parents[1]
SURV = PKG / 'scenarios' / 'surveillance.yaml'
S2 = PKG / 'scenarios' / 'scenario2.yaml'


def test_roles_from_surveillance_yaml(tmp_path):
    info = write_run_info(tmp_path, SURV, ['scenarios/trial.sh', '1'], {})
    assert info['defenders'] == ['x500_gimbal_1', 'x500_gimbal_2', 'x500_gimbal_3']
    assert info['hostiles'] == ['x500_4']


def test_roles_scenario2(tmp_path):
    info = write_run_info(tmp_path, S2, ['scenarios/mpc.sh'], {})
    assert info['hostiles'] == ['x500_2', 'x500_3']
    assert info['defenders'] == ['x500_gimbal_1']


def test_setup_and_seeds_from_command(tmp_path):
    cmd = ['scenarios/trial.sh', '3', '--setup', 'baseline', '--seeds', '0-2']
    info = write_run_info(tmp_path, SURV, cmd, {})
    assert info['setup'] == 'baseline'
    assert info['seeds'] == [0, 1, 2]
    default = write_run_info(tmp_path, SURV, ['scenarios/trial.sh'], {})
    assert default['setup'] == 'yolo-bytetrack-mpc' and default['seeds'] is None


def test_round_trip(tmp_path):
    write_run_info(tmp_path, SURV, ['scenarios/trial.sh', '1'], {'screen': False})
    info = read_run_info(tmp_path)
    assert info.defenders == ['x500_gimbal_1', 'x500_gimbal_2', 'x500_gimbal_3']
    assert info.hostiles == ['x500_4']
    assert info.birds_prefix == 'bird_'
    assert info.options == {'screen': False}
    assert info.config['surveillance']['arc_radius'] == 150.0


def test_cli_as_record_sh_calls_it(tmp_path):
    # record.sh: write RUN --config CFG --options JSON -- CMD...; the options come after RUN.
    from swarm_backend.demo.runinfo import main
    assert main(['write', str(tmp_path), '--config', str(SURV), '--options', '{"yolo": 1}',
                 '--', 'scenarios/trial.sh', '1', '--setup', 'baseline']) == 0
    info = read_run_info(tmp_path)
    assert info.options == {'yolo': 1}
    assert info.setup == 'baseline'
