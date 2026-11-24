"""recording/record.sh --dry-run prints exactly what a real run would do, in order."""
import subprocess
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[4] / 'recording' / 'record.sh'


def plan(*args):
    out = subprocess.run(['bash', str(SCRIPT), '--dry-run', '--name', 'test', *args],
                         capture_output=True, text=True)
    assert out.returncode == 0, out.stderr
    return out.stdout.splitlines()


def parts(lines, verb, section='normal'):
    if section == 'int':
        lines = lines[lines.index('ON INT:') + 1:]
    else:
        lines = lines[:lines.index('ON INT:')] if 'ON INT:' in lines else lines
    return [line.split()[1].rstrip(':') for line in lines if line.startswith(verb + ' ')]


def command(lines, part):
    return next(line for line in lines if line.startswith(f'START {part}:'))


def test_dry_run_start_order():
    lines = plan('scenarios/trial.sh', '1')
    assert parts(lines, 'START') == ['session', 'sim', 'runinfo', 'ui', 'poses', 'yolo', 'cams',
                                     'scenario']
    assert '--headless' in command(lines, 'sim') and 'RECORD_RUN=test' in command(lines, 'sim')
    assert 'scenarios/trial.sh 1' in command(lines, 'scenario')


def test_dry_run_screen_and_watch():
    lines = plan('--screen', '--watch', 'scenarios/trial.sh', '1')
    assert parts(lines, 'START') == ['session', 'sim', 'runinfo', 'ui', 'poses', 'browser',
                                     'screen', 'yolo', 'cams', 'scenario']
    assert '--headless' not in command(lines, 'sim')
    assert '--region ' in command(lines, 'screen')    # one monitor, not every screen


def test_dry_run_options():
    lines = plan('--no-cameras', '--no-yolo', 'scenarios/trial.sh', '1')
    assert 'cams' not in parts(lines, 'START') and 'yolo' not in parts(lines, 'START')
    assert not any('cam_' in line or 'debug_d' in line for line in lines if line.startswith('CHECK'))
    lines = plan('--cameras', 'base_tripod', 'scenarios/trial.sh', '1')
    assert 'camrec base_tripod --no-pose-log' in command(lines, 'cams')


def test_dry_run_ctrl_c_order():
    lines = plan('scenarios/trial.sh', '1')
    assert parts(lines, 'STOP', 'int') == ['scenario', 'detect-records', 'cams', 'yolo', 'poses',
                                           'ui', 'sim', 'session']
    assert any('tmux kill-session -t hive-rec' in line for line in lines[lines.index('ON INT:'):])


def test_needs_a_scenario():
    out = subprocess.run(['bash', str(SCRIPT), '--dry-run'], capture_output=True, text=True)
    assert out.returncode != 0 and 'SCENARIO' in (out.stdout + out.stderr)


# --- final review findings -------------------------------------------------------------------

def test_second_ctrl_c_keeps_stopping():
    lines = plan('scenarios/trial.sh', '1')
    second = [line for line in lines if line.startswith('ON SECOND INT:')]
    assert second and 'still stopping' in second[0]


def test_screen_container_removed_on_stop():
    lines = plan('--screen', 'scenarios/trial.sh', '1')
    stop = next(line for line in lines if line.startswith('STOP screen:'))
    assert 'docker rm -f px4-gazebo-vnc-screen' in stop


def test_browser_gets_its_own_instance():
    lines = plan('--screen', 'scenarios/trial.sh', '1')
    browser = command(lines, 'browser')
    assert '--kiosk' in browser and ('--new-instance' in browser or '--user-data-dir' in browser)


# --- step 2: shoots ----------------------------------------------------------------------------

def test_record_reuse_sim_dry_run():
    lines = plan('--reuse-sim', 'scenarios/trial.sh', '1')
    assert 'sim' not in parts(lines, 'START') and 'session' not in parts(lines, 'START')
    assert 'sim' not in parts(lines, 'STOP') and 'session' not in parts(lines, 'STOP')
    assert 'ui' in parts(lines, 'START')


def test_keep_sim_keeps_sim_and_session():
    lines = plan('--keep-sim', 'scenarios/trial.sh', '1')
    assert 'sim' not in parts(lines, 'STOP') and 'session' not in parts(lines, 'STOP')


def test_pair_written_to_run_json():
    lines = plan('--pair', 'head-on-1', 'scenarios/trial.sh', '1', '--setup', 'baseline')
    assert '"pair": "head-on-1"' in command(lines, 'runinfo')


def test_idle_runs_sleep():
    lines = plan('--no-yolo', 'idle', '60')
    assert command(lines, 'scenario').endswith('sleep 60')


def test_reused_run_does_not_check_gz():
    lines = plan('--reuse-sim', 'scenarios/trial.sh', '1')
    assert not any(line.startswith('CHECK gz') for line in lines)


def test_ctrl_c_does_not_wait_for_landing_unless_sim_kept():
    # The sim is shut down right after, so landing (minutes at the slow sim's pace) is skipped.
    stop = next(line for line in plan('scenarios/trial.sh', '1') if line.startswith('STOP scenario:'))
    assert 'no landing' in stop and '300' not in stop
    kept = next(line for line in plan('--keep-sim', 'scenarios/trial.sh', '1')
                if line.startswith('STOP scenario:'))
    assert 'land' in kept and '300' in kept


def test_quick_end_after_last_engagement_unless_sim_kept():
    quick = [line for line in plan('scenarios/trial.sh', '1') if 'Defenders back to their slots' in line]
    assert quick and 'no landing' in quick[0]
    assert not [line for line in plan('--keep-sim', 'scenarios/trial.sh', '1')
                if 'Defenders back to their slots' in line]
