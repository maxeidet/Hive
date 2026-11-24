"""recording/shoot.sh --dry-run: which runs it would record, with which record.sh flags."""
import subprocess
from pathlib import Path

from swarm_backend.demo.shoot import expand, load_shoot, mark, write_plan

REPO = Path(__file__).resolve().parents[4]
SCRIPT = REPO / 'recording' / 'shoot.sh'
YAML = '''- name: a
  run: scenarios/trial.sh 1
  repeat: 3
'''


def yaml_in_repo(name):
    y = REPO / 'data' / 'recordings' / f'{name}.yaml'   # the container sees only the repo
    y.parent.mkdir(parents=True, exist_ok=True)
    y.write_text(YAML)
    return y


def dry(tmp_path, name, *extra):
    y = yaml_in_repo(name)
    out = subprocess.run(['bash', str(SCRIPT), str(y), '--name', name, '--dry-run', *extra],
                         capture_output=True, text=True)
    assert out.returncode == 0, out.stdout + out.stderr
    return out.stdout.splitlines(), y


def test_dry_run_reuse_flags(tmp_path):
    lines, y = dry(tmp_path, 'test-shoot-flags')
    y.unlink()
    rec = [line for line in lines if line.startswith('RECORD ')]
    assert len(rec) == 3
    assert '--keep-sim' in rec[0] and '--keep-sim' in rec[1] and '--keep-sim' not in rec[2]
    assert '--reuse-sim' not in rec[0] and '--reuse-sim' in rec[1] and '--reuse-sim' in rec[2]
    assert '--name test-shoot-flags/a-1' in rec[0]
    after = lines[lines.index(rec[2]) + 1]
    assert after.startswith('STOP sim')     # the reused sim of the group is stopped after its last run


def test_resume_skips_done(tmp_path):
    name = 'test-shoot-resume'
    folder = REPO / 'data' / 'recordings' / name
    try:
        y = yaml_in_repo(name)
        write_plan(folder, y, expand(load_shoot(y)))
        mark(folder, 'a-1', 'done')
        lines, _ = dry(tmp_path, name)
        assert any(line.startswith('SKIP a-1') for line in lines)
        rec = [line for line in lines if line.startswith('RECORD ')]
        assert len(rec) == 2 and 'a-2' in rec[0]
        assert '--reuse-sim' not in rec[0]  # nothing kept from a run of an earlier session
    finally:
        import shutil
        shutil.rmtree(folder, ignore_errors=True)
        (REPO / 'data' / 'recordings' / f'{name}.yaml').unlink(missing_ok=True)


def test_resume_without_name(tmp_path):
    name = 'test-shoot-auto-20990101-0000'
    folder = REPO / 'data' / 'recordings' / name
    y = REPO / 'data' / 'recordings' / 'test-shoot-auto.yaml'
    try:
        y.parent.mkdir(parents=True, exist_ok=True)
        y.write_text(YAML)
        write_plan(folder, y, expand(load_shoot(y)))
        mark(folder, 'a-1', 'done')
        out = subprocess.run(['bash', str(SCRIPT), str(y), '--dry-run'], capture_output=True, text=True)
        assert out.returncode == 0, out.stdout + out.stderr
        assert f'RESUME {name}' in out.stdout and 'SKIP a-1' in out.stdout
    finally:
        import shutil
        shutil.rmtree(folder, ignore_errors=True)
        y.unlink(missing_ok=True)


def test_dry_run_fresh_sim(tmp_path):
    # --fresh-sim: every run starts and stops its own sim, so a crashed defender (a real
    # collision at a hit, check-birds 24 Nov) can only spoil its own run.
    lines, y = dry(tmp_path, 'test-shoot-fresh', '--fresh-sim')
    y.unlink()
    rec = [line for line in lines if line.startswith('RECORD ')]
    assert len(rec) == 3
    assert not any('--keep-sim' in r or '--reuse-sim' in r for r in rec)
