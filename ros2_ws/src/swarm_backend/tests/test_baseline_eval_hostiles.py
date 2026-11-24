"""baseline_eval --hostiles 2: two hostiles at once (surveillance2.yaml, fly_many)."""
import shutil
from pathlib import Path

import pytest

from swarm_backend.config import load_config
from swarm_backend.scenarios import baseline_eval, common

TWO = str(common.PKG / 'scenarios' / 'surveillance2.yaml')
FIXTURE = Path(__file__).resolve().parent / 'data' / 'baseline_eval_single'


def test_hostiles_2_rows(tmp_path):
    assert baseline_eval.main(['intercept', '--sim', '--quiet', '--config', TWO, '--hostiles', '2',
                               '--aspects', '0', '--speeds', '4', '--seeds', '0',
                               '--out', str(tmp_path)]) == 0
    rows = baseline_eval.load_runs(tmp_path / 'intercept_2.jsonl')
    assert len(rows) == 1 and not (tmp_path / 'intercept.jsonl').exists()
    first, second = rows[0]['hostiles']
    assert (first['id'], second['id']) == (4, 5)
    assert first['start_bearing_deg'] < 0 < second['start_bearing_deg']
    assert rows[0]['outcome'] == f"hits {sum(h['hit'] for h in rows[0]['hostiles'])}/2"
    assert 'hostiles hit' in (tmp_path / 'summary.txt').read_text()


def test_hostiles_2_needs_the_config(tmp_path):
    with pytest.raises(SystemExit, match='surveillance2.yaml'):
        baseline_eval.main(['intercept', '--sim', '--quiet', '--hostiles', '2', '--seeds', '0',
                            '--out', str(tmp_path)])


def test_old_rows_summarise_unchanged(tmp_path):
    shutil.copy(FIXTURE / 'intercept.jsonl', tmp_path / 'intercept.jsonl')
    config = load_config(common.PKG / common.SURVEILLANCE_CONFIG)
    assert baseline_eval.summary_text(tmp_path, config) == \
        (FIXTURE / 'expected_summary.txt').read_text()
