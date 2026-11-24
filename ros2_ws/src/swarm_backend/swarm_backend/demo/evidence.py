"""The earlier Gazebo evaluations' numbers for the film's end card (story `evidence:`).

`baseline_eval` writes the baseline's runs to data/baseline_eval/intercept.jsonl and every
other setup's to data/baseline_eval_<setup>/ (scenarios/baseline_eval.py). Many runs make a
stronger point than a shoot's few pairs, so the compare end card shows both.
"""
import statistics
from pathlib import Path

from swarm_backend.demo.shoot import OURS
from swarm_backend.scenarios.baseline_eval import load_runs


def _summary(rows: list[dict]) -> dict | None:
    if not rows:
        return None
    hits = [r for r in rows if r.get('outcome') == 'hit']
    times = [r['time_to_hit'] for r in hits if r.get('time_to_hit') is not None]
    dists = [r['detect_distance'] for r in rows if r.get('detect_distance') is not None]
    return {'runs': len(rows), 'hits': len(hits), 'hit_rate': len(hits) / len(rows),
            'median_time_to_hit_s': statistics.median(times) if times else None,
            'median_detect_m': statistics.median(dists) if dists else None}


def evidence(base_folder: Path) -> dict[str, dict | None]:
    """{'baseline': summary, 'ours': summary} from `base_folder` (the baseline's) and
    `<base_folder>_yolo-bytetrack-mpc`; None for a setup without runs."""
    base_folder = Path(base_folder)
    ours = base_folder.with_name(f'{base_folder.name}_{OURS}')
    return {'baseline': _summary(load_runs(base_folder / 'intercept.jsonl')),
            'ours': _summary(load_runs(ours / 'intercept.jsonl'))}


def evidence_lines(ev: dict[str, dict | None]) -> list[str]:
    def hits(name):
        s = ev.get(name)
        return f'{name}: no evaluation runs' if s is None else f'{name} {s["hits"]}/{s["runs"]} hits'
    lines = [f'Gazebo evaluation: {hits("baseline")}, {hits("ours")}']
    for key, label, unit in (('median_time_to_hit_s', 'median time to hit', 's'),
                             ('median_detect_m', 'median first detection', 'm')):
        vals = {n: (ev.get(n) or {}).get(key) for n in ('baseline', 'ours')}
        if all(v is not None for v in vals.values()):
            fmt = '{:.1f}' if unit == 's' else '{:.0f}'
            lines.append(f'{label}: baseline {fmt.format(vals["baseline"])} {unit}, '
                         f'ours {fmt.format(vals["ours"])} {unit}')
    return lines
