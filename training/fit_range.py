"""Distance from box size: the fixed 0.5 m rule vs sizes fitted on gendata (offline, no container needed).

    python3 training/fit_range.py [--fit b06 .. b13] [--test b14 b15] [--conf 0.25]

Today (detect/camera.py) the distance is r = f * 0.5 / p: f = 411 px, p = the box's longer
side, 0.5 m = an x500 frame. Every gendata object has its true distance, so the size the box
really sees is s = r * p / f. This fits that size on the --fit runs and checks it on the
--test runs, against the old rule:

  old 0.5 m           r = f * 0.5 / longer side                       (what live uses now)
  class, longer side  r = f * s_class / longer side                   (one size per class)
  class, sqrt(w*h)    r = f * s_class / sqrt(w * h)                   (area instead of side)
  class, w and h      log r = a + b log w + c log h, per class        (shape-aware fit)
  type (oracle)       r = f * s_type / longer side                    (the true drone/bird
                                                                       type is known: an upper
                                                                       bound, not usable live)

Sizes are fitted as medians in log space, so they minimise relative error, on every
labelled object not cut off by the image border. Far objects are fitted as they are: thin
parts (arms, wing tips) vanish at a few px, so the box there is smaller than the pinhole
size, which the w-and-h fit can learn and a fixed size cannot. Evaluated on every labelled
object of the test runs with the label box, and with each YOLO model's own boxes if
`comparedet` has cached its detections (data/results/compare/raw/RUN/*.json; the
predicted class picks the size, the baseline counts as a drone).

Output in data/results/range_fit/: summary.txt (also printed), fit.json (the fitted sizes
and coefficients), error_by_size.png, error_by_distance.png, est_vs_true.png,
size_by_type.png.
"""
import argparse
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
GEN = REPO / 'data' / 'gen'
RAW = REPO / 'data' / 'results' / 'compare' / 'raw'
OUT = REPO / 'data' / 'results' / 'range_fit'
W, H, HFOV = 1280, 720, 2.0
F = (W / 2) / math.tan(HFOV / 2)  # ~411 px, as camera.CameraModel.f
OLD_SIZE = 0.5  # camera.TARGET_SIZE_M
CLASSES = ('drone', 'bird')
SB = [(0, 4), (4, 8), (8, 16), (16, 32), (32, 64), (64, 1e9)]
DB = [(0, 10), (10, 20), (20, 40), (40, 80), (80, 1e9)]
METHODS = ('old 0.5 m', 'class, longer side', 'class, sqrt(w*h)', 'class, w and h', 'type (oracle)')
COLOURS = {'old 0.5 m': '#444444', 'class, longer side': '#3182bd', 'class, sqrt(w*h)': '#31a354',
           'class, w and h': '#e6550d', 'type (oracle)': '#9e9ac8'}

_log = []


def say(text: str = '') -> None:
    print(text, flush=True)
    _log.append(text)


def names(bins):
    return [f'{a:g}-{b:g}' if b < 1e9 else f'{a:g}+' for a, b in bins]


def binof(v, bins):
    return next(n for (a, b), n in zip(bins, names(bins)) if a <= v < b)


# ---------------------------------------------------------------- data

def load_objects(run: str) -> list[dict]:
    """Every labelled object of a run: class, visible box, distance, type (from meta/)."""
    out = []
    for path in sorted((GEN / run / 'meta').glob('*.json')):
        meta = json.load(open(path))
        placed = meta.get('placed', [])
        for o in meta.get('objects', []):
            if not o.get('labelled') or not o.get('distance'):
                continue
            box = o['visible']
            # The type: the placed model whose drawn size matches (both rounded differently).
            kind = None
            if o.get('drawn_px') is not None and placed:
                p = min(placed, key=lambda p: abs(p['drawn_px'] - o['drawn_px']))
                if abs(p['drawn_px'] - o['drawn_px']) < 0.2:
                    kind = p['variant'].split('/')[0]
            out.append({'run': run, 'image': meta['image'].rsplit('.', 1)[0],
                        'cls': CLASSES.index(o['class']), 'w': box['width'], 'h': box['height'],
                        'u': box['u'], 'v': box['v'], 'r': o['distance'],
                        'vis': o.get('visibility', 1.0), 'type': kind,
                        'edge': box['x0'] <= 1 or box['y0'] <= 1 or box['x1'] >= W - 1 or box['y1'] >= H - 1})
    return out


# ---------------------------------------------------------------- the fit

def fit(objs: list[dict]) -> dict:
    good = [o for o in objs if not o['edge'] and min(o['w'], o['h']) >= 1]
    model = {'f_px': F, 'class_longer': {}, 'class_area': {}, 'class_wh': {}, 'type_longer': {}}
    for c, name in enumerate(CLASSES):
        oc = [o for o in good if o['cls'] == c]
        if not oc:
            continue
        model['class_longer'][name] = float(np.exp(np.median([math.log(o['r'] * max(o['w'], o['h']) / F) for o in oc])))
        model['class_area'][name] = float(np.exp(np.median([math.log(o['r'] * math.sqrt(o['w'] * o['h']) / F) for o in oc])))
        # log r = a + b log w + c log h, least squares
        X = np.array([[1, math.log(o['w']), math.log(o['h'])] for o in oc])
        y = np.array([math.log(o['r']) for o in oc])
        model['class_wh'][name] = [float(v) for v in np.linalg.lstsq(X, y, rcond=None)[0]]
    by_type = defaultdict(list)
    for o in good:
        if o['type']:
            by_type[o['type']].append(math.log(o['r'] * max(o['w'], o['h']) / F))
    model['type_longer'] = {k: float(np.exp(np.median(v))) for k, v in sorted(by_type.items())}
    model['n_fit'] = len(good)
    return model


def estimate(model: dict, method: str, cls: int, w: float, h: float, kind: str | None) -> float:
    w, h = max(w, 1.0), max(h, 1.0)
    name = CLASSES[cls]
    if method == 'old 0.5 m':
        return F * OLD_SIZE / max(w, h)
    if method == 'class, longer side':
        return F * model['class_longer'][name] / max(w, h)
    if method == 'class, sqrt(w*h)':
        return F * model['class_area'][name] / math.sqrt(w * h)
    if method == 'class, w and h':
        a, b, c = model['class_wh'][name]
        return math.exp(a + b * math.log(w) + c * math.log(h))
    if method == 'type (oracle)':
        s = model['type_longer'].get(kind) if kind else None
        return F * (s if s else model['class_longer'][name]) / max(w, h)
    raise ValueError(method)


# ---------------------------------------------------------------- evaluation

def stats(pairs):
    """pairs: (estimate, truth). Median |error| %, 90 % under, median bias (est/true), share within 25 %."""
    if not pairs:
        return None
    rel = np.array([e / t for e, t in pairs])
    ab = np.abs(rel - 1)
    return {'n': len(pairs), 'median_abs': float(np.median(ab)), 'p90_abs': float(np.percentile(ab, 90)),
            'bias': float(np.median(rel)), 'within25': float(np.mean(ab <= 0.25))}


def fmt(s):
    if s is None:
        return '      -'
    return (f"{100 * s['median_abs']:5.0f} % (90 %: {100 * s['p90_abs']:4.0f} %, "
            f"bias x{s['bias']:.2f}, within 25 %: {100 * s['within25']:3.0f} %)")


def label_box_eval(model, objs):
    """{method: [(est, true, size px, class, distance)]} on the label boxes."""
    res = {}
    for m in METHODS:
        res[m] = [(estimate(model, m, o['cls'], o['w'], o['h'], o['type']), o['r'],
                   max(o['w'], o['h']), o['cls'], o['r']) for o in objs]
    return res


def detector_boxes(runs, objs, conf):
    """{detector: [(pred cls, w, h, truth obj)]}: each detector's own box for the objects it
    found (centre within max(3 px, half the object's size), highest score first)."""
    by_image = defaultdict(list)
    for o in objs:
        by_image[(o['run'], o['image'])].append(o)
    out = defaultdict(list)
    for run in runs:
        for cache in sorted((RAW / run).glob('*.json')):
            d = json.load(open(cache))
            thr = 0 if d['detector'] == 'baseline' else conf
            for image, dets in d['dets'].items():
                truth = by_image.get((run, image), [])
                used = set()
                for c, u, v, s, w, h in sorted((x for x in dets if x[3] >= thr), key=lambda x: -x[3]):
                    best = None
                    for i, o in enumerate(truth):
                        dd = math.dist((u, v), (o['u'], o['v']))
                        if i not in used and dd <= max(3, max(o['w'], o['h']) / 2) and (best is None or dd < best[1]):
                            best = (i, dd)
                    if best:
                        used.add(best[0])
                        out[d['detector']].append((max(c, 0), w, h, truth[best[0]]))
    return out


def report_table(title, res):
    say(title)
    say(f'  {"method":20s} relative distance error: median (90 % under, median est/true, share within 25 %)')
    for m, rows in res.items():
        say(f'  {m:20s} {fmt(stats([(e, t) for e, t, *_ in rows]))}')


def by_bins(res, idx, bins, cls=None):
    labels = names(bins)
    table = {}
    for m, rows in res.items():
        groups = defaultdict(list)
        for row in rows:
            if cls is None or row[3] == cls:
                groups[binof(row[idx], bins)].append(row[:2])
        table[m] = {b: stats(groups[b]) for b in labels}
    return table


def show_bins(title, table, bins):
    labels = names(bins)
    say(title + '  (median |error| %)')
    say('  ' + ' ' * 20 + ''.join(f'{b:>9s}' for b in labels))
    for m, t in table.items():
        say(f'  {m:20s}' + ''.join(f'{100 * t[b]["median_abs"]:8.0f}%' if t[b] else '        -' for b in labels))
    say('  ' + 'n'.ljust(20) + ''.join(f'{(next(iter(table.values()))[b] or {"n": 0})["n"]:9d}' for b in labels))


# ---------------------------------------------------------------- plots

def plots(res, model, fit_objs, out):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        say('No plots: matplotlib is not installed here')
        return
    for idx, bins, fname, xlabel in ((2, SB, 'error_by_size.png', 'box size (px, longer side)'),
                                     (4, DB, 'error_by_distance.png', 'true distance (m)')):
        fig, axes = plt.subplots(1, 2, figsize=(14, 4.8))
        for ax, c in zip(axes, (0, 1)):
            t = by_bins(res, idx, bins, c)
            labels = names(bins)
            for m in METHODS:
                ax.plot(labels, [100 * t[m][b]['median_abs'] if t[m][b] else float('nan') for b in labels],
                        'o:' if 'oracle' in m else 'o-', color=COLOURS[m], label=m, lw=2)
            ns = [(t[METHODS[0]][b] or {'n': 0})['n'] for b in labels]
            ax.set_xticks(range(len(labels)), [f'{b}\n(n={n})' for b, n in zip(labels, ns)])
            ax.set(xlabel=xlabel, ylabel='median |distance error| (%)', title=f'{CLASSES[c]}s, label boxes')
            ax.set_ylim(0, None)
            ax.grid(alpha=0.3)
            ax.legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(out / fname, dpi=110)
        plt.close(fig)

    shown = ('old 0.5 m', 'class, longer side', 'class, w and h')
    fig, axes = plt.subplots(1, len(shown), figsize=(5.2 * len(shown), 5), sharey=True)
    for ax, m in zip(axes, shown):
        for c, mk in ((0, 'o'), (1, '^')):
            rows = [r for r in res[m] if r[3] == c]
            ax.scatter([r[1] for r in rows], [r[0] for r in rows], s=6, alpha=0.4, marker=mk, label=f'{CLASSES[c]}s')
        lim = (1, 300)
        ax.plot(lim, lim, 'k--', lw=1)
        ax.set(xscale='log', yscale='log', xlim=lim, ylim=lim, xlabel='true distance (m)', title=m)
        ax.grid(alpha=0.3, which='both')
        ax.legend(fontsize=8)
    axes[0].set_ylabel('estimated distance (m)')
    fig.tight_layout()
    fig.savefig(out / 'est_vs_true.png', dpi=110)
    plt.close(fig)

    # Why one size cannot fit: the size the box sees, per drone/bird type.
    by_type = defaultdict(list)
    for o in fit_objs:
        if o['type'] and not o['edge']:
            by_type[o['type']].append(o['r'] * max(o['w'], o['h']) / F)
    kinds = sorted(by_type, key=lambda k: np.median(by_type[k]))
    fig, ax = plt.subplots(figsize=(max(8, 0.9 * len(kinds)), 4.8))
    ax.boxplot([by_type[k] for k in kinds], showfliers=False)
    ax.set_xticks(range(1, len(kinds) + 1), [f'{k}\n(n={len(by_type[k])})' for k in kinds], rotation=30, fontsize=8)
    ax.axhline(OLD_SIZE, color='#444444', ls='--', label='0.5 m (live today)')
    for name, col in zip(CLASSES, ('#3182bd', '#e6550d')):
        if name in model['class_longer']:
            ax.axhline(model['class_longer'][name], color=col, ls=':', label=f'fitted {name} {model["class_longer"][name]:.2f} m')
    ax.set(ylabel='size the box sees: distance x longer side / f (m)', title='apparent size by type (fit runs)')
    ax.grid(alpha=0.3, axis='y')
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out / 'size_by_type.png', dpi=110)
    plt.close(fig)


# ---------------------------------------------------------------- main

def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--fit', nargs='+', default=['b06', 'b07', 'b08', 'b09', 'b10', 'b11', 'b12', 'b13'], help='runs to fit on')
    p.add_argument('--test', nargs='+', default=['b14', 'b15'], help='runs to check on')
    p.add_argument('--conf', type=float, default=0.25, help='YOLO threshold for the detector boxes')
    p.add_argument('--out', type=Path, default=OUT)
    a = p.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)

    fit_objs = [o for r in a.fit for o in load_objects(r)]
    test_objs = [o for r in a.test for o in load_objects(r)]
    model = fit(fit_objs)
    json.dump(model, open(a.out / 'fit.json', 'w'), indent=1)

    say(f'Fit on {" ".join(a.fit)}: {model["n_fit"]} objects (not cut by the border); '
        f'test on {" ".join(a.test)}: {len(test_objs)} labelled objects')
    say(f'f = {F:.1f} px. Fitted size per class (longer side): ' +
        ', '.join(f'{k} {v:.3f} m' for k, v in model['class_longer'].items()) + f' (live uses {OLD_SIZE} m)')
    say('  sqrt(w*h): ' + ', '.join(f'{k} {v:.3f} m' for k, v in model['class_area'].items()))
    say('  log r = a + b log w + c log h: ' +
        ', '.join(f'{k} a={v[0]:.2f} b={v[1]:.2f} c={v[2]:.2f}' for k, v in model['class_wh'].items()) +
        '  (b + c = -1 would be pure pinhole)')
    say('  by type (longer side): ' + ', '.join(f'{k} {v:.2f}' for k, v in model['type_longer'].items()))

    res = label_box_eval(model, test_objs)
    for c, name in enumerate(CLASSES):
        report_table(f'\n=== {name}s, label boxes', {m: [r for r in rows if r[3] == c] for m, rows in res.items()})
    for c, name in enumerate(CLASSES):
        say('')
        show_bins(f'{name}s by size (px)', by_bins(res, 2, SB, c), SB)
        show_bins(f'{name}s by distance (m)', by_bins(res, 4, DB, c), DB)

    dets = detector_boxes(a.test, test_objs, a.conf)
    if not dets:
        say(f'\nNo detector boxes: run comparedet on {" ".join(a.test)} first to check with them too.')
    for det, rows in dets.items():
        dres = {m: [(estimate(model, m, c, w, h, o['type']), o['r'], max(o['w'], o['h']), o['cls'], o['r'])
                    for c, w, h, o in rows] for m in METHODS if m != 'type (oracle)'}
        report_table(f'\n=== {det} boxes ({len(rows)} found objects'
                     f'{"" if det == "baseline" else f", conf >= {a.conf}, predicted class"}), both classes', dres)

    plots(res, model, fit_objs, a.out)
    (a.out / 'summary.txt').write_text('\n'.join(_log) + '\n')
    print(f'\nResults in {a.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
