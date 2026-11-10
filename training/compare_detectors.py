"""Baseline vs several YOLO models on labelled gendata runs (`mount-backend.sh comparedet`).

    python3 training/compare_detectors.py data/gen/b14 data/gen/b15 \
        --weights models/A.onnx models/B.onnx [--conf 0.25] [--redo]

Every detector runs on every image of the runs (all .onnx in models/ when --weights is left
out, plus the baseline). Detections are cached per run and detector in
data/results/compare/raw/RUN/DETECTOR.json, so adding a model later only runs that model
(--redo runs them all again). A detection counts when its centre is within max(3 px, half
the object's size) of a labelled object; the class is ignored for that and reported
separately in the confusion matrix.

Output in data/results/compare/RUNS/ (e.g. b14+b15/):
  summary.txt        recall by size and distance, false alarms/image, confusion, centre
                     error, speed, threshold sweep (also printed)
  compare.png        recall by size/distance (drones, birds), recall vs false alarms, speed
  threshold.png      recall and false alarms vs confidence threshold, per model (without the
                     off-scale detectors, OFF_SCALE: dvb), and threshold_dvb.png for those
  confusion.png      drone/bird/background confusion per model, wrong-class rate by size
  localisation.png   centre error (degrees) by size and its distribution
  false_alarms_X.png crops of every false alarm of detector X (check for own-body hits)
"""
import argparse
import csv
import json
import math
import sys
import time
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np

from swarm_backend.detect import candidates
from swarm_backend.detect.yolo_onnx import YoloOnnx

REPO = Path(__file__).resolve().parents[1]
RESULTS = REPO / 'data' / 'results' / 'compare'
W, H, HFOV = 1280, 720, 2.0
F = (W / 2) / math.tan(HFOV / 2)  # focal length in px (~411)
SB = [(0, 4), (4, 8), (8, 16), (16, 32), (32, 64), (64, 1e9)]
DB = [(0, 10), (10, 20), (20, 40), (40, 80), (80, 1e9)]
CLASSES = ('drone', 'bird')
SWEEP = (0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9)
TABLE_CONFS = (0.1, 0.25, 0.5)
COLOURS = ['#3182bd', '#e6550d', '#31a354', '#756bb1', '#d6616b', '#8c6d31']
BASE_COLOUR = '#444444'
# Detectors whose false alarms (several per image) dwarf everyone else's (a few per hundred
# images): left out of the shared false-alarm plots, which are then readable, and drawn alone
# in threshold_<name>.png.
OFF_SCALE = ('dvb',)

_log = []


def say(text: str = '') -> None:
    print(text, flush=True)
    _log.append(text)


# ---------------------------------------------------------------- running the detectors

def det_name(weights: Path | None) -> str:
    if weights is None:
        return 'baseline'
    return weights.stem.replace('yolo26n-', '').replace('-1280', '')


def load_truth(run: Path) -> list[dict]:
    dist = defaultdict(list)
    for r in csv.DictReader(open(run / 'objects.csv')):
        if r['labelled'] == 'True':
            dist[r['image']].append(float(r['distance']) if r['distance'] else None)
    rows = []
    for p in sorted((run / 'images').glob('*.png')):
        lines = (run / 'labels' / f'{p.stem}.txt').read_text().splitlines()
        truth = [(int(c), float(x) * W, float(y) * H, float(w) * W, float(h) * H)
                 for c, x, y, w, h in (l.split() for l in lines if l.strip())]
        d = dist.get(p.stem, [])
        rows.append({'run': run.name, 'image': p.stem, 'path': str(p), 'truth': truth,
                     'dist': d if len(d) == len(truth) else [None] * len(truth)})
    return rows


def detect_run(run: Path, rows: list[dict], weights: Path | None, redo: bool) -> dict:
    """Detections of one detector on one run: {image: [(cls, u, v, score, w, h)]}, times.
    The baseline's 'score' is its contrast and its class -1; YOLO keeps everything >= 0.05."""
    name = det_name(weights)
    cache = RESULTS / 'raw' / run.name / f'{name}.json'
    if cache.exists() and not redo:
        d = json.load(open(cache))
        # Only use it if it covers every image of the run (a run regenerated or a test on
        # a few images leaves a cache that does not).
        if all(r['image'] in d['dets'] for r in rows):
            return d
        say(f'{run.name}: cached {name} does not cover all {len(rows)} images, running it again')
    if weights is None:
        settings = candidates.Settings()
        fn = lambda rgb: [(-1, c.x + c.width / 2, c.y + c.height / 2, c.contrast, c.width, c.height)
                          for c in candidates.find_candidates(rgb, None, settings)]
    else:
        model = YoloOnnx(weights)
        fn = lambda rgb: [(b.cls, (b.x1 + b.x2) / 2, (b.y1 + b.y2) / 2, b.score,
                           b.x2 - b.x1, b.y2 - b.y1) for b in model(rgb, 0.05)]
    say(f'{run.name}: running {name} on {len(rows)} images')
    dets, times = {}, []
    for n, r in enumerate(rows, 1):
        rgb = cv2.cvtColor(cv2.imread(r['path']), cv2.COLOR_BGR2RGB)
        t = time.perf_counter()
        dets[r['image']] = fn(rgb)
        times.append(time.perf_counter() - t)
        if n % 25 == 0:
            print(f'  {n}/{len(rows)}', flush=True)
    d = {'detector': name, 'weights': str(weights) if weights else None, 'dets': dets,
         'times': times[1:]}  # the first frame includes start-up
    cache.parent.mkdir(parents=True, exist_ok=True)
    json.dump(d, open(cache, 'w'))
    return d


# ---------------------------------------------------------------- scoring

def binof(v, bins):
    return None if v is None else next(n for (a, b), n in zip(bins, names(bins)) if a <= v < b)


def names(bins):
    return [f'{a:g}-{b:g}' if b < 1e9 else f'{a:g}+' for a, b in bins]


def ray(u, v):
    x, y = u - W / 2, v - H / 2
    n = math.sqrt(x * x + y * y + F * F)
    return x / n, y / n, F / n


def angle_deg(a, b) -> float:
    ra, rb = ray(*a), ray(*b)
    return math.degrees(math.acos(max(-1.0, min(1.0, sum(p * q for p, q in zip(ra, rb))))))


def match(truth, dets, thr):
    """Greedy by score: [(det, truth index or None)] for the detections >= thr."""
    used = [False] * len(truth)
    out = []
    for det in sorted((d for d in dets if d[3] >= thr), key=lambda d: -d[3]):
        best = None
        for i, (tc, tu, tv, tw, th) in enumerate(truth):
            dd = math.dist(det[1:3], (tu, tv))
            if not used[i] and dd <= max(3, max(tw, th) / 2) and (best is None or dd < best[1]):
                best = (i, dd)
        if best:
            used[best[0]] = True
        out.append((det, best and best[0]))
    return out


def evaluate(rows: list[dict], dets: dict, thr: float) -> dict:
    size, dist = defaultdict(lambda: [0, 0]), defaultdict(lambda: [0, 0])
    conf = np.zeros((3, 3), int)  # truth drone/bird/background x predicted drone/bird/missed
    wrong_by_size = defaultdict(lambda: [0, 0])  # wrong class, hits
    loc, false = [], []  # (size px, error px, error deg), (row, det)
    for r in rows:
        truth = r['truth']
        hit = [None] * len(truth)
        for det, i in match(truth, dets[r['image']], thr):
            if i is None:
                false.append((r, det))
                if det[0] >= 0:
                    conf[2, det[0]] += 1
                continue
            hit[i] = det
            tc, tu, tv, tw, th = truth[i]
            loc.append((max(tw, th), math.dist(det[1:3], (tu, tv)), angle_deg(det[1:3], (tu, tv))))
            if det[0] >= 0:
                conf[tc, det[0]] += 1
                b = binof(max(tw, th), SB)
                wrong_by_size[(tc, b)][0] += det[0] != tc
                wrong_by_size[(tc, b)][1] += 1
        for (tc, _, _, tw, th), h, dm in zip(truth, hit, r['dist']):
            if h is None:
                conf[tc, 2] += 1
            for tab, b in ((size, binof(max(tw, th), SB)), (dist, binof(dm, DB))):
                if b:
                    tab[(tc, b)][0] += h is not None
                    tab[(tc, b)][1] += 1
    return {'size': size, 'dist': dist, 'fa': len(false) / len(rows), 'false': false,
            'conf': conf, 'wrong_by_size': wrong_by_size, 'loc': loc,
            'wrong': int(conf[0, 1] + conf[1, 0])}


def scores(rows, dets):
    hits, false = [], []
    for r in rows:
        for det, i in match(r['truth'], dets[r['image']], 0.0):
            (false if i is None else hits).append(det[3])
    return hits, false


def recall(tab, c):
    return sum(v[0] for (cc, _), v in tab.items() if cc == c), sum(v[1] for (cc, _), v in tab.items() if cc == c)


# ---------------------------------------------------------------- text report

def show(title, tab, bins):
    say(title)
    for c, name in enumerate(CLASSES):
        say('    ' + name.ljust(6) + '  '.join(f'{b:>6}: {tab[(c, b)][0]:3d}/{tab[(c, b)][1]:<3d}'
                                             for b in names(bins)))


def report(rows, results, conf_main):
    say(f'{len(rows)} images, {sum(len(r["truth"]) for r in rows)} labelled objects')
    for name, d in results.items():
        thrs = (0,) if name == 'baseline' else TABLE_CONFS
        for thr in thrs:
            e = evaluate(rows, d['dets'], thr)
            dr, br = recall(e['size'], 0), recall(e['size'], 1)
            say(f'\n=== {name}' + (f' conf >= {thr}' if thr else '') +
                f': drones {dr[0]}/{dr[1]}, birds {br[0]}/{br[1]}, false alarms/image {e["fa"]:.3f}' +
                (f', wrong class {e["wrong"]}' if thr else ''))
            show('  by size (px)', e['size'], SB)
            show('  by distance (m)', e['dist'], DB)
            errs = sorted(x[2] for x in e['loc'])
            if errs:
                say(f'  centre error: median {errs[len(errs) // 2]:.2f} deg, '
                    f'90% under {errs[int(len(errs) * .9)]:.2f} deg')
            if thr == conf_main:
                c = e['conf']
                say(f'  confusion (rows truth, cols predicted drone / bird / missed):')
                for i, n in enumerate(('drone', 'bird', 'backgr.')):
                    say(f'    {n:8s}' + ''.join(f'{v:7d}' for v in c[i]))
    say('\nThreshold sweep (all objects, class ignored): recall / false alarms per image')
    say('  conf ' + ''.join(f'{n[:22]:>24s}' for n in results if n != 'baseline'))
    sw = {n: scores(rows, d['dets']) for n, d in results.items() if n != 'baseline'}
    n_obj = sum(len(r['truth']) for r in rows)
    for t in SWEEP:
        say(f'  {t:4.2f} ' + ''.join(f'{sum(h >= t for h in hi) / n_obj:13.3f} /{sum(f >= t for f in fa) / len(rows):7.3f}  '
                                    for hi, fa in sw.values()))
    say('\nSpeed (CPU, this machine):')
    for name, d in results.items():
        v = sorted(d['times'])
        say(f'  {name:24s} median {1000 * v[len(v) // 2]:5.0f} ms/frame, 90% under {1000 * v[int(len(v) * .9)]:5.0f} ms')
    return sw


# ---------------------------------------------------------------- plots

def colour_of(results):
    cols, k = {}, 0
    for n in results:
        if n == 'baseline':
            cols[n] = BASE_COLOUR
        else:
            cols[n] = COLOURS[k % len(COLOURS)]
            k += 1
    return cols


def plot_compare(rows, results, ev, sw, conf_main, out, title, plt):
    cols = colour_of(results)
    fig, axes = plt.subplots(2, 3, figsize=(18, 9.5))
    lab = lambda n: n if n == 'baseline' else f'{n} (conf {conf_main})'
    for row, c in enumerate((0, 1)):
        for col, (key, bins, xlabel) in enumerate((('size', SB, 'object size (px, longer side)'),
                                                    ('dist', DB, 'distance (m)'))):
            ax = axes[row][col]
            labels = names(bins)
            for n, e in ev.items():
                tab = e[key]
                rec = [tab[(c, b)][0] / tab[(c, b)][1] if tab[(c, b)][1] else float('nan') for b in labels]
                ax.plot(labels, rec, 'o-', label=lab(n), color=cols[n], lw=2)
            counts = [next(iter(ev.values()))[key][(c, b)][1] for b in labels]
            ax.set_xticks(range(len(labels)), [f'{b}\n(n={k})' for b, k in zip(labels, counts)])
            ax.set_ylim(-0.03, 1.03)
            ax.set_xlabel(xlabel)
            ax.set_ylabel(f'recall, {CLASSES[c]}s')
            ax.grid(alpha=0.3)
            if row == 0 and col == 0:
                ax.legend(loc='lower right', fontsize=8)
    ax = axes[0][2]
    n_obj = sum(len(r['truth']) for r in rows)
    for n, (hi, fa) in sw.items():
        if n in OFF_SCALE:
            continue
        ax.plot([sum(f >= t for f in fa) / len(rows) for t in SWEEP],
                [sum(h >= t for h in hi) / n_obj for t in SWEEP], 'o-', color=cols[n], label=n, ms=4)
        e = ev[n]
        ax.scatter(e['fa'], sum(recall(e['size'], c)[0] for c in (0, 1)) / n_obj,
                   s=160, facecolors='none', edgecolors=cols[n], lw=2)
    if 'baseline' in ev:
        e = ev['baseline']
        ax.scatter(e['fa'], sum(recall(e['size'], c)[0] for c in (0, 1)) / n_obj, s=120,
                   color=BASE_COLOUR, label='baseline', zorder=3)
    ax.set_xscale('symlog', linthresh=0.01)
    ax.set_xlabel('false alarms per image (symlog)')
    ax.set_ylabel('recall, all objects')
    off = [n for n in sw if n in OFF_SCALE]
    ax.set_title(f'recall vs false alarms over thresholds 0.05-0.9 (ring = conf {conf_main})'
                 + (f'\n{", ".join(off)} off scale: see threshold_{off[0]}.png' if off else ''), fontsize=10)
    ax.set_ylim(0, 1.03)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    ax = axes[1][2]
    med = {n: 1000 * sorted(d['times'])[len(d['times']) // 2] for n, d in results.items()}
    bars = ax.bar(range(len(med)), list(med.values()), color=[cols[n] for n in med])
    ax.set_xticks(range(len(med)), list(med), rotation=15, fontsize=8)
    for b, v in zip(bars, med.values()):
        ax.annotate(f'{v:.0f} ms\n({1000 / v:.1f} fps)', (b.get_x() + b.get_width() / 2, v),
                    ha='center', va='bottom', fontsize=8)
    ax.set_ylabel('median ms per frame (CPU)')
    ax.set_title('speed', fontsize=10)
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(out / 'compare.png', dpi=110)
    plt.close(fig)


def plot_threshold(rows, results, sw, out, plt, only=None, fname='threshold.png'):
    """only: draw just these detectors; None: all but OFF_SCALE."""
    cols = colour_of(results)
    sw = {n: v for n, v in sw.items() if (n in only if only else n not in OFF_SCALE)}
    if not sw:
        return
    n_obj = sum(len(r['truth']) for r in rows)
    fig, (a1, a2, a3) = plt.subplots(1, 3, figsize=(18, 4.8))
    for n, (hi, fa) in sw.items():
        a1.plot(SWEEP, [sum(h >= t for h in hi) / n_obj for t in SWEEP], 'o-', color=cols[n], label=n)
        a2.plot(SWEEP, [sum(f >= t for f in fa) / len(rows) for t in SWEEP], 's--', color=cols[n], label=n)
    a1.set(xlabel='confidence threshold', ylabel='recall, all objects', ylim=(0, 1.03), title='recall vs threshold')
    a2.set(xlabel='confidence threshold', ylabel='false alarms per image', title='false alarms vs threshold')
    for a in (a1, a2):
        a.grid(alpha=0.3)
        a.legend(fontsize=8)
    bins = [i / 20 for i in range(21)]
    for n, (hi, fa) in sw.items():
        a3.hist(fa, bins=bins, histtype='step', lw=2, color=cols[n], label=f'{n} false alarms (n={len(fa)})')
        a3.hist(hi, bins=bins, histtype='step', lw=1, ls=':', color=cols[n], label=f'{n} hits (n={len(hi)})')
    a3.set_yscale('log')
    a3.set(xlabel='YOLO confidence', ylabel='detections (log)', title='confidence of hits (dotted) vs false alarms')
    a3.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(out / fname, dpi=110)
    plt.close(fig)


def plot_confusion(ev, results, conf_main, out, plt):
    models = [n for n in ev if n != 'baseline']
    if not models:
        return
    cols = colour_of(results)
    fig, axes = plt.subplots(1, len(models) + 1, figsize=(4.6 * (len(models) + 1), 4.4))
    for ax, n in zip(axes, models):
        c = ev[n]['conf'].astype(float)
        norm = c / np.maximum(c.sum(axis=1, keepdims=True), 1)
        norm[2] = c[2] / max(c[2].sum(), 1)
        ax.imshow(norm, cmap='Blues', vmin=0, vmax=1)
        for i in range(3):
            for j in range(3):
                if i == 2 and j == 2:
                    continue
                ax.text(j, i, f'{int(c[i, j])}\n{norm[i, j]:.0%}', ha='center', va='center',
                        color='white' if norm[i, j] > 0.6 else 'black', fontsize=9)
        ax.set_xticks(range(3), ['drone', 'bird', 'missed'])
        ax.set_yticks(range(3), ['drone', 'bird', 'background'])
        ax.set_xlabel('predicted')
        ax.set_ylabel('truth')
        ax.set_title(f'{n} (conf {conf_main})', fontsize=10)
    ax = axes[-1]
    labels = names(SB)
    for n in models:
        wb = ev[n]['wrong_by_size']
        for c, ls in ((0, '-'), (1, '--')):
            ax.plot(labels, [wb[(c, b)][0] / wb[(c, b)][1] if wb[(c, b)][1] else float('nan') for b in labels],
                    'o' + ls, color=cols[n], label=f'{n}, {CLASSES[c]}s')
    ax.set_xlabel('object size (px)')
    ax.set_ylabel('wrong class, share of found objects')
    ax.set_title('class mistakes by size', fontsize=10)
    ax.set_ylim(0, 1)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(out / 'confusion.png', dpi=110)
    plt.close(fig)


def plot_localisation(ev, results, conf_main, out, plt):
    cols = colour_of(results)
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 4.6))
    labels = names(SB)
    for n, e in ev.items():
        by = defaultdict(list)
        for s, _, deg in e['loc']:
            by[binof(s, SB)].append(deg)
        a1.plot(labels, [float(np.median(by[b])) if by[b] else float('nan') for b in labels], 'o-',
                color=cols[n], label=n)
        errs = sorted(x[2] for x in e['loc'])
        if errs:
            a2.plot(errs, np.arange(1, len(errs) + 1) / len(errs), color=cols[n], label=n, lw=2)
    a1.set(xlabel='object size (px)', ylabel='median centre error (deg)',
           title=f'centre error of found objects by size (YOLO conf {conf_main})')
    a2.set(xlabel='centre error (deg)', ylabel='share of found objects', xlim=(0, None),
           title='distribution of centre error')
    for a in (a1, a2):
        a.grid(alpha=0.3)
        a.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out / 'localisation.png', dpi=110)
    plt.close(fig)


def false_alarm_sheet(name, e, out, plt, limit=60):
    """A grid of crops around each false alarm (3x the box, at least 48 px) with its score,
    image and whether it touches the image border (where the own body's legs and
    propellers show up)."""
    fa = sorted(e['false'], key=lambda x: -x[1][3])[:limit]
    path = out / f'false_alarms_{name}.png'
    if not fa:
        path.unlink(missing_ok=True)
        return 0
    cols = 10
    rws = math.ceil(len(fa) / cols)
    fig, axes = plt.subplots(rws, cols, figsize=(cols * 1.7, rws * 1.95), squeeze=False)
    border = 0
    cache = {}
    for ax, (r, (c, u, v, s, w, h)) in zip(axes.flat, fa):
        img = cache.setdefault(r['path'], cv2.cvtColor(cv2.imread(r['path']), cv2.COLOR_BGR2RGB))
        half = max(24, 1.5 * max(w, h))
        x0, x1 = int(max(0, u - half)), int(min(W, u + half))
        y0, y1 = int(max(0, v - half)), int(min(H, v + half))
        ax.imshow(img[y0:y1, x0:x1])
        ax.add_patch(plt.Rectangle((u - w / 2 - x0, v - h / 2 - y0), w, h, fill=False,
                                   ec='red' if c != 1 else 'lime', lw=1))
        edge = u - w / 2 <= 2 or v - h / 2 <= 2 or u + w / 2 >= W - 2 or v + h / 2 >= H - 2
        border += edge
        kind = '' if c < 0 else CLASSES[c][0]
        ax.set_title(f'{kind} {s:.2f}' + (' EDGE' if edge else '') + f'\n{r["image"]}', fontsize=6)
    for ax in axes.flat:
        ax.axis('off')
    fig.suptitle(f'{name}: false alarms ({len(fa)} of {len(e["false"])}, highest score first; '
                 'EDGE = touches the image border)', fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return border


# ---------------------------------------------------------------- main

def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('runs', type=Path, nargs='+', help='gendata runs with images/, labels/, objects.csv')
    p.add_argument('--weights', type=Path, nargs='*',
                   help='YOLO .onnx files (default: every .onnx directly under models/)')
    p.add_argument('--no-baseline', action='store_true', help='leave out the baseline detector')
    p.add_argument('--conf', type=float, default=0.25,
                   help='YOLO threshold for the plots, confusion matrix and false-alarm crops')
    p.add_argument('--redo', action='store_true', help='run the detectors again, ignore the cache')
    p.add_argument('--out', type=Path, help='output folder (default data/results/compare/RUN+RUN)')
    a = p.parse_args()

    weights = a.weights or sorted((REPO / 'models').glob('*.onnx'))
    detectors = ([] if a.no_baseline else [None]) + list(weights)
    out = a.out or RESULTS / '+'.join(r.name for r in a.runs)
    out.mkdir(parents=True, exist_ok=True)

    rows, results = [], {}
    for run in a.runs:
        rr = load_truth(run)
        rows += rr
        for w in detectors:
            d = detect_run(run, rr, w, a.redo)
            res = results.setdefault(d['detector'], {'dets': {}, 'times': []})
            res['dets'].update(d['dets'])
            res['times'] += d['times']

    say(f'Runs: {" ".join(r.name for r in a.runs)}; detectors: {", ".join(results)}')
    sw = report(rows, results, a.conf)
    ev = {n: evaluate(rows, d['dets'], 0 if n == 'baseline' else a.conf) for n, d in results.items()}

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        say('No plots: matplotlib is not installed here')
        plt = None
    if plt:
        title = f'{" + ".join(r.name for r in a.runs)}: {len(rows)} images'
        plot_compare(rows, results, ev, sw, a.conf, out, title, plt)
        plot_threshold(rows, results, sw, out, plt)
        for n in OFF_SCALE:
            plot_threshold(rows, results, sw, out, plt, only=(n,), fname=f'threshold_{n}.png')
        plot_confusion(ev, results, a.conf, out, plt)
        plot_localisation(ev, results, a.conf, out, plt)
        say(f'\nFalse alarms (YOLO conf {a.conf}), crops in false_alarms_*.png:')
        for n, e in ev.items():
            edge = false_alarm_sheet(n, e, out, plt)
            say(f'  {n:24s} {len(e["false"]):4d} false alarms, {edge} of the shown ones touch the image border')
    (out / 'summary.txt').write_text('\n'.join(_log) + '\n')
    print(f'\nResults in {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
