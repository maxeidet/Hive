"""Baseline vs several YOLO models on labelled gendata runs (`mount-backend.sh comparedet`).

    python3 training/compare_detectors.py data/gen/b14 data/gen/b15 \
        --weights models/A.onnx models/B.onnx [--conf 0.25] [--redo]

Every detector runs on every image of the runs (all .onnx in models/ when --weights is left
out, plus the baseline). Detections are cached per run and detector in
data/results/compare/raw/RUN/DETECTOR.json, so adding a model later only runs that model
(--redo runs them all again). A detection counts when its centre is within max(3 px, half
the object's size) of a labelled object; the class is ignored for that and reported
separately in the confusion matrix.

Output in data/results/compare/RUNS/ (e.g. b14+b15/); every plot is a .pdf (vector, for the report):
  summary.txt        recall by size and distance, false alarms/image, confusion, centre
                     error, speed, threshold sweep (also printed)
  compare.pdf        recall by size/distance (drones, birds), recall vs false alarms, speed
  recall_by_size_bar.pdf  recall by size as grouped bars, one bar per detector (conf --conf)
  precision_by_size_bar.pdf  precision by detected box size: called drone, called bird
  precision_recall_all_by_size.pdf  precision and recall by size, class ignored, side by side
  single/            every panel of the multi-panel figures above as its own PDF
  dataset.pdf        the evaluation set: drones and birds by size, distance, per image
  pr_curve.pdf       precision vs recall over every YOLO threshold, with average precision:
                     drones, birds (right class needed) and all objects (class ignored)
  threshold.pdf      recall and false alarms vs confidence threshold, per model (without the
                     off-scale detectors, OFF_SCALE: dvb), and threshold_dvb.pdf for those
  confusion.pdf      drone/bird/background confusion per model, wrong-class rate by size
  localisation.pdf   centre error (degrees) by size and its distribution
  false_alarms_X.pdf crops of every false alarm of detector X (check for own-body hits)
  final/             for the report: dataset, recall_by_size_bar, precision_by_size_bar, pr_curve, speed_bar, the best model's
                     confusion matrix and results_and_method.pdf (training/final_report.py)
  per_model/X/       the plots above for detector X alone (threshold and confusion: YOLO only)
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
# in threshold_<name>.pdf.
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
    prec = defaultdict(lambda: [0, 0])  # detection size bin -> [hits, detections]
    # (class given, detection size bin) -> [on a real object of that class, detections]
    prec_cls = defaultdict(lambda: [0, 0])
    for r in rows:
        truth = r['truth']
        hit = [None] * len(truth)
        for det, i in match(truth, dets[r['image']], thr):
            pb = binof(max(det[4], det[5]), SB)
            prec[pb][0] += i is not None
            prec[pb][1] += 1
            if det[0] >= 0:
                prec_cls[(det[0], pb)][0] += i is not None and truth[i][0] == det[0]
                prec_cls[(det[0], pb)][1] += 1
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
            'wrong': int(conf[0, 1] + conf[1, 0]), 'prec': prec, 'prec_cls': prec_cls,
            'tp': sum(v[0] for v in prec.values()), 'ndet': sum(v[1] for v in prec.values())}


def scores_cls(rows, dets):
    """Per class c: (scores of detections called c that are on a real c, scores of the
    other detections called c, number of labelled c objects). The matching is the same
    class-blind one; a detection called drone on a bird counts against drone precision."""
    out = {}
    for c in (0, 1):
        hits, false = [], []
        for r in rows:
            for det, i in match(r['truth'], dets[r['image']], 0.0):
                if det[0] == c:
                    (hits if i is not None and r['truth'][i][0] == c else false).append(det[3])
        out[c] = (hits, false, sum(1 for r in rows for t in r['truth'] if t[0] == c))
    return out


def prec_of(e, c=None):
    """(correct, detections): class c given (prec_cls), or class ignored (prec)."""
    if c is None:
        return e['tp'], e['ndet']
    v = [x for (cc, _), x in e['prec_cls'].items() if cc == c]
    return sum(x[0] for x in v), sum(x[1] for x in v)


def scores(rows, dets):
    hits, false = [], []
    for r in rows:
        for det, i in match(r['truth'], dets[r['image']], 0.0):
            (false if i is None else hits).append(det[3])
    return hits, false


def pr_curve(hits, false, n_obj):
    """Precision and recall at every score threshold (class ignored), highest first, and
    the average precision (area under the curve, precision made non-increasing first, as in
    Pascal VOC). match() is greedy by score, so the matches of the detections >= t are the
    same at every t and one matching at threshold 0 gives the whole curve."""
    s = sorted([(h, 1) for h in hits] + [(f, 0) for f in false], key=lambda x: -x[0])
    tp = np.cumsum([x[1] for x in s])
    n = np.arange(1, len(s) + 1)
    rec, pre, thr = tp / max(n_obj, 1), tp / n, np.array([x[0] for x in s])
    if not len(s):
        return rec, pre, thr, 0.0
    env = np.maximum.accumulate(pre[::-1])[::-1]
    ap = float(np.sum(np.diff(np.concatenate(([0], rec))) * env))
    return rec, pre, thr, ap


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
                f': drones {dr[0]}/{dr[1]}, birds {br[0]}/{br[1]}, false alarms/image {e["fa"]:.3f}, '
                f'precision {e["tp"]}/{e["ndet"]} = {e["tp"] / max(e["ndet"], 1):.3f}' +
                (f', wrong class {e["wrong"]}' if thr else ''))
            show('  by size (px)', e['size'], SB)
            show('  by distance (m)', e['dist'], DB)
            say('  precision by detection size (px): ' + '  '.join(
                f'{b}: {e["prec"][b][0]}/{e["prec"][b][1]}' for b in names(SB)))
            if thr:
                say('  precision per class given: ' + ', '.join(
                    f'{CLASSES[c]} {prec_of(e, c)[0]}/{prec_of(e, c)[1]}' for c in (0, 1)))
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
    say('\nAverage precision (area under the precision-recall curve, class ignored):')
    for n, (hi, fa) in sw.items():
        sc = scores_cls(rows, results[n]['dets'])
        say(f'  {n:24s} AP {pr_curve(hi, fa, n_obj)[3]:.3f} (class ignored), '
            + ', '.join(f'{CLASSES[c]}s {pr_curve(*sc[c])[3]:.3f}' for c in (0, 1)))
    say('\nSpeed (CPU, this machine):')
    for name, d in results.items():
        v = sorted(d['times'])
        say(f'  {name:24s} median {1000 * v[len(v) // 2]:5.0f} ms/frame, 90% under {1000 * v[int(len(v) * .9)]:5.0f} ms')
    return sw


# ---------------------------------------------------------------- plots

def save(fig, path: Path) -> None:
    """Every plot as a .pdf: vector, for the report (crops in the false-alarm sheets stay
    images, embedded at their own resolution)."""
    fig.savefig(path.with_suffix('.pdf'))


_PALETTE = {}  # detector -> colour, fixed over all detectors so single-model plots match


def colour_of(results):
    if _PALETTE:
        return _PALETTE
    cols, k = {}, 0
    for n in results:
        if n == 'baseline':
            cols[n] = BASE_COLOUR
        else:
            cols[n] = COLOURS[k % len(COLOURS)]
            k += 1
    return cols


def plot_compare(rows, results, ev, sw, conf_main, out, title, plt, off_scale=OFF_SCALE):
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
        if n in off_scale:
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
    off = [n for n in sw if n in off_scale]
    ax.set_title(f'recall vs false alarms over thresholds 0.05-0.9 (ring = conf {conf_main})'
                 + (f'\n{", ".join(off)} off scale: see threshold_{off[0]}.pdf' if off else ''), fontsize=10)
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
    ax.set_ylim(0, 1.18 * max(med.values()))  # room for the labels above the bars
    ax.set_ylabel('median ms per frame (CPU)')
    ax.set_title('speed', fontsize=10)
    fig.suptitle(title)
    fig.tight_layout()
    save(fig, out / 'compare.pdf')
    plt.close(fig)


def figure(panels, out, fname, suptitle, plt, single=None, size=(8, 5.4)):
    """One figure with a panel per entry of panels [(single file name, panel title,
    draw(ax))], saved as out/fname; with single, every panel also alone as
    single/<its file name>, titled with the figure's title and the panel's."""
    n = len(panels)
    fig, axes = plt.subplots(1, n, figsize=(size[0] * n, size[1]), squeeze=False)
    for ax, (_, t, draw) in zip(axes[0], panels):
        draw(ax)
        ax.set_title(t, fontsize=11)
    fig.suptitle(suptitle)
    fig.tight_layout()
    save(fig, out / fname)
    plt.close(fig)
    if single is None:
        return
    single.mkdir(parents=True, exist_ok=True)
    for name, t, draw in panels:
        fig, ax = plt.subplots(figsize=size)
        draw(ax)
        ax.set_title(f'{suptitle}\n{t}', fontsize=10)
        fig.tight_layout()
        save(fig, single / name)
        plt.close(fig)


def _label(n, conf_main, c=None):
    if n == 'baseline':
        return 'baseline (no class: all its detections)' if c is not None else 'baseline'
    return f'{n} (conf {conf_main})'


def _bars(ax, cols, conf_main, tabs, labels, ticks=None, c=None, counts_on_bars=True):
    """Grouped bars, one per detector: tabs[n] = [[hits, total] per bin]; % (and n) on top."""
    x = np.arange(len(labels))
    width = 0.8 / max(len(tabs), 1)
    for k, (n, tab) in enumerate(tabs.items()):
        v = [t[0] / t[1] if t[1] else float('nan') for t in tab]
        pos = x - 0.4 + width * (k + 0.5)
        ax.bar(pos, v, width * 0.92, color=cols[n], label=_label(n, conf_main, c))
        for p_, y, t in zip(pos, v, tab):
            if not math.isnan(y):
                ax.annotate(f'{y:.0%}\nn={t[1]}' if counts_on_bars else f'{y:.0%}', (p_, y),
                            ha='center', va='bottom', fontsize=5.5 if counts_on_bars else 6.5)
    ax.set_xticks(x, ticks or labels)
    ax.set_ylim(0, 1.15)
    ax.grid(axis='y', alpha=0.3)
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.16 if ticks else -0.13),
              ncol=2, fontsize=8, frameon=False)


def _prec_tabs(ev, c, labels):
    """Precision counts per bin: detections called class c, or all detections (c None). The
    baseline gives no class, so in the per-class panels it shows its all-detections precision."""
    return {n: [e['prec'][b] if c is None or n == 'baseline' else e['prec_cls'][(c, b)] for b in labels]
            for n, e in ev.items()}


def _recall_all_tabs(ev, labels):
    return {n: [[e['size'][(0, b)][0] + e['size'][(1, b)][0], e['size'][(0, b)][1] + e['size'][(1, b)][1]]
                for b in labels] for n, e in ev.items()}


def plot_recall_bar(ev, results, conf_main, out, title, plt, single=None):
    """Recall per object size bin (SB, longer side px), one bar per detector, one panel each
    for drones and birds; bin counts under the ticks."""
    cols = colour_of(results)
    labels = names(SB)

    def panel(c):
        def draw(ax):
            tabs = {n: [e['size'][(c, b)] for b in labels] for n, e in ev.items()}
            counts = [next(iter(ev.values()))['size'][(c, b)][1] for b in labels]
            _bars(ax, cols, conf_main, tabs, labels, [f'{b}\n(n={k})' for b, k in zip(labels, counts)],
                  counts_on_bars=False)
            ax.set(xlabel='object size (px, longer side)', ylabel='recall (share of labelled objects found)')
        return (f'recall_by_size_bar_{CLASSES[c]}s.pdf', f'{CLASSES[c]}s', draw)

    figure([panel(0), panel(1)], out, 'recall_by_size_bar.pdf', f'{title}: recall by object size',
           plt, single)


def _ring(rec, pre, thr, conf_main):
    """The point of a precision-recall curve at threshold conf_main."""
    k = int(np.sum(thr >= conf_main)) - 1
    return (rec[k], pre[k]) if k >= 0 else (0.0, 1.0)


def plot_pr(rows, results, ev, sw, conf_main, out, title, plt, single=None):
    """Precision-recall curves over every YOLO threshold (ring at --conf) with AP: drones and
    birds (a detection is right only with the right class), and all objects with the class
    ignored, where the baseline (no score, no class) is one point."""
    cols = colour_of(results)
    n_obj = sum(len(r['truth']) for r in rows)

    def panel(c):
        def draw(ax):
            for n, (hi, fa) in sw.items():
                if c is None:
                    rec, pre, thr, ap = pr_curve(hi, fa, n_obj)
                else:
                    rec, pre, thr, ap = pr_curve(*scores_cls(rows, results[n]['dets'])[c])
                ax.plot(rec, pre, color=cols[n], lw=2, label=f'{n} (AP {ap:.2f})')
                if len(rec):
                    ax.scatter(*_ring(rec, pre, thr, conf_main), s=160, facecolors='none',
                               edgecolors=cols[n], lw=2, zorder=3)
            if c is None and 'baseline' in ev:
                e = ev['baseline']
                ax.scatter(sum(recall(e['size'], k)[0] for k in (0, 1)) / n_obj, e['tp'] / max(e['ndet'], 1),
                           s=120, color=BASE_COLOUR, label='baseline (no score: one point)', zorder=3)
            ax.set(xlabel='recall', ylabel='precision (share of detections that are right)',
                   xlim=(0, 1.02), ylim=(0, 1.03))
            ax.grid(alpha=0.3)
            ax.legend(loc='center left', fontsize=8)
        what = 'all objects, class ignored' if c is None else f'{CLASSES[c]}s (right class needed)'
        slug = 'all' if c is None else f'{CLASSES[c]}s'
        return (f'pr_curve_{slug}.pdf', what, draw)

    figure([panel(0), panel(1), panel(None)], out, 'pr_curve.pdf',
           f'{title}: precision vs recall over YOLO thresholds (ring = conf {conf_main})', plt, single,
           size=(6.5, 6))


def plot_precision_bar(ev, results, conf_main, out, title, plt, single=None):
    """Precision per detection size bin (the detected box's longer side, so false alarms have
    a size too), one bar per detector, for detections called drone and called bird (right only
    on a real object of that class). The baseline gives no class: its bar is the precision of
    all its detections."""
    cols = colour_of(results)
    labels = names(SB)

    def panel(c):
        def draw(ax):
            _bars(ax, cols, conf_main, _prec_tabs(ev, c, labels), labels, c=c)
            ax.set(xlabel='detected box size (px, longer side)',
                   ylabel='precision (share of detections that are right)')
        return (f'precision_by_size_bar_{CLASSES[c]}s.pdf', f'detections called {CLASSES[c]}', draw)

    figure([panel(0), panel(1)], out, 'precision_by_size_bar.pdf',
           f'{title}: precision by detection size and class (n = detections in the bin)', plt, single)


def plot_pr_all_bar(ev, results, conf_main, out, title, plt, single=None):
    """Class ignored, drones and birds together: precision by detected box size and recall by
    labelled object size (precision_recall_all_by_size.pdf; alone: precision_by_size_bar_all,
    recall_by_size_bar_all)."""
    cols = colour_of(results)
    labels = names(SB)

    def draw_p(ax):
        _bars(ax, cols, conf_main, _prec_tabs(ev, None, labels), labels)
        ax.set(xlabel='detected box size (px, longer side)',
               ylabel='precision (share of detections on a real object)')

    def draw_r(ax):
        _bars(ax, cols, conf_main, _recall_all_tabs(ev, labels), labels)
        ax.set(xlabel='object size (px, longer side)', ylabel='recall (share of labelled objects found)')

    figure([('precision_by_size_bar_all.pdf', 'precision, all classes (n = detections)', draw_p),
            ('recall_by_size_bar_all.pdf', 'recall, all classes (n = labelled objects)', draw_r)],
           out, 'precision_recall_all_by_size.pdf',
           f'{title}: precision and recall by size, drones and birds together', plt, single)


CLASS_COLOURS = ('#4c4c9d', '#b8860b')  # drones, birds: unlike any detector's colour


def plot_dataset(rows, out, title, plt, single=None):
    """What the evaluation runs hold: labelled drones and birds by size (longer side, px),
    by distance (m), and how many objects each image has."""
    def count_panel(key, bins, xlabel, slug, t):
        def draw(ax):
            labels = names(bins)
            x = np.arange(len(labels))
            for c in (0, 1):
                vals = [max(tr[3], tr[4]) if key == 'size' else d
                        for r in rows for tr, d in zip(r['truth'], r['dist']) if tr[0] == c]
                cnt = [sum(1 for v in vals if v is not None and binof(v, bins) == b) for b in labels]
                bars = ax.bar(x - 0.2 + 0.4 * c, cnt, 0.38, color=CLASS_COLOURS[c],
                              label=f'{CLASSES[c]}s (n={len(vals)})')
                for b_, v in zip(bars, cnt):
                    ax.annotate(str(v), (b_.get_x() + b_.get_width() / 2, v), ha='center', va='bottom', fontsize=8)
            ax.set_xticks(x, labels)
            ax.set(xlabel=xlabel, ylabel='labelled objects')
            ax.grid(axis='y', alpha=0.3)
            ax.legend(fontsize=9)
        return (f'dataset_{slug}.pdf', t, draw)

    per = [(sum(t[0] == 0 for t in r['truth']), sum(t[0] == 1 for t in r['truth'])) for r in rows]
    empty = sum(1 for p in per if p == (0, 0))

    def draw_per(ax):
        x = np.arange(max(max(p) for p in per) + 1)
        for c in (0, 1):
            cnt = [sum(1 for p in per if p[c] == k) for k in x]
            bars = ax.bar(x - 0.2 + 0.4 * c, cnt, 0.38, color=CLASS_COLOURS[c], label=f'{CLASSES[c]}s')
            for b_, v in zip(bars, cnt):
                ax.annotate(str(v), (b_.get_x() + b_.get_width() / 2, v), ha='center', va='bottom', fontsize=8)
        ax.set_xticks(x)
        ax.set(xlabel='objects of that class in the image', ylabel='images')
        ax.grid(axis='y', alpha=0.3)
        ax.legend(fontsize=9)

    figure([count_panel('size', SB, 'object size (px, longer side)', 'size', 'by object size'),
            count_panel('dist', DB, 'distance to the camera (m)', 'distance', 'by distance'),
            ('dataset_per_image.pdf', f'per image ({empty} of {len(rows)} images have no object)', draw_per)],
           out, 'dataset.pdf', f'{title}: labelled objects in the evaluation set', plt, single,
           size=(6.5, 4.8))


def plot_threshold(rows, results, sw, out, plt, only=None, fname='threshold.pdf'):
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
    save(fig, out / fname)
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
    save(fig, out / 'confusion.pdf')
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
    save(fig, out / 'localisation.pdf')
    plt.close(fig)


def false_alarm_sheet(name, e, out, plt, limit=60):
    """A grid of crops around each false alarm (3x the box, at least 48 px) with its score,
    image and whether it touches the image border (where the own body's legs and
    propellers show up)."""
    fa = sorted(e['false'], key=lambda x: -x[1][3])[:limit]
    path = out / f'false_alarms_{name}.pdf'
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
    save(fig, path)
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
        _PALETTE.update(colour_of(results))
        title = f'{" + ".join(r.name for r in a.runs)}: {len(rows)} images'
        plot_compare(rows, results, ev, sw, a.conf, out, title, plt)
        sg = out / 'single'
        plot_recall_bar(ev, results, a.conf, out, title, plt, sg)
        plot_precision_bar(ev, results, a.conf, out, title, plt, sg)
        plot_pr_all_bar(ev, results, a.conf, out, title, plt, sg)
        plot_pr(rows, results, ev, sw, a.conf, out, title, plt, sg)
        plot_dataset(rows, out, title, plt, sg)
        plot_threshold(rows, results, sw, out, plt)
        for n in OFF_SCALE:
            plot_threshold(rows, results, sw, out, plt, only=(n,), fname=f'threshold_{n}.pdf')
        plot_confusion(ev, results, a.conf, out, plt)
        plot_localisation(ev, results, a.conf, out, plt)
        say(f'\nFalse alarms (YOLO conf {a.conf}), crops in false_alarms_*.pdf:')
        for n, e in ev.items():
            edge = false_alarm_sheet(n, e, out, plt)
            say(f'  {n:24s} {len(e["false"]):4d} false alarms, {edge} of the shown ones touch the image border')
        # The same plots for each detector alone (false-alarm crops are per detector already).
        for n in results:
            one, ev1 = {n: results[n]}, {n: ev[n]}
            sw1 = {n: sw[n]} if n in sw else {}
            o = out / 'per_model' / n
            o.mkdir(parents=True, exist_ok=True)
            t = f'{n}, {title}'
            plot_compare(rows, one, ev1, sw1, a.conf, o, t, plt, off_scale=())
            plot_recall_bar(ev1, one, a.conf, o, t, plt, o / 'single')
            plot_precision_bar(ev1, one, a.conf, o, t, plt, o / 'single')
            plot_pr_all_bar(ev1, one, a.conf, o, t, plt, o / 'single')
            plot_pr(rows, one, ev1, sw1, a.conf, o, t, plt, o / 'single')
            plot_threshold(rows, one, sw1, o, plt, only=(n,))
            plot_confusion(ev1, one, a.conf, o, plt)
            plot_localisation(ev1, one, a.conf, o, plt)
        say(f'Single-detector versions of the plots in {out / "per_model"}/DETECTOR/')
        from final_report import write_final  # training/, next to this file
        final, best = write_final(rows, results, ev, sw, a.conf, out, [r.name for r in a.runs], plt,
                                  plot_recall_bar, title, colour_of(results), pr_curve, names, SB, recall,
                                  plot_precision_bar, plot_pr, scores_cls, prec_of, plot_dataset,
                                  plot_pr_all_bar)
        say(f'Report subset (best model {best}) and results_and_method.pdf in {final}')
    (out / 'summary.txt').write_text('\n'.join(_log) + '\n')
    print(f'\nResults in {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
