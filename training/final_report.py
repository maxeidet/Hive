"""The report-ready subset of `compare_detectors.py`, written to RUNS/final/ on every run.

  results_and_method.pdf   what the figures show, and how the data, models and evaluation
                           were made
  double/                  the figures with two or three panels: dataset, recall_by_size_bar
                           (drones, birds), precision_by_size_bar (called drone, called bird),
                           precision_recall_all_by_size (class ignored), pr_curve (drones,
                           birds, class ignored)
  single/                  every panel of those as its own PDF (<figure>_<panel>.pdf;
                           precision_by_size_bar_all and recall_by_size_bar_all are the two
                           panels of precision_recall_all_by_size), plus speed_bar and
                           confusion_matrix_BEST (the YOLO model with the highest mAP)

The method facts that are not in this repo's data (the training ran on a LiU school computer) are
in TRAINING below: update them when a model is retrained.
"""
import math
import textwrap

import numpy as np

# How the three YOLO models were trained (LiU school computer, RTX 4000 Ada GPU, Ultralytics 8.4, YOLO26n). Not
# readable from the .onnx files, so kept here by hand.
TRAINING = {
    'dvb': 'stage 1 only: COCO yolo26n.pt fine-tuned on the Drone vs Bird images '
           '(640 px, batch 32, mosaic off; best epoch 8, val mAP50 0.71, mAP50-95 0.49). '
           'Has never seen a Gazebo image.',
    'dvb-gazebo': 'stage 1 (the dvb model above), then stage 2 on the Gazebo set '
                  '(early-stopped at epoch 98, best epoch 83, val mAP50 0.777, mAP50-95 0.607).',
    'gazebo-only': 'COCO yolo26n.pt, then stage 2 on the Gazebo set only '
                   '(best epoch 86, val mAP50 0.827, mAP50-95 0.655).',
}
HARDWARE = 'Apple M1 (8 cores), in the px4-gazebo-vnc Docker container (Linux aarch64)'


def best_model(rows, results, sw, n_obj, pr_curve, scores_cls):
    """AP per YOLO model: class ignored ('all'), per class (0 drone, 1 bird) and their mean
    ('mean', i.e. mAP at our centre matching); the best model has the highest mean."""
    aps = {}
    for n, (hi, fa) in sw.items():
        sc = scores_cls(rows, results[n]['dets'])
        a = {'all': pr_curve(hi, fa, n_obj)[3], 0: pr_curve(*sc[0])[3], 1: pr_curve(*sc[1])[3]}
        a['mean'] = (a[0] + a[1]) / 2
        aps[n] = a
    return (max(aps, key=lambda n: aps[n]['mean']) if aps else None), aps


def speed_rows(results):
    rows = []
    base = None
    for n, d in results.items():
        v = sorted(d['times'])
        med, p90 = 1000 * v[len(v) // 2], 1000 * v[int(len(v) * .9)]
        if n == 'baseline':
            base = med
        rows.append([n, med, p90])
    return [[n, f'{m:.0f}', f'{p:.0f}', f'{1000 / m:.1f}', f'{m / base:.0f}x' if base else '-']
            for n, m, p in rows]


def plot_speed_bar(results, cols, out, plt):
    """Median ms per frame per detector as bars (as in compare.pdf), ms and fps on each bar."""
    med = {n: 1000 * sorted(d['times'])[len(d['times']) // 2] for n, d in results.items()}
    fig, ax = plt.subplots(figsize=(6.5, 4.5))
    bars = ax.bar(range(len(med)), list(med.values()), color=[cols[n] for n in med])
    ax.set_xticks(range(len(med)), list(med), fontsize=9)
    for b, v in zip(bars, med.values()):
        ax.annotate(f'{v:.0f} ms\n({1000 / v:.1f} fps)', (b.get_x() + b.get_width() / 2, v),
                    ha='center', va='bottom', fontsize=9)
    ax.set_ylim(0, 1.18 * max(med.values()))
    ax.set_ylabel('median ms per frame (CPU)')
    ax.grid(axis='y', alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / 'speed_bar.pdf')
    plt.close(fig)


def plot_confusion_matrix(name, e, conf_main, out, plt):
    c = e['conf'].astype(float)
    norm = c / np.maximum(c.sum(axis=1, keepdims=True), 1)
    norm[2] = c[2] / max(c[2].sum(), 1)
    fig, ax = plt.subplots(figsize=(5.2, 4.6))
    ax.imshow(norm, cmap='Blues', vmin=0, vmax=1)
    for i in range(3):
        for j in range(3):
            if i == 2 and j == 2:
                continue
            ax.text(j, i, f'{int(c[i, j])}\n{norm[i, j]:.0%}', ha='center', va='center',
                    color='white' if norm[i, j] > 0.6 else 'black', fontsize=10)
    ax.set_xticks(range(3), ['drone', 'bird', 'missed'])
    ax.set_yticks(range(3), ['drone', 'bird', 'background'])
    ax.set_xlabel('predicted')
    ax.set_ylabel('truth')
    ax.set_title(f'{name}, conf >= {conf_main}', fontsize=10)
    fig.tight_layout()
    fig.savefig(out / f'confusion_matrix_{name}.pdf')
    plt.close(fig)


# ---------------------------------------------------------------- the text

def _pct(a, b):
    return f'{a / b:.0%}' if b else '-'


def _sparse(rows, names, SB):
    """One sentence on the size bins with few objects, where the percentages are rough."""
    few = []
    for c, cname in ((0, 'drones'), (1, 'birds')):
        for (lo, hi), b in zip(SB, names(SB)):
            k = sum(1 for r in rows for t in r['truth'] if t[0] == c and lo <= max(t[3], t[4]) < hi)
            if k < 30:
                few.append(f'{cname} at {b} px ({k})')
    if not few:
        return 'Every size bin holds at least 30 objects of each class.'
    return ('Every size bin holds at least 30 objects of each class except ' + ', '.join(few)
            + ', so the percentages there are rough.')


def text_blocks(rows, results, ev, aps, best, conf_main, runs, names, SB, recall, prec_of):
    n_img = len(rows)
    n_obj = sum(len(r['truth']) for r in rows)
    n_dr = sum(1 for r in rows for t in r['truth'] if t[0] == 0)
    n_bd = n_obj - n_dr
    n_empty = sum(1 for r in rows if not r['truth'])
    run_list = ' and '.join(runs)
    yolo = [n for n in ev if n != 'baseline']

    def rec_all(e):
        return sum(recall(e['size'], c)[0] for c in (0, 1))

    B = []
    h = lambda t: B.append(('h', t))
    p = lambda t: B.append(('p', t))
    tab = lambda rows_, head, caption: B.append(('t', head, rows_, caption))

    B.append(('title', 'Drone and bird detection: results and method'))
    p(f'Evaluated on the Gazebo test runs {run_list}: {n_img} images with {n_obj} labelled '
      f'objects ({n_dr} drones, {n_bd} birds) and {n_empty} images with nothing in them. '
      f'YOLO detections count from confidence {conf_main}; the baseline has no confidence. '
      f'Figures with several panels are in double/, and every panel is also in single/ as its '
      f'own file (<figure>_<panel>.pdf). dataset.pdf shows how the drones and birds spread over '
      f'size, distance and images. '
      + _sparse(rows, names, SB))

    h('1. Results')
    p('Table 1 gives the overall numbers. Recall per class is the share of labelled drones '
      '(birds) that were found, whatever class the detector gave them; class mistakes are '
      'counted in the confusion matrix instead. Precision per class is the share of '
      'detections called drone (bird) that are on a real drone (bird), so a bird called a '
      'drone lowers drone precision. "Precision all" ignores the class: the share of '
      'detections that are on any real object, the only precision the baseline has, since it '
      'gives no class. Average precision (AP) is the area under the precision-recall curve '
      'over all confidence thresholds (pr_curve.pdf), per class; the mean of the two is '
      'the mAP.')
    t1 = []
    for n, e in ev.items():
        dr, br = recall(e['size'], 0), recall(e['size'], 1)
        pc = ['-', '-'] if n == 'baseline' else [_pct(*prec_of(e, c)) for c in (0, 1)]
        a = aps.get(n)
        t1.append([n, _pct(dr[0], dr[1]), _pct(br[0], br[1])] + pc +
                  [_pct(e['tp'], e['ndet']), f'{e["fa"]:.3f}'] +
                  ([f'{a[0]:.2f}', f'{a[1]:.2f}', f'{a["mean"]:.2f}'] if a else ['-'] * 3))
    tab(t1, ['detector', 'R drones', 'R birds', 'P drones', 'P birds', 'P all', 'FA / image',
             'AP drones', 'AP birds', 'mAP'],
        f'Table 1. Overall results on {run_list} (YOLO at conf >= {conf_main}). R = recall, '
        f'P = precision, FA = false alarms.')
    if best:
        others = [n for n in yolo if n != best]
        p(f'The best model is {best} (highest mAP, {aps[best]["mean"]:.2f}). '
          + ('; '.join(f'{n} reaches {aps[n]["mean"]:.2f}' for n in others) + '. ' if others else ''))
        eb = ev[best]
        p(f'For {best}, {_pct(*prec_of(eb, 0))} of the detections called drone are drones and '
          f'{_pct(*prec_of(eb, 1))} of those called bird are birds, while with the class '
          f'ignored {_pct(eb["tp"], eb["ndet"])} are on a real object. So nearly all of the '
          f'precision it loses comes from drones and birds taken for each other, not from boxes '
          f'on empty sky, and bird precision suffers most because birds are fewer.')
    p('Precision by detection size (precision_by_size_bar.pdf) bins the detections by the size '
      'of the box the detector drew, since a false alarm has no labelled object to take a '
      'size from. It has a panel for detections called drone and one for detections called '
      'bird. The baseline gives no class, so its bar there is the precision of all its '
      'detections, the same in both panels. precision_by_size_bar_all.pdf is precision with '
      'the class ignored; recall_by_size_bar_all.pdf is recall by size with drones and birds '
      'counted together. precision_recall_all_by_size.pdf puts precision and recall with the '
      'class ignored side by side, drones and birds counted together.')
    p('Recall by object size (recall_by_size_bar.pdf, Table 2) shows where each detector stops working. '
      'Size is the longer side of the labelled box in pixels; with this camera (1280 x 720, '
      '114.6 degrees wide, focal length about 411 px) a 0.5 m drone is 4 px wide at about 51 m '
      'and 20 px at about 10 m.')
    labels = names(SB)
    t2 = []
    for n, e in ev.items():
        for c, cname in enumerate(('drones', 'birds')):
            t2.append([f'{n}, {cname}'] + [_pct(*e['size'][(c, b)]) if e['size'][(c, b)][1] else '-'
                                           for b in labels])
    counts = [f'{b} px' for b in labels]
    tab(t2, ['detector'] + counts, 'Table 2. Recall by object size (longer side of the '
        'labelled box).')
    if best and 'baseline' in ev:
        eb, ey = ev['baseline'], ev[best]
        small = labels[0]
        p(f'The models trained on Gazebo images find most objects from 8 px up and over 90 % '
          f'from 16 px up. Below '
          f'4 px they find few ({_pct(*ey["size"][(0, small)])} of drones for {best}), while the '
          f'baseline finds {_pct(*eb["size"][(0, small)])}: there a dark dot against the sky is '
          f'all there is to see, and that is exactly what the baseline looks for. The baseline '
          f'falls off above about 32 px because it only accepts objects up to 40 px and a '
          f'large object is no longer a single dark blob. The pretrained dvb model, which '
          f'never saw a Gazebo image, only starts finding objects above 32 px.')
    if best:
        c = ev[best]['conf']
        p(f'Confusion matrix of {best} (confusion_matrix_{best}.pdf): of the drones found, '
          f'{_pct(c[0, 1], c[0, 0] + c[0, 1])} were called birds; of the birds found, '
          f'{_pct(c[1, 0], c[1, 0] + c[1, 1])} were called drones. The share of class mistakes '
          f'is highest under 8 px, where the two look alike, and close to zero from 16 px up. '
          f'In the matrix, the drone and bird rows are shares of each true class; the '
          f'background row is the false alarms, split by the class they were given. Of its {int(c[2].sum())} false '
          f'alarms on background, {int(c[2, 0])} were called drones.')
    sp = speed_rows(results)
    p('Speed (speed_bar.pdf) is the median time per 1280 x 720 frame on the CPU, including '
      f'pre- and post-processing, the first frame left out, measured on {HARDWARE}. '
      + '; '.join(f'{r[0]}: {r[1]} ms' for r in sp) + '. Absolute times depend on the computer; '
      'the ratio between detectors is what carries over. A YOLO26n at 1280 px on a CPU is far '
      'from the 1-10 ms per frame aimed at for a drone-class computer.')

    h('2. How the data was made')
    p('All images come from the Gazebo simulator with a bare-camera generator '
      '(mount-backend.sh gendata, swarm_backend/gen_data.py) in the hive_base world. The '
      'camera has the settings of the x500_gimbal drone camera: 1280 x 720 px, 2.0 rad '
      '(114.6 degrees) horizontal field of view, no distortion. For each image the camera gets '
      'a random position, heading and tilt and the sun a random elevation. Each image then '
      'gets up to three drones and zero to two birds, each placed at a random pixel and at the '
      'distance that gives a chosen drawn size between 2 and 100 px (weighted towards about 4 '
      'and 20 px). About 15 % of the images are left empty, and an unlabelled drone body sits '
      'where the camera drone\'s own legs and propellers would be seen.')
    p('Drones: look-only copies of five PX4 drone models (x500 0.66 m, px4vision 0.47 m, '
      'omniquad 0.6 m, quadtailsitter 0.7 m, standard_vtol 2.1 m, picked 35/20/15/15/15 %), '
      'half of them repainted in a plain colour (black, grey, white, red or olive) so the '
      'model learns "small drone" and not one airframe. Birds: five meshes in a flying pose '
      '(dove, seagull, gull, eagle, soaring bird), scaled to realistic wingspans and shown '
      'without wing animation.')
    p('Labels come from Gazebo\'s bounding-box camera, which shares the pose and settings of '
      'the image camera and reports the visible box of every labelled model, so partly hidden '
      'objects get the box of what is seen. Objects under 2 px or fully hidden get no label '
      '(they are not counted as missed). Each image gets a YOLO label file plus a record '
      'of every object\'s class, size and distance.')
    p('Runs of 200 images: b01-b04 and b06-b15 (b05 was dropped: its birds were missing). '
      'Split by run, so no scene appears in two parts: validation b03 and b13, test b14 and '
      'b15, training the other ten runs. Every result here is on the test runs only.')

    h('3. How the models were trained')
    p('All three are YOLO26n (the smallest YOLO26), trained with Ultralytics 8.4 on the GPU '
      'of a LiU school computer (NVIDIA RTX 4000 Ada), all layers trained, with two classes: drone and bird. They are exported to '
      'ONNX at 1280 px input and run with ONNX Runtime on the CPU, the same way the live '
      'detector node runs them.')
    p('Stage 1, real images: the Drone vs Bird data set (Mendeley Data, CC BY 4.0), relabelled '
      'from the file names (B = bird, D = drone) and split again into validation and test by '
      'blocks of frames from the same scene, so near-identical frames do not leak between '
      'parts. Trained at 640 px.')
    p('Stage 2, Gazebo images: 1280 px input, AdamW with learning rate 0.002, no frozen layers, '
      'batch 16, mosaic augmentation on, up to 100 epochs with early stopping after 15 '
      'epochs without improvement on the validation runs. The weights of the best epoch are '
      'kept.')
    for n in yolo:
        if n in TRAINING:
            p(f'{n}: {TRAINING[n]}')
    p('The baseline needs no training: it marks pixels clearly darker than the sky around '
      'them, joins them into regions and keeps the regions of a plausible size (up to 40 px) '
      'that are surrounded by sky. It cannot tell a drone from a bird and has no confidence '
      'score.')

    h('4. How the detectors were evaluated')
    p(f'Every detector runs on every image of {run_list} (training/compare_detectors.py, '
      'mount-backend.sh comparedet). A detection counts as finding a labelled object when its '
      'box centre lies within max(3 px, half the object\'s longer side) of the object\'s '
      'centre. Detections are taken highest confidence first, and each one claims the '
      'nearest object not yet claimed, so one object is found at most once and a second box '
      'on it is a false alarm. The class is ignored for this matching and scored separately '
      'in the confusion matrix. Detections that match nothing are false alarms.')
    p('Matching by centre distance instead of box overlap (IoU) is deliberate: what the '
      'drone uses downstream is the direction to the target, which comes from the box centre, '
      'and for objects of a few pixels a one-pixel box error would already ruin the IoU.')
    p('Recall is counted per labelled object and binned by object size; precision is counted '
      'per detection and binned by the detected box\'s size. The precision-recall curves and '
      'AP use one greedy matching at the lowest threshold (0.05): because matching is in '
      'order of confidence, the matches above any higher threshold are the same, so one '
      'matching gives the whole curve. AP is the area under that curve with the precision '
      'made non-increasing first, as in Pascal VOC. ROC curves are not used: a detector has no '
      'countable true negatives, so a false positive rate is not defined.')

    h('Credits')
    p('Drone vs Bird data set: Mendeley Data, CC BY 4.0. Bird meshes: "Dove Bird Rigged" by '
      'FourthGreen (Sketchfab, CC BY 4.0); "Flying seagull", "Flying gull" and "Bird" by Poly '
      'by Google (Poly Pizza, CC BY 3.0); "Eagle" by Robert Mirabelle (Poly Pizza, CC BY 3.0).')
    return B


def render_text(blocks, path, plt):
    """A4 pages of headings, wrapped paragraphs and simple tables, with matplotlib."""
    from matplotlib.backends.backend_pdf import PdfPages
    W_IN, H_IN, M = 8.27, 11.69, 0.85
    top, bottom = 1 - M / H_IN, M / H_IN
    lh = 0.205 / H_IN  # line height, 9.5 pt text
    pdf = PdfPages(path)
    state = {'fig': None, 'y': top}

    def new_page():
        if state['fig'] is not None:
            pdf.savefig(state['fig'])
            plt.close(state['fig'])
        state['fig'] = plt.figure(figsize=(W_IN, H_IN))
        state['y'] = top

    def need(lines):
        if state['y'] - lines * lh < bottom:
            new_page()

    def line(text, x=M / W_IN, **kw):
        state['fig'].text(x, state['y'], text, va='top', ha='left', **kw)
        state['y'] -= lh

    new_page()
    for b in blocks:
        if b[0] == 'title':
            line(b[1], fontsize=15, weight='bold')
            state['y'] -= lh
        elif b[0] == 'h':
            need(4)
            state['y'] -= 0.5 * lh
            line(b[1], fontsize=12, weight='bold')
            state['y'] -= 0.3 * lh
        elif b[0] == 'p':
            ls = textwrap.wrap(b[1], 98)
            need(min(len(ls), 3))
            for l in ls:
                need(1)
                line(l, fontsize=9.5)
            state['y'] -= 0.5 * lh
        elif b[0] == 't':
            head, rows, caption = b[1], b[2], b[3]
            need(len(rows) * 1.25 + 3)
            line(caption, fontsize=8.5, style='italic')
            state['y'] -= 0.2 * lh
            n = len(rows) + 1
            need(n * 1.25 + 1)
            fig = state['fig']
            h = n * lh * 1.25
            ax = fig.add_axes([M / W_IN, state['y'] - h, 1 - 2 * M / W_IN, h])
            ax.axis('off')
            first = 0.17
            widths = [first] + [(1 - first) / (len(head) - 1)] * (len(head) - 1)
            t = ax.table(cellText=rows, colLabels=head, loc='upper center', cellLoc='center',
                         bbox=[0, 0, 1, 1], colWidths=widths)
            t.auto_set_font_size(False)
            t.set_fontsize(7 if len(head) > 7 else 7.5)
            for (r, _), cell in t.get_celld().items():
                if r == 0:
                    cell.set_text_props(weight='bold')
            state['y'] -= h + 0.8 * lh
    pdf.savefig(state['fig'])
    plt.close(state['fig'])
    pdf.close()


def write_final(rows, results, ev, sw, conf_main, out, runs, plt, plot_recall_bar, title, cols,
                pr_curve, names, SB, recall, plot_precision_bar, plot_pr, scores_cls, prec_of,
                plot_dataset, plot_pr_all_bar):
    final = out / 'final'
    double, single = final / 'double', final / 'single'
    for d in (final, double, single):
        d.mkdir(parents=True, exist_ok=True)
        for f in d.glob('*.pdf'):
            f.unlink()  # every file is written again; the best model may have changed
    n_obj = sum(len(r['truth']) for r in rows)
    best, aps = best_model(rows, results, sw, n_obj, pr_curve, scores_cls)
    plot_dataset(rows, double, title, plt, single)
    plot_recall_bar(ev, results, conf_main, double, title, plt, single)
    plot_precision_bar(ev, results, conf_main, double, title, plt, single)
    plot_pr_all_bar(ev, results, conf_main, double, title, plt, single)
    plot_pr(rows, results, ev, sw, conf_main, double, title, plt, single)
    plot_speed_bar(results, cols, single, plt)
    if best:
        plot_confusion_matrix(best, ev[best], conf_main, single, plt)
    render_text(text_blocks(rows, results, ev, aps, best, conf_main, runs, names, SB, recall, prec_of),
                final / 'results_and_method.pdf', plt)
    return final, best
