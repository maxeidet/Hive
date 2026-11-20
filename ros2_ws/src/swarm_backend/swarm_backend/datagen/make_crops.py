"""Cut-outs around the baseline's candidates, for YOLO as a second stage (`mount-backend.sh makecrops`).

    ./mount-backend.sh makecrops                          data/gen -> data/crops (scaled)
    ./mount-backend.sh makecrops --context 4 --size 96    looser scaled cut-outs, smaller
    ./mount-backend.sh makecrops --mode fixed --out /workspace/data/crops_fixed

Approach A of "Training data and YOLO training" in the vault: on the drone the baseline
(dark object against the sky) finds candidates, and YOLO only looks at an enlarged cut-out
around each one. So the training images for that YOLO are made the same way: the baseline
runs on every image that gendata / genlive made, and a cut-out is taken around each
candidate:
  - positives: cut-outs with a labelled drone or bird inside (its box moved into the
    cut-out and scaled with it), off-centre as much as the baseline's candidates really are;
  - negatives: the baseline's false alarms (cloud edges, trees, the drone's own parts),
    at most --negatives per image, picked at random. These are what YOLO has to reject.
Two ways to size a cut-out (--mode); either way it is square, centred on the candidate,
moved inwards at the image edge, and resized to --size px:
  scaled (default)  --context times the candidate's size (at least --min-crop px), so the
                    target always fills about 1/context of the cut-out whatever its
                    distance: one scale for YOLO to learn. Far, small targets are enlarged
                    the most (a 4 px drone: 12 px cut, 128 px out, 11x). Default: context
                    3, at least 12 px, out 128 px, so targets from 4 px up fill about a third
                    and only 2-3 px ones get more room (24 px gave 4-8 px drones only a
                    sixth, measured 18 Oct). Relies on the candidate's size being about
                    right; --min-crop keeps very small candidates from getting too tight.
  fixed             always --crop px of the image (more for a candidate wider than half of
                    it), so the target's size in the cut-out varies with its distance and
                    some surroundings are kept. Default: 64 px cut, 256 px out (a 4 px
                    drone becomes 16 px).
Train and compare both on the same test runs (the split is kept). --missed also cuts
around labelled drones the baseline did not find.

Output in --out/<run>/images|labels (YOLO), crops.csv (one row per cut-out) and data.yaml
with the same split by run as --data's data.yaml. It also prints the baseline's candidate
recall per pixel size: the ceiling for the two stages together, since YOLO never sees a
drone the baseline missed.
"""
import argparse
import csv
import json
import math
import random
import sys
from pathlib import Path

import cv2

from swarm_backend.detect import detectors

DEFAULT_DATA = Path('/workspace/data/gen')
DEFAULT_OUT = Path('/workspace/data/crops')
BINS = [(2, 4), (4, 8), (8, 16), (16, 32), (32, 64), (64, 128)]


def read_labels(path: Path, width: int, height: int) -> list[tuple]:
    """(class, x0, y0, x1, y1) in pixels from a YOLO label file."""
    out = []
    if path.exists():
        for line in path.read_text().split('\n'):
            if line.strip():
                c, u, v, w, h = (float(x) for x in line.split())
                u, v, w, h = u * width, v * height, w * width, h * height
                out.append((int(c), u - w / 2, v - h / 2, u + w / 2, v + h / 2))
    return out


def side_for(size: float, mode: str, crop: int = 64, context: float = 3.0,
             min_crop: int = 12) -> int:
    """The cut-out's width in image px for a candidate `size` px across (see the modes)."""
    if mode == 'scaled':
        return int(max(min_crop, math.ceil(context * size)))
    return int(max(crop, math.ceil(2 * size)))


def window(u: float, v: float, side: int, width: int, height: int):
    """The square cut-out (x0, y0, side) around (u, v), moved inwards so it stays inside
    the image (and no wider than the image)."""
    side = int(min(side, width, height))
    x0 = int(round(u - side / 2))
    y0 = int(round(v - side / 2))
    return min(max(x0, 0), width - side), min(max(y0, 0), height - side), side


def crop_labels(boxes: list[tuple], x0: int, y0: int, side: int, min_inside: float = 0.5):
    """YOLO lines (fractions of the cut-out) for the boxes at least `min_inside` inside it."""
    lines = []
    for c, bx0, by0, bx1, by1 in boxes:
        area = (bx1 - bx0) * (by1 - by0)
        cx0, cy0 = max(bx0, x0), max(by0, y0)
        cx1, cy1 = min(bx1, x0 + side), min(by1, y0 + side)
        if cx1 <= cx0 or cy1 <= cy0 or area <= 0 or (cx1 - cx0) * (cy1 - cy0) < min_inside * area:
            continue
        lines.append(f'{c} {((cx0 + cx1) / 2 - x0) / side:.6f} {((cy0 + cy1) / 2 - y0) / side:.6f} '
                     f'{(cx1 - cx0) / side:.6f} {(cy1 - cy0) / side:.6f}')
    return lines


def cut_through(boxes: list[tuple], x0: int, y0: int, side: int, min_inside: float = 0.5) -> bool:
    """True when the cut-out cuts through an object: part of it is inside, but less than
    `min_inside`, so it gets no label although some of it shows. Such cut-outs are left
    out; as negatives they would teach YOLO that a piece of a drone is background."""
    for _, bx0, by0, bx1, by1 in boxes:
        area = (bx1 - bx0) * (by1 - by0)
        w = min(bx1, x0 + side) - max(bx0, x0)
        h = min(by1, y0 + side) - max(by0, y0)
        if area > 0 and w > 0 and h > 0 and 0 < w * h < min_inside * area:
            return True
    return False


def found_by(box: tuple, candidates: list) -> bool:
    """A candidate's centre within max(3 px, half the object's size) of the object's centre
    (the same rule as training/train_yolo.py's scoring)."""
    _, x0, y0, x1, y1 = box
    u, v, size = (x0 + x1) / 2, (y0 + y1) / 2, max(x1 - x0, y1 - y0)
    return any(math.dist((c.u, c.v), (u, v)) <= max(3.0, size / 2) for c in candidates)


def dedupe(candidates: list, min_dist: float) -> list:
    """Candidates without the ones whose centre is within `min_dist` of a stronger one
    (they would give nearly the same cut-out)."""
    kept = []
    for c in sorted(candidates, key=lambda c: -c.score):
        if all(math.dist(c.centre, k.centre) >= min_dist for k in kept):
            kept.append(c)
    return kept


def size_bin(size: float) -> str:
    return next((f'{a}-{b} px' for a, b in BINS if a <= size < b), 'other')


def split_yaml(data_yaml: Path, out: Path, runs: list[str]) -> str:
    """data.yaml for the cut-outs with the same train / val / test runs as `data_yaml`."""
    text = data_yaml.read_text() if data_yaml.exists() else ''
    lines, kept = [], set()
    for line in text.splitlines():
        if line.startswith('path:'):
            lines.append(f'path: {out}')
        elif line.strip().startswith('- '):
            run = line.strip()[2:].split('/')[0]
            if run in runs:
                lines.append(line)
                kept.add(run)
        else:
            lines.append(line)
    if not text:
        lines = [f'path: {out}', 'train:'] + [f'  - {r}/images' for r in runs] + \
                ['val: []', 'test: []', 'names:', '  0: drone', '  1: bird']
    return '# Written by make_crops.py, same split as the full images.\n' + '\n'.join(
        l for l in lines if not l.startswith('# Written')) + '\n'


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--data', type=Path, default=DEFAULT_DATA,
                        help='folder with the runs and data.yaml (default %(default)s)')
    parser.add_argument('--runs', nargs='*', help='only these runs (default: all)')
    parser.add_argument('--out', type=Path, default=DEFAULT_OUT)
    parser.add_argument('--mode', choices=('scaled', 'fixed'), default='scaled',
                        help='how big a cut-out is, see above (default %(default)s)')
    parser.add_argument('--context', type=float, default=3.0,
                        help='scaled: cut-out width / candidate size (default %(default)s)')
    parser.add_argument('--min-crop', type=int, default=12,
                        help='scaled: smallest cut-out, px (default %(default)s)')
    parser.add_argument('--crop', type=int, default=64,
                        help='fixed: cut-out width, px (default %(default)s)')
    parser.add_argument('--size', type=int,
                        help='width the cut-out is resized to, px (default: 128 scaled, '
                             '256 fixed)')
    parser.add_argument('--negatives', type=int, default=5,
                        help='most false-alarm cut-outs per image (default %(default)s)')
    parser.add_argument('--missed', action='store_true',
                        help='also cut around labelled drones the baseline did not find')
    parser.add_argument('--seed', type=int, default=0)
    detectors.add_options(parser)
    args = parser.parse_args(argv)

    if args.size is None:
        args.size = 128 if args.mode == 'scaled' else 256

    def side(size: float) -> int:
        return side_for(size, args.mode, args.crop, args.context, args.min_crop)

    detect = detectors.make('baseline', args)
    rng = random.Random(args.seed)
    runs = args.runs or sorted(p.name for p in args.data.iterdir() if (p / 'images').is_dir())
    recall = {}  # size bin -> [found, total]
    totals = {'images': 0, 'positive': 0, 'negative': 0, 'missed': 0, 'cut_through': 0}
    for run in runs:
        src = args.data / run
        dst = args.out / run
        (dst / 'images').mkdir(parents=True, exist_ok=True)
        (dst / 'labels').mkdir(parents=True, exist_ok=True)
        with open(dst / 'crops.csv', 'w', newline='') as f:
            writer = csv.DictWriter(f, ['crop', 'image', 'x0', 'y0', 'side', 'source', 'objects',
                                        'candidate_px', 'score'])
            writer.writeheader()
            for image_path in sorted((src / 'images').glob('*.png')):
                bgr = cv2.imread(str(image_path))
                if bgr is None:
                    continue
                height, width = bgr.shape[:2]
                rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                boxes = read_labels(src / 'labels' / f'{image_path.stem}.txt', width, height)
                candidates = dedupe(detect(rgb, None),
                                    (args.min_crop if args.mode == 'scaled' else args.crop) / 4)
                totals['images'] += 1
                for box in boxes:
                    if box[0] == 0:  # drones: the recall that matters
                        key = size_bin(max(box[3] - box[1], box[4] - box[2]))
                        hit = found_by(box, candidates)
                        recall.setdefault(key, [0, 0])
                        recall[key][0] += hit
                        recall[key][1] += 1

                jobs = []  # (u, v, size, source, score)
                negatives = []
                for c in candidates:
                    x0, y0, w = window(c.u, c.v, side(c.size), width, height)
                    job = (c.u, c.v, c.size, 'candidate', c.score)
                    (jobs if crop_labels(boxes, x0, y0, w) else negatives).append(job)
                jobs += rng.sample(negatives, min(args.negatives, len(negatives)))
                if args.missed:
                    for box in boxes:
                        if not found_by(box, candidates):
                            u, v = (box[1] + box[3]) / 2, (box[2] + box[4]) / 2
                            size = max(box[3] - box[1], box[4] - box[2])
                            jitter = side(size) / 6  # off-centre like a candidate would be
                            jobs.append((u + rng.uniform(-jitter, jitter),
                                         v + rng.uniform(-jitter, jitter), size, 'missed', 0.0))

                for k, (u, v, size, source, score) in enumerate(jobs):
                    x0, y0, w = window(u, v, side(size), width, height)
                    if cut_through(boxes, x0, y0, w):
                        totals['cut_through'] += 1
                        continue
                    lines = crop_labels(boxes, x0, y0, w)
                    name = f'{image_path.stem}_c{k:02d}'
                    cut = bgr[y0:y0 + w, x0:x0 + w]
                    cv2.imwrite(str(dst / 'images' / f'{name}.png'),
                                cv2.resize(cut, (args.size, args.size),
                                           interpolation=cv2.INTER_LINEAR))
                    (dst / 'labels' / f'{name}.txt').write_text(
                        ''.join(line + '\n' for line in lines))
                    writer.writerow({'crop': name, 'image': image_path.name, 'x0': x0, 'y0': y0,
                                     'side': w, 'source': source, 'objects': len(lines),
                                     'candidate_px': round(size, 1), 'score': round(score, 1)})
                    if source == 'missed':
                        totals['missed'] += 1
                    else:
                        totals['positive' if lines else 'negative'] += 1

    (args.out / 'data.yaml').write_text(split_yaml(args.data / 'data.yaml', args.out, runs))
    (args.out / 'settings.json').write_text(json.dumps(
        {'mode': args.mode, 'context': args.context, 'min_crop': args.min_crop,
         'crop': args.crop, 'size': args.size, 'negatives': args.negatives,
         'missed': args.missed, 'kernel': args.kernel, 'contrast': args.contrast,
         'runs': runs}, indent=1))
    print(f'{args.mode} cut-outs, {args.size} px. '
          f'{totals["images"]} images -> {totals["positive"]} cut-outs with a target, '
          f'{totals["negative"]} false-alarm cut-outs'
          + (f', {totals["missed"]} around missed drones' if args.missed else '')
          + f' ({totals["cut_through"]} left out: they cut through a drone or bird)'
          + f'\nin {args.out} (data.yaml: same split as {args.data / "data.yaml"})')
    print('\nBaseline candidate recall (drones): the most the two stages can find')
    for key in [f'{a}-{b} px' for a, b in BINS] + ['other']:
        if key in recall:
            found, total = recall[key]
            print(f'  {key:>10}: {found:4d}/{total:<4d} {found / total:.2f}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
