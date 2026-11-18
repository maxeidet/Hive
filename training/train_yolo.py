"""Fine-tune YOLO26 on the Gazebo images from gen_data.py and score it by pixel size.

    python3 train_yolo.py --data /data/gen/data.yaml                 train + score
    python3 train_yolo.py --data /data/gen/data.yaml --epochs 3       quick try
    python3 train_yolo.py --data /data/gen/data.yaml --score runs/yolo26n/weights/best.pt

Runs where Ultralytics and a GPU are (Stellar, Colab; see README.md here), not in the
simulation container. The data folder is the one `mount-backend.sh gendata` writes:
data.yaml next to one folder per run (images/, labels/, meta/), split by run.

After training it runs the best weights on the test runs and writes, in the run folder,
size_recall.csv and prints: recall per pixel-size bin of the drones (the report's "recall
vs pixel size"), and false alarms per image, at a few confidence thresholds. A detection
counts as finding an object when it has the same class and its centre is within
max(3 px, half the object's size) of the object's centre (matching by centre, not IoU,
as the plan says: IoU is too harsh on boxes a few pixels wide).
"""
import argparse
import csv
import math
import sys
from pathlib import Path

NAMES = {0: 'drone', 1: 'bird'}
BINS = [(2, 4), (4, 8), (8, 16), (16, 32), (32, 64), (64, 128)]
THRESHOLDS = (0.1, 0.25, 0.5)


def local_yaml(data: Path, out: Path) -> Path:
    """A copy of data.yaml whose `path` is the folder it is in now (gen_data.py writes the
    container's /workspace/data/gen, which is wrong on Stellar or Colab)."""
    lines = data.read_text().splitlines()
    fixed = [f'path: {data.parent.resolve()}' if line.startswith('path:') else line
             for line in lines]
    out.write_text('\n'.join(fixed) + '\n')
    return out


def split_dirs(data: Path, split: str) -> list[Path]:
    """The image folders listed under `split:` in data.yaml (a list of run/images)."""
    root, dirs, current = data.parent, [], None
    for line in data.read_text().splitlines():
        if line.startswith('path:'):
            root = Path(line.split(':', 1)[1].strip())
        elif line and not line.startswith(' ') and line.endswith(':'):
            current = line[:-1]
        elif current == split and line.strip().startswith('- '):
            dirs.append(root / line.strip()[2:])
    return dirs


def read_labels(path: Path, width: int, height: int) -> list[tuple]:
    """(class, u, v, w, h) in pixels from a YOLO label file."""
    out = []
    if path.exists():
        for line in path.read_text().split('\n'):
            if line.strip():
                c, u, v, w, h = line.split()
                out.append((int(c), float(u) * width, float(v) * height,
                            float(w) * width, float(h) * height))
    return out


def match(truth: list[tuple], found: list[tuple]) -> tuple[list[bool], int]:
    """Which true objects were found, and how many detections found nothing (false
    alarms). truth: (class, u, v, w, h); found: (class, u, v, w, h, confidence), each
    detection used once, the most confident first."""
    hit = [False] * len(truth)
    false_alarms = 0
    for c, u, v, *_ in sorted(found, key=lambda d: -d[5]):
        best, best_dist = None, None
        for i, (tc, tu, tv, tw, th) in enumerate(truth):
            dist = math.dist((u, v), (tu, tv))
            if not hit[i] and tc == c and dist <= max(3.0, max(tw, th) / 2):
                if best_dist is None or dist < best_dist:
                    best, best_dist = i, dist
        if best is None:
            false_alarms += 1
        else:
            hit[best] = True
    return hit, false_alarms


def size_bin(size: float) -> str | None:
    for a, b in BINS:
        if a <= size < b:
            return f'{a}-{b} px'
    return None


def score(model, image_dirs: list[Path], imgsz: int, device) -> list[dict]:
    """Rows of size_recall.csv: per threshold and size bin, drones found / drones."""
    images = sorted(p for d in image_dirs for p in d.glob('*.png'))
    if not images:
        raise SystemExit(f'No test images in {", ".join(map(str, image_dirs))}')
    counts = {t: {'images': 0, 'false_alarms': 0, 'bins': {}} for t in THRESHOLDS}
    for result in (r for p in images for r in model.predict(str(p), imgsz=imgsz, conf=min(THRESHOLDS),
                                device=device, verbose=False)):  # one at a time: a list is loaded as ONE batch
        path = Path(result.path)
        height, width = result.orig_shape
        truth = read_labels(path.parent.parent / 'labels' / f'{path.stem}.txt', width, height)
        boxes = result.boxes
        detections = [(int(c), float(x), float(y), float(w), float(h), float(p))
                      for (x, y, w, h), c, p in zip(boxes.xywh.tolist(), boxes.cls.tolist(),
                                                    boxes.conf.tolist())]
        for t in THRESHOLDS:
            hit, false_alarms = match(truth, [d for d in detections if d[5] >= t])
            counts[t]['images'] += 1
            counts[t]['false_alarms'] += false_alarms
            for (c, _, _, w, h), found in zip(truth, hit):
                key = (NAMES[c], size_bin(max(w, h)))
                n_found, n = counts[t]['bins'].get(key, (0, 0))
                counts[t]['bins'][key] = (n_found + found, n + 1)
    rows = []
    for t, c in counts.items():
        for (name, bin_), (n_found, n) in sorted(c['bins'].items(), key=lambda kv: str(kv[0])):
            rows.append({'threshold': t, 'class': name, 'size': bin_ or 'other',
                         'found': n_found, 'objects': n, 'recall': round(n_found / n, 3),
                         'false_alarms_per_image': round(c['false_alarms'] / c['images'], 3)})
    return rows


def print_rows(rows: list[dict]) -> None:
    for t in THRESHOLDS:
        part = [r for r in rows if r['threshold'] == t]
        if not part:
            continue
        print(f'\nconfidence >= {t}: {part[0]["false_alarms_per_image"]} false alarms per image')
        for r in part:
            print(f'  {r["class"]:6} {r["size"]:>10}: {r["found"]:4d}/{r["objects"]:<4d} '
                  f'recall {r["recall"]:.2f}')


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--data', type=Path, required=True, help='data.yaml from gendata --yaml')
    parser.add_argument('--model', default='yolo26n.pt',
                        help='starting weights (COCO; downloaded by Ultralytics)')
    parser.add_argument('--imgsz', type=int, default=1280,
                        help='training and test image size; 1280 keeps a 4 px drone 4 px '
                             '(the default 640 halves it)')
    parser.add_argument('--epochs', type=int, default=50)
    parser.add_argument('--batch', type=int, default=8, help='lower it if the GPU runs out of memory')
    parser.add_argument('--device', default=None, help='e.g. 0 for the first GPU, cpu')
    parser.add_argument('--project', type=Path, default=Path('runs'))
    parser.add_argument('--name', default=None, help='run name (default: model name)')
    parser.add_argument('--score', type=Path, help='only score these weights on the test runs')
    parser.add_argument('--no-score', action='store_true',
                        help='skip scoring by pixel size (for data without our run split, '
                             'e.g. a real drone-vs-bird set)')
    parser.add_argument('--freeze', type=int,
                        help='freeze the first N layers (YOLO26: 0-10 = backbone, 11-22 = '
                             'neck, 23 = head); default: train everything')
    parser.add_argument('--lr0', type=float,
                        help='starting learning rate (default: Ultralytics chooses); lower it '
                             'when fine-tuning an already trained model, e.g. 0.002')
    parser.add_argument('--mosaic', type=float,
                        help='mosaic augmentation probability (Ultralytics default 1.0); 0 for '
                             'data whose images are already mosaics')
    parser.add_argument('--patience', type=int, default=15,
                        help='stop after this many epochs without val improvement')
    args = parser.parse_args(argv)

    from ultralytics import YOLO

    data = local_yaml(args.data, args.data.with_name('data_local.yaml'))
    name = args.name or Path(args.model).stem
    if args.score:
        model, out = YOLO(str(args.score)), args.score.parent.parent
    else:
        model = YOLO(args.model)
        extra = {k: v for k, v in (('freeze', args.freeze), ('lr0', args.lr0),
                                   ('mosaic', args.mosaic)) if v is not None}
        if args.lr0 is not None:
            extra['optimizer'] = 'AdamW'  # 'auto' would ignore lr0
        model.train(data=str(data), imgsz=args.imgsz, epochs=args.epochs, batch=args.batch,
                    device=args.device, project=str(args.project.resolve()), name=name,
                    patience=args.patience,  # early stopping on val
                    scale=0.2,     # the default 0.5 shrinks tiny drones out of existence
                    fliplr=0.5, hsv_v=0.5,  # brightness: the generator's sun barely shows
                    **extra)
        out = Path(model.trainer.save_dir)
        model = YOLO(str(model.trainer.best))

    if args.no_score:
        print(f'\nBest weights: {out / "weights" / "best.pt"}')
        return 0
    test = split_dirs(data, 'test') or split_dirs(data, 'val')
    rows = score(model, test, args.imgsz, args.device)
    with open(out / 'size_recall.csv', 'w', newline='') as f:
        writer = csv.DictWriter(f, list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print_rows(rows)
    print(f'\nWrote {out / "size_recall.csv"}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
