"""Unpack the Seraphim drone set and make a YOLO set in our class order (0 drone, 1 bird).

    hf download lgrzybowski/seraphim-drone-detection-dataset --repo-type dataset --local-dir SRC
    python prep_seraphim.py --src SRC --raw RAW --out OUT
    python prep_seraphim.py --src SRC --raw RAW --out OUT --add /nobackup/$USER/dvb/ours_resplit
    python prep_seraphim.py ... --limit 500                       quick test

Seraphim (https://huggingface.co/datasets/lgrzybowski/seraphim-drone-detection-dataset,
CC BY 4.0, credit it in the report): ~83 000 images, 640x640, one class (drone), from 23
open sources (real photos, marketing images and renders). On the Hub the images and labels
are zipped in batches (train/images/batch_001.zip, train/labels/batch_001.zip, ...), with
train and test splits and no val. Here:

  - every batch zip is unpacked (flat) into RAW/<split>/images and RAW/<split>/labels;
    already unpacked .jpg/.txt files next to the zips are used as they are;
  - every box becomes class 0 (drone); polygons become boxes;
  - Seraphim's train -> our train, Seraphim's test -> our val (held out by its author);
  - --add DIR also links another prepared set (e.g. prep_dvb.py's ours_resplit, which has
    the birds Seraphim lacks) into train and val, with a prefix so names don't clash.

Then train with train_yolo.py --data OUT/data.yaml --imgsz 640 --no-score ...
"""
import argparse
import random
import shutil
import sys
import zipfile
from collections import Counter
from pathlib import Path

SPLITS = (('train', 'train'), ('test', 'val'), ('valid', 'val'), ('val', 'val'))
IMAGE_EXTS = ('.jpg', '.jpeg', '.png', '.JPG', '.JPEG', '.PNG')


def unpack(src: Path, raw: Path) -> None:
    """Every <split>/{images,labels}/*.zip under src, unpacked flat into raw/<split>/..."""
    zips = sorted(src.rglob('*.zip'))
    print(f'{len(zips)} zip files under {src}')
    for i, archive in enumerate(zips, 1):
        kind = 'labels' if 'labels' in archive.parts else 'images'
        split = next((p for p in archive.parts if p in dict(SPLITS)), None)
        if split is None:
            print(f'  skipped {archive} (no train/test in its path)')
            continue
        dest = raw / split / kind
        dest.mkdir(parents=True, exist_ok=True)
        done = dest / f'.done_{archive.stem}'
        if done.exists():
            continue
        print(f'  [{i}/{len(zips)}] {archive.relative_to(src)}', flush=True)
        with zipfile.ZipFile(archive) as z:
            for member in z.infolist():
                name = Path(member.filename).name
                if member.is_dir() or not name or name.startswith('.'):
                    continue
                with z.open(member) as fin, open(dest / name, 'wb') as fout:
                    shutil.copyfileobj(fin, fout)
        done.touch()


def box_line(parts: list[str]) -> str:
    xy = list(map(float, parts[1:]))
    if len(xy) > 4:  # segmentation polygon -> box
        xs, ys = xy[0::2], xy[1::2]
        xy = [(min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2, max(xs) - min(xs), max(ys) - min(ys)]
    return '0 ' + ' '.join(f'{v:.6f}' for v in xy[:4])


def put_file(src: Path, dst: Path) -> None:
    """Hard link if possible (no extra disk), else copy."""
    if dst.exists():
        return
    try:
        dst.hardlink_to(src)
    except (OSError, AttributeError):
        shutil.copy2(src, dst)


def find_image(images: Path, stem: str) -> Path | None:
    return next((images / (stem + e) for e in IMAGE_EXTS if (images / (stem + e)).exists()), None)


def pairs(folder: Path) -> list[tuple[Path, Path | None]]:
    """(image, label or None) for every image in folder/images; an image without a label
    file is a background image (no drone)."""
    images, labels = folder / 'images', folder / 'labels'
    out = []
    for img in sorted(p for p in images.iterdir() if p.suffix in IMAGE_EXTS) if images.is_dir() else []:
        lab = labels / f'{img.stem}.txt'
        out.append((img, lab if lab.exists() else None))
    return out


def convert(raw: Path, out: Path, limit: int | None, add: list[Path], seed: int = 0) -> None:
    if out.exists():
        shutil.rmtree(out)
    counts, boxes = Counter(), Counter()
    for ours in ('train', 'val'):
        (out / ours / 'images').mkdir(parents=True, exist_ok=True)
        (out / ours / 'labels').mkdir(parents=True, exist_ok=True)
    for split, ours in SPLITS:
        found = pairs(raw / split)
        if limit:
            n = limit if ours == 'train' else max(limit // 5, 50)
            found = random.Random(seed).sample(found, min(n, len(found)))
        for img, lab in found:
            lines = [box_line(line.split()) for line in lab.read_text().splitlines()
                     if len(line.split()) >= 5] if lab else []
            (out / ours / 'labels' / f'{img.stem}.txt').write_text(
                '\n'.join(lines) + ('\n' if lines else ''))
            put_file(img, out / ours / 'images' / img.name)
            counts[(ours, 'seraphim')] += 1
            boxes[(ours, 0)] += len(lines)
    for other in add:
        prefix = other.name + '_'
        for ours in ('train', 'val'):
            found = pairs(other / ours)
            if limit:
                n = limit if ours == 'train' else max(limit // 5, 50)
                found = random.Random(seed).sample(found, min(n, len(found)))
            for img, lab in found:
                text = lab.read_text() if lab else ''
                for line in text.splitlines():
                    if line.split():
                        boxes[(ours, int(line.split()[0]))] += 1
                (out / ours / 'labels' / f'{prefix}{img.stem}.txt').write_text(text)
                put_file(img, out / ours / 'images' / f'{prefix}{img.name}')
                counts[(ours, other.name)] += 1
    print('images per split and source:', dict(sorted(counts.items())))
    print('boxes per split and class (0 drone, 1 bird):', dict(sorted(boxes.items())))
    if not counts[('train', 'seraphim')] or not counts[('val', 'seraphim')]:
        raise SystemExit(f'No Seraphim images in train or val: look at the folders under {raw}.')
    (out / 'data.yaml').write_text(f'path: {out.resolve().as_posix()}\ntrain: train/images\n'
                                   f'val: val/images\nnames:\n  0: drone\n  1: bird\n')
    print(f'Wrote {out / "data.yaml"}')


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--src', type=Path, required=True,
                        help='the downloaded dataset (hf download ... --local-dir SRC)')
    parser.add_argument('--raw', type=Path, required=True, help='where to unpack the zips')
    parser.add_argument('--out', type=Path, required=True, help='the YOLO set to write')
    parser.add_argument('--add', type=Path, nargs='*', default=[],
                        help='other prepared sets (with train/ and val/) to mix in, e.g. '
                             'the Drone vs Bird ours_resplit for the birds')
    parser.add_argument('--limit', type=int,
                        help='only this many train images per source (val a fifth, at least '
                             '50), for a quick test')
    args = parser.parse_args(argv)
    unpack(args.src, args.raw)
    for split, _ in SPLITS[:2]:  # unpacked in place by Hugging Face's own script?
        for kind, exts in (('images', IMAGE_EXTS), ('labels', ('.txt',))):
            here = args.src / split / kind
            if here.is_dir() and any(p.suffix in exts for p in here.iterdir()):
                dest = args.raw / split / kind
                dest.mkdir(parents=True, exist_ok=True)
                for p in here.iterdir():
                    if p.suffix in exts:
                        put_file(p, dest / p.name)
    convert(args.raw, args.out, args.limit, args.add)
    return 0


if __name__ == '__main__':
    sys.exit(main())
