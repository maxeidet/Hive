"""Download the real Drone vs Bird set and put it in our class order (0 drone, 1 bird).

    python prep_dvb.py                         download (~1 GB) + convert -> dvb_ours/data.yaml
    python prep_dvb.py --zip dvb.zip           use a zip you already downloaded
    python prep_dvb.py --limit 300             only 300 train images (quick test on a slow computer)
    python prep_dvb.py --resplit 0.2           plain val/test images mostly into train (see below)

The same steps as cells 2-3 of colab_dvb_then_gazebo.ipynb, as a plain script that works on
Windows, Linux and macOS (no wget/unzip needed). Data: Drone vs Bird, Mendeley
(https://data.mendeley.com/datasets/6ghdz52pd7/2), CC BY 4.0: credit it in the report.

The class is taken from the file name: bird images are called "BTR (...)", "BVA (...)" etc.
and drone images "DTR (...)", "DVA (...)". The class numbers inside the label files can't be
trusted (on 18 Oct they all came out as one class, so every bird was trained as a drone).
The set's train split is all Roboflow-augmented 2x2 mosaics, its val and test splits plain
video frames (5 Nov: val mAP stayed flat while train improved). --resplit pools the plain
val/test images, cuts them into blocks of consecutive frame numbers (one scene each), and
shares the blocks out, so train also has plain images and no scene is in two splits.

Then train with:

    python train_yolo.py --data dvb_ours/data.yaml --imgsz 640 --no-score --mosaic 0 ...
"""
import argparse
import random
import re
import shutil
import sys
import urllib.request
import zipfile
from collections import Counter
from pathlib import Path

URL = 'https://data.mendeley.com/public-api/zip/6ghdz52pd7/download/2'
SPLITS = (('train', 'train'), ('valid', 'val'), ('val', 'val'), ('test', 'test'))


def class_from_name(name: str) -> int | None:
    first = name.strip().upper()[:1]
    return 0 if first == 'D' else 1 if first == 'B' else None


def download(url: str, dest: Path) -> None:
    print(f'Downloading {url}\n  -> {dest} (about 1 GB)')

    def progress(blocks, block_size, total):
        done = blocks * block_size
        text = f'{done / 1e6:7.0f} MB' + (f' of {total / 1e6:.0f} MB' if total > 0 else '')
        print('\r  ' + text, end='', flush=True)

    tmp = dest.with_suffix('.part')
    urllib.request.urlretrieve(url, tmp, progress)
    tmp.replace(dest)
    print()


def unpack(archive: Path, folder: Path) -> None:
    """Unzip, then unzip any zips inside (Mendeley sometimes wraps the set in another zip)."""
    print(f'Unpacking {archive} -> {folder}')
    with zipfile.ZipFile(archive) as z:
        z.extractall(folder)
    for inner in sorted(folder.rglob('*.zip')):
        target = inner.with_suffix('')
        if not target.exists():
            print(f'  inner zip {inner.name}')
            with zipfile.ZipFile(inner) as z:
                z.extractall(target)


def box_line(cls: int, parts: list[str]) -> str:
    xy = list(map(float, parts[1:]))
    if len(xy) > 4:  # segmentation polygon -> box
        xs, ys = xy[0::2], xy[1::2]
        xy = [(min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2, max(xs) - min(xs), max(ys) - min(ys)]
    return f'{cls} ' + ' '.join(f'{v:.6f}' for v in xy[:4])


def put_file(src: Path, dst: Path) -> None:
    """Hard link if possible (no extra disk, works on Windows without admin), else copy."""
    if dst.exists():
        return
    try:
        dst.hardlink_to(src)
    except (OSError, AttributeError):
        shutil.copy2(src, dst)


def scene_block(name: str, block: int) -> tuple[str, int]:
    """("DV", 7) for "DV (362).txt" with block 50: the images of the val/test sets are video
    frames numbered in order, so nearby numbers are the same scene."""
    m = re.match(r'\s*(.*?)\s*\((\d+)\)', Path(name).stem)
    return (m.group(1), int(m.group(2)) // block) if m else (Path(name).stem, 0)


def assign(raw: Path, resplit: float | None, block: int, seed: int
           ) -> dict[str, list[Path]]:
    """Label files per split of ours. Without resplit: the set's own split. With it, the
    plain (non-mosaic) val and test images are pooled, cut into blocks of consecutive frame
    numbers, and the blocks shared out: `resplit` of them to val, half that to test, the
    rest to train, so one scene never lands in two splits."""
    found: dict[str, list[Path]] = {'train': [], 'val': [], 'test': []}
    for split, ours in SPLITS:
        dirs = [d for d in raw.rglob('labels') if d.is_dir() and d.parent.name == split]
        found[ours] += sorted(p for d in dirs for p in d.glob('*.txt'))
    if not resplit:
        return found
    pool = found['val'] + found['test']
    blocks = sorted({scene_block(p.name, block) for p in pool})
    random.Random(seed).shuffle(blocks)
    n_val = max(1, round(len(blocks) * resplit))
    n_test = max(1, round(len(blocks) * resplit / 2))
    where = {b: 'val' if i < n_val else 'test' if i < n_val + n_test else 'train'
             for i, b in enumerate(blocks)}
    out = {'train': list(found['train']), 'val': [], 'test': []}
    for p in pool:
        out[where[scene_block(p.name, block)]].append(p)
    print(f'resplit: {len(pool)} plain val/test images in {len(blocks)} scene blocks of '
          f'{block} frames -> {n_val} blocks val, {n_test} test, the rest train')
    return out


def convert(raw: Path, out: Path, limit: int | None, seed: int = 0,
            resplit: float | None = None, block: int = 50) -> Counter:
    if out.exists():
        shutil.rmtree(out)
    counts, classes, skipped = Counter(), Counter(), Counter()
    for ours, files in assign(raw, resplit, block, seed).items():
        if limit and ours == 'train':
            files = random.Random(seed).sample(files, min(limit, len(files)))
        elif limit:
            files = random.Random(seed).sample(files, min(max(limit // 5, 50), len(files)))
        (out / ours / 'images').mkdir(parents=True, exist_ok=True)
        (out / ours / 'labels').mkdir(parents=True, exist_ok=True)
        for lab in files:
            cls = class_from_name(lab.name)
            if cls is None:
                skipped[lab.name[:3]] += 1
                continue
            image = next((lab.parent.parent / 'images' / (lab.stem + ext)
                          for ext in ('.jpg', '.jpeg', '.png', '.JPG', '.PNG')
                          if (lab.parent.parent / 'images' / (lab.stem + ext)).exists()), None)
            if image is None:
                skipped['no image'] += 1
                continue
            lines = [box_line(cls, line.split()) for line in lab.read_text().splitlines()
                     if len(line.split()) >= 5]
            classes[(ours, cls)] += len(lines)
            (out / ours / 'labels' / lab.name).write_text('\n'.join(lines) + ('\n' if lines else ''))
            put_file(image, out / ours / 'images' / image.name)
            counts[ours] += 1
    print('images per split:', dict(counts))
    print('boxes per split and class (0 drone, 1 bird):', dict(sorted(classes.items())))
    if skipped:
        print('skipped:', dict(skipped))
    missing = [(s, c) for s in ('train', 'val') for c in (0, 1) if classes[(s, c)] == 0]
    if missing:
        raise SystemExit(f'Both classes must be in train and val; missing {missing}. '
                         f'Look at the folders under {raw}.')
    (out / 'data.yaml').write_text(f'path: {out.resolve().as_posix()}\ntrain: train/images\n'
                                   f'val: val/images\ntest: test/images\n'
                                   f'names:\n  0: drone\n  1: bird\n')
    print(f'Wrote {out / "data.yaml"}')
    return counts


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--zip', type=Path, default=Path('dvb.zip'),
                        help='the downloaded zip (downloaded here if missing)')
    parser.add_argument('--raw', type=Path, default=Path('dvb'), help='where to unpack it')
    parser.add_argument('--out', type=Path, default=Path('dvb_ours'), help='converted set')
    parser.add_argument('--limit', type=int,
                        help='only this many train images (val/test a fifth, at least 50), '
                             'for a quick test')
    parser.add_argument('--resplit', type=float, metavar='FRACTION',
                        help='move the plain (non-mosaic) val/test images into the split, '
                             'keeping this fraction of their scenes as val (half of it as '
                             'test), e.g. 0.2; without it the set\'s own split is used')
    parser.add_argument('--block', type=int, default=50,
                        help='frames per scene block for --resplit (default %(default)s)')
    args = parser.parse_args(argv)
    if not args.raw.exists():
        if not args.zip.exists():
            download(URL, args.zip)
        unpack(args.zip, args.raw)
    else:
        print(f'{args.raw} already unpacked (delete it to unpack again)')
    convert(args.raw, args.out, args.limit, resplit=args.resplit, block=args.block)
    return 0


if __name__ == '__main__':
    sys.exit(main())
