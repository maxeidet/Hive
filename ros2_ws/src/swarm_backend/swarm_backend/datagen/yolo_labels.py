"""YOLO label files from the truth saved with each image (`mount-backend.sh yololabels`).

    ./mount-backend.sh yololabels data/Manualimages            labels/ next to the images
    ./mount-backend.sh yololabels data/run_03 --min-px 2

For every <name>.png with a <name>.json (save_images.py), writes <folder>/labels/<name>.txt
with one line per drone or bird in view: `class x_centre y_centre width height`, the last
four as fractions of the image size, which is the format Ultralytics trains on. Classes:
0 drone, 1 bird. An image with nothing in view gets an empty file (an empty-sky example).

Objects smaller than --min-px across, or mostly outside the image, are left out and listed
in <folder>/labels/ignored.csv, so they can be told apart from misses when scoring. The
boxes are projected from a box around each model (ground_truth.py), so trees in the way
are not taken into account.
"""
import argparse
import csv
import json
import sys
from pathlib import Path

CLASSES = {'drone': 0, 'bird': 1}


def yolo_lines(truth: dict, min_px: float = 2.0, min_in_image: float = 0.5):
    """(label lines, ignored objects) for one image's truth."""
    width, height = truth['camera']['width'], truth['camera']['height']
    lines, ignored = [], []
    for obj in truth.get('objects', []):
        box = obj.get('box')
        if box is None:
            continue  # out of view
        w, h = box['x1'] - box['x0'], box['y1'] - box['y0']
        if max(w, h) < min_px or box['in_image'] < min_in_image:
            ignored.append({'name': obj['name'], 'class': obj['class'],
                            'size_px': round(max(w, h), 1), 'in_image': box['in_image'],
                            'distance': obj['distance']})
            continue
        u, v = (box['x0'] + box['x1']) / 2, (box['y0'] + box['y1']) / 2
        lines.append(f"{CLASSES[obj['class']]} {u / width:.6f} {v / height:.6f} "
                     f"{w / width:.6f} {h / height:.6f}")
    return lines, ignored


def write_labels(folder: Path, min_px: float = 2.0) -> tuple[int, int, int]:
    """Writes folder/labels/; returns (images, objects labelled, objects ignored)."""
    out = folder / 'labels'
    out.mkdir(exist_ok=True)
    images = labelled = 0
    rows = []
    for meta in sorted(folder.glob('*.json')):
        if not meta.with_suffix('.png').exists():
            continue
        truth = json.loads(meta.read_text())
        if 'objects' not in truth:
            print(f'{meta.name}: no `objects` (saved before boxes were added), skipped')
            continue
        lines, ignored = yolo_lines(truth, min_px)
        (out / f'{meta.stem}.txt').write_text(''.join(line + '\n' for line in lines))
        images += 1
        labelled += len(lines)
        rows += [{'image': meta.stem, **row} for row in ignored]
    with open(out / 'ignored.csv', 'w', newline='') as f:
        writer = csv.DictWriter(f, ['image', 'name', 'class', 'size_px', 'in_image', 'distance'])
        writer.writeheader()
        writer.writerows(rows)
    return images, labelled, len(rows)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('folder', type=Path, help='folder with the .png and .json files')
    parser.add_argument('--min-px', type=float, default=2.0,
                        help='leave out objects smaller than this, px (default %(default)s)')
    args = parser.parse_args(argv)
    images, labelled, ignored = write_labels(args.folder, args.min_px)
    print(f'{images} images: {labelled} objects labelled, {ignored} ignored '
          f'-> {args.folder / "labels"}')
    return 0 if images else 1


if __name__ == '__main__':
    sys.exit(main())
