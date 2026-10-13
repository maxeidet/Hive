"""Run the baseline detector on saved images and save what it finds.

    python3 -m swarm_backend.detect.run_on_images data/Manualimages --out data/results/baseline

For every PNG it writes the image with a red box on each candidate, and one results.csv
with a row per candidate (an image with none gets one row with empty fields). Runs on a
laptop: it needs only NumPy and OpenCV (pip install opencv-python), not ROS.
"""
import argparse
import csv
import sys
from pathlib import Path

import cv2

from swarm_backend.detect.candidates import Settings, draw, find_candidates

FIELDS = ['image', 'candidate', 'x', 'y', 'width', 'height', 'area', 'contrast', 'darkest']


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('images', type=Path, help='folder with PNG images')
    parser.add_argument('--out', type=Path, required=True, help='folder for the results')
    parser.add_argument('--contrast', type=float, default=Settings.contrast,
                        help='grey levels darker than the sky around it (default %(default)s)')
    args = parser.parse_args(argv)
    settings = Settings(contrast=args.contrast)

    paths = sorted(args.images.glob('*.png'))
    if not paths:
        print(f'No PNG images in {args.images}')
        return 1
    args.out.mkdir(parents=True, exist_ok=True)
    rows = []
    for path in paths:
        bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if bgr is None:
            print(f'{path.name}: could not be read, skipped')
            continue
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        candidates = find_candidates(rgb, settings)
        cv2.imwrite(str(args.out / path.name), cv2.cvtColor(draw(rgb, candidates), cv2.COLOR_RGB2BGR))
        sizes = ', '.join(f'{max(c.width, c.height)} px at ({c.centre[0]:.0f}, {c.centre[1]:.0f})'
                          for c in candidates)
        print(f'{path.name}: {len(candidates)} candidate(s)' + (f': {sizes}' if sizes else ''))
        if not candidates:
            rows.append({'image': path.name})
        for number, c in enumerate(candidates, start=1):
            rows.append({'image': path.name, 'candidate': number, 'x': c.x, 'y': c.y,
                         'width': c.width, 'height': c.height, 'area': c.area,
                         'contrast': round(c.contrast, 1), 'darkest': c.darkest})
    with open(args.out / 'results.csv', 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    print(f'Results in {args.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
