"""Run the baseline detector on saved images and write copies with the boxes drawn.

No ROS, so it runs on a laptop: python -m swarm_backend.detect.annotate imgs/*.png --out boxes/
Use it to tune the CandidateConfig settings on frames saved by `detect --save`.
"""
import argparse
import sys
from pathlib import Path

import cv2

from swarm_backend.detect.candidates import CandidateConfig, above_line, draw, find_candidates


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('images', type=Path, nargs='+')
    parser.add_argument('--out', type=Path, required=True, help='folder for the annotated copies')
    parser.add_argument('--horizon', type=float, default=None, help='image row of the horizon')
    parser.add_argument('--kernel', type=int, default=CandidateConfig.kernel)
    parser.add_argument('--k-sigma', type=float, default=CandidateConfig.k_sigma)
    parser.add_argument('--min-contrast', type=float, default=CandidateConfig.min_contrast)
    args = parser.parse_args(argv)

    config = CandidateConfig(kernel=args.kernel, k_sigma=args.k_sigma,
                             min_contrast=args.min_contrast)
    args.out.mkdir(parents=True, exist_ok=True)
    for path in args.images:
        if path.stem.endswith('_boxes'):
            continue
        bgr = cv2.imread(str(path))
        if bgr is None:
            print(f'{path}: not an image')
            continue
        rgb = bgr[..., ::-1]
        mask = None if args.horizon is None else above_line(rgb.shape, args.horizon)
        boxes = find_candidates(rgb, mask, config)
        labels = [f'{b.width}px {b.contrast:.0f}' for b in boxes]
        out = draw(rgb, boxes, (255, 220, 0), labels=labels)
        cv2.imwrite(str(args.out / path.name), out[..., ::-1])
        print(f'{path.name}: ' + (', '.join(
            f'({b.u:.0f}, {b.v:.0f}) {b.width}x{b.height}px contrast {b.contrast:.0f}'
            for b in boxes) or 'nothing'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
