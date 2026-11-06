"""Draw YOLO labels on images, with a zoomed cut-out of every box, to check them by eye.

    python show_labels.py RUN_DIR [--n 5] [--out FILE]  (default RUN_DIR/labels_check.png)

RUN_DIR has images/ and labels/ (one gendata run, e.g. data/gen/b01, or any YOLO split
folder). Picks the images with the most boxes. Red = drone (class 0), blue = bird (1).
"""
import argparse
from pathlib import Path

import cv2
import numpy as np

COL = {0: (0, 0, 255), 1: (255, 200, 0)}
NAME = {0: 'drone', 1: 'bird'}


def panel(img_p: Path, lab_p: Path) -> np.ndarray:
    img = cv2.imread(str(img_p))
    h, w = img.shape[:2]
    out, crops = img.copy(), []
    for line in lab_p.read_text().splitlines() if lab_p.exists() else []:
        if not line.strip():
            continue
        c, u, v, bw, bh = line.split()[:5]
        c = int(c)
        u, v, bw, bh = float(u) * w, float(v) * h, float(bw) * w, float(bh) * h
        x0, y0, x1, y1 = (int(round(u - bw / 2)), int(round(v - bh / 2)),
                          int(round(u + bw / 2)), int(round(v + bh / 2)))
        col = COL.get(c, (0, 255, 0))
        cv2.rectangle(out, (x0 - 3, y0 - 3), (x1 + 3, y1 + 3), col, 2)
        cv2.putText(out, f'{NAME.get(c, c)} {bw:.0f}x{bh:.0f}px', (x0, max(15, y0 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
        m = int(max(bw, bh) * 1.5) + 6
        cx0, cy0 = max(0, int(u) - m), max(0, int(v) - m)
        crop = img[cy0:int(v) + m, cx0:int(u) + m].copy()
        cv2.rectangle(crop, (x0 - cx0, y0 - cy0), (x1 - cx0, y1 - cy0), col, 1)
        crop = cv2.resize(crop, (200, 200), interpolation=cv2.INTER_NEAREST)
        cv2.putText(crop, str(NAME.get(c, c)), (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
        crops.append(crop)
    strip = np.full((210, w, 3), 40, np.uint8)
    for i, cr in enumerate(crops[:w // 210]):
        strip[5:205, 5 + i * 210:205 + i * 210] = cr
    cv2.putText(out, img_p.stem, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
    return np.vstack([out, strip])


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('run', type=Path)
    p.add_argument('--n', type=int, default=5, help='images to show (default 5)')
    p.add_argument('--out', type=Path, help='default: RUN_DIR/labels_check.png')
    a = p.parse_args()
    a.out = a.out or a.run / 'labels_check.png'
    labels = sorted((a.run / 'labels').glob('*.txt'),
                    key=lambda f: -len([l for l in f.read_text().splitlines() if l.strip()]))
    panels = []
    for lab in labels[:a.n]:
        img = next((a.run / 'images' / (lab.stem + e) for e in ('.png', '.jpg')
                    if (a.run / 'images' / (lab.stem + e)).exists()), None)
        if img is not None:
            panels.append(panel(img, lab))
    width = max(x.shape[1] for x in panels)
    panels = [np.pad(x, ((0, 0), (0, width - x.shape[1]), (0, 0))) for x in panels]
    cv2.imwrite(str(a.out), np.vstack(panels))
    print(f'Wrote {a.out} ({len(panels)} images)')


if __name__ == '__main__':
    main()
