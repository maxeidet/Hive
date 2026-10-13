"""Run a detector on saved images and save what it finds (`mount-backend.sh predict`).

    python3 -m swarm_backend.detect.predict baseline
    python3 -m swarm_backend.detect.predict baseline --images data/Manualimages --out data/results/baseline

For every PNG it writes the image with a red box on each detection, and one results.csv
with a row per detection (an image with none gets one row with empty fields). Each detection
gets an estimated distance (from its size in pixels) and direction (from where it is in the
image). When the image has the truth file that snap saves, these are compared with the true
distance and direction, drawn on the image and plotted in <out>/plots. Methods:
`baseline` (dark object against the sky, sky_candidates.py) and `yolo` (not trained yet).
Needs only NumPy and OpenCV, not ROS.
"""
import argparse
import csv
import json
import math
import sys
from pathlib import Path

import cv2

from swarm_backend.detect.camera import GIMBAL_HFOV, TARGET_SIZE_M, CameraModel, angle_deg
from swarm_backend.detect.sky_candidates import Candidate, Settings, find_candidates

FIELDS = ['image', 'candidate', 'x', 'y', 'width', 'height', 'area', 'contrast', 'darkest',
          'size_px', 'est_distance', 'est_dir_x', 'est_dir_y', 'est_dir_z',
          'target', 'true_distance', 'true_dir_x', 'true_dir_y', 'true_dir_z',
          'distance_error', 'direction_error_deg', 'pixel_error']
DATA = Path('/workspace/data')  # /workspace is the repo on the host
MATCH_PX = 40.0  # a detection this close to where a drone really shows is that drone
RED, GREEN = (255, 0, 0), (0, 160, 0)  # estimated, true (RGB)


def baseline(args):
    settings = Settings(contrast=args.contrast)
    return lambda rgb: find_candidates(rgb, settings)


def yolo(args):
    raise NotImplementedError(
        'The YOLO detector is not here yet: it needs trained weights first (see "Detection '
        'implementation plan" in the vault, steps 6-8). Use baseline for now.')


# Each returns a function from an RGB image to its detections (a list of Candidate).
METHODS = {'baseline': baseline, 'yolo': yolo}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('method', choices=METHODS, nargs='?', default='baseline')
    parser.add_argument('--images', type=Path, default=DATA / 'Manualimages',
                        help='folder with PNG images (default %(default)s)')
    parser.add_argument('--out', type=Path,
                        help='folder for the results (default data/results/<method>)')
    parser.add_argument('--contrast', type=float, default=Settings.contrast,
                        help='baseline: grey levels darker than the sky around it '
                             '(default %(default)s)')
    parser.add_argument('--no-plots', action='store_true', help='skip the plots')
    args = parser.parse_args(argv)
    out = args.out or DATA / 'results' / args.method
    try:
        detect = METHODS[args.method](args)
    except NotImplementedError as e:
        print(e)
        return 1

    paths = sorted(args.images.glob('*.png'))
    if not paths:
        print(f'No PNG images in {args.images}')
        return 1
    out.mkdir(parents=True, exist_ok=True)
    rows = []
    for path in paths:
        bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if bgr is None:
            print(f'{path.name}: could not be read, skipped')
            continue
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        truth = load_truth(path)
        found: list[Candidate] = detect(rgb)
        image_rows = [describe(path.name, number, c, truth, rgb.shape[1], rgb.shape[0])
                      for number, c in enumerate(found, start=1)]
        cv2.imwrite(str(out / path.name),
                    cv2.cvtColor(annotate(rgb, found, image_rows, truth), cv2.COLOR_RGB2BGR))
        print(f'{path.name}: {len(found)} found' + ''.join(summary(r) for r in image_rows))
        rows.extend(image_rows or [{'image': path.name}])
    with open(out / 'results.csv', 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    with_detection = len({r['image'] for r in rows if r.get('candidate')})
    print(f'{args.method}: something found in {with_detection} of {len(paths)} images. '
          f'Results in {out}')
    matched = [r for r in rows if r.get('target') is not None]
    if matched and not args.no_plots:
        plot(matched, out / 'plots', args.method)
    return 0


def load_truth(image: Path) -> dict | None:
    """The .json that snap saved with the image (ground_truth.py), if there is one."""
    path = image.with_suffix('.json')
    return json.loads(path.read_text()) if path.exists() else None


def describe(image: str, number: int, c: Candidate, truth: dict | None,
             width: int, height: int) -> dict:
    """One results.csv row: the detection, the distance and direction estimated from it,
    and, when the image has truth, the same for the drone it is and the errors."""
    size_px = max(c.width, c.height)
    row = {'image': image, 'candidate': number, 'x': c.x, 'y': c.y, 'width': c.width,
           'height': c.height, 'area': c.area, 'contrast': round(c.contrast, 1),
           'darkest': c.darkest, 'size_px': size_px}
    # Without truth there is no camera orientation either, so only the distance.
    hfov = truth['camera']['hfov_rad'] if truth else GIMBAL_HFOV
    camera = CameraModel(hfov, width, height)
    row['est_distance'] = round(camera.range_from_width(size_px), 2)
    if truth is None:
        return row
    # The camera's orientation is taken from the truth: on the drone it will come from the
    # gimbal's attitude. So the direction error below is from the detection alone.
    cam = truth['camera']
    direction = camera.direction(*c.centre, cam['forward'], cam['right'], cam['down'])
    row.update(zip(('est_dir_x', 'est_dir_y', 'est_dir_z'), (round(v, 5) for v in direction)))
    in_view = [d for d in truth['drones'] if d['pixel'] is not None]
    target = min(in_view, key=lambda d: math.dist(d['pixel'], c.centre), default=None)
    if target is None or math.dist(target['pixel'], c.centre) > MATCH_PX:
        return row  # not a drone: a false alarm
    row.update({
        'target': target['id'],
        'true_distance': target['distance'],
        'distance_error': round(row['est_distance'] - target['distance'], 2),
        'direction_error_deg': round(angle_deg(direction, tuple(target['direction'])), 2),
        'pixel_error': round(math.dist(target['pixel'], c.centre), 1),
    })
    row.update(zip(('true_dir_x', 'true_dir_y', 'true_dir_z'), target['direction']))
    return row


def summary(row: dict) -> str:
    text = f"\n    {row['size_px']} px: estimated {row['est_distance']:.1f} m"
    if row.get('target') is not None:
        text += (f", true {row['true_distance']:.1f} m (drone {row['target']}), "
                 f"direction off by {row['direction_error_deg']:.1f} deg")
    elif 'est_dir_x' in row:
        text += ', no drone there (false alarm)'
    return text


def annotate(rgb, found: list[Candidate], rows: list[dict], truth: dict | None, pad: int = 12):
    """The image with a red box and the numbers on every detection. With truth: a green
    cross where each drone really is, and from the image centre a red arrow along the
    estimated direction and a green one along the true direction of each detected drone."""
    out = rgb.copy()
    height, width = out.shape[:2]
    centre = (width // 2, height // 2)
    font = cv2.FONT_HERSHEY_SIMPLEX
    if truth is not None:
        detected = {row.get('target') for row in rows}
        for drone in truth['drones']:
            if drone['pixel'] is None:
                continue
            u, v = (round(p) for p in drone['pixel'])
            cv2.drawMarker(out, (u, v), GREEN, cv2.MARKER_CROSS, 24, 1)
            if drone['id'] in detected:
                cv2.arrowedLine(out, centre, (u, v), GREEN, 2, cv2.LINE_AA, tipLength=0.04)
            else:  # in view but not detected: say which drone the cross is
                text = f"drone {drone['id']}, {drone['distance']:.1f} m, not detected"
                cv2.putText(out, text, (u + 16, v + 4), font, 0.5, (255, 255, 255), 3, cv2.LINE_AA)
                cv2.putText(out, text, (u + 16, v + 4), font, 0.5, GREEN, 1, cv2.LINE_AA)
    for c, row in zip(found, rows):
        u, v = (round(p) for p in c.centre)
        if 'est_dir_x' in row:
            cv2.arrowedLine(out, centre, (u, v), RED, 1, cv2.LINE_AA, tipLength=0.04)
        cv2.rectangle(out, (c.x - pad, c.y - pad), (c.x + c.width + pad, c.y + c.height + pad),
                      RED, 2)
        lines = [(f"est {row['est_distance']:.1f} m ({row['size_px']} px)", RED)]
        if row.get('target') is not None:
            lines.append((f"true {row['true_distance']:.1f} m (drone {row['target']})", GREEN))
            lines.append((f"direction off {row['direction_error_deg']:.1f} deg", RED))
            lines.append((_vector('est', row, 'est_dir'), RED))
            lines.append((_vector('true', row, 'true_dir'), GREEN))
        elif 'est_dir_x' in row:
            lines.append(('no drone here', RED))
        x, y = c.x + c.width + pad + 8, c.y - pad + 12
        if x > width - 330:
            x = max(0, c.x - pad - 330)
        for text, colour in lines:
            cv2.putText(out, text, (x, y), font, 0.5, (255, 255, 255), 3, cv2.LINE_AA)
            cv2.putText(out, text, (x, y), font, 0.5, colour, 1, cv2.LINE_AA)
            y += 18
    if truth is not None:
        legend = 'red: estimated   green: true   arrows from the image centre   vectors: north, east, down'
        cv2.putText(out, legend, (10, height - 12), font, 0.5, (255, 255, 255), 3, cv2.LINE_AA)
        cv2.putText(out, legend, (10, height - 12), font, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    return out


def _vector(label: str, row: dict, key: str) -> str:
    return f"{label} dir ({row[key + '_x']:+.3f}, {row[key + '_y']:+.3f}, {row[key + '_z']:+.3f})"


def plot(rows: list[dict], out: Path, method: str) -> None:
    """Plots of the estimates against the truth, one point per detected drone."""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('No plots: matplotlib is not installed here')
        return
    out.mkdir(parents=True, exist_ok=True)
    true = [r['true_distance'] for r in rows]
    est = [r['est_distance'] for r in rows]
    size = [r['size_px'] for r in rows]
    n = len(rows)

    fig, ax = plt.subplots(figsize=(5, 5))
    limit = max(true + est) * 1.1
    ax.plot([0, limit], [0, limit], color='grey', linestyle='--', label='perfect')
    ax.scatter(true, est, color='tab:red', label=f'{method} ({n} detections)')
    ax.set(xlabel='True distance (m)', ylabel='Estimated distance (m)', xlim=(0, limit),
           ylim=(0, limit), title=f'Distance from box size, assuming a {TARGET_SIZE_M} m drone')
    ax.legend()
    fig.savefig(out / 'distance_estimated_vs_true.png', dpi=150, bbox_inches='tight')
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(6, 4))
    ax.axhline(0, color='grey', linestyle='--')
    ax.scatter(true, [100 * r['distance_error'] / r['true_distance'] for r in rows],
               color='tab:red')
    ax.set(xlabel='True distance (m)', ylabel='Distance error (% of true distance)',
           title='Distance error against distance')
    fig.savefig(out / 'distance_error.png', dpi=150, bbox_inches='tight')
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(6, 4))
    ax.scatter(true, [r['direction_error_deg'] for r in rows], color='tab:red')
    ax.set(xlabel='True distance (m)', ylabel='Direction error (degrees)', ylim=(0, None),
           title='Angle between estimated and true direction')
    fig.savefig(out / 'direction_error.png', dpi=150, bbox_inches='tight')
    plt.close(fig)

    # What the drone's real width would have to be for r = f s / p to give the true distance.
    fig, ax = plt.subplots(figsize=(6, 4))
    ax.axhline(TARGET_SIZE_M, color='grey', linestyle='--', label=f'assumed {TARGET_SIZE_M} m')
    ax.scatter(size, [TARGET_SIZE_M * t / e for t, e in zip(true, est)], color='tab:red',
               label='size that would fit')
    ax.set(xlabel='Box size (px)', ylabel='Drone size (m)', ylim=(0, None),
           title='Apparent size of the drone')
    ax.legend()
    fig.savefig(out / 'apparent_size.png', dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Plots in {out}')


if __name__ == '__main__':
    sys.exit(main())
