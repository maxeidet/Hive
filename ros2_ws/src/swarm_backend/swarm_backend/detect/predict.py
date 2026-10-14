"""Run a detector on saved images and save what it finds (`mount-backend.sh predict`).

    python3 -m swarm_backend.detect.predict baseline
    python3 -m swarm_backend.detect.predict baseline --images data/Manualimages --out data/results/baseline

For every PNG it writes the image with a red box on each detection, and one results.csv
with a row per detection (an image with none gets one row with empty fields). Each detection
gets an estimated distance (from its size in pixels) and direction (from where it is in the
image). When the image has the truth file that snap saves, these are compared with the true
distance and direction, drawn on the image and plotted in <out>/plots. It also gives the
target_offset angles the live detector publishes, and compares them with the truth twice:
without roll correction (what the live node sends today) and corrected with the camera's
true roll (what it will send once the roll comes from PX4). The first argument is the
detector, any name in detectors.py (`baseline`, `yolo`). Needs only NumPy and OpenCV.
"""
import argparse
import csv
import json
import math
import sys
from pathlib import Path

import cv2

from swarm_backend.detect import detectors
from swarm_backend.detect.camera import (
    GIMBAL_HFOV, TARGET_SIZE_M, CameraModel, angle_deg, camera_roll, true_offset)
from swarm_backend.detect.detectors import Detection

FIELDS = ['image', 'candidate', 'x', 'y', 'width', 'height', 'score', 'label',
          'size_px', 'est_distance', 'est_dir_x', 'est_dir_y', 'est_dir_z',
          'offset_x_deg', 'offset_y_deg', 'roll_deg', 'offset_rolled_x_deg', 'offset_rolled_y_deg',
          'target', 'true_distance', 'true_dir_x', 'true_dir_y', 'true_dir_z',
          'true_offset_x_deg', 'true_offset_y_deg',
          'distance_error', 'direction_error_deg', 'pixel_error',
          'offset_error_deg', 'offset_rolled_error_deg',
          'px4_direction_error_deg', 'position_n', 'position_e', 'position_d', 'position_error_m']
DATA = Path('/workspace/data')  # /workspace is the repo on the host
MATCH_PX = 40.0  # a detection this close to where a drone really shows is that drone
RED, GREEN = (255, 0, 0), (0, 160, 0)  # estimated, true (RGB)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('method', choices=detectors.DETECTORS, nargs='?', default='baseline',
                        help='which detector to run (default %(default)s)')
    parser.add_argument('--images', type=Path, default=DATA / 'Manualimages',
                        help='folder with PNG images (default %(default)s)')
    parser.add_argument('--out', type=Path,
                        help='folder for the results (default data/results/<method>)')
    parser.add_argument('--no-plots', action='store_true', help='skip the plots')
    detectors.add_options(parser)
    args = parser.parse_args(argv)
    out = args.out or DATA / 'results' / args.method
    try:
        detect = detectors.make(args.method, args)
    except NotImplementedError as e:
        print(e)
        return 1

    # live.py --save also writes <name>_boxes.png copies with its own boxes drawn on them.
    paths = sorted(p for p in args.images.glob('*.png') if not p.stem.endswith('_boxes'))
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
        found: list[Detection] = detect(rgb, None)
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
    orientation_summary(paths)
    matched = [r for r in rows if r.get('target') is not None]
    if matched and not args.no_plots:
        plot(matched, out / 'plots', args.method)
    return 0


def orientation_summary(paths: list[Path]) -> None:
    """How far the camera orientation PX4 reported was from the truth, over all images whose
    .json has it (snap saves it; task 2 of the baseline attack plan)."""
    errors = [t['px4_camera'] for t in map(load_truth, paths)
              if t is not None and t.get('px4_camera')]
    if not errors:
        return
    for key, label in (('forward_error_deg', 'where it looks'), ('roll_error_deg', 'roll')):
        values = sorted(e[key] for e in errors)
        print(f'PX4 camera orientation, {label}: median {values[len(values) // 2]:.2f} deg, '
              f'worst {values[-1]:.2f} deg over {len(values)} images')


def load_truth(image: Path) -> dict | None:
    """The .json that snap saved with the image (ground_truth.py), if there is one."""
    path = image.with_suffix('.json')
    return json.loads(path.read_text()) if path.exists() else None


def describe(image: str, number: int, c: Detection, truth: dict | None,
             width: int, height: int) -> dict:
    """One results.csv row: the detection, the distance and direction estimated from it,
    and, when the image has truth, the same for the drone it is and the errors."""
    size_px = c.size
    row = {'image': image, 'candidate': number, 'x': c.x, 'y': c.y, 'width': c.width,
           'height': c.height, 'score': round(c.score, 2), 'label': c.label, 'size_px': size_px}
    # Without truth there is no camera orientation either, so only the distance.
    hfov = truth['camera']['hfov_rad'] if truth else GIMBAL_HFOV
    camera = CameraModel(hfov, width, height)
    row['est_distance'] = round(camera.range_from_width(size_px), 2)
    offset = camera.offset_angles(*c.centre)  # what the live node publishes today: no roll
    row['offset_x_deg'], row['offset_y_deg'] = (round(math.degrees(a), 2) for a in offset)
    if truth is None:
        return row
    # The camera's orientation is taken from the truth: on the drone it will come from the
    # gimbal's attitude. So the direction error below is from the detection alone.
    cam = truth['camera']
    direction = camera.direction(*c.centre, cam['forward'], cam['right'], cam['down'])
    row.update(zip(('est_dir_x', 'est_dir_y', 'est_dir_z'), (round(v, 5) for v in direction)))
    roll = camera_roll(cam['forward'], cam['right'])
    rolled = camera.offset_angles(*c.centre, roll)
    row['roll_deg'] = round(math.degrees(roll), 1)
    row['offset_rolled_x_deg'], row['offset_rolled_y_deg'] = (
        round(math.degrees(a), 2) for a in rolled)
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
    true = true_offset(tuple(target['direction']), tuple(cam['forward']))
    row['true_offset_x_deg'], row['true_offset_y_deg'] = (round(math.degrees(a), 2) for a in true)
    row['offset_error_deg'] = round(math.degrees(math.dist(offset, true)), 2)
    row['offset_rolled_error_deg'] = round(math.degrees(math.dist(rolled, true)), 2)
    # target_position as the live node computes it: PX4's camera orientation, not the truth's.
    px4 = truth.get('px4_camera')
    if px4:
        axes = (px4['forward'], px4['right'], px4['down'])
        px4_direction = camera.direction(*c.centre, *axes)
        row['px4_direction_error_deg'] = round(
            angle_deg(px4_direction, tuple(target['direction'])), 2)
        position = camera.position(*c.centre, size_px, *axes)
        true_position = [t - p for t, p in zip(target['position'], cam['position'])]
        row['position_n'], row['position_e'], row['position_d'] = (round(v, 2) for v in position)
        row['position_error_m'] = round(math.dist(position, true_position), 2)
    return row


def summary(row: dict) -> str:
    text = f"\n    {row['size_px']} px: estimated {row['est_distance']:.1f} m"
    if row.get('target') is not None:
        text += (f", true {row['true_distance']:.1f} m (drone {row['target']}), "
                 f"direction off by {row['direction_error_deg']:.1f} deg; offset off by "
                 f"{row['offset_error_deg']:.1f} deg without roll correction, "
                 f"{row['offset_rolled_error_deg']:.1f} deg with it (roll {row['roll_deg']:+.0f} deg)")
        if row.get('position_error_m') is not None:
            text += (f"; target_position (PX4 orientation) off by {row['position_error_m']:.1f} m, "
                     f"direction {row['px4_direction_error_deg']:.1f} deg")
    elif 'est_dir_x' in row:
        text += ', no drone there (false alarm)'
    return text


def annotate(rgb, found: list[Detection], rows: list[dict], truth: dict | None, pad: int = 12):
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
            lines.append((f"offset ({row['offset_x_deg']:+.1f}, {row['offset_y_deg']:+.1f}) deg, "
                          f"rolled ({row['offset_rolled_x_deg']:+.1f}, "
                          f"{row['offset_rolled_y_deg']:+.1f})", RED))
            lines.append((f"true offset ({row['true_offset_x_deg']:+.1f}, "
                          f"{row['true_offset_y_deg']:+.1f}) deg, roll {row['roll_deg']:+.0f}",
                          GREEN))
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

    fig, ax = plt.subplots(figsize=(6, 4))
    roll = [abs(r['roll_deg']) for r in rows]
    ax.scatter(roll, [r['offset_error_deg'] for r in rows], color='tab:red',
               label='not roll-corrected (live node today)')
    ax.scatter(roll, [r['offset_rolled_error_deg'] for r in rows], color='tab:green',
               label='corrected with the true roll')
    ax.set(xlabel='Camera roll (degrees, either way)', ylabel='target_offset error (degrees)',
           ylim=(0, None), title='target_offset against the truth')
    ax.legend()
    fig.savefig(out / 'offset_error.png', dpi=150, bbox_inches='tight')
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
