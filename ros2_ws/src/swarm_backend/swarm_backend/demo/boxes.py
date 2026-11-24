"""The detectors' confirmed target boxes, for zooming in on the YOLO view (story `layout: zoom`).

The scenarios start each defender's detector with `--record /tmp/detect_<id>.jsonl`
(detect/live.py result_record: per image its `stamp`, image `width`/`height`, the `confirmed`
tracks as [u, v, width, height, track id] with (u, v) the box centre, and the `target` track
id). recording/record.sh copies them into the run folder as detect_d<id>.jsonl. Boxes are
scaled to the debug video's size (debug_d<id>.json), which may differ from the camera's.
"""
import bisect
import json
from dataclasses import dataclass
from pathlib import Path


@dataclass
class Box:
    sim: float
    u: float
    v: float
    w: float
    h: float
    track: int


def load_boxes(folder: Path, drone: int) -> list[Box]:
    folder = Path(folder)
    path = folder / f'detect_d{drone}.jsonl'
    if not path.exists():
        return []
    meta_path = folder / f'debug_d{drone}.json'
    meta = json.loads(meta_path.read_text()) if meta_path.exists() else {}
    out = []
    for raw in path.read_text().splitlines():
        try:
            r = json.loads(raw)
        except json.JSONDecodeError:
            continue
        target = r.get('target')
        if target is None:
            continue
        sx = (meta.get('width') or r['width']) / r['width']
        sy = (meta.get('height') or r['height']) / r['height']
        for u, v, w, h, track in r.get('confirmed') or []:
            if track == target:
                out.append(Box(float(r['stamp']), u * sx, v * sy, w * sx, h * sy, int(track)))
    out.sort(key=lambda b: b.sim)
    return out


def box_at(boxes: list[Box], sim: float, max_gap: float = 0.5) -> Box | None:
    i = bisect.bisect_right([b.sim for b in boxes], sim) - 1
    if i < 0 or sim - boxes[i].sim > max_gap:
        return None
    return boxes[i]


def first_box(boxes: list[Box], start: float, end: float) -> Box | None:
    return next((b for b in boxes if start <= b.sim <= end), None)


def crop_rect(cu: float, cv: float, width: int, height: int, zoom: float) -> tuple[int, int, int, int]:
    """(x0, y0, w, h) of a 1/zoom crop centred on (cu, cv), shifted to stay inside the image."""
    cw, ch = int(width / zoom), int(height / zoom)
    x0 = int(round(min(max(cu - cw / 2, 0), width - cw)))
    y0 = int(round(min(max(cv - ch / 2, 0), height - ch)))
    return x0, y0, cw, ch
