"""The tactical map: a top-down view of a recorded run, drawn afterwards, as a video source.

    python3 -m swarm_backend.demo.map_video RUN --start S --duration D [--out F] [--size WxH]
    (recording/video.sh map ...; recording/video.sh final cuts it by itself)

Drawn from poses.jsonl (true positions) and ui.jsonl (gimbal headings, detections), at 25 fps
on sim time like every other source: frame i shows sim time S + i / 25. No browser and no
GPU, so it never pops anything up and is sharp at any size. In the operator UI's colours:
defenders blue with their camera cones, the hostile grey until a defender has seen it and
red after, birds as small grey dots, the search arc from the run's config, and a timeline of
the takes' moments along the bottom. Before the first pose the first picture is held.
"""
import argparse
import json
import math
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

from swarm_backend.demo.runinfo import read_run_info
from swarm_backend.demo.takes import CONFIG, Take, find_takes, graveyard, in_graveyard
from swarm_backend.ui.camera_video import load_poses, pose_at
from swarm_backend.ui.replay import ReplayUi, load_recording

FPS = 25.0
# BGR, as the operator UI (ui/static/index.html): --bg, --friend, --hostile, --unknown/grey
BG = (19, 17, 10)
FRIEND = (255, 200, 92)
HOSTILE = (93, 93, 255)
GREY = (153, 148, 138)
BIRD = (80, 200, 80)                  # birds: green, as in the camera clips
GRID = (52, 46, 34)
AREA = (154, 212, 95)
TEXT = (235, 235, 235)
GHOST = (176, 172, 165)               # the baseline in a ghost overlay (GREY, lighter)
CONE_DEG, CONE_M = 60.0, 40.0
VELOCITY_LEAD_S = 2.0                 # the commanded velocity arrow: where it is in 2 s (as the UI)
TIMELINE_H = 46                       # px at 1080 high, scaled with the height
CLOCK_BOX = (0, 0, 420, 60)           # x0, y0, x1, y1 of the sim time text (at 1920x1080)
DEFAULTS = {'half_size_m': 260.0, 'centre_enu': [0.0, 110.0], 'trail_s': 10.0}


@dataclass
class MapState:
    sims: list[float]
    rows: list[dict]
    ui: ReplayUi
    defenders: list[str]
    hostiles: list[str]
    birds_prefix: str
    config: dict
    takes: list[Take]
    grave: tuple[float, float]
    view: dict
    size: tuple[int, int]
    bird_names: list[str] = field(default_factory=list)  # in any pose row (birds spawn mid-run)


def map_state(folder: Path, cfg: dict | None = None, size: tuple[int, int] = (1920, 1080)) -> MapState:
    folder = Path(folder)
    if cfg is None:
        try:
            cfg = json.loads(CONFIG.read_text()).get('map', {})
        except (OSError, ValueError):
            cfg = {}
    info = read_run_info(folder)
    sims, rows = load_poses(folder / 'poses.jsonl')
    return MapState(sims, rows, ReplayUi(load_recording(folder / 'ui.jsonl')), info.defenders,
                    info.hostiles, info.birds_prefix, info.config, find_takes(folder),
                    graveyard(), {**DEFAULTS, **cfg}, size,
                    sorted({n for r in rows for n in r if n.startswith(info.birds_prefix)}))


def _scale(st: MapState) -> float:
    w, h = st.size
    return (h - TIMELINE_H * h / 1080) / 2 / st.view['half_size_m']


def project(st: MapState, x: float, y: float) -> tuple[int, int]:
    """Gazebo ENU (x east, y north) to pixels, north up."""
    w, h = st.size
    cx, cy = st.view['centre_enu']
    s = _scale(st)
    return int(round(w / 2 + (x - cx) * s)), int(round((h - TIMELINE_H * h / 1080) / 2 - (y - cy) * s))


def _pos(st, name, sim):
    p = pose_at(st.sims, st.rows, name, sim)
    return None if p is None else p[0]


def _drone_id(name: str) -> int | None:
    tail = name.rsplit('_', 1)[-1]
    return int(tail) if tail.isdigit() else None


def _take_at(st, sim) -> Take | None:
    now = [t for t in st.takes if t.moments['start'] - 1e-6 <= sim]
    return now[-1] if now else (st.takes[0] if st.takes else None)


def render_frame(st: MapState, sim: float) -> np.ndarray:
    w, h = st.size
    k = h / 1080
    img = np.full((h, w, 3), BG, np.uint8)
    t = max(sim, st.sims[0]) if st.sims else sim  # held before the first pose
    s = _scale(st)
    line = lambda n: max(1, int(round(n * k)))  # noqa: E731

    # range rings around the base, the search arc
    surv = st.config.get('surveillance') or {}
    bn, be = (surv.get('base_position') or [0.0, 0.0])[:2]
    bu, bv = project(st, be, bn)
    for r in range(50, 1000, 50):
        if r * s > max(w, h):
            break
        cv2.circle(img, (bu, bv), int(r * s), GRID, line(1), cv2.LINE_AA)
    if surv.get('arc_radius'):
        head, arc, rad = surv.get('threat_heading_deg', 0.0), surv.get('arc_deg', 90.0), surv['arc_radius']
        pts = [project(st, be + rad * math.sin(math.radians(a)), bn + rad * math.cos(math.radians(a)))
               for a in np.linspace(head - arc / 2, head + arc / 2, 60)]
        cv2.polylines(img, [np.array(pts, np.int32)], False, AREA, line(2), cv2.LINE_AA)
    cv2.rectangle(img, (bu - line(9), bv - line(9)), (bu + line(9), bv + line(9)), TEXT, line(2))
    cv2.circle(img, (bu, bv), line(3), TEXT, -1)

    ui_line = st.ui.at(t)['state']
    by_id = {d.get('id'): d for d in ui_line.get('drones', [])}
    take = _take_at(st, t)
    spotted = bool(take and 'first_sighting' in take.moments and t >= take.moments['first_sighting'])
    trail_s = st.view['trail_s']

    def trail(name, colour, alive=lambda p: True):
        pts = []
        for j in range(int(trail_s * 5), -1, -1):
            p = _pos(st, name, t - j / 5)
            if p is not None and alive(p):
                pts.append(project(st, p[0], p[1]))
        if len(pts) > 1:
            cv2.polylines(img, [np.array(pts, np.int32)], False, colour, line(2), cv2.LINE_AA)

    # birds
    for name in st.bird_names:
        p = _pos(st, name, t)
        if p is not None:
            cv2.circle(img, project(st, p[0], p[1]), line(3), BIRD, -1, cv2.LINE_AA)

    # defenders with camera cones
    font = cv2.FONT_HERSHEY_SIMPLEX
    for name in st.defenders:
        p = _pos(st, name, t)
        if p is None:
            continue
        u, v = project(st, p[0], p[1])
        view = by_id.get(_drone_id(name)) or {}
        gh = view.get('gimbal_heading_deg')
        if gh is not None:
            a0, a1 = math.radians(gh - CONE_DEG / 2), math.radians(gh + CONE_DEG / 2)
            r = CONE_M * s
            cone = np.array([(u, v), (u + r * math.sin(a0), v - r * math.cos(a0)),
                             (u + r * math.sin(a1), v - r * math.cos(a1))], np.int32)
            over = img.copy()
            cv2.fillPoly(over, [cone], FRIEND)
            cv2.addWeighted(over, 0.22, img, 0.78, 0, img)
        trail(name, FRIEND)
        engaged = view.get('status') in ('intercept', 'target lost', 'hit')
        cv2.circle(img, (u, v), line(9 if engaged else 7), FRIEND, -1, cv2.LINE_AA)
        cv2.putText(img, f'D{_drone_id(name)}', (u + line(12), v - line(8)), font, 0.6 * k,
                    FRIEND, line(2), cv2.LINE_AA)

    # truth vs. belief: each defender's estimate of the target, while the target is alive
    alive_targets = [p for p in (_pos(st, n, t) for n in st.hostiles)
                     if p is not None and not in_graveyard(p, st.grave)]
    rect = (0, 0, w, int(h - TIMELINE_H * k))
    for name in st.defenders:
        view = by_id.get(_drone_id(name)) or {}
        me = _pos(st, name, t)
        if me is None:
            continue
        mu, mv = project(st, me[0], me[1])
        attack = view.get('attack') or {}
        vel = attack.get('velocity')
        if vel and view.get('status') in ('intercept', 'target lost'):
            end = project(st, me[0] + VELOCITY_LEAD_S * vel['east'], me[1] + VELOCITY_LEAD_S * vel['north'])
            ok, a, b = cv2.clipLine(rect, (mu, mv), end)
            if ok:
                cv2.arrowedLine(img, a, b, FRIEND, line(2), cv2.LINE_AA, tipLength=0.2)
        aim = attack.get('target')
        if aim and alive_targets:
            au, av = project(st, aim['east'], aim['north'])
            r = line(6)
            for d0, d1 in (((-r, -r), (r, r)), ((-r, r), (r, -r))):
                ok, a, b = cv2.clipLine(rect, (au + d0[0], av + d0[1]), (au + d1[0], av + d1[1]))
                if ok:
                    cv2.line(img, a, b, AREA, line(2), cv2.LINE_AA)
        det = view.get('detection')
        if not det or not alive_targets:
            continue
        eu, ev = project(st, det['east'], det['north'])
        # what the camera sees: a dashed line from the defender to its estimate
        ok, a, b = cv2.clipLine(rect, (mu, mv), (eu, ev))
        if ok:
            n_dash = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / line(10)))
            for j in range(0, n_dash, 2):
                p0 = (int(a[0] + (b[0] - a[0]) * j / n_dash), int(a[1] + (b[1] - a[1]) * j / n_dash))
                p1 = (int(a[0] + (b[0] - a[0]) * (j + 1) / n_dash), int(a[1] + (b[1] - a[1]) * (j + 1) / n_dash))
                cv2.line(img, p0, p1, HOSTILE, line(1), cv2.LINE_AA)
        if 0 <= eu < w and 0 <= ev < rect[3]:
            cv2.circle(img, (eu, ev), line(9), HOSTILE, line(2), cv2.LINE_AA)
        truth = min(alive_targets, key=lambda p: math.hypot(p[0] - det['east'], p[1] - det['north']))
        tu, tv = project(st, truth[0], truth[1])
        ok, a, b = cv2.clipLine(rect, (eu, ev), (tu, tv))
        if ok:
            cv2.line(img, a, b, GREY, line(1), cv2.LINE_AA)
            err = math.dist((det['east'], det['north'], det['altitude']), truth)
            cv2.putText(img, f'err {err:.1f} m', (a[0] + line(12), a[1] + line(18)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5 * k, GREY, line(1), cv2.LINE_AA)

    # hostiles: gone once in the graveyard
    for name in st.hostiles:
        p = _pos(st, name, t)
        if p is None or in_graveyard(p, st.grave):
            continue
        colour = HOSTILE if spotted else GREY
        trail(name, colour, alive=lambda q: not in_graveyard(q, st.grave))
        u, v = project(st, p[0], p[1])
        r = line(11)
        diamond = np.array([(u, v - r), (u + r, v), (u, v + r), (u - r, v)], np.int32)
        cv2.fillPoly(img, [diamond], colour, cv2.LINE_AA)

    # timeline: the whole recording, the takes' moments as ticks, a cursor
    y0 = int(h - TIMELINE_H * k)
    cv2.rectangle(img, (0, y0), (w, h), (12, 10, 6), -1)
    if st.sims:
        lo, hi = st.sims[0], st.sims[-1]
        x_at = lambda s_: int(line(20) + (w - 2 * line(20)) * (s_ - lo) / max(1e-6, hi - lo))  # noqa: E731
        ym = y0 + int(TIMELINE_H * k / 2)
        cv2.line(img, (x_at(lo), ym), (x_at(hi), ym), GRID, line(2))
        for tk in st.takes:
            for m, at in tk.moments.items():
                if m in ('first_sighting', 'hit', 'release'):
                    colour = {'hit': HOSTILE, 'first_sighting': AREA}.get(m, GREY)
                    cv2.line(img, (x_at(at), ym - line(9)), (x_at(at), ym + line(9)), colour, line(3))
        cv2.circle(img, (x_at(min(max(t, lo), hi)), ym), line(7), TEXT, -1, cv2.LINE_AA)

    cv2.putText(img, f'sim {sim:7.1f} s', (line(20), line(42)), font, 1.0 * k, TEXT, line(2),
                cv2.LINE_AA)
    return img


def render_ghost(ours: MapState, base: MapState, t_ours: float, t_base: float) -> np.ndarray:
    """Ours as render_frame draws it, with the baseline's run of the same pair on top as a
    grey ghost (its defenders, trails and hostile), each at its own sim time; past a run's
    end its last picture is held (pose_at holds the last pose)."""
    img = render_frame(ours, t_ours)
    w, h = ours.size
    k = h / 1080
    line = lambda n: max(1, int(round(n * k)))  # noqa: E731
    layer = np.zeros_like(img)
    t = min(max(t_base, base.sims[0]), base.sims[-1]) if base.sims else t_base
    trail_s = base.view['trail_s']
    names = list(base.defenders) + list(base.hostiles)
    attacker = None
    take = _take_at(base, t)
    if take is not None and take.attacker is not None:
        attacker = next((n for n in base.defenders if n.endswith(f'_{take.attacker}')), None)
    for name in names:
        alive = lambda q: not in_graveyard(q, base.grave)  # noqa: E731
        pts = []
        for j in range(int(trail_s * 5), -1, -1):
            p = _pos(base, name, t - j / 5)
            if p is not None and alive(p):
                pts.append(project(ours, p[0], p[1]))
        if len(pts) > 1:
            cv2.polylines(layer, [np.array(pts, np.int32)], False, GHOST, line(2), cv2.LINE_AA)
        p = _pos(base, name, t)
        if p is None or not alive(p):
            continue
        u, v = project(ours, p[0], p[1])
        if name in base.hostiles:
            r = line(10)
            cv2.polylines(layer, [np.array([(u, v - r), (u + r, v), (u, v + r), (u - r, v)], np.int32)],
                          True, GHOST, line(2), cv2.LINE_AA)
        else:
            cv2.circle(layer, (u, v), line(7), GHOST, line(2), cv2.LINE_AA)
            if name == attacker:
                cv2.putText(layer, 'baseline', (u + line(12), v + line(18)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5 * k, GHOST, line(1), cv2.LINE_AA)
    mask = layer.any(axis=2)
    img[mask] = layer[mask]
    return img


def write_ghost(ours_folder: Path, base_folder: Path, ours_start: float, base_start: float,
                duration: float, out: Path, size: tuple[int, int] = (1920, 1080),
                fps: float = FPS) -> Path:
    ours, base = map_state(ours_folder, size=size), map_state(base_folder, size=size)
    w, h = size
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    enc = subprocess.Popen(['ffmpeg', '-v', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'bgr24',
                            '-s', f'{w}x{h}', '-r', f'{fps:g}', '-i', '-', '-c:v', 'libx264',
                            '-preset', 'medium', '-crf', '18', '-pix_fmt', 'yuv420p', str(out)],
                           stdin=subprocess.PIPE)
    try:
        for i in range(int(round(duration * fps))):
            enc.stdin.write(render_ghost(ours, base, ours_start + i / fps, base_start + i / fps).tobytes())
    finally:
        enc.stdin.close()
        enc.wait()
    if enc.returncode != 0:
        raise RuntimeError(f'ffmpeg failed writing {out}')
    return out


def write_map(folder: Path, start: float, duration: float, out: Path,
              size: tuple[int, int] = (1920, 1080), fps: float = FPS) -> Path:
    st = map_state(folder, size=size)
    w, h = size
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    enc = subprocess.Popen(['ffmpeg', '-v', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'bgr24',
                            '-s', f'{w}x{h}', '-r', f'{fps:g}', '-i', '-', '-c:v', 'libx264',
                            '-preset', 'medium', '-crf', '18', '-pix_fmt', 'yuv420p', str(out)],
                           stdin=subprocess.PIPE)
    try:
        for i in range(int(round(duration * fps))):
            enc.stdin.write(render_frame(st, start + i / fps).tobytes())
    finally:
        enc.stdin.close()
        enc.wait()
    if enc.returncode != 0:
        raise RuntimeError(f'ffmpeg failed writing {out}')
    return out


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('run', type=Path)
    p.add_argument('--start', type=float, required=True)
    p.add_argument('--duration', type=float, required=True)
    p.add_argument('--size', default='1920x1080')
    p.add_argument('--out', type=Path)
    args = p.parse_args(argv)
    size = tuple(int(v) for v in args.size.lower().split('x'))
    out = args.out or args.run / f'map_{args.start:g}.mp4'
    write_map(args.run, args.start, args.duration, out, size)
    print(f'Wrote {out}: the map from sim time {args.start:g} s, {args.duration:g} s')
    return 0


if __name__ == '__main__':
    sys.exit(main())
