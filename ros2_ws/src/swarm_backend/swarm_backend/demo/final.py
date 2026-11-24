"""Builds the demo film of a recorded run from a story file (recording/README.md).

    python3 -m swarm_backend.demo.final RUN --story recording/stories/default.yaml
        [--preview] [--skip-missing] [--take N]
    (recording/video.sh final [--story NAME] [--preview] ...)

Finds the run's takes and moments, resolves every scene of the story to a sim range, cuts
each source it needs once over all scenes (with the existing cut commands, cached in
RUN/cache/), encodes one clip per scene and card into RUN/scenes/<story>/ (01_..., for
slides), and joins them into RUN/final_<story>.mp4: H.264, 1920x1080, 25 fps, yuv420p,
faststart. --preview: 960x540, fast, into preview_<story>.mp4. A scene whose moment or
recording is missing stops the build before anything is encoded, with every problem listed;
--skip-missing builds the other scenes.
"""
import argparse
import re
import statistics
import subprocess
import sys
from pathlib import Path

from swarm_backend.demo.map_video import write_ghost, write_map
from swarm_backend.demo.story import (Resolved, Scene, StoryError, load_story, recorded_file,
                                      resolve)
from swarm_backend.demo.takes import Take, save_takes, shoot_takes
from swarm_backend.scenarios.common import PKG, REPO

FPS = 25
FULL, PREVIEW = (1920, 1080), (960, 540)
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
FONT_BOLD = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
FADE = 0.4            # seconds of fade in and out per scene
PAD = 1.0             # sim seconds cut on each side of what the scenes need
CARD_BG = '0x0a1113'
SCREEN_PY = REPO / 'recording' / 'screen.py'


class CutError(RuntimeError):
    pass


def encode_args(preview: bool) -> list[str]:
    if preview:
        return ['-c:v', 'libx264', '-preset', 'ultrafast', '-crf', '28', '-pix_fmt', 'yuv420p']
    return ['-c:v', 'libx264', '-preset', 'medium', '-crf', '18', '-pix_fmt', 'yuv420p']


def text(s: str) -> str:
    """drawtext's text, escaped for the filter graph."""
    # A straight quote cannot be escaped inside drawtext's quoted text: use the typographic one.
    s = s.replace("'", '\u2019')
    return s.replace('\\', '\\\\').replace(':', '\\:').replace(',', '\\,').replace('%', '\\%')


def slug(s: str) -> str:
    return re.sub(r'[^a-z0-9]+', '-', s.lower()).strip('-')[:40] or 'scene'


def role_label(role: str, attacker: int | None, setup: str | None = None) -> str:
    """The panel's title. `setup`: the take's (baseline: the blob detector, not YOLO)."""
    detector = 'baseline detector' if setup == 'baseline' else 'YOLO'
    if role.startswith('plot:'):
        from swarm_backend.demo.plots import PLOTS
        return PLOTS[role[5:]][1]
    labels = {'map': 'Tactical map', 'ui': 'Operator UI', 'chase': 'Chase camera',
              'wide': 'Wide shot', 'yolo': f'Drone {attacker} camera - {detector} + tracker'}
    if role in labels:
        return labels[role]
    kind, _, arg = role.partition(':')
    return f'Drone {arg} camera - {detector} + tracker' if kind == 'yolo' else arg.replace('_', ' ')


# ---- cutting the sources ----------------------------------------------------------------------

def needed_cuts(resolved: list[Resolved]) -> dict[tuple, tuple[float, float]]:
    """Per (run folder, source file), the sim range all scenes need, padded by PAD."""
    out = {}
    for r in resolved:
        for key in r.sources.values():
            k = (r.take.folder if r.take else None, key)
            lo, hi = out.get(k, (r.start, r.end))
            out[k] = (min(lo, r.start), max(hi, r.end))
    return {k: (round(lo - PAD, 3), round(hi + PAD, 3)) for k, (lo, hi) in out.items()}


def source_file(folder: Path, key: str) -> Path:
    return recorded_file(folder, key)


def cut_source(folder: Path, key: str, start: float, end: float, preview: bool) -> Path:
    """The source `key` from sim `start` to `end`, cached in RUN/cache/; reused while it is
    newer than what it was cut from."""
    cache = folder / 'cache' / ('preview' if preview else 'full')
    cache.mkdir(parents=True, exist_ok=True)
    out = cache / f'{key}_{start:.2f}_{end:.2f}.mp4'
    src = source_file(folder, key)
    if out.exists() and out.stat().st_mtime > src.stat().st_mtime:
        return out
    dur = f'{end - start:.3f}'
    if key == 'map':
        write_map(folder, start, end - start, out, PREVIEW if preview else FULL)
        return out
    if key.startswith('plot_'):
        from swarm_backend.demo.plots import write_plot
        from swarm_backend.demo.takes import find_takes
        mid = (start + end) / 2
        takes = find_takes(folder)
        take = next((t for t in reversed(takes) if t.moments['start'] <= mid), takes[0])
        W, H = PREVIEW if preview else FULL
        write_plot(folder, take, key[len('plot_'):], start, end, out, (W // 2, H // 2))
        return out
    if key.startswith('debug_d'):
        cmd = [sys.executable, '-m', 'swarm_backend.ui.debug_video', 'cut', str(folder),
               '--id', key[len('debug_d'):], '--start', f'{start:.3f}', '--duration', dur]
    elif key.startswith('cam_'):
        cmd = [sys.executable, '-m', 'swarm_backend.ui.camera_video', 'overlay', str(folder),
               '--angle', key[len('cam_'):], '--start', f'{start:.3f}', '--duration', dur]
    elif key == 'screen':
        cmd = [sys.executable, str(SCREEN_PY), 'cut', str(folder), '--start', f'{start:.3f}',
               '--duration', dur]
    else:
        raise CutError(f'unknown source {key}')
    res = subprocess.run(cmd + ['--out', str(out)], cwd=PKG, capture_output=True, text=True)
    if res.returncode != 0:
        out.unlink(missing_ok=True)
        msg = (res.stderr.strip().splitlines() or [f'exit {res.returncode}'])[-1]
        raise CutError(msg)
    return out


# ---- encoding ---------------------------------------------------------------------------------

def _panel(label: str, w: int, h: int) -> str:
    return (f'scale={w}:{h}:force_original_aspect_ratio=decrease,'
            f'pad={w}:{h}:(ow-iw)/2:(oh-ih)/2:color=black,setsar=1,'
            f"drawtext=fontfile={FONT}:text='{text(label)}':x=w-tw-14:y=12:fontsize={max(14, h // 26)}:"
            f'fontcolor=white:box=1:boxcolor=black@0.55:boxborderw=8')


def label_filters(r: Resolved, H: int, x0: str = '24', y0: int = 0, offset: float = 0.0) -> str:
    """drawtext filters (with a leading comma) for the scene's insight labels, or ''."""
    if r.scene.insights is None or r.take is None:
        return ''
    from swarm_backend.demo.insights import insights_for
    out = ''
    fs = max(14, H // 34)
    for lab in insights_for(r.take, r.start, r.end, r.scene.speed, r.scene.insights or None):
        y = y0 + H // 12 + int(lab.row * fs * 1.9)
        out += (f",drawtext=fontfile={FONT_BOLD}:text='{text(lab.text)}':x={x0}:y={y}:fontsize={fs}:"
                f'fontcolor=white:box=1:boxcolor=0x0a1113@0.75:boxborderw={max(4, H // 120)}:'
                f"enable='between(t\\,{offset + lab.at:.3f}\\,{offset + lab.until:.3f})'")
    return out


def scene_clip(r: Resolved, cuts: dict, out: Path, preview: bool, label: str | None = None,
               extra: str | None = None) -> Path:
    """extra: drawtext filters to use instead of the scene's own insight labels (zoom)."""
    """One scene: its sources from their cuts (path, cut start), laid out, captioned, faded."""
    W, H = PREVIEW if preview else FULL
    s = r.scene
    length = (r.end - r.start) / s.speed
    size = {'full': (W, H), 'side': (W // 2, H // 2), 'grid': (W // 2, H // 2)}[s.layout]
    inputs, parts = [], []
    for k, role in enumerate(s.show):
        path, cut_start = cuts[(r.take.folder, r.sources[role])]
        inputs += ['-ss', f'{r.start - cut_start:.3f}', '-t', f'{r.end - r.start + 0.5:.3f}',
                   '-i', str(path)]
        attacker = r.take.attacker if r.take else None
        setup = r.take.facts.get('setup') if r.take else None
        parts.append(f'[{k}:v]fps={FPS},setpts=(PTS-STARTPTS)/{s.speed},'
                     f'{_panel(label or role_label(role, attacker, setup), *size)}[p{k}]')
    pins = ''.join(f'[p{k}]' for k in range(len(s.show)))
    body = {'full': f'{pins}null',
            'side': f'{pins}hstack=inputs=2,pad={W}:{H}:0:(oh-ih)/2:color=black',
            'grid': f'{pins}xstack=inputs=4:layout=0_0|w0_0|0_h0|w0_h0'}[s.layout]
    fs = H // 27
    parts.append(
        f'{body},trim=duration={length:.3f},setpts=PTS-STARTPTS{label_filters(r, H) if extra is None else extra},'
        f"drawtext=fontfile={FONT_BOLD}:text='{text(s.caption or '')}':x=(w-tw)/2:y=h-th-{H // 30}:"
        f'fontsize={fs}:fontcolor=white:box=1:boxcolor=black@0.65:boxborderw={H // 70},'
        f"drawtext=fontfile={FONT}:text='{text(f'sim {r.start:.0f}-{r.end:.0f} s')}':"
        f'x=w-tw-{H // 45}:y=h-th-{H // 25}:fontsize={H // 45}:fontcolor=white@0.8,'
        f'fade=t=in:st=0:d={FADE},fade=t=out:st={max(0.0, length - FADE):.3f}:d={FADE},'
        f'format=yuv420p,setsar=1[out]')
    out.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(['ffmpeg', '-v', 'error', '-y', *inputs, '-filter_complex', ';'.join(parts),
                    '-map', '[out]', '-r', str(FPS), '-t', f'{length:.3f}', *encode_args(preview),
                    '-an', str(out)], check=True)
    return out


def result_text(t: Take) -> str:
    """A side's result for the compare layout; with several hostiles, how many were hit."""
    f = t.facts
    c = f.get('closest_pass_m')
    if f.get('hits'):
        hit = f['hits'].split('/')[0] != '0'
        s = f.get('sighting_to_hit_s')
        if hit:
            return f"{f['hits']} hit" + (f', last {s:.1f} s after sighting' if s is not None else '')
        return f"{f['hits']} hit" + (f', closest {c:.1f} m' if c is not None else '')
    if f.get('hit'):
        s = f.get('sighting_to_hit_s')
        return f'hit {s:.1f} s after sighting' if s is not None else 'hit'
    return f'no hit, closest {c:.1f} m' if c is not None else 'no hit'


SIDE_NAMES = {'ours': 'Improved version', 'baseline': 'Baseline'}


def compare_clip(ours: Resolved, base: Resolved, cuts, out: Path, preview: bool) -> Path:
    """A baseline/ours pair side by side (baseline left), synced on release; each side's
    result appears at its own hit (or near the end without one)."""
    W, H = PREVIEW if preview else FULL
    s = ours.scene
    length = (ours.end - ours.start) / s.speed
    role = s.show[0]
    inputs, parts = [], []
    fs = H // 32
    for k, r in enumerate((base, ours)):
        path, cut_start = cuts[(r.take.folder, r.sources[role])]
        inputs += ['-ss', f'{r.start - cut_start:.3f}', '-t', f'{r.end - r.start + 0.5:.3f}',
                   '-i', str(path)]
        hit = r.take.moments.get('hit')
        at = (hit - r.start) / s.speed if hit is not None and r.start <= hit <= r.end else max(0.0, length - 1.5)
        label = f'{SIDE_NAMES[r.side]} - {r.take.facts.get("setup")}'
        parts.append(
            f'[{k}:v]fps={FPS},setpts=(PTS-STARTPTS)/{s.speed},{_panel(label, W // 2, H // 2)},'
            f"drawtext=fontfile={FONT_BOLD}:text='{text(result_text(r.take))}':x=(w-tw)/2:y=h-th-{H // 40}:"
            f'fontsize={fs}:fontcolor=white:box=1:boxcolor=black@0.6:boxborderw={H // 90}:'
            f"enable='gte(t,{at:.3f})'{label_filters(r, H // 2)}[p{k}]")
    fs2 = H // 27
    parts.append(
        f'[p0][p1]hstack=inputs=2,pad={W}:{H}:0:(oh-ih)/2:color=black,'
        f'trim=duration={length:.3f},setpts=PTS-STARTPTS,'
        f"drawtext=fontfile={FONT_BOLD}:text='{text(s.caption or '')}':x=(w-tw)/2:y=h-th-{H // 30}:"
        f'fontsize={fs2}:fontcolor=white:box=1:boxcolor=black@0.65:boxborderw={H // 70},'
        f"drawtext=fontfile={FONT}:text='{text(f'release +{ours.start - ours.take.moments['release']:.0f} s')}':"
        f'x=w-tw-{H // 45}:y=h-th-{H // 25}:fontsize={H // 45}:fontcolor=white@0.8,'
        f'fade=t=in:st=0:d={FADE},fade=t=out:st={max(0.0, length - FADE):.3f}:d={FADE},'
        f'format=yuv420p,setsar=1[out]')
    out.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(['ffmpeg', '-v', 'error', '-y', *inputs, '-filter_complex', ';'.join(parts),
                    '-map', '[out]', '-r', str(FPS), '-t', f'{length:.3f}', *encode_args(preview),
                    '-an', str(out)], check=True)
    return out


def zoom_clip(r: Resolved, cuts, out: Path, preview: bool) -> Path:
    """The YOLO view cropped 1/zoom around the locked target's box (smoothed, kept inside the
    picture), with `freeze` seconds of the first locked frame first; then captioned as a
    normal full scene."""
    import cv2
    import numpy as np

    from swarm_backend.demo.boxes import box_at, crop_rect, first_box, load_boxes
    W, H = PREVIEW if preview else FULL
    s = r.scene
    key = r.sources[s.show[0]]
    drone = int(key[len('debug_d'):])
    boxes = load_boxes(r.take.folder, drone)
    path, cut_start = cuts[(r.take.folder, key)]
    first_sim = min([r.start] + [b.sim for b in boxes if r.start <= b.sim <= r.end][:1])
    i0 = max(0, int(round((first_sim - cut_start) * FPS)))
    i1 = int(round((r.end - cut_start) * FPS)) + 1
    cap = cv2.VideoCapture(str(path))
    cap.set(cv2.CAP_PROP_POS_FRAMES, i0)  # only the scene's frames, not the whole cut
    frames = []
    ok, img = cap.read()
    while ok and len(frames) < i1 - i0:
        frames.append(img)
        ok, img = cap.read()
    cap.release()
    if not frames:
        raise CutError(f'{path} has no frames')
    fh, fw = frames[0].shape[:2]

    def frame_at(sim):
        return frames[min(len(frames) - 1, max(0, int(round((sim - cut_start) * FPS)) - i0))]

    centres = []
    def crop(sim):
        b = box_at(boxes, sim)
        if b is not None:
            centres.append((b.u, b.v))
        cu, cv_ = (np.mean([c[0] for c in centres[-8:]]), np.mean([c[1] for c in centres[-8:]])) \
            if centres else (fw / 2, fh / 2)
        x0, y0, cw, ch = crop_rect(cu, cv_, fw, fh, s.zoom)
        img = cv2.resize(frame_at(sim)[y0:y0 + ch, x0:x0 + cw], (W, H), interpolation=cv2.INTER_CUBIC)
        if b is not None:
            kx, ky = W / cw, H / ch
            p0 = (int((b.u - b.w / 2 - x0) * kx), int((b.v - b.h / 2 - y0) * ky))
            p1 = (int((b.u + b.w / 2 - x0) * kx), int((b.v + b.h / 2 - y0) * ky))
            cv2.rectangle(img, p0, p1, (93, 93, 255), max(1, H // 360))
        return img

    out.parent.mkdir(parents=True, exist_ok=True)
    tmp = out.with_name(out.stem + '.zoom.mp4')
    enc = subprocess.Popen(['ffmpeg', '-v', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'bgr24',
                            '-s', f'{W}x{H}', '-r', str(FPS), '-i', '-', *encode_args(preview),
                            str(tmp)], stdin=subprocess.PIPE)
    n_scene = int(round((r.end - r.start) / s.speed * FPS))
    first = first_box(boxes, r.start, r.end)
    try:
        if s.freeze > 0 and first is not None:
            still = crop(first.sim)
            cv2.putText(still, 'YOLO + tracker: locked', (W // 40, H - H // 8), cv2.FONT_HERSHEY_SIMPLEX,
                        H / 900, (255, 255, 255), max(1, H // 400), cv2.LINE_AA)
            for _ in range(int(round(s.freeze * FPS))):
                enc.stdin.write(still.tobytes())
            centres.clear()
        for i in range(n_scene):
            enc.stdin.write(crop(r.start + i * s.speed / FPS).tobytes())
    finally:
        enc.stdin.close()
        enc.wait()
    total = s.freeze + (r.end - r.start) / s.speed if first is not None else (r.end - r.start) / s.speed
    label = f'yolo:{drone}'
    single = Resolved(Scene(**{**s.__dict__, 'layout': 'full', 'speed': 1.0, 'show': [label],
                               'caption': s.caption}),
                      r.take, r.start, r.start + total, {label: 'zoom'})
    try:
        H = (PREVIEW if preview else FULL)[1]
        labels = label_filters(r, H, offset=s.freeze if first is not None else 0.0)
        return scene_clip(single, {(r.take.folder, 'zoom'): (tmp, r.start)}, out, preview,
                          label=f'Drone {drone} camera, zoomed {s.zoom:g}x', extra=labels)
    finally:
        tmp.unlink(missing_ok=True)


def ghost_clip(ours: Resolved, base: Resolved, out: Path, preview: bool) -> Path:
    """Both runs of a pair on one map (baseline grey), synced on release, as one full scene."""
    size = PREVIEW if preview else FULL
    cache = ours.take.folder / 'cache' / ('preview' if preview else 'full')
    src = cache / f'ghost_{ours.take.pair}_{ours.start:.2f}_{ours.end:.2f}.mp4'
    if not src.exists():
        write_ghost(ours.take.folder, base.take.folder, ours.start, base.start,
                    ours.end - ours.start, src, size)
    caption = f'{ours.scene.caption or ""}  (blue: ours, grey: baseline)'.strip()
    single = Resolved(Scene(**{**ours.scene.__dict__, 'layout': 'full', 'caption': caption}),
                      ours.take, ours.start, ours.end, {'map': 'ghost'})
    return scene_clip(single, {(ours.take.folder, 'ghost'): (src, ours.start)}, out, preview)


def compare_lines(pairs: list[tuple[Take, Take]]) -> list[str]:
    def count(side):
        return sum(bool(p[side].facts.get('hit')) for p in pairs)
    n = len(pairs)
    lines = [f'These runs: baseline {count(1)}/{n} hits, ours {count(0)}/{n} hits']
    for idx, name in ((1, 'baseline'), (0, 'ours')):
        times = [p[idx].facts['sighting_to_hit_s'] for p in pairs
                 if p[idx].facts.get('sighting_to_hit_s') is not None]
        if times:
            lines.append(f'{name}: {statistics.median(times):.1f} s from sighting to hit (median)')
    return lines


def bars_png(values: dict[str, float], title: str, out: Path, size: tuple[int, int]) -> Path:
    """A small bar chart (hit rate per setup) for a card."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    w, h = size
    fig = plt.figure(figsize=(w / 100, h / 100), dpi=100)
    fig.patch.set_alpha(0)
    ax = fig.add_subplot(111)
    ax.set_facecolor('none')
    names = list(values)
    ax.bar(names, [values[n] for n in names], color=['#8a9499', '#5cc8ff'][:len(names)])
    ax.set_ylim(0, 1)
    ax.set_title(title, color='white')
    ax.tick_params(colors='white')
    for side in ax.spines.values():
        side.set_color('#555')
    fig.savefig(out, transparent=True)
    plt.close(fig)
    return out


def card_clip(title: str, lines: list[str], seconds: float, out: Path, preview: bool,
              image: Path | None = None) -> Path:
    """A text card: the title, then smaller lines, white on the UI's background; an optional
    picture (a chart) under the lines."""
    W, H = PREVIEW if preview else FULL
    draws = [f"drawtext=fontfile={FONT_BOLD}:text='{text(title)}':x=(w-tw)/2:"
             f'y=h/2-th-{H // 40}:fontsize={H // 16}:fontcolor=white']
    for i, line in enumerate(lines):
        draws.append(f"drawtext=fontfile={FONT}:text='{text(line)}':x=(w-tw)/2:"
                     f'y=h/2+{H // 40 + i * H // 20}:fontsize={H // 30}:fontcolor=white@0.85')
    tail = [f'fade=t=in:st=0:d={FADE}', f'fade=t=out:st={max(0.0, seconds - FADE):.3f}:d={FADE}',
            'setsar=1']
    out.parent.mkdir(parents=True, exist_ok=True)
    src = ['-f', 'lavfi', '-i', f'color=c={CARD_BG}:s={W}x{H}:d={seconds:.3f}:r={FPS}']
    if image is None:
        args = ['-vf', ','.join(draws + tail)]
    else:
        y = H // 2 + H // 40 + len(lines) * H // 20
        args = ['-loop', '1', '-i', str(image), '-filter_complex',
                f'[0:v][1:v]overlay=(W-w)/2:{y}:shortest=1,' + ','.join(draws + tail)]
    subprocess.run(['ffmpeg', '-v', 'error', '-y', *src, *args, '-t', f'{seconds:.3f}',
                    *encode_args(preview), '-an', str(out)], check=True)
    return out


def end_card_lines(takes_used: list[Take]) -> list[str]:
    n = len(takes_used)
    hits = [t for t in takes_used if t.facts.get('hit')]
    lines = [f'{len(hits)} of {n} run{"s" if n != 1 else ""} ended in a hit']
    times = [t.facts['sighting_to_hit_s'] for t in hits if t.facts.get('sighting_to_hit_s') is not None]
    if times:
        lines.append(f'{statistics.median(times):.1f} s from first sighting to hit (median)')
    ranges = [t.facts['sighting_range_m'] for t in takes_used if t.facts.get('sighting_range_m') is not None]
    if ranges:
        lines.append(f'first seen at {statistics.median(ranges):.0f} m (median)')
    return lines


def concat(clips: list[Path], out: Path) -> Path:
    listing = out.with_suffix('.txt')
    listing.write_text(''.join(f"file '{c.resolve()}'\n" for c in clips))
    tmp = out.with_name(out.stem + '.joined.mp4')
    subprocess.run(['ffmpeg', '-v', 'error', '-y', '-f', 'concat', '-safe', '0', '-i', str(listing),
                    '-c', 'copy', str(tmp)], check=True)
    subprocess.run(['ffmpeg', '-v', 'error', '-y', '-i', str(tmp), '-c', 'copy',
                    '-movflags', '+faststart', str(out)], check=True)
    tmp.unlink()
    listing.unlink()
    return out


# ---- the build --------------------------------------------------------------------------------

def _label(r: Resolved) -> str:
    return f"scene {r.scene.index} '{r.scene.caption or ''}'"


def _labels(r: Resolved, prefix: str = '', offset: float = 0.0) -> list[tuple[float, float, str]]:
    if r.scene.insights is None or r.take is None:
        return []
    from swarm_backend.demo.insights import insights_for
    return [(offset + lab.at, offset + lab.until, prefix + lab.text)
            for lab in insights_for(r.take, r.start, r.end, r.scene.speed, r.scene.insights or None)]


def _srt_time(t: float) -> str:
    ms = int(round(t * 1000))
    return f'{ms // 3600000:02d}:{ms // 60000 % 60:02d}:{ms // 1000 % 60:02d},{ms % 1000:03d}'


def subtitles(entries: list[tuple[float, float, str]]) -> str:
    """SRT text: numbered entries (start, end in film seconds, text)."""
    blocks = [f'{i}\n{_srt_time(a)} --> {_srt_time(b)}\n{t}\n'
              for i, (a, b, t) in enumerate(sorted(entries, key=lambda e: e[0]), 1)]
    return '\n'.join(blocks)


def build(folder: Path, story_path: Path, preview: bool = False, skip_missing: bool = False,
          take: str | None = None, stills: bool = False) -> Path:
    folder, story_path = Path(folder), Path(story_path)
    try:
        story = load_story(story_path)
    except StoryError as e:
        print(f'Story: {e}')
        raise SystemExit(1) from None
    takes = shoot_takes(folder)
    for run in {t.folder for t in takes}:
        save_takes(run, [t for t in takes if t.folder == run])
    resolved, problems = resolve(story, takes, take)
    bad: set[int] = {int(m.group(1)) for p in problems if (m := re.match(r'scene (\d+)', p))}

    # every scene's recordings must exist
    for r in resolved:
        for role, key in r.sources.items():
            if not source_file(r.take.folder, key).exists():
                where = r.take.folder.name if r.take.folder != folder else ''
                problems.append(f'{_label(r)}: no recording of {role} '
                                f'({where + "/" if where else ""}{source_file(r.take.folder, key).name})')
                bad.add(r.scene.index)

    # a zoom scene needs a locked box in its range
    from swarm_backend.demo.boxes import first_box, load_boxes
    for r in resolved:
        if r.scene.layout == 'zoom' and r.take is not None and r.scene.index not in bad:
            drone = int(r.sources[r.scene.show[0]][len('debug_d'):])
            if first_box(load_boxes(r.take.folder, drone), r.start, r.end) is None:
                problems.append(f'{_label(r)}: drone {drone} confirmed no box between '
                                f'{r.start:.1f} and {r.end:.1f} (detect_d{drone}.jsonl)')
                bad.add(r.scene.index)

    def stop_or_skip():
        if not problems:
            return
        print('Cannot build every scene:')
        for p in problems:
            print(f'  {p}')
        if not skip_missing:
            print('Nothing was built. Fix the story or the run, or use --skip-missing.')
            raise SystemExit(1)
        print('Skipping those scenes (--skip-missing).')

    stop_or_skip()
    scenes = [r for r in resolved if r.scene.index not in bad]

    # cut every needed source once, then check nothing failed before encoding any scene
    cuts, cut_problems = {}, []
    for (run, key), (lo, hi) in needed_cuts([r for r in scenes if r.take and r.scene.layout != 'ghost']).items():
        try:
            cuts[(run, key)] = (cut_source(run, key, lo, hi, preview), lo)
        except (CutError, subprocess.CalledProcessError) as e:
            users = [r for r in scenes if r.take and r.take.folder == run and key in r.sources.values()]
            for r in users:
                cut_problems.append(f'{_label(r)}: {key} could not be cut: {e}')
                bad.add(r.scene.index)
    if cut_problems:
        problems.extend(cut_problems)
        stop_or_skip()
        scenes = [r for r in scenes if r.scene.index not in bad]

    name = story_path.stem
    clip_dir = folder / 'scenes' / name / ('preview' if preview else '')
    for old in clip_dir.glob('*.mp4'):
        old.unlink()
    clips, rows, n = [], [], 0

    def nxt(label):
        nonlocal n
        n += 1
        return clip_dir / f'{n:02d}_{slug(label)}.mp4'

    if story.title:
        clips.append(card_clip(story.title, [story.subtitle] if story.subtitle else [],
                               story.title_seconds, nxt(story.title), preview))
        rows.append(('title', story.title, '-', '-', story.title_seconds,
                     ' - '.join(x for x in (story.title, story.subtitle) if x), []))
    used, used_pairs = [], []
    i = 0
    while i < len(scenes):
        r = scenes[i]
        i += 1
        if r.scene.card is not None:
            clips.append(card_clip(r.scene.card, [], r.scene.duration, nxt(r.scene.card), preview))
            rows.append((str(r.scene.index), r.scene.card, '-', '-', r.scene.duration,
                         r.scene.card, []))
            continue
        if r.side == 'ours':
            base = scenes[i]
            i += 1
            if r.scene.layout == 'ghost':
                clips.append(ghost_clip(r, base, nxt(r.scene.caption or 'ghost'), preview))
            else:
                clips.append(compare_clip(r, base, cuts, nxt(r.scene.caption or 'compare'), preview))
            rows.append((str(r.scene.index), f'{r.scene.caption or ""}  [baseline: '
                         f'{result_text(base.take)} | ours: {result_text(r.take)}]',
                         r.take.pair, f'{r.start:.1f}-{r.end:.1f}', (r.end - r.start) / r.scene.speed,
                         r.scene.caption or '', _labels(base, 'baseline: ') + _labels(r, 'ours: ')))
            if (r.take, base.take) not in used_pairs:
                used_pairs.append((r.take, base.take))
            continue
        if r.scene.layout == 'zoom':
            clips.append(zoom_clip(r, cuts, nxt(r.scene.caption or 'zoom'), preview))
            length = (r.end - r.start) / r.scene.speed + r.scene.freeze
        else:
            clips.append(scene_clip(r, cuts, nxt(r.scene.caption or 'scene'), preview))
            length = (r.end - r.start) / r.scene.speed
        rows.append((str(r.scene.index), r.scene.caption or '', r.take.name or str(r.take.number),
                     f'{r.start:.1f}-{r.end:.1f}', length, r.scene.caption or '',
                     _labels(r, offset=r.scene.freeze if r.scene.layout == 'zoom' else 0.0)))
        if r.take not in used:
            used.append(r.take)
    if story.end_card == 'results' and used:
        clips.append(card_clip('Results', end_card_lines(used), story.end_seconds,
                               nxt('results'), preview))
        rows.append(('end', 'Results', '-', '-', story.end_seconds,
                     'Results: ' + '; '.join(end_card_lines(used)), []))
    if story.end_card == 'compare' and (used_pairs or story.evidence):
        lines = compare_lines(used_pairs) if used_pairs else []
        image = None
        if story.evidence:
            from swarm_backend.demo.evidence import evidence, evidence_lines
            ev_path = Path(story.evidence)
            ev_path = ev_path if ev_path.is_absolute() else REPO / ev_path
            ev = evidence(ev_path)
            lines += evidence_lines(ev)
            rates = {n: ev[n]['hit_rate'] for n in ('baseline', 'ours') if ev.get(n)}
            if rates:
                W, H = PREVIEW if preview else FULL
                image = bars_png(rates, 'Gazebo evaluation: hit rate',
                                 clip_dir / 'evidence_bars.png', (W // 3, H // 4))
        clips.append(card_clip('Baseline vs. improved version', lines, story.end_seconds,
                               nxt('compare'), preview, image))
        rows.append(('end', 'Baseline vs. improved version', '-', '-', story.end_seconds,
                     'Baseline vs. improved version: ' + '; '.join(lines), []))
    if not clips:
        print('No scene left to build.')
        raise SystemExit(1)

    out = concat(clips, folder / f'{"preview" if preview else "final"}_{name}.mp4')
    entries, t = [], 0.0
    for row in rows:
        length, text_, labels = row[4], row[5], row[6]
        if text_:
            entries.append((t, t + length, text_))
        entries += [(t + a, t + b, lab) for a, b, lab in labels]
        t += length
    if stills:
        from swarm_backend.demo.stills import gif, stills_pdf
        pairs_ = [(c, row[5] or row[1]) for c, row in zip(clips, rows)]
        stills_pdf([c for c, _ in pairs_], [t for _, t in pairs_], folder / f'stills_{name}.pdf')
        for c, (idx, cap, *_rest) in zip(clips, rows):
            if 'hit' in (cap or '').lower():
                g, size = gif(c, folder / f'gif_{c.stem}.gif')
                print(f'Wrote {g} ({size / 1e6:.1f} MB)')
    srt = out.with_suffix('.srt')
    srt.write_text(subtitles(entries))
    print(f'{"#":>5}  {"take":>4}  {"sim":>13}  {"s":>5}  caption')
    for idx, cap, tk, sim, length, *_ in rows:
        print(f'{idx:>5}  {tk:>4}  {sim:>13}  {length:5.1f}  {cap}')
    print(f'Wrote {out} ({sum(r[4] for r in rows):.1f} s; one clip per scene in {clip_dir})')
    return out


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('run', type=Path)
    p.add_argument('--story', type=Path, default=REPO / 'recording' / 'stories' / 'default.yaml')
    p.add_argument('--preview', action='store_true', help='960x540, fast: for trying the story')
    p.add_argument('--skip-missing', action='store_true', help='build the scenes that can be built')
    p.add_argument('--take', help='use this take for every scene (best or a number)')
    p.add_argument('--stills', action='store_true', help='also a PDF of stills and GIFs of hit scenes')
    args = p.parse_args(argv)
    build(args.run, args.story, args.preview, args.skip_missing, args.take, stills=args.stills)
    return 0


if __name__ == '__main__':
    sys.exit(main())
