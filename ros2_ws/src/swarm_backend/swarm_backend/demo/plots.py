"""Graphs on sim time as a video source (story `show: [chase, "plot:range"]`).

The whole take's curve is drawn faint once (matplotlib), and per frame the part up to the
current sim time bold with a dot and its value (OpenCV), so a minute of graph takes seconds.
The take's first sighting and hit are vertical ticks. A new graph is one entry in PLOTS.
"""
import math
from pathlib import Path

import numpy as np

from swarm_backend.demo.takes import Take, TakeData, est_error_samples, in_graveyard, graveyard, take_data

STEP_S = 0.2
BG = '#0a1113'
COLOUR = '#5cc8ff'


def _samples(d: TakeData, f):
    m = d.take.moments
    t, out = m['start'], []
    grave = graveyard()
    while t <= m['end']:
        h = d.pos(d.take.target, t)
        if h is not None and not in_graveyard(h, grave):
            v = f(t, h)
            if v is not None:
                out.append((t, v))
        t += STEP_S
    return out


def _range(d: TakeData):
    name = d.attacker_name()
    def f(t, h):
        a = d.pos(name, t) if name else None
        return None if a is None else math.dist(a, h)
    return _samples(d, f)


def _base_distance(d: TakeData):
    surv = (d.ui.lines[0]['state'].get('search_area') or {}) if d.ui else {}
    bn, be = 0.0, 0.0
    centre = surv.get('centre') if isinstance(surv, dict) else None
    if centre:
        bn, be = centre[:2]
    return _samples(d, lambda t, h: math.hypot(h[0] - be, h[1] - bn))


PLOTS = {
    'range': (_range, 'Attacker to target', 'm'),
    'base_distance': (_base_distance, 'Target to base', 'm'),
    'est_error': (est_error_samples, 'Position error of the estimate', 'm'),
}


def plot_data(folder: Path, take: Take, name: str) -> tuple[list[float], list[float], str, str]:
    f, title, unit = PLOTS[name]
    pts = f(take_data(folder, take))
    return [p[0] for p in pts], [p[1] for p in pts], title, unit


def write_plot(folder: Path, take: Take, name: str, start: float, end: float, out: Path,
               size: tuple[int, int] = (960, 540), dots: list | None = None, fps: float = 25.0) -> Path:
    import subprocess

    import cv2
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    sims, values, title, unit = plot_data(folder, take, name)
    w, h = size
    fig = plt.figure(figsize=(w / 100, h / 100), dpi=100, facecolor=BG)
    ax = fig.add_axes([0.1, 0.14, 0.86, 0.74], facecolor=BG)
    ax.plot(sims, values, color=COLOUR, alpha=0.25, lw=1.5)
    ax.set_xlim(start, end)
    if values:
        lo, hi = min(values), max(values)
        ax.set_ylim(max(0.0, lo - 0.05 * (hi - lo + 1)), hi + 0.05 * (hi - lo + 1))
    for m, colour in (('first_sighting', '#5fd49a'), ('hit', '#ff5d5d')):
        at = take.moments.get(m)
        if at is not None and start <= at <= end:
            ax.axvline(at, color=colour, lw=1, alpha=0.8)
    ax.set_title(f'{title} [{unit}]', color='white', fontsize=max(8, h // 40))
    ax.tick_params(colors='#aaaaaa', labelsize=max(6, h // 60))
    ax.set_xlabel('sim time [s]', color='#aaaaaa', fontsize=max(6, h // 60))
    for spine in ax.spines.values():
        spine.set_color('#444444')
    fig.canvas.draw()
    bg = cv2.cvtColor(np.asarray(fig.canvas.buffer_rgba())[:, :, :3], cv2.COLOR_RGB2BGR).copy()
    to_px = ax.transData.transform
    plt.close(fig)
    pts = [to_px((s, v)) for s, v in zip(sims, values)]
    pts = [(int(x), int(h - y)) for x, y in pts]

    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    enc = subprocess.Popen(['ffmpeg', '-v', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'bgr24',
                            '-s', f'{w}x{h}', '-r', f'{fps:g}', '-i', '-', '-c:v', 'libx264',
                            '-preset', 'medium', '-crf', '18', '-pix_fmt', 'yuv420p', str(out)],
                           stdin=subprocess.PIPE)
    try:
        n = int(round((end - start) * fps))
        for i in range(n):
            t = start + i / fps
            img = bg.copy()
            k = int(np.searchsorted(sims, t, side='right'))
            if k > 1:
                cv2.polylines(img, [np.array(pts[:k], np.int32)], False, (255, 200, 92), max(2, h // 200), cv2.LINE_AA)
            if k > 0:
                x, y = pts[k - 1]
                cv2.circle(img, (x, y), max(4, h // 90), (255, 255, 255), -1, cv2.LINE_AA)
                cv2.putText(img, f'{values[k - 1]:.1f} {unit}', (x + 8, y - 8), cv2.FONT_HERSHEY_SIMPLEX,
                            h / 900, (255, 255, 255), max(1, h // 500), cv2.LINE_AA)
                if dots is not None:
                    dots.append((x, y))
            elif dots is not None:
                dots.append((0, 0))
            enc.stdin.write(img.tobytes())
    finally:
        enc.stdin.close()
        enc.wait()
    return out
