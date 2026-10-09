"""How long a crossing target stays in view if the gimbal stops following it
(Target System, Camera Choice, "Keeping the target in view"), animated in a loop.

Top: the narrow and the wide camera's images; the target starts in the centre and
drifts across at the angular rate omega = v_perp / r (equation eq:angrate). Bottom:
the time until it leaves each view, against range, with the current range marked.

    python report/plots/time_in_view.py
    python report/plots/time_in_view.py --gif
"""
import math

import common
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle

import model
from common import BAD, INK, INK_2, INK_3, SERIES, SURFACE

FRAME_S = 0.05
MAX_LOOP_S = 6.0
RANGES = np.geomspace(5, 1000, 300)


def main():
    args = common.cli(__doc__)
    fig = plt.figure(figsize=(10.5, 8.5))
    ax_n = fig.add_axes((0.07, 0.62, 0.38, 0.3))
    ax_w = fig.add_axes((0.55, 0.62, 0.38, 0.3))
    ax_c = fig.add_axes((0.09, 0.27, 0.86, 0.27))
    sl = {
        'r': common.slider(fig, (0.22, 0.19, 0.55, 0.022), 'Range r (m)', 5, 500, 50, 1),
        'v': common.slider(fig, (0.22, 0.155, 0.55, 0.022), 'Crossing speed (m/s)', 1, 40, 17, 1),
        'n': common.slider(fig, (0.22, 0.12, 0.55, 0.022), 'Narrow FOV (°)', 5, 60, 20, 1),
        'w': common.slider(fig, (0.22, 0.085, 0.55, 0.022), 'Wide FOV (°)', 60, 150, 115, 1),
        'speed': common.slider(fig, (0.22, 0.05, 0.55, 0.022), 'Playback (×)', 0.1, 2.0, 0.5, 0.1),
    }
    st = {'t': 0.0}

    def view(ax, fov, color, label):
        ax.clear()
        theta_v = model.vertical_fov_deg(fov)
        ax.add_patch(Rectangle((-fov / 2, -theta_v / 2), fov, theta_v, fill=True, fc='#f4f7fb',
                               ec=INK_3, lw=1.2))
        ax.plot([0], [0], '+', color=INK_3, markersize=12)
        lim = max(fov, 20) / 2 * 1.25
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim * 0.75, lim * 0.75)
        ax.set_aspect('equal')
        ax.grid(False)
        ax.set_xlabel('degrees')
        dot, = ax.plot([], [], 'o', color=SERIES[1], markersize=10, mec=SURFACE, mew=1.5)
        return {'ax': ax, 'fov': fov, 'dot': dot, 'label': label, 'color': color}

    def reset(_=None):
        r, v = sl['r'].val, sl['v'].val
        st['views'] = [view(ax_n, sl['n'].val, SERIES[0], 'Narrow'),
                       view(ax_w, sl['w'].val, SERIES[2], 'Wide')]
        st['omega'] = math.degrees(model.angular_rate(v, r))       # deg/s
        times = [model.time_in_view(vw['fov'], v, r) for vw in st['views']]
        st['times'] = times
        st['loop'] = min(max(times), MAX_LOOP_S) + 1.0
        st['t'] = 0.0

        ax_c.clear()
        for vw in st['views']:
            ax_c.plot(RANGES, [model.time_in_view(vw['fov'], v, x) for x in RANGES],
                      color=vw['color'], label=f'{vw["label"]} camera, {vw["fov"]:g}°')
        for vw, t in zip(st['views'], times):
            ax_c.plot(r, t, 'o', color=vw['color'], markersize=8, mec=SURFACE, mew=1.5)
        ax_c.axvline(r, color=INK_3, lw=1, ls='--')
        ax_c.set_xscale('log')
        ax_c.set_yscale('log')
        ax_c.set_xlim(RANGES[0], RANGES[-1])
        ax_c.set_xlabel('range to the target (m)')
        ax_c.set_ylabel('time until it leaves the view (s)')
        ax_c.set_title(f'Target crossing at {v:g} m/s: ω = {st["omega"]:.1f}°/s at {r:g} m')
        ax_c.legend(loc='upper left', fontsize=9)
        step(None, advance=False)

    def step(_frame, advance=True):
        if advance:
            st['t'] += FRAME_S * sl['speed'].val
            if st['t'] > st['loop']:
                st['t'] = 0.0
        t = st['t']
        for vw, t_out in zip(st['views'], st['times']):
            x = st['omega'] * t
            lost = x > vw['fov'] / 2
            vw['dot'].set_data([min(x, vw['fov'] / 2 * 1.2)], [0])
            vw['dot'].set_alpha(0.35 if lost else 1.0)
            if lost:
                state = f'lost after {t_out:.2f} s'
            elif t_out > MAX_LOOP_S:
                state = f'in view for {t_out:.1f} s'
            else:
                state = 'in view'
            vw['ax'].set_title(f'{vw["label"]} camera, {vw["fov"]:g}°: {state}   (t = {t:.2f} s)',
                               fontsize=10, color=BAD if lost else INK)
        return ()

    for key, s_ in sl.items():
        if key != 'speed':
            s_.on_changed(reset)
    reset()
    frames = math.ceil(st['loop'] / (FRAME_S * sl['speed'].val)) + 1
    anim = FuncAnimation(fig, step, frames=frames if args.gif else None,
                         interval=FRAME_S * 1000, cache_frame_data=False)
    common.finish(fig, 'time_in_view', args, anim=anim, fps=int(1 / FRAME_S))
    return fig, sl, anim


if __name__ == '__main__':
    main()
