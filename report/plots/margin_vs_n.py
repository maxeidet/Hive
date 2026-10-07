"""Worst-case intercept margin against the number of drones (Target System, Camera
Choice example). The second line, with two drones away on an intercept, is extra:
the report does not cover the gaps it leaves.

Blue: the full ring of N drones. Orange: the same ring while two drones are away on
an intercept (N - 2 left). The intercept is feasible above zero. Hover to read the
values; the radio buttons pick the image width.

    python report/plots/margin_vs_n.py
    python report/plots/margin_vs_n.py --save
"""
import common
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import RadioButtons

import model
from common import INK, INK_2, INK_3, SERIES

NS = np.arange(3, 41)


def margins(s, away=0):
    vals = []
    for n in NS:
        m = model.margin_s(s, n - away) if n - away >= 2 else None
        vals.append(np.nan if m is None else m)
    return np.array(vals)


def first_feasible(ms):
    ok = np.where(ms >= 0)[0]
    return int(NS[ok[0]]) if len(ok) else None


def main():
    args = common.cli(__doc__)
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_axes((0.09, 0.32, 0.68, 0.6))
    sl = {
        'fov': common.slider(fig, (0.2, 0.19, 0.55, 0.03), 'FOV (°)', 5, 120, 20, 1),
        'R': common.slider(fig, (0.2, 0.14, 0.55, 0.03), 'Radius r (km)', 0.3, 3.0, 1.0, 0.05),
        'vh': common.slider(fig, (0.2, 0.09, 0.55, 0.03), 'Hostile speed (m/s)', 4, 30, 17, 1),
        'p': common.slider(fig, (0.2, 0.04, 0.55, 0.03), 'p_min (px)', 2, 20, 4, 1),
    }
    radio_ax = fig.add_axes((0.81, 0.55, 0.16, 0.18))
    radio_ax.set_title('image width', fontsize=9, color=INK_2)
    radio = RadioButtons(radio_ax, ['1280 px', '1920 px', '3840 px'], active=1,
                         activecolor=SERIES[0])
    st = {}

    def scene():
        width = int(radio.value_selected.split()[0])
        return model.Scene(fov_deg=sl['fov'].val, radius_m=sl['R'].val * 1000,
                           v_h=sl['vh'].val, p_min_px=sl['p'].val, width_px=width)

    def update(_=None):
        s = scene()
        full, away = margins(s), margins(s, away=2)
        st.update(full=full, away=away)
        ax.clear()
        ax.axhline(0, color=INK_3, lw=1)
        ax.plot(NS, full, color=SERIES[0], marker='o', markersize=4, label='full ring (N)')
        ax.plot(NS, away, color=SERIES[1], marker='o', markersize=4,
                label='two drones away (N − 2)')
        for ms, offset, ha in ((full, (-8, 10), 'right'), (away, (8, -18), 'left')):
            n = first_feasible(ms)
            if n:
                ax.annotate(f'N = {n}', (n, ms[n - NS[0]]), offset, textcoords='offset points',
                            fontsize=9, color=INK, ha=ha)
        ax.set_xlim(NS[0], NS[-1])
        ax.set_xlabel('drones on the ring, N')
        ax.set_ylabel('worst-case margin t_h − t_i (s)')
        ax.set_title(f'Intercept margin, FOV {s.fov_deg:g}° at {s.width_px} px, '
                     f'r = {s.radius_m / 1000:g} km (missing points: hostile passes unseen)')
        ax.legend(loc='lower right', fontsize=9)
        fig.canvas.draw_idle()

    def describe(x):
        i = int(np.clip(round(x - NS[0]), 0, len(NS) - 1))

        def f(v):
            return 'unseen' if np.isnan(v) else f'{v:+.1f} s'
        return f'N = {NS[i]}\nfull ring   {f(st["full"][i]):>8}\nN − 2       {f(st["away"][i]):>8}'

    for s_ in sl.values():
        s_.on_changed(update)
    radio.on_clicked(update)
    update()
    common.hover_readout(fig, ax, describe)
    common.finish(fig, 'margin_vs_n', args)
    return fig, sl, radio


if __name__ == '__main__':
    main()
