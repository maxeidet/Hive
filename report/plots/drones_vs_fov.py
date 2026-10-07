"""Drones needed against the camera's field of view (Target System, Camera Choice).

Solid lines: drones needed for a feasible intercept. Dashed: drones needed for
coverage without gaps. One colour per image width. Sliders change the perimeter
radius, the smallest detectable size, the hostile speed and the gimbal turn time;
hover over the plot to read the values.

    python report/plots/drones_vs_fov.py
    python report/plots/drones_vs_fov.py --save
"""
import math

import common
import matplotlib.pyplot as plt
import numpy as np

import model
from common import INK, INK_2, SERIES

WIDTHS = [(1280, '1280 px'), (1920, '1920 px'), (3840, '3840 px (4K)')]
FOVS = np.arange(5, 121, 1.0)
CAP = 500


def curves(base):
    out = []
    for width, _ in WIDTHS:
        feas, gaps = [], []
        for fov in FOVS:
            s = base.with_(fov_deg=float(fov), width_px=width)
            n = model.n_feasible(s, CAP)
            feas.append(n if n else np.nan)
            gaps.append(model.n_no_gaps(s))
        out.append((np.array(feas, float), np.array(gaps, float)))
    return out


def main():
    args = common.cli(__doc__)
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_axes((0.09, 0.32, 0.86, 0.6))
    sl = {
        'R': common.slider(fig, (0.2, 0.19, 0.6, 0.03), 'Radius r (km)', 0.3, 3.0, 1.0, 0.05),
        'p': common.slider(fig, (0.2, 0.14, 0.6, 0.03), 'p_min (px)', 2, 20, 4, 1),
        'vh': common.slider(fig, (0.2, 0.09, 0.6, 0.03), 'Hostile speed (m/s)', 4, 30, 17, 1),
        'tg': common.slider(fig, (0.2, 0.04, 0.6, 0.03), 'Gimbal turn t_g (s)', 0.1, 2.0, 0.5, 0.1),
    }
    st = {}

    def update(_=None):
        base = model.Scene(radius_m=sl['R'].val * 1000, p_min_px=sl['p'].val,
                           v_h=sl['vh'].val, t_g=sl['tg'].val)
        data = curves(base)
        st['data'] = data
        ax.clear()
        for (width, label), (feas, gaps), color in zip(WIDTHS, data, SERIES):
            ax.plot(FOVS, feas, color=color, label=f'{label}: feasible intercept')
            ax.plot(FOVS, gaps, color=color, ls='--', lw=1.5, label=f'{label}: no gaps')
        # The simulated platform's camera: 115 degrees at 1280 px.
        n = model.n_feasible(base.with_(fov_deg=115, width_px=1280), CAP)
        ax.plot(115, n if n else CAP, marker='v' if not n else 'o', color=INK, markersize=9,
                ls='', zorder=5)
        ax.annotate('platform camera' + ('' if n else ': not feasible'),
                    (115, n if n else CAP), (-8, -14), textcoords='offset points',
                    ha='right', fontsize=9, color=INK_2)
        ax.set_yscale('log')
        ax.set_ylim(1, CAP)
        ax.set_xlim(FOVS[0], FOVS[-1])
        ax.set_xlabel('field of view (°)')
        ax.set_ylabel('drones needed on the ring')
        ax.set_title(f'Drones needed, r = {sl["R"].val:g} km, p_min = {sl["p"].val:g} px, '
                     f'v_h = {sl["vh"].val:g} m/s')
        ax.legend(loc='upper left', fontsize=8, ncol=1)
        ax.text(0.99, 0.02, 'steps: the number of scan positions ⌈180°/FOV⌉ drops by one',
                transform=ax.transAxes, ha='right', fontsize=8, color=INK_2)
        fig.canvas.draw_idle()

    def describe(x):
        i = int(np.clip(round(x - FOVS[0]), 0, len(FOVS) - 1))
        lines = [f'FOV {FOVS[i]:.0f}°     feasible   no gaps']
        for (_, label), (feas, gaps) in zip(WIDTHS, st['data']):
            f = '>500' if math.isnan(feas[i]) else f'{feas[i]:.0f}'
            lines.append(f'{label:<13}{f:>7}{gaps[i]:>9.0f}')
        return '\n'.join(lines)

    for s_ in sl.values():
        s_.on_changed(update)
    update()
    common.hover_readout(fig, ax, describe)
    common.finish(fig, 'drones_vs_fov', args)
    return fig, sl


if __name__ == '__main__':
    main()
