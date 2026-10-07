"""Pixels on target against range (Target System, Detection range).

One line per camera; the dots mark each camera's detection range r_det, where the
target shrinks below p_min. The dotted line is Johnson's detection criterion
(about two pixels). Sliders change p_min and the hostile drone's size.

    python report/plots/pixels_vs_range.py
    python report/plots/pixels_vs_range.py --save
"""
import common
import matplotlib.pyplot as plt
import numpy as np

import model
from common import INK_2, INK_3, SERIES

CAMERAS = [
    (115, 1280, 'platform: 115°, 1280 px'),
    (60, 1920, '60°, 1920 px'),
    (20, 1920, '20°, 1920 px'),
    (20, 3840, '20°, 3840 px'),
]
RANGES = np.geomspace(10, 5000, 400)
JOHNSON_PX = 2.0


def main():
    args = common.cli(__doc__)
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_axes((0.09, 0.25, 0.86, 0.67))
    sl = {
        'p': common.slider(fig, (0.2, 0.1, 0.6, 0.03), 'p_min (px)', 1, 20, 4, 0.5),
        's': common.slider(fig, (0.2, 0.05, 0.6, 0.03), 'Drone size s (m)', 0.2, 2.0, 0.5, 0.05),
    }

    def scenes():
        return [model.Scene(fov_deg=fov, width_px=w, p_min_px=sl['p'].val, size_m=sl['s'].val)
                for fov, w, _ in CAMERAS]

    def update(_=None):
        ax.clear()
        p_min = sl['p'].val
        for (fov, w, label), s, color in zip(CAMERAS, scenes(), SERIES):
            ax.plot(RANGES, model.pixels_on_target(s, RANGES), color=color, label=label)
            r = model.detection_range_m(s)
            ax.plot(r, p_min, 'o', color=color, markersize=8, mec='white', mew=1.5, zorder=5)
            ax.annotate(f'{r:.0f} m', (r, p_min), (4, 6), textcoords='offset points',
                        fontsize=9, color=INK_2)
        ax.axhline(p_min, color=INK_2, lw=1.2)
        ax.text(RANGES[0] * 1.1, p_min * 1.08, f'p_min = {p_min:g} px', fontsize=9, color=INK_2)
        ax.axhline(JOHNSON_PX, color=INK_3, lw=1.2, ls=':')
        ax.text(RANGES[0] * 1.1, JOHNSON_PX * 0.8, 'Johnson: detection ≈ 2 px', fontsize=9,
                color=INK_2, va='top')
        ax.set_xscale('log')
        ax.set_yscale('log')
        ax.set_xlim(RANGES[0], RANGES[-1])
        ax.set_ylim(0.5, 2000)
        ax.set_xlabel('range to the hostile drone (m)')
        ax.set_ylabel(f'pixels across a {sl["s"].val:g} m drone')
        ax.set_title('Pixels on target; dots mark the detection range r_det')
        ax.legend(loc='upper right', fontsize=9)
        fig.canvas.draw_idle()

    def describe(x):
        lines = [f'range {x:,.0f} m']
        for (_, _, label), s in zip(CAMERAS, scenes()):
            lines.append(f'{label:<25}{model.pixels_on_target(s, x):8.1f} px')
        return '\n'.join(lines)

    for s_ in sl.values():
        s_.on_changed(update)
    update()
    common.hover_readout(fig, ax, describe)
    common.finish(fig, 'pixels_vs_range', args)
    return fig, sl


if __name__ == '__main__':
    main()
