"""Which assumption matters most? (Target System, Additional Assumptions.)

Tornado chart: each assumption is lowered (blue) and raised (orange) by the slider's
percentage while all others keep their base value, and the bars show the number of
drones needed. The longest bars are the assumptions worth checking first.

    python report/plots/sensitivity.py
    python report/plots/sensitivity.py --save
"""
import common
import matplotlib.pyplot as plt
from matplotlib.widgets import RadioButtons

import model
from common import INK, INK_2, INK_3, SERIES

CAP = 500
PARAMS = [
    ('radius_m', 'perimeter radius r'),
    ('fov_deg', 'field of view'),
    ('width_px', 'image width'),
    ('size_m', 'hostile drone size s'),
    ('p_min_px', 'smallest detectable size p_min'),
    ('v_h', 'hostile speed v_h'),
    ('v_i', 'interceptor speed v_i'),
    ('accel', 'interceptor acceleration a'),
    ('t_g', 'gimbal turn time t_g'),
    ('sector_deg', 'scanned sector'),
]


def needed(s, metric):
    n = model.n_feasible(s, CAP) if metric == 'intercept' else model.n_no_gaps(s)
    return n if n else CAP


def vary(base, attr, factor):
    value = getattr(base, attr) * factor
    if attr == 'width_px':
        value = int(round(value))
    if attr == 'sector_deg':
        value = min(value, 360.0)
    return base.with_(**{attr: value})


def main():
    args = common.cli(__doc__)
    fig = plt.figure(figsize=(10.5, 7))
    ax = fig.add_axes((0.27, 0.22, 0.52, 0.68))
    pct = common.slider(fig, (0.27, 0.08, 0.5, 0.03), 'Change (±%)', 5, 50, 25, 5)
    radio_ax = fig.add_axes((0.83, 0.6, 0.15, 0.15))
    radio_ax.set_title('drones needed for', fontsize=9, color=INK_2)
    radio = RadioButtons(radio_ax, ['intercept', 'no gaps'], activecolor=SERIES[0])

    def update(_=None):
        p = pct.val / 100
        metric = radio.value_selected
        base = model.Scene()
        n0 = needed(base, metric)
        rows = []
        for attr, label in PARAMS:
            lo = needed(vary(base, attr, 1 - p), metric)
            hi = needed(vary(base, attr, 1 + p), metric)
            rows.append((max(abs(lo - n0), abs(hi - n0)), label, lo, hi, getattr(base, attr)))
        rows.sort()

        ax.clear()
        for y, (_, label, lo, hi, value) in enumerate(rows):
            for n, color in ((lo, SERIES[0]), (hi, SERIES[1])):
                ax.barh(y, n - n0, left=n0, height=0.6, color=color, alpha=0.9)
                if n != n0:
                    text = '>500' if n >= CAP else str(n)
                    ax.text(n + (0.3 if n > n0 else -0.3), y, text, va='center',
                            ha='left' if n > n0 else 'right', fontsize=8, color=INK_2)
        ax.set_yticks(range(len(rows)))
        ax.set_yticklabels([f'{label} ({value:g})' for _, label, _, _, value in rows], fontsize=9)
        ax.axvline(n0, color=INK, lw=1.2)
        ax.grid(axis='y', visible=False)
        ax.set_xlabel(f'drones needed for {"a feasible intercept" if metric == "intercept" else "no gaps"}')
        ax.set_title(f'Base: N = {n0}. Each assumption changed by ±{pct.val:g} %, '
                     'the others kept', fontsize=10)
        lo_patch = plt.Rectangle((0, 0), 1, 1, color=SERIES[0])
        hi_patch = plt.Rectangle((0, 0), 1, 1, color=SERIES[1])
        ax.legend([lo_patch, hi_patch], [f'assumption −{pct.val:g} %', f'assumption +{pct.val:g} %'],
                  loc='lower right', fontsize=9)
        left = min(min(r[2], r[3]) for r in rows)
        right = min(max(max(r[2], r[3]) for r in rows), CAP)
        span = max(right - left, 4)
        ax.set_xlim(max(0, left - 0.15 * span), right + 0.15 * span)
        fig.canvas.draw_idle()

    pct.on_changed(update)
    radio.on_clicked(update)
    update()
    common.finish(fig, 'sensitivity', args)
    return fig, pct, radio


if __name__ == '__main__':
    main()
