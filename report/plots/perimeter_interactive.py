"""Interactive perimeter plot: sliders for the field of view, the number of cameras per
drone, the number of drones and the perimeter radius, with the coverage and the
worst-case intercept margin.

Several cameras sit side by side on each drone's gimbal (the camera-array option in
Camera Choice): one gimbal position then covers cameras x FOV, so fewer positions are
scanned. When the cameras cover the whole outward sector, nothing is scanned and the
target is seen at once.

    python report/plots/perimeter_interactive.py
    python report/plots/perimeter_interactive.py --save
"""
import common
import matplotlib.pyplot as plt

import model
from common import BAD, INK, INK_2
from perimeter import draw, legend_handles


def summary(s):
    r_det = model.detection_range_m(s)
    y = model.y_max(s)
    margin = model.margin_s(s)
    n_feasible = model.n_feasible(s)
    positions = model.scan_positions(s)
    rows = [
        ('Detection range', f'{r_det:.0f} m'),
        ('Spacing d', f'{2 * y:.0f} m'),
        ('Worst gap y_max', f'{y:.0f} m'),
        None,
        ('Cameras cover', f'{min(s.n_cameras * s.fov_deg, 360):.0f}°'),
        ('Scan positions', 'none' if positions == 1 else str(positions)),
        ('Time to depart t_d', f'{model.t_depart(s):.1f} s'),
        ('Pixels per frame', f'{s.n_cameras}× one camera'),
        None,
        ('Coverage', 'no gaps' if r_det >= y else 'GAPS'),
        ('Intercept margin', 'unseen' if margin is None else f'{margin:+.1f} s'),
        None,
        ('N for no gaps', str(model.n_no_gaps(s))),
        ('N for intercept', str(n_feasible) if n_feasible else '> 500'),
    ]
    text = '\n'.join('' if row is None else f'{row[0]:<19}{row[1]:>12}' for row in rows)
    return text, margin is not None and margin >= 0


def main():
    args = common.cli(__doc__)
    fig = plt.figure(figsize=(11, 8))
    ax = fig.add_axes((0.02, 0.27, 0.62, 0.65))
    verdict = fig.text(0.67, 0.9, '', fontsize=13, weight='bold', va='top')
    info = fig.text(0.67, 0.84, '', family='monospace', fontsize=10, color=INK_2, va='top')
    fig.legend(handles=legend_handles(), loc='upper left', bbox_to_anchor=(0.66, 0.36),
               fontsize=9)

    sliders = {
        'fov': common.slider(fig, (0.18, 0.18, 0.42, 0.025), 'FOV per camera (°)', 5, 120, 20, 1),
        'cams': common.slider(fig, (0.18, 0.13, 0.42, 0.025), 'Cameras per drone', 1, 12, 1, 1),
        'n': common.slider(fig, (0.18, 0.08, 0.42, 0.025), 'Drones N', 2, 40, 12, 1),
        'r': common.slider(fig, (0.18, 0.03, 0.42, 0.025), 'Radius r (km)', 0.2, 3.0, 1.0, 0.05),
    }

    def update(_=None):
        s = model.Scene(fov_deg=sliders['fov'].val, n_cameras=int(sliders['cams'].val),
                        n_drones=int(sliders['n'].val), radius_m=sliders['r'].val * 1000)
        draw(ax, s)
        text, ok = summary(s)
        info.set_text(text)
        verdict.set_text('Intercept feasible' if ok else 'Intercept NOT feasible')
        verdict.set_color(INK if ok else BAD)
        fig.canvas.draw_idle()

    for sl in sliders.values():
        sl.on_changed(update)
    update()
    common.finish(fig, 'perimeter_interactive', args)
    return fig, sliders


if __name__ == '__main__':
    main()
