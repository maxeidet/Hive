"""Triangulation baseline (Base Station): how accurate is the target's position from
two drones' bearings, and which second drone should the base station send?

Left: top view. The interceptor (drone 0) and a partner drone k positions away both
measure the bearing to the target with noise sigma; the cloud shows the triangulated
positions (crossing point of the two bearing lines) over many trials. Right: the
position error against which partner is used. Two dimensions only (top view).

    python report/plots/triangulation.py
    python report/plots/triangulation.py --save
"""
import math

import common
import matplotlib.pyplot as plt
import numpy as np

from common import INK, INK_2, INK_3, SERIES, SURFACE

TRIALS = 600


def ring_pos(radius, n, i):
    a = 2 * math.pi * i / n
    return np.array([radius * math.cos(a), radius * math.sin(a)])


def estimates(p_a, p_b, target, sigma_rad, rng, trials=TRIALS):
    """Crossing points of two noisy bearing lines."""
    def noisy_dirs(p):
        true = math.atan2(*(target - p)[::-1])
        ang = true + rng.normal(0, sigma_rad, trials)
        return np.stack([np.cos(ang), np.sin(ang)], axis=1)
    u_a, u_b = noisy_dirs(p_a), noisy_dirs(p_b)
    det = u_a[:, 0] * (-u_b[:, 1]) - u_a[:, 1] * (-u_b[:, 0])
    rhs = p_b - p_a
    s = (rhs[0] * (-u_b[:, 1]) - rhs[1] * (-u_b[:, 0])) / np.where(np.abs(det) < 1e-9, np.nan, det)
    pts = p_a + s[:, None] * u_a
    return pts[np.isfinite(pts).all(axis=1) & (s > 0)]


def rms_error(pts, target):
    return float(np.sqrt(np.mean(np.sum((pts - target) ** 2, axis=1)))) if len(pts) else math.nan


def main():
    args = common.cli(__doc__)
    fig = plt.figure(figsize=(11.5, 7.5))
    ax = fig.add_axes((0.04, 0.3, 0.5, 0.62))
    ax_e = fig.add_axes((0.62, 0.36, 0.35, 0.52))
    sl = {
        'N': common.slider(fig, (0.2, 0.2, 0.55, 0.025), 'Drones N', 4, 30, 12, 1),
        'k': common.slider(fig, (0.2, 0.155, 0.55, 0.025), 'Partner (positions away)', 1, 15, 1, 1),
        'D': common.slider(fig, (0.2, 0.11, 0.55, 0.025), 'Target outside ring (m)', 50, 1000, 400, 10),
        'sig': common.slider(fig, (0.2, 0.065, 0.55, 0.025), 'Bearing error σ (°)', 0.05, 3.0, 0.5, 0.05),
        'R': common.slider(fig, (0.2, 0.02, 0.55, 0.025), 'Radius r (km)', 0.3, 3.0, 1.0, 0.05),
    }

    def update(_=None):
        n, d, radius = int(sl['N'].val), sl['D'].val, sl['R'].val * 1000
        k = int(min(sl['k'].val, n // 2))
        sigma = math.radians(sl['sig'].val)
        rng = np.random.default_rng(0)
        mid = math.pi / n                      # target midway between drones 0 and 1
        target = np.array([(radius + d) * math.cos(mid), (radius + d) * math.sin(mid)])
        p_a, p_b = ring_pos(radius, n, 0), ring_pos(radius, n, k)
        pts = estimates(p_a, p_b, target, sigma, rng)
        err = rms_error(pts, target)
        ang = math.degrees(abs(math.atan2(*(target - p_a)[::-1]) - math.atan2(*(target - p_b)[::-1])))
        ang = min(ang, 360 - ang)

        ax.clear()
        theta = np.linspace(0, 2 * math.pi, 400)
        ax.plot(radius * np.cos(theta), radius * np.sin(theta), color=INK_2, lw=1.2)
        for i in range(n):
            p = ring_pos(radius, n, i)
            ax.plot(*p, 'o', color=INK_3, markersize=6, mec=SURFACE)
        ax.scatter(pts[:, 0], pts[:, 1], s=6, color=SERIES[1], alpha=0.35, lw=0,
                   label=f'triangulated, {len(pts)} trials')
        for p, label, color in ((p_a, 'interceptor (0)', SERIES[0]), (p_b, f'partner ({k})', SERIES[2])):
            ax.plot([p[0], target[0]], [p[1], target[1]], color=color, lw=1)
            ax.plot(*p, 'o', color=color, markersize=10, mec=SURFACE, mew=1.5, label=label, zorder=5)
        ax.plot(*target, 'X', color=INK, markersize=10, zorder=6, label='true target')
        box = np.array([p_a, p_b, target])
        lo, hi = box.min(axis=0), box.max(axis=0)
        pad = 0.25 * max(hi - lo) + 50
        ax.set_xlim(lo[0] - pad, hi[0] + pad)
        ax.set_ylim(lo[1] - pad, hi[1] + pad)
        ax.set_aspect('equal')
        ax.set_xlabel('m')
        ax.set_title(f'Angle between bearings {ang:.0f}°, RMS error {err:.0f} m', fontsize=11)
        ax.legend(loc='lower left', fontsize=8)

        ax_e.clear()
        ks = np.arange(1, n // 2 + 1)
        errs = [rms_error(estimates(p_a, ring_pos(radius, n, kk), target, sigma,
                                    np.random.default_rng(0), 300), target) for kk in ks]
        ax_e.plot(ks, errs, color=SERIES[0], marker='o', markersize=5)
        ax_e.plot(k, errs[k - 1], 'o', color=SERIES[2], markersize=11, mec=SURFACE, mew=1.5)
        ax_e.set_yscale('log')
        ax_e.set_xlabel('partner drone, positions away from the interceptor')
        ax_e.set_ylabel('RMS position error (m)')
        ax_e.set_title(f'σ = {sl["sig"].val:g}°, target {d:g} m outside the ring', fontsize=10)
        best = int(ks[int(np.nanargmin(errs))])
        ax_e.annotate(f'best: {best}', (best, errs[best - 1]), (10, 10), textcoords='offset points',
                      ha='left', fontsize=9, color=INK)
        fig.canvas.draw_idle()

    for s_ in sl.values():
        s_.on_changed(update)
    update()
    common.finish(fig, 'triangulation', args)
    return fig, sl


if __name__ == '__main__':
    main()
