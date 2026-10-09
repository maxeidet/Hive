"""Perimeter plots: the protected base, the ring of defenders, the hostile
spawn circle and what each drone's camera covers. Static grid for the report:

    python report/plots/perimeter.py
    python report/plots/perimeter.py --drones 5 10 12 --fov 20 60 115 --radius 1.5

Writes report/figures/perimeter.pdf and .png. For sliders, run
perimeter_interactive.py. Distances are in kilometres: the ring is at r, hostile
drones spawn at r + 1.
"""
import argparse
import math
import sys

import matplotlib

if __name__ == '__main__':
    matplotlib.use('Agg')
import common  # noqa: E402  (style)
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Circle, Patch, Wedge  # noqa: E402

import model  # noqa: E402
from common import INK, INK_2, INK_3, SERIES, SURFACE  # noqa: E402

DRONE, HOSTILE = SERIES[0], SERIES[1]


def draw(ax, s, title=True):
    """Draw one perimeter scene for Scene s on ax (in kilometres)."""
    ax.clear()
    r = s.radius_m / 1000
    spawn = r + 1.0
    r_det = model.detection_range_m(s) / 1000

    ax.add_patch(Circle((0, 0), r, fill=False, color=INK_2, lw=1.2))
    ax.add_patch(Circle((0, 0), spawn, fill=False, color=INK_3, lw=1.2, ls=(0, (4, 3))))

    for i in range(s.n_drones):
        bearing = 360.0 * i / s.n_drones
        x, y = r * math.cos(math.radians(bearing)), r * math.sin(math.radians(bearing))
        ax.add_patch(Wedge((x, y), r_det, bearing - s.sector_deg / 2,
                           bearing + s.sector_deg / 2, color=DRONE, alpha=0.10, lw=0))
        for c in range(s.n_cameras):           # cameras side by side, centred outward
            centre = bearing + (c - (s.n_cameras - 1) / 2) * s.fov_deg
            ax.add_patch(Wedge((x, y), r_det, centre - s.fov_deg / 2, centre + s.fov_deg / 2,
                               color=DRONE, alpha=0.35, lw=0.6, ec=SURFACE))
        ax.plot(x, y, 'o', color=DRONE, markersize=7, mec=SURFACE, mew=1.5, zorder=4)

    mid = math.radians(180.0 / s.n_drones)
    tip = 0.06 * spawn
    ax.annotate('', xy=(tip * math.cos(mid), tip * math.sin(mid)),
                xytext=(spawn * math.cos(mid), spawn * math.sin(mid)),
                arrowprops=dict(arrowstyle='-|>', color=HOSTILE, lw=2, mutation_scale=14),
                zorder=5)
    ax.plot(spawn * math.cos(mid), spawn * math.sin(mid), 'o', color=HOSTILE,
            markersize=8, mec=SURFACE, mew=1.5, zorder=6)

    ax.plot(0, 0, 'o', color=INK, markersize=7, zorder=6)
    ax.text(0, -0.05 * spawn, 'base', ha='center', va='top', fontsize=8, color=INK_2)
    ax.text(0, r - 0.03 * spawn, 'r', ha='center', va='top', fontsize=9, color=INK_2)
    ax.text(0, spawn * 1.02, 'r + 1', ha='center', va='bottom', fontsize=9, color=INK_2)

    if title:
        covered = model.detection_range_m(s) >= model.y_max(s)
        cams = f', {s.n_cameras} cameras' if s.n_cameras > 1 else ''
        ax.set_title(f'N = {s.n_drones}, FOV {s.fov_deg:g}°{cams}, r = {r:g} km\n'
                     f'range {r_det * 1000:.0f} m, {"no gaps" if covered else "gaps"}',
                     fontsize=10)

    lim = spawn * 1.1
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_aspect('equal')
    ax.axis('off')


def legend_handles():
    return [
        Line2D([], [], color=INK_2, lw=1.2, label='perimeter (r)'),
        Line2D([], [], color=INK_3, lw=1.2, ls=(0, (4, 3)), label='hostile spawn (r + 1 km)'),
        Line2D([], [], marker='o', ls='', color=DRONE, markersize=8, label='defender'),
        Patch(color=DRONE, alpha=0.35, label='camera view (FOV)'),
        Patch(color=DRONE, alpha=0.10, label='scanned sector'),
        Line2D([], [], color=HOSTILE, lw=2, marker='>', markersize=6,
               label='hostile path (worst case)'),
    ]


def main():
    p = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    p.add_argument('--drones', type=int, nargs='+', default=[5, 10, 12])
    p.add_argument('--fov', type=float, nargs='+', default=[20, 115])
    p.add_argument('--radius', type=float, default=1.0, help='perimeter radius r, km')
    p.add_argument('--width', type=int, default=1920, help='image width, pixels')
    p.add_argument('--pmin', type=float, default=4, help='smallest detectable size, pixels')
    args = p.parse_args()
    base = model.Scene(radius_m=args.radius * 1000, width_px=args.width, p_min_px=args.pmin)

    rows, cols = len(args.drones), len(args.fov)
    fig, axes = plt.subplots(rows, cols, figsize=(max(3.4 * cols, 5.0), 3.6 * rows + 0.6),
                             squeeze=False)
    for i, n in enumerate(args.drones):
        for j, fov in enumerate(args.fov):
            draw(axes[i][j], base.with_(n_drones=n, fov_deg=fov))

    handles = legend_handles()
    ncol = 3 if cols > 1 else 2
    fig.legend(handles=handles, loc='lower center', ncol=ncol, fontsize=9)
    legend_height = 0.25 * math.ceil(len(handles) / ncol) + 0.2
    fig.tight_layout(rect=(0, legend_height / fig.get_figheight(), 1, 1))

    common.FIGURES.mkdir(parents=True, exist_ok=True)
    for ext in ('pdf', 'png'):
        fig.savefig(common.FIGURES / f'perimeter.{ext}', dpi=200)
    print(f'Wrote {common.FIGURES / "perimeter.pdf"} and .png')


if __name__ == '__main__':
    sys.exit(main())
