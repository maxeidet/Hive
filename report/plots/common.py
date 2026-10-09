"""Shared look and command-line options for the plots in this folder.

Every plot script accepts:
    --save   write report/figures/<name>.pdf and .png of the start view, no window
    --gif    (animated plots) also write report/figures/<name>.gif, one loop
"""
import argparse
import sys
import warnings
from pathlib import Path

# --save draws only the start view of an animated plot; the animation never runs.
warnings.filterwarnings('ignore', message='Animation was deleted without rendering')

import matplotlib

if '--save' in sys.argv or '--gif' in sys.argv:
    matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.widgets import Slider  # noqa: E402

# Text inks and the reference categorical palette, slots in fixed order.
INK = '#0b0b0b'
INK_2 = '#52514e'
INK_3 = '#8a8985'
GRID = '#e6e5e1'
SURFACE = '#ffffff'
SERIES = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100']
BAD = '#e34948'

FIGURES = Path(__file__).resolve().parent.parent / 'figures'

plt.rcParams.update({
    'font.size': 10,
    'axes.edgecolor': INK_3,
    'axes.labelcolor': INK_2,
    'axes.titlecolor': INK,
    'axes.titlesize': 11,
    'axes.spines.top': False,
    'axes.spines.right': False,
    'axes.grid': True,
    'grid.color': GRID,
    'grid.linewidth': 0.8,
    'xtick.color': INK_2,
    'ytick.color': INK_2,
    'lines.linewidth': 2,
    'legend.frameon': False,
    'legend.labelcolor': INK_2,
    'figure.facecolor': SURFACE,
    'savefig.facecolor': SURFACE,
})


def cli(doc):
    p = argparse.ArgumentParser(description=doc.strip().split('\n')[0])
    p.add_argument('--save', action='store_true', help='save the start view to report/figures')
    p.add_argument('--gif', action='store_true', help='animated plots: save one loop as a GIF')
    return p.parse_args()


def slider(fig, rect, label, vmin, vmax, vinit, step=None):
    return Slider(fig.add_axes(rect), label, vmin, vmax, valinit=vinit, valstep=step,
                  color=SERIES[0], initcolor='none')


def hover_readout(fig, ax, describe, corner=(0.02, 0.98)):
    """Show describe(x) in a corner of ax while the mouse is over it. Call once; the
    readout survives ax.clear()."""
    box = {}

    def text():
        if box.get('text') not in ax.texts:
            box['text'] = ax.text(*corner, '', transform=ax.transAxes, va='top', fontsize=9,
                                  family='monospace', color=INK, zorder=10,
                                  bbox=dict(boxstyle='round,pad=0.4', fc=SURFACE, ec=GRID))
        return box['text']

    def on_move(event):
        t = text()
        if event.inaxes is ax and event.xdata is not None:
            t.set_text(describe(event.xdata))
            t.set_visible(True)
        else:
            t.set_visible(False)
        fig.canvas.draw_idle()

    fig.canvas.mpl_connect('motion_notify_event', on_move)


def finish(fig, name, args, anim=None, frames=None, fps=20):
    """Show the window, or save the start view (and a GIF of one loop)."""
    if args.save or args.gif:
        FIGURES.mkdir(parents=True, exist_ok=True)
        if anim is not None and args.gif:
            anim.save(FIGURES / f'{name}.gif', writer='pillow', fps=fps)
            print(f'Wrote {FIGURES / (name + ".gif")}')
        if args.save:
            for ext in ('pdf', 'png'):
                fig.savefig(FIGURES / f'{name}.{ext}', dpi=200)
            print(f'Wrote {FIGURES / (name + ".pdf")} and .png')
        return
    plt.show()
