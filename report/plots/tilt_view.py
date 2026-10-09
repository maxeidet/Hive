"""Why every camera sits on the gimbal (Target System, "Rejected: a wide camera fixed
to the drone"), animated in a loop.

Left: side view of a drone pitching forward and back as it accelerates and brakes.
The filled cone is a camera fixed to the drone, the dashed one a camera on the
gimbal, which stays level. The fixed camera loses a target at elevation epsilon
when epsilon + pitch > theta_v / 2 (equation eq:tiltview).
Right: the elevations each camera sees against the pitch.

    python report/plots/tilt_view.py
    python report/plots/tilt_view.py --gif
"""
import math

import common
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon, Wedge

import model
from common import BAD, INK, INK_2, INK_3, SERIES, SURFACE

FRAME_S = 0.05
PERIOD_S = 6.0
CONE_R = 3.0
TARGET_R = 2.4


def main():
    args = common.cli(__doc__)
    fig = plt.figure(figsize=(11, 7))
    ax = fig.add_axes((0.03, 0.27, 0.45, 0.65))
    ax_p = fig.add_axes((0.57, 0.3, 0.4, 0.6))
    sl = {
        'fov': common.slider(fig, (0.22, 0.17, 0.55, 0.025), 'Camera FOV (°)', 20, 150, 115, 1),
        'eps': common.slider(fig, (0.22, 0.12, 0.55, 0.025), 'Target elevation (°)', -10, 60, 20, 1),
        'pitch': common.slider(fig, (0.22, 0.07, 0.55, 0.025), 'Max pitch (°)', 5, 45, 30, 1),
        'a': common.slider(fig, (0.22, 0.02, 0.55, 0.025), 'Acceleration a (m/s²)', 0.5, 8, 2, 0.5),
    }
    st = {'t': 0.0}

    def reset(_=None):
        theta_v = model.vertical_fov_deg(sl['fov'].val)
        eps, p_max = sl['eps'].val, sl['pitch'].val
        st.update(theta_v=theta_v, eps=eps, p_max=p_max)

        ax.clear()
        ax.set_xlim(-1.2, 3.4)
        ax.set_ylim(-2.3, 2.3)
        ax.set_aspect('equal')
        ax.axis('off')
        ax.axhline(0, color=INK_3, lw=1, ls=':')
        ax.text(3.35, 0.05, 'horizon', ha='right', fontsize=8, color=INK_2)
        st['fixed'] = ax.add_patch(Wedge((0, 0), CONE_R, 0, 1, color=SERIES[0], alpha=0.25, lw=0))
        g = ax.add_patch(Wedge((0, 0), CONE_R, -theta_v / 2, theta_v / 2, fill=False,
                               ec=SERIES[2], lw=1.5, ls='--'))
        g.set_zorder(3)
        st['body'] = ax.add_patch(Polygon(np.zeros((4, 2)), closed=True, color=INK, zorder=5))
        tx, ty = TARGET_R * math.cos(math.radians(eps)), TARGET_R * math.sin(math.radians(eps))
        st['target'], = ax.plot([tx], [ty], 'o', color=SERIES[1], markersize=11, mec=SURFACE,
                                mew=1.5, zorder=6)
        st['status'] = ax.text(-1.15, 2.2, '', va='top', fontsize=10, family='monospace')

        ax_p.clear()
        pitches = np.linspace(-45, 45, 200)
        ax_p.fill_between(pitches, -theta_v / 2 - pitches, theta_v / 2 - pitches,
                          color=SERIES[0], alpha=0.2, lw=0, label='fixed camera sees')
        ax_p.axhline(theta_v / 2, color=SERIES[2], lw=1.5, ls='--', label='gimbal camera sees')
        ax_p.axhline(-theta_v / 2, color=SERIES[2], lw=1.5, ls='--')
        ax_p.axhline(eps, color=SERIES[1], lw=2, label=f'target at {eps:g}°')
        tilt = model.tilt_deg(sl['a'].val)
        ax_p.axvline(tilt, color=INK_3, lw=1, ls=':')
        ax_p.text(tilt + 1, -theta_v / 2 - 40, f'a = {sl["a"].val:g} m/s²\n→ {tilt:.1f}°',
                  fontsize=8, color=INK_2)
        st['marker'] = ax_p.axvline(0, color=INK, lw=1.5)
        ax_p.set_xlim(-45, 45)
        ax_p.set_ylim(-theta_v / 2 - 50, theta_v / 2 + 50)
        ax_p.set_xlabel('pitch, nose down positive (°)')
        ax_p.set_ylabel('elevation above the horizon (°)')
        ax_p.set_title(f'Vertical field of view {theta_v:.0f}° (FOV {sl["fov"].val:g}°, 16:9)',
                       fontsize=10)
        ax_p.legend(loc='upper right', fontsize=8)
        step(None, advance=False)

    def step(_frame, advance=True):
        if advance:
            st['t'] = (st['t'] + FRAME_S) % PERIOD_S
        pitch = st['p_max'] * math.sin(2 * math.pi * st['t'] / PERIOD_S)   # nose down > 0
        centre = -pitch
        half = st['theta_v'] / 2
        w = st['fixed']
        w.set_theta1(centre - half)
        w.set_theta2(centre + half)
        in_view = abs(st['eps'] - centre) <= half
        w.set_color(SERIES[0] if in_view else BAD)
        w.set_alpha(0.25 if in_view else 0.18)
        c, s = math.cos(math.radians(centre)), math.sin(math.radians(centre))
        body = np.array([[-0.5, -0.08], [0.5, -0.08], [0.5, 0.08], [-0.5, 0.08]])
        st['body'].set_xy(body @ np.array([[c, s], [-s, c]]))
        st['marker'].set_xdata([pitch, pitch])
        gimbal_ok = abs(st['eps']) <= half
        st['status'].set_text(f'pitch {pitch:+5.1f}°\n'
                              f'fixed camera:  {"in view" if in_view else "LOST"}\n'
                              f'gimbal camera: {"in view" if gimbal_ok else "out of view"}')
        st['status'].set_color(INK if in_view else BAD)
        return ()

    for s_ in sl.values():
        s_.on_changed(reset)
    reset()
    frames = int(PERIOD_S / FRAME_S)
    anim = FuncAnimation(fig, step, frames=frames if args.gif else None,
                         interval=FRAME_S * 1000, cache_frame_data=False)
    common.finish(fig, 'tilt_view', args, anim=anim, fps=int(1 / FRAME_S))
    return fig, sl, anim


if __name__ == '__main__':
    main()
