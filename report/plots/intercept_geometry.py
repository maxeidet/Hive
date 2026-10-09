"""Animated worst-case intercept (Target System, Intercept feasibility), in a loop.

The ring is drawn locally straight (the x axis), outside is up. A hostile drone flies
straight in, midway between two defenders, at lateral distance y = y_max from
the nearest one. It enters the detection range r_det; in the worst case the scanning
camera has just looked past it, so it is detected one sweep later and the interceptor
departs t_d after entry. The intercept works if the interceptor reaches the hostile
drone's path before the hostile drone reaches the ring (margin t_h - t_i >= 0).

    python report/plots/intercept_geometry.py
    python report/plots/intercept_geometry.py --gif      # one loop as a GIF
"""
import math

import common
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle, Wedge

import model
from common import BAD, INK, INK_2, INK_3, SERIES, SURFACE

FRIEND, HOSTILE = SERIES[0], SERIES[1]
SPAWN_OUT_M = 1000.0        # hostile spawn is 1 km outside the ring (r + 1)
FRAME_S = 0.05              # real seconds per frame


class Run:
    """Timeline of one worst-case run for Scene s."""

    def __init__(self, s):
        self.s = s
        self.r_det = model.detection_range_m(s)
        self.y = model.y_max(s)
        self.seen = self.r_det > self.y
        self.t_ring = SPAWN_OUT_M / s.v_h
        self.cols = math.ceil(s.sector_deg / s.fov_deg)
        self.positions = model.scan_positions(s)
        if self.seen:
            inside = math.sqrt(self.r_det ** 2 - self.y ** 2)
            self.t_enter = max(0.0, (SPAWN_OUT_M - inside) / s.v_h)
            self.t_detect = self.t_enter + model.t_sweep(s)
            self.t_depart = self.t_enter + model.t_depart(s)
            self.t_arrive = self.t_depart + model.flight_time(s, self.y)
            self.margin = self.t_ring - self.t_arrive
            entry_bearing = math.degrees(math.atan2(self.hostile_y(self.t_enter), self.y))
            self.k_entry = int(entry_bearing // s.fov_deg)
        else:
            self.t_enter = self.t_detect = self.t_depart = self.t_arrive = math.inf
            self.margin = None
        end = self.t_ring if not self.seen else max(self.t_ring, min(self.t_arrive, 3 * self.t_ring))
        self.t_end = end + 2.0

    def hostile_y(self, t):
        return max(0.0, SPAWN_OUT_M - self.s.v_h * t)

    def interceptor_x(self, t):
        if t <= self.t_depart:
            return 0.0
        tau = t - self.t_depart
        s = self.s
        t_acc = s.v_i / s.accel
        d = 0.5 * s.accel * tau ** 2 if tau < t_acc else s.v_i * (tau - t_acc / 2)
        return min(d, self.y)

    def camera_bearing(self, t):
        """Centre of the narrow camera's view, degrees from the ring (x axis)."""
        s = self.s
        if t >= self.t_detect:
            return math.degrees(math.atan2(self.hostile_y(t), self.y - self.interceptor_x(t)))
        if not self.seen:
            k = int(t // s.t_g) % self.cols
        else:
            # Worst case: the camera has just left the entry bearing at t_enter.
            k = (self.k_entry + 1 + math.floor((t - self.t_enter) / s.t_g)) % self.cols
        return (k + 0.5) * s.fov_deg


def main():
    args = common.cli(__doc__)
    fig = plt.figure(figsize=(11.5, 7.5))
    ax = fig.add_axes((0.05, 0.27, 0.58, 0.68))
    status = fig.text(0.67, 0.93, '', family='monospace', fontsize=10, color=INK_2, va='top')
    verdict = fig.text(0.67, 0.42, '', fontsize=13, weight='bold', va='top')

    sl = {
        'N': common.slider(fig, (0.16, 0.19, 0.45, 0.025), 'Drones N', 4, 30, 12, 1),
        'fov': common.slider(fig, (0.16, 0.15, 0.45, 0.025), 'FOV (°)', 5, 90, 20, 1),
        'R': common.slider(fig, (0.16, 0.11, 0.45, 0.025), 'Radius r (km)', 0.3, 3.0, 1.0, 0.05),
        'vh': common.slider(fig, (0.16, 0.07, 0.45, 0.025), 'Hostile speed (m/s)', 4, 30, 17, 1),
        'speed': common.slider(fig, (0.16, 0.03, 0.45, 0.025), 'Playback (×)', 1, 10, 4, 1),
    }
    st = {}

    def reset(_=None):
        s = model.Scene(n_drones=int(sl['N'].val), fov_deg=sl['fov'].val,
                        radius_m=sl['R'].val * 1000, v_h=sl['vh'].val)
        run = Run(s)
        st.update(run=run, t=0.0)
        ax.clear()
        y = run.y
        xlim = (-0.5 * y - 150, 2.5 * y + 150)
        ax.set_xlim(*xlim)
        ax.set_ylim(-250, SPAWN_OUT_M + 150)
        ax.set_aspect('equal')
        ax.set_xlabel('along the ring (m)')
        ax.set_ylabel('distance outside the ring (m)')
        ax.axhline(0, color=INK_2, lw=1.2)
        ax.axhline(SPAWN_OUT_M, color=INK_3, lw=1.2, ls=(0, (4, 3)))
        ax.text(xlim[0] + 20, 15, 'ring', color=INK_2, fontsize=9)
        ax.text(xlim[0] + 20, SPAWN_OUT_M + 15, 'hostile spawn (r + 1)', color=INK_2, fontsize=9)
        for x in (0.0, 2 * y):
            ax.plot(x, 0, 'o', color=FRIEND, markersize=9, mec=SURFACE, mew=1.5, zorder=5)
        if run.seen:
            ax.add_patch(Circle((0, 0), run.r_det, fill=False, color=FRIEND, lw=1, ls='--',
                                alpha=0.6))
        ax.plot([y, y], [0, SPAWN_OUT_M], color=HOSTILE, lw=1, ls=':', alpha=0.7)
        ax.plot(y, 0, 'X', color=INK, markersize=9, zorder=6)
        ax.annotate('meeting point', (y, 0), (y + 30, -120), fontsize=9, color=INK_2)
        ax.annotate('', (y, -60), (0, -60),
                    arrowprops=dict(arrowstyle='<->', color=INK_3, lw=1))
        ax.text(y / 2, -95, f'y = {y:.0f} m', ha='center', fontsize=9, color=INK_2)
        ax.set_title(f'Worst case: N = {s.n_drones}, FOV {s.fov_deg:g}°, '
                     f'r_det = {run.r_det:.0f} m', fontsize=11)
        st['wedge'] = ax.add_patch(Wedge((0, 0), run.r_det, 0, 1, color=FRIEND, alpha=0.3,
                                         lw=0))
        st['hostile'], = ax.plot([], [], 'o', color=HOSTILE, markersize=10, mec=SURFACE,
                                 mew=1.5, zorder=7)
        st['interceptor'], = ax.plot([], [], 'o', color=FRIEND, markersize=11, mec=INK,
                                     mew=1.5, zorder=7)
        st['trail'], = ax.plot([], [], color=FRIEND, lw=2)
        if run.margin is None:
            verdict.set_text('Not feasible: passes unseen')
            verdict.set_color(BAD)
        elif run.margin >= 0:
            verdict.set_text(f'Feasible: margin {run.margin:+.1f} s')
            verdict.set_color(INK)
        else:
            verdict.set_text(f'Too late: margin {run.margin:+.1f} s')
            verdict.set_color(BAD)
        step(None, advance=False)

    def step(_frame, advance=True):
        run = st['run']
        if advance:
            st['t'] += FRAME_S * sl['speed'].val
            if st['t'] > run.t_end:
                st['t'] = 0.0
        t = st['t']
        hy = run.hostile_y(t)
        ix = run.interceptor_x(t)
        st['hostile'].set_data([run.y], [hy])
        st['interceptor'].set_data([ix], [0])
        st['trail'].set_data([0, ix], [0, 0])
        bearing = run.camera_bearing(t)
        fov = run.s.fov_deg
        w = st['wedge']
        w.set_center((ix, 0))
        w.set_theta1(bearing - fov / 2)
        w.set_theta2(bearing + fov / 2)
        w.set_radius(max(run.r_det, 1.0))

        def phase():
            if t < run.t_enter:
                return 'outside detection range'
            if t < run.t_detect:
                return 'in range, camera scanning'
            if t < run.t_depart:
                return 'detected, gimbal turning'
            if ix < run.y:
                return 'interceptor flying'
            return 'interceptor at meeting point'

        def ts(v):
            return '   --  ' if v == math.inf else f'{v:6.1f} s'

        status.set_text('\n'.join([
            f'time              {t:6.1f} s',
            f'phase             {phase()}',
            '',
            f'enters range      {ts(run.t_enter)}',
            f'detected          {ts(run.t_detect)}',
            f'departs           {ts(run.t_depart)}',
            f'reaches path      {ts(run.t_arrive)}',
            f'hostile at ring   {ts(run.t_ring)}',
            '',
            f't_d (worst case)  {model.t_depart(run.s):6.1f} s',
            f'scan positions    {run.positions:6d}',
        ]))
        return ()

    for s_ in sl.values():
        if s_ is not sl['speed']:
            s_.on_changed(reset)
    reset()
    run = st['run']
    frames = math.ceil(run.t_end / (FRAME_S * sl['speed'].val)) + 1
    anim = FuncAnimation(fig, step, frames=frames if args.gif else None,
                         interval=FRAME_S * 1000, cache_frame_data=False)
    common.finish(fig, 'intercept_geometry', args, anim=anim, fps=int(1 / FRAME_S))
    return fig, sl, anim


if __name__ == '__main__':
    main()
