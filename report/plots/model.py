"""The report's assumptions and equations (Setup Overview, Target System), shared by
every plot in this folder so that the figures always match the report's numbers.

Change a default here and every plot follows.
"""
import math
from dataclasses import dataclass, replace

G = 9.81


@dataclass(frozen=True)
class Scene:
    radius_m: float = 1000.0       # R_p, perimeter radius
    n_drones: int = 12             # N
    fov_deg: float = 20.0          # theta, horizontal field of view (per camera)
    n_cameras: int = 1             # cameras side by side on each drone's gimbal
    width_px: int = 1920           # W, image width
    aspect: float = 9 / 16         # H / W
    size_m: float = 0.5            # s, hostile drone size
    p_min_px: float = 4.0          # smallest detectable size
    sector_deg: float = 180.0      # Theta_s, outward sector the gimbal scans
    patrol_alt_m: float = 50.0     # h
    hostile_alt_m: tuple = (20.0, 120.0)
    v_i: float = 12.0              # interceptor top speed
    v_h: float = 17.0              # hostile speed
    accel: float = 2.0             # a, interceptor acceleration
    t_g: float = 0.5               # gimbal turn and settle time

    def with_(self, **changes):
        return replace(self, **changes)


def focal_px(fov_deg, width_px):
    """Equation eq:focal."""
    return (width_px / 2) / math.tan(math.radians(fov_deg) / 2)


def pixels_on_target(s, r_m):
    return focal_px(s.fov_deg, s.width_px) * s.size_m / r_m


def detection_range_m(s):
    """Equation eq:rdet."""
    return focal_px(s.fov_deg, s.width_px) * s.size_m / s.p_min_px


def vertical_fov_deg(fov_deg, aspect=9 / 16):
    """Equation eq:vfov."""
    return 2 * math.degrees(math.atan(math.tan(math.radians(fov_deg) / 2) * aspect))


def scan_positions(s):
    """Gimbal positions needed to cover the sector (columns x rows). With several
    cameras side by side, one position covers n_cameras * fov_deg."""
    r = detection_range_m(s)
    lo, hi = s.hostile_alt_m
    band = math.degrees(math.atan((hi - s.patrol_alt_m) / r) - math.atan((lo - s.patrol_alt_m) / r))
    rows = max(1, math.ceil(band / vertical_fov_deg(s.fov_deg, s.aspect)))
    return math.ceil(s.sector_deg / (s.fov_deg * s.n_cameras)) * rows


def t_sweep(s):
    """Equation eq:sweep. Zero when the cameras see the whole sector at once."""
    n = scan_positions(s)
    return 0.0 if n == 1 else n * s.t_g


def t_depart(s):
    """Equation eq:delay: worst case from entering detection range to departure.
    Without scanning, the target is seen at once (t_d ~ 0, as in the report)."""
    return 0.0 if scan_positions(s) == 1 else t_sweep(s) + s.t_g


def y_max(s, n=None):
    """Equation eq:ymax."""
    return math.pi * s.radius_m / (n or s.n_drones)


def flight_time(s, distance_m):
    """Time to fly distance_m from standstill, accelerating at a up to v_i."""
    d_acc = s.v_i ** 2 / (2 * s.accel)
    if distance_m <= d_acc:
        return math.sqrt(2 * distance_m / s.accel)
    return distance_m / s.v_i + s.v_i / (2 * s.accel)


def margin_s(s, n=None, depart=None):
    """t_h - t_i (equations eq:th, eq:ti) at the worst case y = y_max.
    None when a hostile drone can pass between two drones unseen."""
    r = detection_range_m(s)
    y = y_max(s, n)
    if y >= r:
        return None
    t_h = math.sqrt(r ** 2 - y ** 2) / s.v_h
    t_i = (t_depart(s) if depart is None else depart) + flight_time(s, y)
    return t_h - t_i


def n_no_gaps(s):
    """Equation eq:ncoverage."""
    return math.ceil(math.pi * s.radius_m / detection_range_m(s))


def n_feasible(s, limit=500, depart=None):
    """Smallest N with a non-negative worst-case margin, or None above limit."""
    for n in range(2, limit + 1):
        m = margin_s(s, n, depart)
        if m is not None and m >= 0:
            return n
    return None


def angular_rate(v_perp, r_m):
    """Equation eq:angrate, rad/s."""
    return v_perp / r_m


def time_in_view(fov_deg, v_perp, r_m):
    """Time until a target at the image centre leaves a still camera's view."""
    return math.radians(fov_deg) / 2 / angular_rate(v_perp, r_m)


def tilt_deg(accel):
    """Equation eq:tilt."""
    return math.degrees(math.atan(accel / G))


def max_visible_elevation(theta_v_deg, pitch_deg):
    """Equation eq:tiltview: highest elevation a fixed camera still sees."""
    return theta_v_deg / 2 - pitch_deg
