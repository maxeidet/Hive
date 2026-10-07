# Plots for the report

Every plot uses the report's assumptions and equations from `model.py`, so the figures always match the tables in the report. Change a default in `model.py` and every plot follows.

Run a script to open it in a window, from this folder:

```bash
python intercept_geometry.py
```

Options for every script:
- `--save`: write `report/figures/<name>.pdf` and `.png` of the start view, without opening a window
- `--gif`: animated plots only, write one loop as `report/figures/<name>.gif`

| Script | Kind | Section | Shows |
|---|---|---|---|
| `perimeter.py` | static grid | Target System | Ring, spawn circle and camera coverage for several N and FOV (`--drones`, `--fov`, `--radius`) |
| `perimeter_interactive.py` | sliders | Target System | The same for one scene: FOV, N, radius, with coverage and intercept margin |
| `intercept_geometry.py` | animation | Intercept feasibility | The worst-case intercept: scan, detection, departure, race to the meeting point |
| `drones_vs_fov.py` | sliders, hover | Camera Choice | Drones needed against FOV, per image width |
| `margin_vs_n.py` | sliders, hover | Camera Choice, Base Station | Intercept margin against N, full ring and with two drones away |
| `pixels_vs_range.py` | sliders, hover | Detection range | Pixels on target against range, p_min and Johnson's criterion |
| `time_in_view.py` | animation | Keeping the target in view | How long a crossing target stays in a narrow and a wide view |
| `sensitivity.py` | slider, buttons | Additional Assumptions | Which assumption changes the number of drones most |
| `tilt_view.py` | animation | Rejected: fixed wide camera | A fixed camera loses the target when the drone pitches; the gimbal does not |
| `triangulation.py` | sliders | Base Station | Position error from two drones' bearings, and which partner drone is best |

`model.py` holds the equations, `common.py` the shared style and options. Needs `matplotlib`, `numpy` and, for GIFs, `Pillow`.
