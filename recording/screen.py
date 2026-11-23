"""Screen-record the live operator UI, then put the recording on simulation time.

The UI is filmed live, from the browser window on this computer's screen. (Rendering it
afterwards in headless Chromium was tried and removed: on Stellar Chromium hung at start with
the GPU, and without it the page's blurred panels never finished a frame.)

record  (`recording/video.sh screen`, during the run, in a container of its own with the host's
        IPC namespace: x11grab's shared-memory images (MIT-SHM) fail in the sim container)
    Click the browser window that shows http://localhost:8090 (or give --region). ffmpeg
    films that part of the screen at 25 fps into data/recordings/<run>/screen.mkv until
    Ctrl-C. Every frame is stamped with the computer's clock (x11grab + -copyts), and
    `ui --record` saves the computer's clock next to the sim time, so the two line up
    exactly. Keep the window where it is and uncovered while it films.

cut     (`recording/video.sh screen-cut --start S --duration D`, afterwards)
    screen_<S>.mp4: the recording from sim time S, D sim seconds, 25 fps. Its first frame
    shows sim time S, as in `video.sh debug`, `cam` and `replay`, so they compose. If the
    sim ran slower or faster than real time, the clip is sped up or slowed down to match.

Runs in the container (ffmpeg and xwininfo are in the hive-px4-vnc image).
"""
import argparse
import json
import os
import re
import subprocess
import sys
from pathlib import Path

FPS = 25


# ---- record --------------------------------------------------------------------------

def pick_window() -> tuple[int, int, int, int]:
    """(x, y, width, height) of the window the user clicks, from xwininfo."""
    print('Click the browser window with the operator UI...', flush=True)
    out = subprocess.run(['xwininfo'], capture_output=True, text=True, check=True).stdout
    return parse_xwininfo(out)


def parse_xwininfo(text: str) -> tuple[int, int, int, int]:
    def field(name):
        m = re.search(rf'{name}:\s+(-?\d+)', text)
        if not m:
            raise ValueError(f'xwininfo gave no "{name}"')
        return int(m.group(1))
    return (field('Absolute upper-left X'), field('Absolute upper-left Y'),
            field('Width'), field('Height'))


def parse_region(arg: str) -> tuple[int, int, int, int]:
    """WIDTHxHEIGHT+X+Y, as X11 geometry."""
    m = re.fullmatch(r'(\d+)x(\d+)\+(-?\d+)\+(-?\d+)', arg)
    if not m:
        raise SystemExit(f'--region {arg}: use WIDTHxHEIGHT+X+Y, e.g. 1920x1080+0+0')
    w, h, x, y = map(int, m.groups())
    return x, y, w, h


def even(region: tuple[int, int, int, int]) -> tuple[int, int, int, int]:
    """H.264 (yuv420p) wants even sizes; a negative corner (window half off screen) is clipped."""
    x, y, w, h = region
    if x < 0:
        w, x = w + x, 0
    if y < 0:
        h, y = h + y, 0
    return x, y, w - w % 2, h - h % 2


def record_command(display: str, region: tuple[int, int, int, int], fps: int, out: Path) -> list[str]:
    x, y, w, h = region
    # -copyts keeps x11grab's timestamps, which are the computer's clock: the alignment.
    # veryfast, two threads: the sim runs beside it on a shared machine.
    return ['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y',
            '-f', 'x11grab', '-framerate', str(fps), '-video_size', f'{w}x{h}',
            '-draw_mouse', '0', '-i', f'{display}+{x},{y}', '-copyts',
            '-c:v', 'libx264', '-preset', 'veryfast', '-crf', '20', '-threads', '2',
            '-pix_fmt', 'yuv420p', str(out)]


def record(args) -> int:
    display = os.environ.get('DISPLAY')
    if not display:
        raise SystemExit('No DISPLAY: run `video.sh screen` in a terminal on this computer\'s desktop.')
    region = even(parse_region(args.region) if args.region else pick_window())
    out = args.run / 'screen.mkv'
    (args.run / 'screen.json').write_text(json.dumps(
        {'display': display, 'region': region, 'fps': args.fps}, indent=1) + '\n')
    x, y, w, h = region
    print(f'Filming {w}x{h} at ({x}, {y}) of {display} into {out}. Keep the window there and '
          'uncovered. Ctrl-C stops it.', flush=True)
    proc = subprocess.Popen(record_command(display, region, args.fps, out))
    try:
        proc.wait()
    except KeyboardInterrupt:  # ffmpeg got the Ctrl-C too and closes the file
        proc.wait()
    start = first_frame_wall(out)
    if start is None:
        print('Nothing was filmed.', flush=True)
        return 1
    print(f'Saved {out} (first frame at {start:.2f}, the computer\'s clock)', flush=True)
    return 0


# ---- cut -----------------------------------------------------------------------------

def first_frame_wall(video: Path) -> float | None:
    if not video.exists():
        return None
    out = subprocess.run(['ffprobe', '-v', 'error', '-show_entries', 'format=start_time',
                          '-of', 'csv=p=0', str(video)], capture_output=True, text=True).stdout
    try:
        return float(out.strip())
    except ValueError:
        return None


def clock_pairs(ui_jsonl: Path) -> list[tuple[float, float]]:
    """(sim, wall) of every UI state, sorted by sim; a torn last line is skipped."""
    pairs = []
    for line in ui_jsonl.read_text().splitlines():
        try:
            d = json.loads(line)
        except json.JSONDecodeError:
            continue
        if d.get('wall') is not None:
            pairs.append((d['sim'], d['wall']))
    pairs.sort()
    return pairs


def wall_at(pairs: list[tuple[float, float]], sim: float) -> float:
    """The computer's clock at sim time `sim`: linear between the two nearest UI states,
    extrapolated at real-time speed outside them."""
    if sim <= pairs[0][0]:
        return pairs[0][1] - (pairs[0][0] - sim)
    if sim >= pairs[-1][0]:
        return pairs[-1][1] + (sim - pairs[-1][0])
    lo, hi = 0, len(pairs) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if pairs[mid][0] <= sim:
            lo = mid
        else:
            hi = mid
    (s0, w0), (s1, w1) = pairs[lo], pairs[hi]
    return w0 + (sim - s0) / (s1 - s0) * (w1 - w0) if s1 > s0 else w0


def cut_command(src: Path, out: Path, offset: float, wall_len: float, duration: float,
                fps: int = FPS) -> list[str]:
    """`offset` s into the file (wall), `wall_len` wall seconds stretched to `duration`."""
    speed = duration / wall_len
    return ['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y',
            '-ss', f'{offset:.3f}', '-t', f'{wall_len + 0.5:.3f}', '-i', str(src),
            '-vf', f'setpts=(PTS-STARTPTS)*{speed:.6f},fps={fps}',
            '-t', f'{duration:.3f}', '-c:v', 'libx264', '-preset', 'medium', '-crf', '18',
            '-pix_fmt', 'yuv420p', '-an', str(out)]


def cut(args) -> int:
    src = args.run / 'screen.mkv'
    ui = args.run / 'ui.jsonl'
    start_wall = first_frame_wall(src)
    if start_wall is None:
        raise SystemExit(f'No screen recording in {args.run}: run `video.sh screen` during the run.')
    if not ui.exists():
        raise SystemExit(f'No {ui}: the screen recording is put on sim time with the UI '
                         'recording, so `ui --record auto` has to run too.')
    pairs = clock_pairs(ui)
    w0 = wall_at(pairs, args.start)
    w1 = wall_at(pairs, args.start + args.duration)
    offset = w0 - start_wall
    if offset < 0:
        raise SystemExit(f'The screen recording starts {-offset:.1f} s after sim time '
                         f'{args.start:g}: use a later --start.')
    out = args.out or args.run / f'screen_{args.start:g}.mp4'
    subprocess.run(cut_command(src, out, offset, w1 - w0, args.duration), check=True)
    print(f'Wrote {out}: sim time {args.start:g} to {args.start + args.duration:g} s '
          f'({w1 - w0:.1f} s on the computer\'s clock)')
    return 0


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = p.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('record')
    r.add_argument('run', type=Path, help='the run folder')
    r.add_argument('--region', help='WIDTHxHEIGHT+X+Y instead of clicking a window')
    r.add_argument('--fps', type=int, default=FPS)
    c = sub.add_parser('cut')
    c.add_argument('run', type=Path, help='the run folder')
    c.add_argument('--start', type=float, required=True, help='sim time of the first frame')
    c.add_argument('--duration', type=float, required=True, help='sim seconds')
    c.add_argument('--out', type=Path)
    args = p.parse_args(argv)
    return record(args) if args.cmd == 'record' else cut(args)


if __name__ == '__main__':
    sys.exit(main())
