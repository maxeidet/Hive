"""Renders a recorded operator UI (ui/replay.py) to an mp4, frame by frame on sim time.

Frame i of the video shows the page at simulation time START + i / FPS, the same rule as
Gazebo's video recorder on sim time. So a UI video and a Gazebo replay clip that start at
the same sim time line up frame by frame (recording/README.md).

How: the recording is served as if it were the live UI node (ReplayUi), and a headless
Chromium (Playwright) opens the page in replay mode with its clock under our control
(page.clock: Date, performance.now, timers and animation frames are virtual). For every
frame the clock is moved on by exactly 1 / FPS, the page's poll for that moment is
answered, and a screenshot goes to ffmpeg. It runs as fast as Chromium draws (about
5-10 frames a second), not in real time, and needs no simulation.

    python3 -m swarm_backend.ui.render data/recordings/<run>/ui.jsonl --list
    python3 -m swarm_backend.ui.render data/recordings/<run>/ui.jsonl --start 312 --duration 90
        [--fps 25] [--size 1920x1080] [--out data/recordings/<run>/ui_312.mp4]

Needs Playwright with Chromium and ffmpeg (both in the hive-px4-vnc image). Start it with
`recording/video.sh ui ...` on the host.
"""
import argparse
import subprocess
import sys
import threading
from http.server import ThreadingHTTPServer
from pathlib import Path

from swarm_backend.ui.replay import ReplayUi, frame_times, load_recording

WARMUP_S = 3.0  # sim seconds the page runs before the first frame: trails, sparklines, glides


def list_events(replay: ReplayUi) -> None:
    end = replay.sims[-1]
    print(f'Recording: sim time {replay.start:.1f} to {end:.1f} s '
          f'({end - replay.start:.0f} s, {len(replay.sims)} states)')
    print(f'{"sim time":>9}  {"T+":>7}  event')
    for sim, t, e in replay.event_table():
        detail = f'  ({e["detail"]})' if e.get('detail') else ''
        print(f'{sim:9.1f}  {t:7.1f}  {e["text"]}{detail}')
    print('Use a sim time as --start (here and in `video.sh replay --start`): '
          'a few seconds before what you want to show.')


def render(recording: Path, out: Path, start: float, duration: float | None, fps: float = 25.0,
           size: tuple[int, int] = (1920, 1080), warmup: float = WARMUP_S,
           collapse_events: bool = False) -> Path:
    from playwright.sync_api import sync_playwright

    from swarm_backend.nodes.operator_ui import STATIC_DIR, make_handler
    from swarm_backend.ui.model import load_map

    replay = ReplayUi(load_recording(recording))
    replay.map = load_map(STATIC_DIR)
    if duration is None:
        duration = replay.sims[-1] - start
    times = frame_times(start, duration, fps)
    if start + duration > replay.sims[-1] + 1.0:
        print(f'Note: the recording ends at sim time {replay.sims[-1]:.1f} s; '
              'the last frames repeat its last state.')

    server = ThreadingHTTPServer(('127.0.0.1', 0), make_handler(replay))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    port = server.server_address[1]

    out.parent.mkdir(parents=True, exist_ok=True)
    ffmpeg = subprocess.Popen(
        ['ffmpeg', '-v', 'error', '-y', '-f', 'image2pipe', '-c:v', 'png', '-framerate', str(fps),
         '-i', '-', '-c:v', 'libx264', '-preset', 'medium', '-crf', '16', '-pix_fmt', 'yuv420p',
         '-movflags', '+faststart', str(out)], stdin=subprocess.PIPE)

    first = start - warmup
    wall = replay.wall_at(first)
    step_ms = 1000.0 / fps
    try:
        with sync_playwright() as pw:
            browser = pw.chromium.launch()
            page = browser.new_page(viewport={'width': size[0], 'height': size[1]},
                                    device_scale_factor=1, color_scheme='dark',
                                    reduced_motion='no-preference')
            if collapse_events:
                page.add_init_script("try { localStorage.setItem('hive.eventsCollapsed', '1') } catch (e) {}")
            else:
                page.add_init_script("try { localStorage.setItem('hive.eventsCollapsed', '0') } catch (e) {}")
            page.clock.install(time=int(wall * 1000) if wall else None)
            page.goto(f'http://127.0.0.1:{port}/?replay={first:.3f}', wait_until='load')
            # Stop the clock: from here on it only moves when we move it.
            page.clock.pause_at(page.evaluate('Date.now()') + 1)
            page.evaluate('window.hiveIdle()')
            elapsed_ms = page.evaluate('performance.now()') - page.evaluate(
                'window.hiveReplayStart')  # virtual ms the page has already run
            if elapsed_ms > warmup * 1000:
                raise RuntimeError(f'The page took {elapsed_ms / 1000:.1f} s (virtual) to load, '
                                   f'more than the {warmup} s warm-up: use a longer --warmup.')

            def advance_to(target_ms: float) -> None:
                nonlocal elapsed_ms
                # In steps of one frame, each poll answered before the next can start.
                while elapsed_ms < target_ms - 0.5:
                    dt = min(step_ms, target_ms - elapsed_ms)
                    page.clock.run_for(int(round(dt)))
                    elapsed_ms += int(round(dt))
                    page.evaluate('window.hiveIdle()')

            advance_to(warmup * 1000)
            for i, _ in enumerate(times):
                advance_to(warmup * 1000 + i * step_ms)
                page.evaluate('window.hiveDraw()')
                ffmpeg.stdin.write(page.screenshot(type='png', animations='disabled'))
                if i % int(fps * 10) == 0:
                    print(f'  sim {times[i]:.1f} s  ({i}/{len(times)} frames)', flush=True)
            browser.close()
    finally:
        ffmpeg.stdin.close()
        ffmpeg.wait()
        server.shutdown()
    if ffmpeg.returncode != 0:
        raise RuntimeError('ffmpeg failed')
    print(f'Saved {out}: sim time {start:.2f} to {start + len(times) / fps:.2f} s, '
          f'{len(times)} frames at {fps:g} fps, {size[0]}x{size[1]}')
    return out


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('recording', type=Path, help='ui.jsonl from `ui --record`, or its folder')
    parser.add_argument('--list', action='store_true', help='print the events with sim times and stop')
    parser.add_argument('--start', type=float, help='sim time of the first frame (seconds)')
    parser.add_argument('--duration', type=float, default=None,
                        help='seconds of sim time (default: to the end of the recording)')
    parser.add_argument('--fps', type=float, default=25.0)
    parser.add_argument('--size', default='1920x1080', help='WIDTHxHEIGHT of the page and video')
    parser.add_argument('--warmup', type=float, default=WARMUP_S,
                        help='sim seconds the page runs before the first frame')
    parser.add_argument('--collapse-events', action='store_true', help='event log minimised')
    parser.add_argument('--out', type=Path, default=None,
                        help='mp4 (default: ui_<start>.mp4 next to the recording)')
    args = parser.parse_args(argv)

    recording = args.recording / 'ui.jsonl' if args.recording.is_dir() else args.recording
    if args.list:
        list_events(ReplayUi(load_recording(recording)))
        return 0
    if args.start is None:
        parser.error('--start is needed (see --list for the sim times of the events)')
    width, height = (int(v) for v in args.size.lower().split('x'))
    out = args.out or recording.parent / f'ui_{args.start:g}.mp4'
    render(recording, out, args.start, args.duration, args.fps, (width, height), args.warmup,
           args.collapse_events)
    return 0


if __name__ == '__main__':
    sys.exit(main())
