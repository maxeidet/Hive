"""The final cut of a recorded run: scenes one after the other, each with its own layout.

Nothing covers anything else: a scene shows one picture, two side by side, or a 2x2 grid,
with a label on each picture and a caption for the scene. Edit SCENES below to re-cut.

The sources are the run's clips that all start at the same sim time BASE (cut them first):
    video.sh screen-cut --start 131 --duration 105      -> screen_131.mp4    (the UI)
    video.sh debug --id 3 --start 131 --duration 105    -> debug_d3_131.mp4  (YOLO view)
    video.sh cam --angle attacker_watch --start 131 ... -> cam_attacker_watch_131.mp4
    video.sh cam --angle arc_side --start 131 ...       -> cam_arc_side_131.mp4

    recording/video.sh final [--out final_cut.mp4] [RUN]        (ffmpeg in the container)

For another run: set BASE, ATTACKER and the SCENES' times, and cut the sources from BASE.
"""
import argparse
import subprocess
import sys
from pathlib import Path

BASE = 131.0          # sim time of the sources' first frame
W, H, FPS = 1920, 1080, 25
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
FONT_BOLD = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
FADE = 0.4            # seconds of fade in and out per scene

ATTACKER = 3          # the drone whose YOLO view is shown (video.sh events: who intercepts)
SOURCES = {
    'ui': (f'screen_{BASE:g}.mp4', 'Operator UI'),
    'yolo': (f'debug_d{ATTACKER}_{BASE:g}.mp4', f'Drone {ATTACKER} camera - YOLO + ByteTrack'),
    'chase': (f'cam_attacker_watch_{BASE:g}.mp4', 'Chase camera'),
    'wide': (f'cam_arc_side_{BASE:g}.mp4', 'Wide shot'),
}

# (caption, sim start, sim end, layout, sources, speed). speed 8 = 8x faster, 0.5 = slow motion.
SCENES = [
    ('Defenders arm and take off', 131.0, 145.0, 'side', ['wide', 'ui'], 1.0),
    ('Patrol - the hostile approaches (8x speed)', 145.0, 212.0, 'full', ['ui'], 8.0),
    ('Drone 3 detects the hostile and intercepts', 212.0, 224.2, 'grid', ['chase', 'yolo', 'ui', 'wide'], 1.0),
    ('The hit - replay at half speed', 221.5, 224.2, 'side', ['chase', 'yolo'], 0.5),
    ('Aftermath - the defenders return', 224.2, 236.0, 'side', ['wide', 'ui'], 1.0),
]


def text(s: str) -> str:
    """drawtext's text, escaped for the filter graph."""
    return s.replace('\\', '\\\\').replace(':', '\\:').replace("'", "\\'").replace(',', '\\,')


def panel(label: str, w: int, h: int) -> str:
    """Scale to w x h (letterboxed) and put the label in the top right (the overlays use the top left)."""
    return (f'scale={w}:{h}:force_original_aspect_ratio=decrease,'
            f'pad={w}:{h}:(ow-iw)/2:(oh-ih)/2:color=black,setsar=1,'
            f"drawtext=fontfile={FONT}:text='{text(label)}':x=w-tw-14:y=12:fontsize={max(18, h // 26)}:"
            f'fontcolor=white:box=1:boxcolor=black@0.55:boxborderw=8')


def scene_filter(i: int, scene, first_input: int) -> tuple[str, list[tuple[str, float, float]]]:
    """The filter chain of one scene (output [s<i>]) and its inputs (file, seek, length)."""
    caption, start, end, layout, names, speed = scene
    inputs = [(SOURCES[n][0], start - BASE, end - start) for n in names]
    length = (end - start) / speed
    parts = []
    size = {'full': (W, H), 'side': (W // 2, H // 2), 'grid': (W // 2, H // 2)}[layout]
    for k, n in enumerate(names):
        parts.append(f'[{first_input + k}:v]fps={FPS},setpts=(PTS-STARTPTS)/{speed},'
                     f'{panel(SOURCES[n][1], *size)}[p{i}_{k}]')
    pins = ''.join(f'[p{i}_{k}]' for k in range(len(names)))
    if layout == 'full':
        body = f'{pins}null'
    elif layout == 'side':
        body = f'{pins}hstack=inputs=2,pad={W}:{H}:0:(oh-ih)/2:color=black'
    else:
        body = f'{pins}xstack=inputs=4:layout=0_0|w0_0|0_h0|w0_h0'
    parts.append(
        f'{body},trim=duration={length:.3f},setpts=PTS-STARTPTS,'
        f"drawtext=fontfile={FONT_BOLD}:text='{text(caption)}':x=(w-tw)/2:y=h-th-36:fontsize=40:"
        f'fontcolor=white:box=1:boxcolor=black@0.65:boxborderw=16,'
        f"drawtext=fontfile={FONT}:text='{text(f'sim {start:.0f}-{end:.0f} s')}':x=w-tw-24:y=h-th-44:"
        f'fontsize=24:fontcolor=white@0.8,'
        f'fade=t=in:st=0:d={FADE},fade=t=out:st={length - FADE:.3f}:d={FADE},format=yuv420p[s{i}]')
    return ';'.join(parts), inputs


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('run', type=Path)
    p.add_argument('--out', default='final_cut.mp4')
    args = p.parse_args(argv)
    missing = [f for f, _ in SOURCES.values() if not (args.run / f).exists()]
    if missing:
        raise SystemExit(f'Missing in {args.run}: {", ".join(missing)} (cut them first, see --help).')
    graphs, cmd_inputs, n = [], [], 0
    for i, scene in enumerate(SCENES):
        graph, inputs = scene_filter(i, scene, n)
        graphs.append(graph)
        for f, seek, length in inputs:
            cmd_inputs += ['-ss', f'{seek:.3f}', '-t', f'{length + 0.5:.3f}', '-i', str(args.run / f)]
        n += len(inputs)
    graphs.append(''.join(f'[s{i}]' for i in range(len(SCENES))) + f'concat=n={len(SCENES)}:v=1:a=0[out]')
    out = args.run / args.out
    cmd = ['ffmpeg', '-v', 'error', '-y', *cmd_inputs, '-filter_complex', ';'.join(graphs),
           '-map', '[out]', '-c:v', 'libx264', '-preset', 'medium', '-crf', '18', '-r', str(FPS),
           '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(out)]
    subprocess.run(cmd, check=True)
    total = sum((e - s) / sp for _, s, e, _, _, sp in SCENES)
    print(f'Wrote {out}: {len(SCENES)} scenes, {total:.1f} s')
    return 0


if __name__ == '__main__':
    sys.exit(main())
