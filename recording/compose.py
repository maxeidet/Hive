"""Puts a Gazebo replay clip and the rendered UI video together, synced on sim time.

Both videos are on simulation time: frame i shows START + i / fps. When both start at the
same sim time (`video.sh replay --start S` and `video.sh ui --start S`), --gz-offset is 0.
If a check on a visible event (the hit: the hostile drone jumps away in Gazebo, the UI
logs it at the same moment) shows they are off, nudge with --gz-offset: the sim seconds
the Gazebo clip's first frame comes AFTER the UI's first frame (negative: before).

Layouts (all 25 fps, H.264, no audio):
  pip   the UI full size, the Gazebo clip as an inset (--pip-width, --pip-corner). The UI
        rendered with --collapse-events leaves the right side free for it.
  side  side by side, both scaled to --height (default 720): 2 x 1280 x 720
  stack Gazebo on top, UI below, both scaled to --width (default 1280)

    python3 compose.py --ui ui_312.mp4 --gazebo hostile_chase.mp4 --layout pip --out demo.mp4
    python3 compose.py --ui ui_312.mp4 --gazebo d2_watch.mp4 --layout side --gz-offset 0.12 --out demo.mp4

Runs in the container (ffmpeg is in the hive-px4-vnc image): `recording/video.sh compose ...`.
"""
import argparse
import json
import subprocess
import sys


def probe(path: str) -> dict:
    out = subprocess.run(['ffprobe', '-v', 'error', '-select_streams', 'v:0', '-show_entries',
                          'stream=width,height,r_frame_rate:format=duration', '-of', 'json', path],
                         capture_output=True, text=True, check=True).stdout
    data = json.loads(out)
    s = data['streams'][0]
    num, den = (float(v) for v in s['r_frame_rate'].split('/'))
    return {'width': s['width'], 'height': s['height'], 'fps': num / den,
            'duration': float(data['format']['duration'])}


def trims(gz_offset: float, ui_len: float, gz_len: float) -> tuple[float, float, float]:
    """(seconds cut from the start of the UI video, of the Gazebo clip, length of the
    result) so that both show the same sim time from the first frame on."""
    ui_cut, gz_cut = (gz_offset, 0.0) if gz_offset >= 0 else (0.0, -gz_offset)
    length = min(ui_len - ui_cut, gz_len - gz_cut)
    if length <= 0:
        raise ValueError('The two videos do not overlap in sim time')
    return ui_cut, gz_cut, length


def filter_graph(layout: str, fps: float, pip_width: int, corner: str, height: int,
                 width: int) -> str:
    base = f'[0:v]fps={fps},setpts=PTS-STARTPTS[ui];[1:v]fps={fps},setpts=PTS-STARTPTS[gz];'
    if layout == 'pip':
        margin = 24
        x = f'{margin}' if 'l' in corner else f'W-w-{margin}'
        y = f'{margin}' if 't' in corner else f'H-h-{margin}'
        return (base + f'[gz]scale={pip_width}:-2,pad=iw+4:ih+4:2:2:color=0x5cc8ff[in];'
                f'[ui][in]overlay={x}:{y}:shortest=1[out]')
    if layout == 'side':
        return (base + f'[gz]scale=-2:{height}[a];[ui]scale=-2:{height}[b];'
                '[a][b]hstack=inputs=2:shortest=1[out]')
    if layout == 'stack':
        return (base + f'[gz]scale={width}:-2[a];[ui]scale={width}:-2[b];'
                '[a][b]vstack=inputs=2:shortest=1[out]')
    raise ValueError(f'Unknown layout {layout}')


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--ui', required=True, help='the UI video (video.sh ui)')
    parser.add_argument('--gazebo', required=True, help='a Gazebo replay clip (video.sh replay)')
    parser.add_argument('--out', required=True)
    parser.add_argument('--layout', choices=('pip', 'side', 'stack'), default='pip')
    parser.add_argument('--gz-offset', type=float, default=0.0,
                        help='sim s the Gazebo clip starts after the UI video (negative: before)')
    parser.add_argument('--duration', type=float, default=None, help='cut the result to this long')
    parser.add_argument('--fps', type=float, default=25.0)
    parser.add_argument('--pip-width', type=int, default=760)
    parser.add_argument('--pip-corner', choices=('br', 'bl', 'tr', 'tl'), default='br')
    parser.add_argument('--height', type=int, default=720, help='side: height of both')
    parser.add_argument('--width', type=int, default=1280, help='stack: width of both')
    args = parser.parse_args(argv)

    ui, gz = probe(args.ui), probe(args.gazebo)
    ui_cut, gz_cut, length = trims(args.gz_offset, ui['duration'], gz['duration'])
    if args.duration:
        length = min(length, args.duration)
    print(f'UI {ui["width"]}x{ui["height"]} {ui["duration"]:.1f} s, Gazebo {gz["width"]}x{gz["height"]} '
          f'{gz["duration"]:.1f} s at {gz["fps"]:.1f} fps; cutting {ui_cut:.2f} s / {gz_cut:.2f} s, '
          f'result {length:.1f} s')
    if abs(gz['fps'] - args.fps) > 0.5:
        print(f'Note: the Gazebo clip is {gz["fps"]:.1f} fps; it is resampled to {args.fps:g} fps.')
    graph = filter_graph(args.layout, args.fps, args.pip_width, args.pip_corner, args.height,
                         args.width)
    cmd = ['ffmpeg', '-v', 'error', '-stats', '-y',
           '-ss', f'{ui_cut:.3f}', '-i', args.ui, '-ss', f'{gz_cut:.3f}', '-i', args.gazebo,
           '-filter_complex', graph, '-map', '[out]', '-t', f'{length:.3f}', '-an',
           '-c:v', 'libx264', '-preset', 'medium', '-crf', '18', '-pix_fmt', 'yuv420p',
           '-movflags', '+faststart', args.out]
    subprocess.run(cmd, check=True)
    print(f'Saved {args.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
