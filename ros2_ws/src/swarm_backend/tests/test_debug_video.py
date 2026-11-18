import json
import shutil
import subprocess
import time
from types import SimpleNamespace

import pytest

from swarm_backend.ui import debug_video
from swarm_backend.ui.debug_video import DroneVideo, FrameClock, cut_command, image_rgb


def test_frames_hold_the_newest_image_at_or_before_their_time():
    clock = FrameClock(fps=25)
    assert clock.add(100.0) == 0           # the first image: START = 100.0
    assert clock.add(100.1) == 3           # frames at 100.00, .04, .08 show image 1
    assert clock.add(100.12) == 0          # 100.12 is a frame time: not before it
    assert clock.add(100.13) == 1          # the frame at 100.12 shows image 2 (100.1)
    assert clock.finish() == 1             # the frame at 100.16 shows the last image
    assert clock.written == 5


def test_an_exact_frame_time_does_not_round_into_an_extra_frame():
    clock = FrameClock(fps=25)
    clock.add(10.0)
    assert clock.add(10.04) == 1           # 0.04 * 25 = 1.0000000000000009 in floats


def test_old_images_are_dropped():
    clock = FrameClock(fps=25)
    clock.add(5.0)
    clock.add(5.2)
    assert clock.add(5.1) is None
    assert clock.add(5.2) is None


def test_a_long_gap_is_filled_with_the_held_image():
    clock = FrameClock(fps=25)
    clock.add(0.0)
    assert clock.add(2.0) == 50


def test_finish_without_images_writes_nothing():
    assert FrameClock().finish() == 0


def image_msg(width, height, encoding, data, step=None):
    return SimpleNamespace(width=width, height=height, encoding=encoding,
                           step=step or width * 3, data=data)


def test_image_rgb_strips_row_padding_and_swaps_bgr():
    padded = bytes([1, 2, 3, 9, 4, 5, 6, 9])   # 1x2 image, 1 pad byte per row
    assert image_rgb(image_msg(1, 2, 'rgb8', padded, step=4)) == bytes([1, 2, 3, 4, 5, 6])
    assert image_rgb(image_msg(1, 1, 'bgr8', bytes([1, 2, 3]))) == bytes([3, 2, 1])
    assert image_rgb(image_msg(1, 1, 'mono8', bytes([1]))) is None


def test_cut_seeks_into_the_recording_or_holds_its_first_image():
    later = cut_command('d.mkv', 'o.mp4', rec_start=100.0, start=120.0, duration=55.0)
    assert later[later.index('-ss') + 1] == '20.000'
    assert later[later.index('-t') + 1] == '55.000'
    assert 'tpad' not in later[later.index('-vf') + 1]
    earlier = cut_command('d.mkv', 'o.mp4', rec_start=100.0, start=97.5, duration=None)
    assert '-ss' not in earlier and '-t' not in earlier
    assert 'tpad=start_duration=2.500:start_mode=clone' in earlier[earlier.index('-vf') + 1]


def frame_count(path):
    out = subprocess.run(['ffprobe', '-v', 'error', '-count_frames', '-select_streams', 'v:0',
                          '-show_entries', 'stream=nb_read_frames', '-of', 'csv=p=0', str(path)],
                         capture_output=True, text=True, check=True).stdout
    return int(out.strip())


def first_pixel(path, at):
    raw = subprocess.run(['ffmpeg', '-v', 'error', '-i', str(path), '-vf', f'select=eq(n\\,{at})',
                          '-frames:v', '1', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-'],
                         capture_output=True, check=True).stdout
    return raw[:3]


@pytest.mark.skipif(shutil.which('ffmpeg') is None, reason='needs ffmpeg (in the container)')
def test_record_and_cut_line_up_on_sim_time(tmp_path):
    w, h = 64, 48
    red, blue = bytes([255, 0, 0]) * (w * h), bytes([0, 0, 255]) * (w * h)
    video = DroneVideo(3, tmp_path)
    video.offer(100.0, w, h, red)      # red from 100.0, blue from 102.0, until 104.0
    video.offer(102.0, w, h, blue)
    video.offer(104.0, w, h, blue)
    time.sleep(0.5)
    video.close()
    meta = json.loads((tmp_path / 'debug_d3.json').read_text())
    assert meta['start'] == 100.0 and meta['images'] == 3
    assert frame_count(tmp_path / 'debug_d3.mkv') == 101       # 100.00 .. 104.00

    args = SimpleNamespace(run=tmp_path, id=3, start=101.0, duration=2.0, out=None)
    debug_video.cut(args)
    clip = tmp_path / 'debug_d3_101.mp4'
    assert frame_count(clip) == 50
    assert first_pixel(clip, 24)[0] > 200                      # sim 101.96: still red
    assert first_pixel(clip, 26)[2] > 200                      # sim 102.04: blue
