import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / 'recording'))
import screen  # noqa: E402  (recording/screen.py)

XWININFO = """
xwininfo: Window id: 0x3a00004 "Hive operator - Mozilla Firefox"

  Absolute upper-left X:  1921
  Absolute upper-left Y:  -3
  Relative upper-left X:  0
  Relative upper-left Y:  0
  Width: 1601
  Height: 900
"""


def test_window_from_xwininfo_is_clipped_to_the_screen_and_made_even():
    region = screen.parse_xwininfo(XWININFO)
    assert region == (1921, -3, 1601, 900)
    assert screen.even(region) == (1921, 0, 1600, 896)  # 3 rows off the top, then even


def test_region_argument():
    assert screen.parse_region('1280x720+10+20') == (10, 20, 1280, 720)
    with pytest.raises(SystemExit):
        screen.parse_region('1280x720')


def test_wall_at_interpolates_and_extrapolates_at_real_time():
    pairs = [(100.0, 5000.0), (101.0, 5002.0)]   # the sim at half speed
    assert screen.wall_at(pairs, 100.5) == 5001.0
    assert screen.wall_at(pairs, 99.0) == 4999.0
    assert screen.wall_at(pairs, 103.0) == 5004.0


def test_cut_stretches_wall_time_to_sim_time():
    cmd = screen.cut_command(Path('s.mkv'), Path('o.mp4'), offset=3.0, wall_len=2.0, duration=1.0)
    assert cmd[cmd.index('-ss') + 1] == '3.000'
    assert 'setpts=(PTS-STARTPTS)*0.500000,fps=25' in cmd
    assert cmd[cmd.index('-vf') + 3] == '1.000'   # the output's -t: sim seconds


ROOT = """
xwininfo: Window id: 0x6b6 (the root window) (has no name)

  Absolute upper-left X:  0
  Absolute upper-left Y:  0
  Width: 1921
  Height: 1080
"""


def test_root_region():
    assert screen.root_region(ROOT) == (0, 0, 1920, 1080)
