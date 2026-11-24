import json
import subprocess

from swarm_backend.demo.fake_run import make_run
from swarm_backend.ui.replay import ReplayUi, load_recording


def test_make_run_layout(tmp_path):
    truth = make_run(tmp_path, takes=2, hit_event=True)
    for name in ('ui.jsonl', 'poses.jsonl', 'run.json', 'debug_d3.mkv', 'debug_d3.json',
                 'cam_arc_side.json', 'cam_attacker_watch.mkv'):
        assert (tmp_path / name).exists(), name
    assert all(len(truth[k]) == 2 for k in ('release', 'sighting', 'hit', 'attacker'))
    texts = [e['text'] for _, _, e in ReplayUi(load_recording(tmp_path / 'ui.jsonl')).event_table()]
    assert 'Drone 3 hit the target' in texts and 'Drone 2 hit the target' in texts
    meta = json.loads((tmp_path / 'cam_arc_side.json').read_text())
    assert {'start', 'end', 'fps', 'width', 'height', 'hfov', 'model', 'angle'} <= set(meta)


def test_video_is_25_fps(tmp_path):
    make_run(tmp_path, takes=1)
    out = subprocess.run(['ffprobe', '-v', 'error', '-select_streams', 'v:0', '-show_entries',
                          'stream=r_frame_rate', '-of', 'csv=p=0', str(tmp_path / 'debug_d3.mkv')],
                         capture_output=True, text=True, check=True).stdout.strip()
    assert out == '25/1'


def test_no_hit_take(tmp_path):
    truth = make_run(tmp_path, takes=2, hit=[True, False], videos=False)
    assert truth['hit'][1] is None
    texts = [e['text'] for _, _, e in ReplayUi(load_recording(tmp_path / 'ui.jsonl')).event_table()]
    assert texts.count('Drone 2 hit the target') == 0
