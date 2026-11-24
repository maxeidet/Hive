import json
import subprocess
from pathlib import Path

import pytest

from swarm_backend.demo.boxes import box_at, crop_rect, first_box, load_boxes
from swarm_backend.demo.fake_run import make_run
from swarm_backend.demo.final import build

RECORD = Path(__file__).resolve().parents[4] / 'recording' / 'record.sh'


def write_records(folder, lines, video_w=640, video_h=360):
    (folder / 'debug_d3.json').write_text(json.dumps({'start': 0, 'end': 10, 'fps': 25.0,
                                                      'width': video_w, 'height': video_h}))
    (folder / 'detect_d3.jsonl').write_text('\n'.join(json.dumps(x) for x in lines) + '\n')


def rec(stamp, confirmed, target, w=1280, h=720):
    return {'stamp': stamp, 'width': w, 'height': h, 'boxes': [], 'confirmed': confirmed, 'target': target}


def test_load_boxes_target_only(tmp_path):
    write_records(tmp_path, [rec(1.0, [[100, 50, 10, 8, 7], [300, 60, 12, 9, 9]], 7),
                             rec(1.1, [[110, 52, 10, 8, 7]], 7), rec(1.2, [], None)],
                  video_w=1280, video_h=720)
    boxes = load_boxes(tmp_path, 3)
    assert [(b.sim, b.u, b.track) for b in boxes] == [(1.0, 100, 7), (1.1, 110, 7)]
    assert box_at(boxes, 1.15).u == 110 and box_at(boxes, 2.0) is None
    assert first_box(boxes, 0.5, 2.0).sim == 1.0


def test_box_scaled_to_video(tmp_path):
    write_records(tmp_path, [rec(1.0, [[100, 50, 10, 8, 7]], 7)])
    b = load_boxes(tmp_path, 3)[0]
    assert (b.u, b.v, b.w, b.h) == (50, 25, 5, 4)


def test_crop_at_edge():
    x0, y0, cw, ch = crop_rect(5, 100, 640, 360, 3.0)
    assert x0 == 0 and (cw, ch) == (213, 120)
    x0, y0, cw, ch = crop_rect(639, 359, 640, 360, 3.0)
    assert x0 + cw == 640 and y0 + ch == 360


ZOOM = '''scenes:
- caption: Locked
  from: first_sighting
  to: first_sighting + 2
  layout: zoom
  show: [yolo]
  freeze: 1
'''


def test_zoom_without_box(tmp_path, capsys):
    run = tmp_path / 'run'
    make_run(run, takes=1, boxes=False)
    (tmp_path / 'z.yaml').write_text(ZOOM)
    with pytest.raises(SystemExit):
        build(run, tmp_path / 'z.yaml', preview=True)
    assert 'confirmed no box' in capsys.readouterr().out


def frame(path, at):
    return subprocess.run(['ffmpeg', '-v', 'error', '-ss', f'{at}', '-i', str(path), '-frames:v', '1',
                           '-f', 'rawvideo', '-pix_fmt', 'gray', '-'], capture_output=True).stdout


def test_zoom_scene_builds(tmp_path):
    run = tmp_path / 'run'
    make_run(run, takes=1)
    (tmp_path / 'z.yaml').write_text(ZOOM)
    out = build(run, tmp_path / 'z.yaml', preview=True)
    dur = float(subprocess.run(['ffprobe', '-v', 'error', '-show_entries', 'format=duration', '-of',
                                'csv=p=0', str(out)], capture_output=True, text=True).stdout)
    assert dur == pytest.approx(3.0, abs=0.15)
    clip = sorted((run / 'scenes' / 'z' / 'preview').glob('*.mp4'))[0]
    import numpy as np
    a, b, c = (np.frombuffer(frame(clip, t), np.uint8).astype(int) for t in (0.5, 0.8, 2.6))
    still, moving = np.abs(a - b).mean(), np.abs(b - c).mean()
    assert still < 1.0 and moving > 3 * still    # frozen (past the fade-in), then moving


def test_dry_run_copies_detect_records():
    out = subprocess.run(['bash', str(RECORD), '--dry-run', 'scenarios/trial.sh', '1'],
                         capture_output=True, text=True).stdout.splitlines()
    stops = [line.split()[1].rstrip(':') for line in out[:out.index('ON INT:')] if line.startswith('STOP ')]
    assert stops[:2] == ['scenario', 'detect-records']


def test_zoom_labels_after_freeze(tmp_path, monkeypatch):
    """The zoom scene's insight labels start after the freeze, at the scene's speed."""
    import swarm_backend.demo.final as final
    seen = []
    real = final.scene_clip
    monkeypatch.setattr(final, 'scene_clip', lambda *a, **k: (seen.append(k.get('extra', '')), real(*a, **k))[1])
    run = tmp_path / 'run'
    make_run(run, takes=1)
    (tmp_path / 'z.yaml').write_text(ZOOM.replace('  freeze: 1\n', '  freeze: 1\n  insights: [seen_at]\n'))
    build(run, tmp_path / 'z.yaml', preview=True)
    assert seen and 'between(t\\,1.000\\,' in seen[0]
