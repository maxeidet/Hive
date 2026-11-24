import json
import subprocess

from swarm_backend.demo.fake_run import make_run
from swarm_backend.demo.final import build
from swarm_backend.demo.stills import gif, stills_pdf


def test_pdf_pages(tmp_path):
    clips = []
    for i in range(3):
        c = tmp_path / f'{i}.mp4'
        subprocess.run(['ffmpeg', '-v', 'error', '-f', 'lavfi', '-i', 'testsrc2=size=320x180:rate=25:duration=2',
                        '-pix_fmt', 'yuv420p', str(c)], check=True)
        clips.append(c)
    out = stills_pdf(clips, ['a', 'b', 'c'], tmp_path / 's.pdf')
    assert out.read_bytes().count(b'/Type /Page\n') + out.read_bytes().count(b'/Type /Page ') + \
        out.read_bytes().count(b'/Type/Page') >= 3


def test_gif_size_capped(tmp_path):
    c = tmp_path / 'c.mp4'
    subprocess.run(['ffmpeg', '-v', 'error', '-f', 'lavfi', '-i', 'testsrc2=size=1920x1080:rate=25:duration=20',
                    '-pix_fmt', 'yuv420p', str(c)], check=True)
    out, size = gif(c, tmp_path / 'c.gif', seconds=20.0, width=1920)
    info = json.loads(subprocess.run(['ffprobe', '-v', 'error', '-show_entries', 'stream=width:format=duration',
                                      '-of', 'json', str(out)], capture_output=True, text=True).stdout)
    assert info['streams'][0]['width'] == 480
    assert float(info['format']['duration']) <= 8.2
    assert size == out.stat().st_size


def test_final_stills(tmp_path):
    run = tmp_path / 'run'
    make_run(run, takes=1, videos=False)
    (tmp_path / 's.yaml').write_text('scenes:\n- caption: Approach\n  from: release\n  to: release + 2\n'
                                     '  layout: full\n  show: [map]\n- caption: The hit\n  from: hit - 1\n'
                                     '  to: hit + 1\n  layout: full\n  show: [map]\n')
    build(run, tmp_path / 's.yaml', preview=True, stills=True)
    assert (run / 'stills_s.pdf').exists()
    assert len(list(run.glob('gif_*_the-hit.gif'))) == 1
