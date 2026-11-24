from swarm_backend.demo.fake_run import make_run
from swarm_backend.demo.final import build, subtitles


def test_srt_format():
    srt = subtitles([(0.0, 2.0, 'Title'), (2.0, 4.5, 'Scene one')])
    assert srt == ('1\n00:00:00,000 --> 00:00:02,000\nTitle\n\n'
                   '2\n00:00:02,000 --> 00:00:04,500\nScene one\n')


def test_srt_times_with_speed(tmp_path):
    run = tmp_path / 'run'
    make_run(run, takes=1, videos=False)
    (tmp_path / 's.yaml').write_text('title: T\ntitle_seconds: 2\nscenes:\n- caption: Slow\n  from: release\n'
                                     '  to: release + 1\n  layout: full\n  show: [map]\n  speed: 0.5\n')
    build(run, tmp_path / 's.yaml', preview=True)
    srt = (run / 'preview_s.srt').read_text()
    assert '00:00:02,000 --> 00:00:04,000\nSlow' in srt

