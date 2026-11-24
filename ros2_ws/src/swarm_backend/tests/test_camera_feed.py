import threading
import time
import urllib.request
from http.server import ThreadingHTTPServer
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from swarm_backend.nodes.operator_ui import camera_request, make_handler
from swarm_backend.ui.camera import Feed, debug_topic, jpeg


def image(encoding='rgb8', width=64, height=48, pad=0):
    channels = 1 if encoding == 'mono8' else 3
    step = width * channels + pad
    data = np.zeros((height, step), np.uint8)
    pixels = data[:, :width * channels].reshape(height, width, channels)
    pixels[:, :width // 2] = (255,) if channels == 1 else (255, 0, 0)  # left half red (rgb8)
    return SimpleNamespace(encoding=encoding, width=width, height=height, step=step,
                           data=data.tobytes())


def decode(body):
    return cv2.imdecode(np.frombuffer(body, np.uint8), cv2.IMREAD_COLOR)  # BGR


def test_debug_topic_matches_the_detector():
    assert debug_topic(0) == '/detection/debug'
    assert debug_topic(2) == '/px4_2/detection/debug'


@pytest.mark.parametrize('encoding, bgr', [('rgb8', (0, 0, 255)), ('bgr8', (255, 0, 0)),
                                           ('mono8', (255, 255, 255))])
def test_jpeg_keeps_the_colours_and_skips_row_padding(encoding, bgr):
    m = image(encoding, pad=4)
    out = decode(jpeg(m.encoding, m.width, m.height, m.step, m.data))
    assert out.shape == (48, 64, 3)
    assert np.allclose(out[24, 10], bgr, atol=40)
    assert np.allclose(out[24, 54], (0, 0, 0), atol=40)


def test_jpeg_scales_large_images_down():
    m = image(width=1280, height=960)
    assert decode(jpeg(m.encoding, m.width, m.height, m.step, m.data)).shape == (720, 960, 3)


def test_jpeg_rejects_other_encodings():
    with pytest.raises(ValueError):
        jpeg('32FC1', 2, 2, 8, bytes(16))


@pytest.mark.parametrize('path, expected', [
    ('/camera/2.mjpg', (2, 'mjpg')), ('/camera/11.jpg?t=5', (11, 'jpg')),
    ('/camera/x.jpg', None), ('/camera/2.png', None), ('/api/state', None)])
def test_camera_request(path, expected):
    assert camera_request(path) == expected


class FakeCameras:
    def __init__(self):
        self.feed = Feed()

    def watch(self, drone_id):
        return self.feed if drone_id == 1 else None


@pytest.fixture
def server():
    ui = SimpleNamespace(cameras=FakeCameras())
    httpd = ThreadingHTTPServer(('127.0.0.1', 0), make_handler(ui))
    httpd.daemon_threads = True
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    yield ui, f'http://127.0.0.1:{httpd.server_address[1]}'
    httpd.shutdown()


def status(url):
    try:
        return urllib.request.urlopen(url, timeout=5).status
    except urllib.error.HTTPError as e:
        return e.code


def test_snapshot_before_and_after_the_first_image(server):
    ui, base = server
    assert status(base + '/camera/7.jpg') == 404  # not a drone
    assert status(base + '/camera/1.jpg') == 404  # no image yet
    ui.cameras.feed.put(image())
    body = urllib.request.urlopen(base + '/camera/1.jpg', timeout=5).read()
    assert decode(body).shape == (48, 64, 3)


def test_stream_sends_each_new_image(server):
    ui, base = server
    ui.cameras.feed.put(image())
    res = urllib.request.urlopen(base + '/camera/1.mjpg', timeout=5)
    assert res.headers['Content-Type'].startswith('multipart/x-mixed-replace')

    def part():
        assert res.readline() == b'--frame\r\n'
        assert res.readline() == b'Content-Type: image/jpeg\r\n'
        length = int(res.readline().split(b':')[1])
        res.readline()
        body = res.read(length)
        res.readline()
        return decode(body)

    assert part().shape == (48, 64, 3)
    threading.Timer(0.2, lambda: ui.cameras.feed.put(image(width=32, height=24))).start()
    start = time.monotonic()
    assert part().shape == (24, 32, 3)
    assert time.monotonic() - start < 3
    res.close()
