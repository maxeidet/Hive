"""Camera feeds for the operator UI: the image RViz shows per drone, as JPEG for a browser.

The picture is the detector's /<ns>/detection/debug (the camera image with the boxes drawn
on it, detect/live.py), so a drone has a feed only while its detector runs. The UI node
subscribes to a drone's topic only while a page watches that drone (plus WATCH_GRACE_S), so
an operator UI nobody looks at costs the simulation no image copies.

No ROS here except in CameraFeeds, which gets the node; jpeg() is plain numpy and OpenCV.
"""
import threading
import time

import cv2
import numpy as np

from swarm_backend.config import default_namespace

WATCH_GRACE_S = 5.0  # keep subscribed this long after the last request (page reloads, tab hops)
MAX_WIDTH = 960  # larger images are scaled down before encoding: the panel is smaller anyway
JPEG_QUALITY = 75


def debug_topic(drone_id: int) -> str:
    """The detector's annotated image (as live.offset_topic(drone_id, 'debug'))."""
    ns = default_namespace(drone_id)
    return f'/{ns}/detection/debug' if ns else '/detection/debug'


def jpeg(encoding: str, width: int, height: int, step: int, data: bytes,
         max_width: int = MAX_WIDTH, quality: int = JPEG_QUALITY) -> bytes:
    """A sensor_msgs/Image's fields (rgb8, bgr8 or mono8) as JPEG bytes."""
    channels = {'rgb8': 3, 'bgr8': 3, 'mono8': 1}.get(encoding)
    if channels is None:
        raise ValueError(f'Unsupported image encoding {encoding}')
    rows = np.frombuffer(data, np.uint8).reshape(height, step)
    img = rows[:, :width * channels].reshape(height, width, channels)
    if encoding == 'rgb8':
        img = img[:, :, ::-1]  # OpenCV writes BGR
    if width > max_width:
        img = cv2.resize(img, (max_width, round(height * max_width / width)),
                         interpolation=cv2.INTER_AREA)
    ok, out = cv2.imencode('.jpg', np.ascontiguousarray(img), [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise ValueError('JPEG encoding failed')
    return out.tobytes()


class Feed:
    """The newest image of one drone, encoded the first time someone asks for it."""

    def __init__(self):
        self.msg = None
        self.received = None  # time.monotonic() of the newest image
        self.seq = 0  # counts images, so a stream sends each one once
        self.wanted = -1e9  # time.monotonic() of the last request
        self.sub = None
        self._jpeg = (None, b'')  # (seq, bytes)
        self.changed = threading.Condition()

    def put(self, msg) -> None:
        with self.changed:
            self.msg, self.received = msg, time.monotonic()
            self.seq += 1
            self.changed.notify_all()

    def frame(self) -> tuple[int, bytes] | None:
        """(seq, JPEG) of the newest image, or None before the first one."""
        with self.changed:
            msg, seq = self.msg, self.seq
        if msg is None:
            return None
        if self._jpeg[0] != seq:
            self._jpeg = (seq, jpeg(msg.encoding, msg.width, msg.height, msg.step,
                                    bytes(msg.data)))
        return self._jpeg

    def wait(self, after_seq: int, timeout: float) -> None:
        with self.changed:
            self.changed.wait_for(lambda: self.seq != after_seq, timeout)


class CameraFeeds:
    """One Feed per drone. watch() is called from the web server's threads; the
    subscriptions are made and dropped on the node's own (spin) thread, by a timer."""

    def __init__(self, node, drone_ids):
        from sensor_msgs.msg import Image
        self._node, self._image = node, Image
        self.feeds = {i: Feed() for i in drone_ids}
        node.create_timer(0.5, self._sync)

    def watch(self, drone_id: int) -> Feed | None:
        feed = self.feeds.get(drone_id)
        if feed is not None:
            feed.wanted = time.monotonic()
        return feed

    def age_s(self, drone_id: int, now: float) -> float | None:
        """Seconds since the newest image, None if none (or nobody watches the drone)."""
        feed = self.feeds.get(drone_id)
        if feed is None or feed.sub is None or feed.received is None:
            return None
        return now - feed.received

    def _sync(self) -> None:
        from rclpy.qos import qos_profile_sensor_data
        now = time.monotonic()
        for drone_id, feed in self.feeds.items():
            watched = now - feed.wanted < WATCH_GRACE_S
            if watched and feed.sub is None:
                feed.sub = self._node.create_subscription(
                    self._image, debug_topic(drone_id), feed.put, qos_profile_sensor_data)
            elif not watched and feed.sub is not None:
                self._node.destroy_subscription(feed.sub)
                feed.sub = None
                with feed.changed:
                    feed.msg = feed.received = None  # an old picture must not look live later
