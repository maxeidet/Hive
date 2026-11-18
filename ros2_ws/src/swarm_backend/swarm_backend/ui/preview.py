"""Offline preview of the operator UI: the real page and HTTP handler, but no ROS and no
simulation. A few made-up drones circle the base, and the search sector and the map area
can be set as in the real UI (they are only kept in memory). For working on the page
without the container.

Run on the host, from ros2_ws/src/swarm_backend:

    python3 -m swarm_backend.ui.preview [--port 8091] [--drones 3]

Then open http://localhost:8091. The background map is ui/static/map.png if there is one.
"""
import argparse
import math
import sys
import threading
import time
import types
from http.server import ThreadingHTTPServer

try:
    import yaml  # noqa: F401  (operator_ui -> config imports it; the preview never reads a config)
except ImportError:
    sys.modules['yaml'] = types.ModuleType('yaml')

from swarm_backend.nodes.operator_ui import STATIC_DIR, make_handler  # noqa: E402
from swarm_backend.ui.model import (EventLog, MapArea, SearchArea, load_map,  # noqa: E402
                                   update_map_area, update_search_area)


class PreviewUi:
    """Stands in for OperatorUi: same state() and setters, made-up drones."""

    def __init__(self, drones: int):
        self.area = SearchArea(45.0, 90.0, 150.0)
        self.map_area = MapArea()
        self.n = drones
        self.events = EventLog()
        self.started = time.monotonic()
        self._lock = threading.Lock()
        for i in range(drones):
            self.events.add(0.0, 'fleet', f'Drone {i} taking off', drone=i)

    def _drone(self, i: int, t: float) -> dict:
        radius, speed = 30.0 + 25.0 * i, 3.0
        a = t * speed / radius + i * 2 * math.pi / max(self.n, 1)
        heading = (math.degrees(a) + 90.0) % 360.0  # flying round, anticlockwise seen from above
        return {'id': i, 'north': radius * math.cos(a), 'east': -radius * math.sin(a),
                'altitude': 15.0 + 3 * i, 'speed': speed, 'heading_deg': (360.0 - heading) % 360.0,
                'gimbal_heading_deg': (self.area.heading_deg + 40 * math.sin(t / 3 + i)) % 360.0,
                'gimbal_pitch_deg': -20.0, 'age_s': 0.1, 'armed': True, 'nav_state': 'OFFBOARD',
                'status': 'searching', 'hostile': False, 'attack': None, 'detection': None,
                'attack_state': None}

    def state(self, events_after: int = 0) -> dict:
        t = time.monotonic() - self.started
        with self._lock:
            return {'search_area': self.area.to_dict(), 'map_area': self.map_area.to_dict(),
                    'drones': [self._drone(i, t) for i in range(self.n)],
                    'map': load_map(STATIC_DIR), 'events': self.events.after(events_after),
                    'uptime_s': round(t, 1)}

    def set_search_area(self, data: dict) -> SearchArea:
        with self._lock:
            self.area = update_search_area(self.area, data)
            self.events.add(time.monotonic() - self.started, 'operator',
                            f'Search sector revision {self.area.revision}')
            return self.area

    def set_map_area(self, data: dict) -> MapArea:
        with self._lock:
            self.map_area = m = update_map_area(self.map_area, data)
            self.events.add(time.monotonic() - self.started, 'operator',
                            f'Map area revision {m.revision}' if m.shape else 'Map area cleared',
                            detail=m.describe())
            return m


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--port', type=int, default=8091)
    parser.add_argument('--drones', type=int, default=3, help='made-up drones circling the base')
    args = parser.parse_args(argv)
    server = ThreadingHTTPServer(('127.0.0.1', args.port), make_handler(PreviewUi(args.drones)))
    server.daemon_threads = True
    print(f'Operator UI preview (no ROS) on http://localhost:{args.port}. Ctrl-C stops it.',
          flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
