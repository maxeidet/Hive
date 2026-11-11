"""Operator UI (`mount-backend.sh ui`): a web page that shows where the drones are and
where their gimbal cameras look, and lets the operator set the search area.

It never commands a drone. It only listens to the PX4 topics of every drone in
swarm.yaml (and their detection and attack state and status topics), and publishes the
search area on /base/search_area for whoever plans the search (std_msgs/String, JSON with
the fields of ui.model.SearchArea, latched: a node that starts later still gets the last
one).

    python3 -m swarm_backend.nodes.operator_ui --config <swarm.yaml> [--port 8090]

Open http://localhost:8090 on the host (the container publishes the port).
"""
import argparse
import json
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from swarm_backend.config import DroneConfig, default_config_path, load_config
from swarm_backend.ui.model import (SearchArea, attack_view, drone_status, drone_view,
                                   load_map, update_search_area)

SEARCH_AREA_TOPIC = '/base/search_area'
STATIC_DIR = Path(__file__).resolve().parent.parent / 'ui' / 'static'
DETECTION_FRESH_S = 2.0  # a detection older than this is not shown
ATTACK_STATE_FRESH_S = 3.0  # the attack node repeats its state every second
STALE_S = 2.0  # no PX4 message for this long: status 'no link' (as STALE_S in index.html)


class DroneMonitor:
    """Read-only view of one PX4 drone: odometry, status, gimbal and its detections."""

    def __init__(self, node, config: DroneConfig):
        from geometry_msgs.msg import PointStamped
        from std_msgs.msg import String
        from px4_msgs.msg import GimbalDeviceAttitudeStatus, VehicleOdometry, VehicleStatus

        from swarm_backend.intercept.attack import STATE_TOPIC, STATUS_TOPIC
        from swarm_backend.intercept.detection import POSITION_TOPIC, topic
        from swarm_backend.px4.px4_drone import PX4_OUT_QOS
        from swarm_backend.px4.topics import PX4Topics

        self.id = config.id
        self.home = config.spawn_ned
        self._earth_flag = GimbalDeviceAttitudeStatus.DEVICE_FLAGS_YAW_IN_EARTH_FRAME
        self._nav_names = {getattr(VehicleStatus, n): n[len('NAVIGATION_STATE_'):]
                           for n in dir(VehicleStatus) if n.startswith('NAVIGATION_STATE_')
                           and n != 'NAVIGATION_STATE_MAX'}
        self._armed_value = VehicleStatus.ARMING_STATE_ARMED
        self.position = self.velocity = self.attitude = None
        self.gimbal_q = None
        self.gimbal_earth_frame = False
        self.armed = self.nav_state = None
        self.last_heard = None  # time.monotonic() of the last PX4 message
        self.detection = None  # (target relative to the drone, world NED; time.monotonic())
        self.attack_state = None  # (label from the attack node, time.monotonic())
        self.attack_status = None  # (attack_view of its status, time.monotonic())

        topics = PX4Topics(config.namespace)
        node.create_subscription(VehicleOdometry, topics.vehicle_odometry, self._on_odometry,
                                 PX4_OUT_QOS)
        node.create_subscription(VehicleStatus, topics.vehicle_status, self._on_status,
                                 PX4_OUT_QOS)
        node.create_subscription(GimbalDeviceAttitudeStatus, topics.gimbal_attitude,
                                 self._on_gimbal, PX4_OUT_QOS)
        node.create_subscription(PointStamped, topic(config.id, POSITION_TOPIC),
                                 self._on_detection, 10)
        node.create_subscription(String, topic(config.id, STATE_TOPIC), self._on_attack_state, 10)
        node.create_subscription(String, topic(config.id, STATUS_TOPIC), self._on_attack_status,
                                 10)

    def _on_odometry(self, msg) -> None:
        local = [float(v) for v in msg.position]
        if any(v != v for v in local):  # NaN until PX4 has a position
            return
        hx, hy, hz = self.home
        self.position = (local[0] + hx, local[1] + hy, local[2] + hz)
        self.velocity = tuple(float(v) for v in msg.velocity)
        self.attitude = tuple(float(v) for v in msg.q)
        self.last_heard = time.monotonic()

    def _on_status(self, msg) -> None:
        self.armed = msg.arming_state == self._armed_value
        self.nav_state = self._nav_names.get(msg.nav_state, str(msg.nav_state))
        self.last_heard = time.monotonic()

    def _on_gimbal(self, msg) -> None:
        q = tuple(float(v) for v in msg.q)
        if any(v != v for v in q):
            return
        self.gimbal_q = q
        self.gimbal_earth_frame = bool(msg.device_flags & self._earth_flag)

    def _on_detection(self, msg) -> None:
        self.detection = ((msg.point.x, msg.point.y, msg.point.z), time.monotonic())

    def _on_attack_state(self, msg) -> None:
        self.attack_state = (msg.data, time.monotonic())

    def _on_attack_status(self, msg) -> None:
        status = attack_view(msg.data)
        if status is not None:
            self.attack_status = (status, time.monotonic())

    def view(self, now: float) -> dict:
        age = None if self.last_heard is None else now - self.last_heard
        view = drone_view(self.id, self.position, self.velocity, self.attitude, self.gimbal_q,
                          self.gimbal_earth_frame, self.armed, self.nav_state, age)
        view['detection'] = None
        view['attack_state'] = None  # None: no attack node running on this drone
        view['attack'] = None
        if self.attack_status is not None and now - self.attack_status[1] < ATTACK_STATE_FRESH_S:
            view['attack'] = self.attack_status[0]
        if self.attack_state is not None and now - self.attack_state[1] < ATTACK_STATE_FRESH_S:
            view['attack_state'] = self.attack_state[0]
        if self.detection is not None and self.position is not None:
            (dn, de, dd), seen = self.detection
            if now - seen < DETECTION_FRESH_S:
                view['detection'] = {'north': round(self.position[0] + dn, 2),
                                     'east': round(self.position[1] + de, 2),
                                     'altitude': round(-(self.position[2] + dd), 2)}
        view['status'] = drone_status(view['attack_state'], self.armed, self.nav_state, age,
                                      STALE_S)
        return view


class OperatorUi:
    """The drone monitors and the search area publisher, on one ROS node.

    ROS callbacks run on the spin thread and the web server on its own threads; reads of
    the monitors' fields are single attribute loads, so they need no lock. The search area
    does, since a change is read-modify-write.
    """

    def __init__(self, node, drones: tuple[DroneConfig, ...], area: SearchArea,
                 hostile_id: int | None = None):
        """`hostile_id`: the drone that plays the target in the simulation (attack.target_id);
        the page draws it as a hostile."""
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String

        self._string = String
        self.monitors = [DroneMonitor(node, d) for d in drones]
        self.hostile_id = hostile_id
        self._lock = threading.Lock()
        self.area = area
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._area_pub = node.create_publisher(String, SEARCH_AREA_TOPIC, latched)
        self._log = node.get_logger()
        self._publish_area()

    def state(self) -> dict:
        now = time.monotonic()
        with self._lock:
            area = self.area.to_dict()
        views = [m.view(now) for m in self.monitors]
        for view in views:
            view['hostile'] = view['id'] == self.hostile_id
        return {'search_area': area, 'drones': views, 'map': load_map(STATIC_DIR)}

    def set_search_area(self, data: dict) -> SearchArea:
        with self._lock:
            self.area = update_search_area(self.area, data)
            self._publish_area()
            area = self.area
        self._log.info(f'Search area {area.revision}: {area.heading_deg:.0f} deg '
                       f'± {area.width_deg / 2:.0f}, {area.radius_m:.0f} m')
        return area

    def _publish_area(self) -> None:
        self._area_pub.publish(self._string(data=json.dumps(self.area.to_dict())))


def make_handler(ui: OperatorUi):

    class Handler(BaseHTTPRequestHandler):

        def do_GET(self):
            if self.path in ('/', '/index.html'):
                self._send(200, (STATIC_DIR / 'index.html').read_bytes(), 'text/html')
            elif self.path.split('?')[0] == '/map.png' and (STATIC_DIR / 'map.png').is_file():
                self._send(200, (STATIC_DIR / 'map.png').read_bytes(), 'image/png')
            elif self.path == '/api/state':
                self._json(200, ui.state())
            else:
                self._json(404, {'error': 'not found'})

        def do_POST(self):
            if self.path != '/api/search_area':
                self._json(404, {'error': 'not found'})
                return
            try:
                length = int(self.headers.get('Content-Length', 0))
                data = json.loads(self.rfile.read(length) or b'{}')
                if not isinstance(data, dict):
                    raise ValueError('Expected a JSON object')
                area = ui.set_search_area(data)
            except ValueError as e:  # json.JSONDecodeError is a ValueError
                self._json(400, {'error': str(e)})
                return
            self._json(200, area.to_dict())

        def _json(self, code: int, body) -> None:
            self._send(code, json.dumps(body).encode(), 'application/json')

        def _send(self, code: int, body: bytes, content_type: str) -> None:
            self.send_response(code)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):  # 5 polls a second would flood the terminal
            pass

    return Handler


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--config', default=None, help='swarm.yaml (default: the installed one)')
    parser.add_argument('--port', type=int, default=8090)
    parser.add_argument('--heading', type=float, default=None,
                        help='first search heading, degrees from north (default: the attack '
                             'section\'s search_heading_deg)')
    parser.add_argument('--width', type=float, default=90.0, help='first arc width, degrees')
    parser.add_argument('--radius', type=float, default=150.0, help='first arc radius, metres')
    args = parser.parse_args(argv)

    config = load_config(args.config or default_config_path())
    heading = config.attack.search_heading_deg if args.heading is None else args.heading
    area = update_search_area(SearchArea(0.0, 90.0, 100.0, revision=-1),
                              {'heading_deg': heading, 'width_deg': args.width,
                               'radius_m': args.radius})

    import rclpy

    rclpy.init()
    node = rclpy.create_node('operator_ui')
    ui = OperatorUi(node, config.drones, area, hostile_id=config.attack.target_id)
    server = ThreadingHTTPServer(('0.0.0.0', args.port), make_handler(ui))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f'Operator UI on http://localhost:{args.port} (drones '
          f'{", ".join(str(d.id) for d in config.drones)}). Ctrl-C stops it.', flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
