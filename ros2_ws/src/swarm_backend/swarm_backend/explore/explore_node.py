"""Exploration node (`mount-backend.sh explorenode`): maps the area it is told to with DAEP.

Waits for commands on /explore/command (std_msgs/String, JSON, see explore/area.py). A start
command with an area runs launch/daep_exploration.launch.py for the box around it: the drone
takes off, flies to the area, DAEP explores it, the map is saved as data/map.bt and the
drone lands. Then the launch is stopped and the node waits for the next area. A stop command
ends the exploration early; the bridge lands the drone. One area at a time.

What it is doing goes out on /explore/status (latched, so a UI that starts later still gets
it): the state, the box, DAEP's coverage of it and the saved map. Each run's launch output is
in /tmp/explore_run_<N>.log.

DAEP is LiU's private code: the daep and daep_msgs forks must be built in the workspace.

    python3 -m swarm_backend.explore.explore_node [--id 0] [--map-range 8]
    ros2 topic pub --once /explore/command std_msgs/msg/String \\
        "{data: '{\\"command\\": \\"start\\", \\"area\\": {\\"circle\\": {\\"centre\\": [-14, 13.5], \\"radius_m\\": 7}}}'}"
"""
import argparse
import os
import re
import signal
import subprocess
import sys
import threading
from dataclasses import replace
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from swarm_backend.explore.area import (
    COMMAND_TOPIC, DONE, EXPLORING, FAILED, FINISHED, LANDING, STARTING, STATUS_TOPIC, STOPPED,
    STOPPING, Command, Progress, Status, last_coverage, parse_command)
from swarm_backend.runner import spinning

ANSI = re.compile(r'\x1b\[[0-9;]*m')
LAND_WAIT_S = 240.0  # for the bridge to land the drone when the launch is stopped
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


class ExploreNode:
    def __init__(self, node: Node, drone_id: int, map_range: float, workdir: Path):
        self.node, self.log = node, node.get_logger()
        self.drone_id, self.map_range, self.workdir = drone_id, map_range, workdir
        self.progress = Progress()
        self.proc: subprocess.Popen | None = None
        self._lock = threading.Lock()
        self._sent: str | None = None
        self._pub = node.create_publisher(String, STATUS_TOPIC, LATCHED)
        node.create_subscription(String, COMMAND_TOPIC, self._on_command, 10)
        node.create_timer(1.0, self._tick)
        self._publish()
        self.log.info(f'Waiting for areas on {COMMAND_TOPIC}; status on {STATUS_TOPIC}')

    # --- commands ----------------------------------------------------------

    def _on_command(self, msg: String) -> None:
        try:
            command = parse_command(msg.data)
        except ValueError as e:
            self.log.warning(f'Bad command: {e}')
            self._set(detail=f'Bad command: {e}')
            return
        if command.kind == 'start':
            self.start(command)
        else:
            self.stop()

    def start(self, command: Command) -> None:
        with self._lock:
            status = self.progress.status
            if status.state not in FINISHED:
                self._set_locked(detail='Busy: stop the current exploration first')
                return
            run = status.run + 1
            self.progress = Progress(Status(STARTING, box=command.box, run=run))
        data = self.workdir / 'data'
        for old in ('coverage.csv', 'map.bt'):  # so the status never shows the last run's
            (data / old).unlink(missing_ok=True)
        cmd = ['ros2', 'launch', 'swarm_backend', 'daep_exploration.launch.py',
               *command.box.launch_args(), f'drone_id:={self.drone_id}',
               f'map_range:={self.map_range:g}']
        self.log.info(f'Run {run}: {" ".join(cmd[4:])}')
        self.proc = subprocess.Popen(
            cmd, cwd=self.workdir, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
            bufsize=1, env={**os.environ, 'PYTHONUNBUFFERED': '1'},
            start_new_session=True)  # Ctrl-C here must not reach it: we stop it ourselves
        threading.Thread(target=self._read, args=(self.proc, run), daemon=True).start()
        self._publish()

    def stop(self) -> None:
        with self._lock:
            if self.progress.status.state in FINISHED:
                return
            self.progress.status = replace(self.progress.status, state=STOPPING,
                                           detail='Stopped by the operator; landing')
        self.log.info('Stopping the exploration; the bridge lands the drone')
        self._interrupt()
        self._publish()

    # --- following the launch ----------------------------------------------

    def _read(self, proc: subprocess.Popen, run: int) -> None:
        with open(f'/tmp/explore_run_{run}.log', 'w') as log:
            for raw in proc.stdout:
                line = ANSI.sub('', raw)
                log.write(line)
                with self._lock:
                    self.progress.line(line)
                    done = self.progress.finished_flying
                    if done:
                        self.progress.status = replace(self.progress.status, state=DONE,
                                                       detail='Map saved, drone landed')
                if done:
                    self.log.info(f'Run {run} done: {self.progress.status.map_path}')
                    self._interrupt()  # DAEP's nodes keep running otherwise
        proc.wait()
        with self._lock:
            s = self.progress.status
            if s.state == STOPPING:
                self.progress.status = replace(s, state=STOPPED, detail='Stopped; drone landed')
            elif s.state not in FINISHED:
                self.progress.status = replace(
                    s, state=FAILED, detail=f'The launch ended (code {proc.returncode}), '
                                            f'see /tmp/explore_run_{run}.log')
        self._publish()

    def _interrupt(self) -> None:
        if self.proc is not None and self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)

    def _tick(self) -> None:
        if self.progress.status.state in (EXPLORING, LANDING):
            try:
                coverage = last_coverage((self.workdir / 'data' / 'coverage.csv').read_text())
            except OSError:
                coverage = None
            if coverage is not None:
                self._set(coverage_pct=coverage)
        self._publish()

    # --- status ------------------------------------------------------------

    def _set(self, **changes) -> None:
        with self._lock:
            self._set_locked(**changes)

    def _set_locked(self, **changes) -> None:
        self.progress.status = replace(self.progress.status, **changes)

    def _publish(self) -> None:
        text = self.progress.status.to_json()
        if text != self._sent:
            self._sent = text
            self._pub.publish(String(data=text))
            s = self.progress.status
            self.log.info(f'Status: {s.state}' + (f' ({s.detail})' if s.detail else '')
                          + (f', {s.coverage_pct} %' if s.coverage_pct is not None else ''))

    def shutdown(self) -> None:
        """Stop a running exploration and wait for the drone to land."""
        if self.proc is not None and self.proc.poll() is None:
            self.stop()
            try:
                self.proc.wait(timeout=LAND_WAIT_S)
            except subprocess.TimeoutExpired:
                self.log.error('The launch did not stop; killing it')
                self.proc.kill()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--id', type=int, default=0, help='PX4 instance of the mapping drone')
    parser.add_argument('--map-range', type=float, default=8.0,
                        help='metres the map takes points from the sensor (and DAEP counts gain '
                             'to); the depth camera sees 19.1 m')
    parser.add_argument('--workdir', type=Path, default=Path.cwd(),
                        help='where data/ (map.bt, coverage.csv) is written (default: here)')
    args = parser.parse_args()

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('explore_node')
    explore = ExploreNode(node, args.id, args.map_range, args.workdir.resolve())

    stopping = threading.Event()

    def stop(*_):
        if not stopping.is_set():  # a second Ctrl-C must not cut the landing short
            stopping.set()
            raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)

    with spinning(node):
        try:
            while not stopping.wait(0.5):
                pass
        except KeyboardInterrupt:
            pass
        explore.shutdown()
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
