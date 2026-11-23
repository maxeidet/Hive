"""The operator UI's recording on Gazebo's clock, for the demo videos (recording/README.md).

During a run, `mount-backend.sh ui --record` writes what the page gets from /api/state, a
few times a second, to a JSON-lines file, each line stamped with Gazebo's simulation time
and the computer's clock (Recorder). Afterwards it is read back (ReplayUi): to put the
screen film of the UI on sim time (recording/screen.py), for the labels on the live camera
videos (ui/camera_video.py), and to list the events with their sim times:

    python3 -m swarm_backend.ui.replay data/recordings/<run>/ui.jsonl    (video.sh events)

One line of the file:
    {"sim": 312.48, "wall": 1763460123.2, "state": {... /api/state without "map" ...}}
`state.events` holds only the events that are new since the line before.

Nothing here imports ROS.
"""
import bisect
import json
import sys
from pathlib import Path


class Recorder:
    """Appends UI states to a .jsonl file. `state_fn(events_after)` is OperatorUi.state."""

    def __init__(self, path: Path, state_fn):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = open(self.path, 'a', buffering=1)  # line buffered: a crash loses one line
        self._state = state_fn
        self._last_event = 0
        self.lines = 0

    def record(self, sim: float | None, wall: float) -> bool:
        """Writes one line; nothing before Gazebo's clock runs (sim None)."""
        if sim is None:
            return False
        state = self._state(self._last_event)
        state.pop('map', None)  # the same every time
        if state.get('events'):
            self._last_event = max(e['id'] for e in state['events'])
        self._file.write(json.dumps({'sim': round(sim, 3), 'wall': round(wall, 3),
                                     'state': state}, separators=(',', ':')) + '\n')
        self.lines += 1
        return True

    def close(self) -> None:
        self._file.close()


def load_recording(path: Path) -> list[dict]:
    """The lines of a recording, sorted by simulation time. A torn last line (the UI was
    killed mid-write) is skipped. Gazebo's clock only goes forward within one run, but a
    restart of Gazebo during a recording starts it at 0 again: then only the part after the
    last restart is kept, since sim times would repeat."""
    lines = []
    with open(path) as fh:
        for raw in fh:
            try:
                line = json.loads(raw)
            except json.JSONDecodeError:
                continue
            if lines and line['sim'] + 1.0 < lines[-1]['sim']:
                lines = []  # Gazebo restarted
            lines.append(line)
    if not lines:
        raise ValueError(f'{path} has no recorded UI states')
    lines.sort(key=lambda line: line['sim'])
    return lines


class ReplayUi:
    """A recording read back: the state shown at any sim time, and the events on sim time."""

    def __init__(self, lines: list[dict]):
        self.lines = lines
        self.sims = [line['sim'] for line in lines]
        self.start = self.sims[0]
        self.events = []  # (sim time it was recorded at, event dict with t on sim time)
        for line in lines:
            for event in line['state'].get('events') or []:
                self.events.append((line['sim'], dict(event, t=round(line['sim'] - self.start, 1))))
        self.events.sort(key=lambda pair: pair[1]['id'])

    def at(self, sim: float) -> dict:
        """The recorded line shown at simulation time `sim` (the first one before the
        recording starts)."""
        i = bisect.bisect_right(self.sims, sim) - 1
        return self.lines[max(i, 0)]

    def event_table(self) -> list[tuple[float, float, dict]]:
        """(absolute sim time, T+ from the start of the recording, event) for every event."""
        return [(at, e['t'], e) for at, e in self.events]


def list_events(replay: ReplayUi) -> None:
    end = replay.sims[-1]
    print(f'Recording: sim time {replay.start:.1f} to {end:.1f} s '
          f'({end - replay.start:.0f} s, {len(replay.sims)} states)')
    print(f'{"sim time":>9}  {"T+":>7}  event')
    for sim, t, e in replay.event_table():
        detail = f'  ({e["detail"]})' if e.get('detail') else ''
        print(f'{sim:9.1f}  {t:7.1f}  {e["text"]}{detail}')
    print('Use a sim time as --start: a few seconds before what you want to show.')


def main(argv: list[str] | None = None) -> int:
    args = sys.argv[1:] if argv is None else argv
    if len(args) != 1 or args[0] in ('-h', '--help'):
        print(__doc__)
        return 0 if args and args[0] in ('-h', '--help') else 1
    path = Path(args[0])
    list_events(ReplayUi(load_recording(path / 'ui.jsonl' if path.is_dir() else path)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
