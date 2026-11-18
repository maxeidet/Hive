"""Recording the operator UI on Gazebo's clock, and playing it back for the demo videos.

During a run, `mount-backend.sh ui --record` writes what the page would get from
/api/state, a few times a second, to a JSON-lines file, each line stamped with Gazebo's
simulation time (Recorder). Afterwards `recording/video.sh ui` serves that file as if it
were the live UI node (ReplayUi, same /api/state) and renders the page frame by frame at
exactly the simulation times of the Gazebo replay clips (ui/render.py), so the two videos
line up to the frame.

One line of the file:
    {"sim": 312.48, "wall": 1763460123.2, "state": {... /api/state without "map" ...}}
`state.events` holds only the events that are new since the line before.

Nothing here imports ROS: the replay and the renderer run without a simulation.
"""
import bisect
import json
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
        state.pop('map', None)  # the same every time; the replay serves the current one
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
    """Answers /api/state from a recording, at a simulation time the page asks for.

    The page in replay mode (index.html?replay=<start sim time>) adds `t=<sim time>` to
    each poll, from its own (virtual, see render.py) clock. The answer is the last recorded
    state at or before t, with every recorded event up to t, and T+ counted on sim time
    from the start of the recording.
    """

    def __init__(self, lines: list[dict]):
        self.lines = lines
        self.sims = [line['sim'] for line in lines]
        self.start = self.sims[0]
        self.events = []  # (sim time it was recorded at, event dict with t on sim time)
        for line in lines:
            for event in line['state'].get('events') or []:
                self.events.append((line['sim'], dict(event, t=round(line['sim'] - self.start, 1))))
        self.events.sort(key=lambda pair: pair[1]['id'])
        self.map = None  # load_map(...) of the current static folder, set by the server

    def at(self, sim: float) -> dict:
        """The recorded line shown at simulation time `sim` (the first one before the
        recording starts)."""
        i = bisect.bisect_right(self.sims, sim) - 1
        return self.lines[max(i, 0)]

    def state(self, events_after: int = 0, sim: float | None = None) -> dict:
        sim = self.start if sim is None else sim
        line = self.at(sim)
        state = dict(line['state'])
        state['events'] = [e for at, e in self.events if at <= sim and e['id'] > events_after]
        state['uptime_s'] = round(max(0.0, sim - self.start), 1)
        state['map'] = self.map
        state['replay'] = {'sim': round(sim, 3), 'recorded_sim': line['sim'],
                           'wall': line.get('wall')}
        return state

    def set_search_area(self, data: dict):
        raise ValueError('This is a replay: the search area cannot be changed')

    def wall_at(self, sim: float) -> float | None:
        """The computer's clock (Unix seconds) when `sim` was recorded, for the page's clock."""
        line = self.at(sim)
        if line.get('wall') is None:
            return None
        return line['wall'] + max(0.0, sim - line['sim'])

    def event_table(self) -> list[tuple[float, float, dict]]:
        """(absolute sim time, T+ in the UI, event) for every event, to pick a start time."""
        return [(at, e['t'], e) for at, e in self.events]


def sim_query(path: str) -> float | None:
    """The `t` query parameter of a request path (the page's sim time), None if missing."""
    from urllib.parse import parse_qs, urlsplit
    try:
        return float(parse_qs(urlsplit(path).query)['t'][0])
    except (KeyError, ValueError, IndexError):
        return None


def frame_times(start: float, duration: float, fps: float) -> list[float]:
    """Simulation time of each video frame: frame i shows start + i / fps, the same rule as
    the Gazebo recorder on sim time (recording/README.md)."""
    if duration <= 0 or fps <= 0:
        raise ValueError('duration and fps must be positive')
    return [start + i / fps for i in range(int(round(duration * fps)))]
