"""Takes, named moments and facts of a recorded run (recording/README.md, `video.sh moments`).

    python3 -m swarm_backend.demo.takes data/recordings/<run>

A recording may hold several engagements (`trial.sh 3`): each is a take, from the hostile
being put in place (a jump of more than TELEPORT_M between two pose rows, not into the
graveyard) until the next one; a placement after which the hostile never flies (put back on
its spawn point when the scenario ends) is no take. The hit is the hostile's jump into the
graveyard (the Gazebo scenarios log no "hit the target" event: the patrol is called back
first), the attacker the defender that went to intercept. Within a take the named moments are found from the UI events
(ui.jsonl) and the true poses (poses.jsonl); every number about the take (the facts) is
computed once here and saved in takes.json, for the film, the ranking and the cards.
"""
import json
import math
import re
import statistics
import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path

from swarm_backend.demo.runinfo import read_run_info
from swarm_backend.scenarios.common import REPO
from swarm_backend.ui.camera_video import load_poses, pose_at
from swarm_backend.ui.replay import ReplayUi, load_recording

MOMENTS = ('start', 'release', 'takeoff', 'first_sighting', 'intercept', 'hit', 'return',
           'landed', 'end',
           # two hostiles at once (baseline_eval --hostiles 2), in run.json's hostiles order;
           # then `hit` is when the last one is down and `first_sighting` the earlier sighting
           'first_sighting_1', 'first_sighting_2', 'hit_1', 'hit_2')
TELEPORT_M = 50.0         # a jump this big between two pose rows: put in place
GRAVEYARD_M = 30.0        # this close (x, y) to graveyard_enu: taken out
RELEASE_MPS = 0.5         # moving at least this fast horizontally (not the climb)...
RELEASE_HOLD_S = 1.0      # ...for this long: released
HIT_M = 2.5               # a defender this close (attack.hit_radius 1.5 m + pose lag)...
HIT_WINDOW_S = 0.5        # ...in this long before the hostile is taken out: a hit
CONTACT_LOOKBACK_S = 30.0  # taken out later than that (a collision, slow clean-up): the contact counts
CONFIG = REPO / 'recording' / 'recording_config.json'

SIGHTING = re.compile(r'^Drone (\d+) sees the target')
INTERCEPT = re.compile(r'^Drone (\d+) .*→ intercept$')
HIT = re.compile(r'^Drone (\d+) hit the target')
RETURN = re.compile(r'^Drone (\d+) back to searching')
ARMED = re.compile(r'^Drone (\d+) armed$')
LANDED = re.compile(r'^Drone (\d+) landed and disarmed')


@dataclass
class Take:
    number: int
    moments: dict[str, float]
    attacker: int | None
    target: str
    facts: dict = field(default_factory=dict)
    folder: Path | None = None
    name: str = ''      # '<run id>/<k>' in a shoot, '<k>' in a single run
    pair: str | None = None  # the baseline/ours pair this run belongs to (shoot compare:)


@dataclass
class TakeData:
    """What a fact function gets: the take, the run's poses, roles and events."""
    take: Take
    sims: list[float]
    rows: list[dict]
    defenders: list[str]
    setup: str | None
    folder: Path | None = None
    ui: ReplayUi | None = None

    def pos(self, name: str, sim: float):
        p = pose_at(self.sims, self.rows, name, sim)
        return None if p is None else p[0]

    def attacker_name(self) -> str | None:
        a = self.take.attacker
        return next((d for d in self.defenders if d.endswith(f'_{a}')), None) if a else None


def graveyard() -> tuple[float, float]:
    try:
        return tuple(json.loads(CONFIG.read_text())['graveyard_enu'][:2])
    except (OSError, KeyError, ValueError):
        return (450.0, -450.0)


def in_graveyard(p, grave) -> bool:
    return p is not None and math.hypot(p[0] - grave[0], p[1] - grave[1]) < GRAVEYARD_M


# ---- facts: one function each; a new fact is one more entry ---------------------------------

def _gap(d: TakeData, a: str, b: str):
    m = d.take.moments
    return round(m[b] - m[a], 2) if a in m and b in m else None


def fact_sighting_range_m(d: TakeData):
    name, s = d.attacker_name(), d.take.moments.get('first_sighting')
    if name is None or s is None:
        return None
    a, h = d.pos(name, s), d.pos(d.take.target, s)
    return None if a is None or h is None else round(math.dist(a, h), 1)


def fact_closest_pass_m(d: TakeData):
    name, m = d.attacker_name(), d.take.moments
    if name is None or 'release' not in m:
        return None
    until = m.get('hit', m['end'])
    grave = graveyard()
    best = None
    for sim, row in zip(d.sims, d.rows):
        if sim < m['release'] or sim > until or name not in row or d.take.target not in row:
            continue
        h = row[d.take.target][:3]
        if in_graveyard(h, grave):
            continue
        dist = math.dist(row[name][:3], h)
        best = dist if best is None else min(best, dist)
    return None if best is None else round(best, 2)


def fact_in_frame_frac(d: TakeData):
    """How much of the engagement the chase camera (cam_attacker_watch) kept the target in
    its picture: the share of 0.2 s samples from first sighting to hit (or the take's end)."""
    if d.folder is None or not (d.folder / 'cam_attacker_watch.json').exists():
        return None
    from swarm_backend.ui.camera_video import project
    meta = json.loads((d.folder / 'cam_attacker_watch.json').read_text())
    cam = meta.get('model', 'rec_cam_attacker_watch')
    m = d.take.moments
    if 'first_sighting' not in m:
        return None
    t, end, inside, total = m['first_sighting'], m.get('hit', m['end']), 0, 0
    while t <= end:
        c, h = pose_at(d.sims, d.rows, cam, t), d.pos(d.take.target, t)
        if c is not None and h is not None:
            total += 1
            uv = project(c[0], c[1], h, meta['hfov'], meta['width'], meta['height'])
            inside += bool(uv and 0 <= uv[0] < meta['width'] and 0 <= uv[1] < meta['height'])
        t += 0.2
    return round(inside / total, 2) if total else None


def est_error_samples(d: TakeData) -> list[tuple[float, float]]:
    """(sim, metres) between the attacker's estimate of the target (its detection in the UI,
    world NED) and the target's true position, from first sighting to hit (or the end)."""
    m, a = d.take.moments, d.take.attacker
    if d.ui is None or a is None or 'first_sighting' not in m:
        return []
    end = m.get('hit', m['end'])
    out = []
    for line in d.ui.lines:
        sim = line['sim']
        if sim < m['first_sighting'] or sim > end:
            continue
        view = next((v for v in line['state'].get('drones', []) if v.get('id') == a), None)
        det = (view or {}).get('detection')
        truth = d.pos(d.take.target, sim)
        if det and truth is not None:
            est = (det['east'], det['north'], det['altitude'])  # NED estimate -> ENU
            out.append((sim, math.dist(est, truth)))
    return out


def fact_est_error_at_sighting_m(d: TakeData):
    s = est_error_samples(d)
    return round(s[0][1], 2) if s else None


def fact_est_error_median_m(d: TakeData):
    s = est_error_samples(d)
    return round(statistics.median(e for _, e in s), 2) if s else None


FACTS = {
    'hit': lambda d: 'hit' in d.take.moments,
    'attacker': lambda d: d.take.attacker,
    'sighting_range_m': fact_sighting_range_m,
    'release_to_sighting_s': lambda d: _gap(d, 'release', 'first_sighting'),
    'sighting_to_hit_s': lambda d: _gap(d, 'first_sighting', 'hit'),
    'closest_pass_m': fact_closest_pass_m,
    'setup': lambda d: d.setup,
    'in_frame_frac': fact_in_frame_frac,
    'est_error_at_sighting_m': fact_est_error_at_sighting_m,
    'est_error_median_m': fact_est_error_median_m,
}


# ---- finding the takes ----------------------------------------------------------------------

def source_ranges(folder: Path) -> dict[str, tuple[float, float]]:
    """Sim range of every recorded source: each clip's .json, ui.jsonl and poses.jsonl."""
    folder = Path(folder)
    ranges = {}
    for meta in folder.glob('*.json'):
        if not (folder / f'{meta.stem}.mkv').exists():
            continue
        try:
            d = json.loads(meta.read_text())
        except ValueError:
            continue
        if d.get('start') is not None and d.get('end') is not None:
            ranges[meta.stem] = (float(d['start']), float(d['end']))
    if (folder / 'ui.jsonl').exists():
        lines = load_recording(folder / 'ui.jsonl')
        ranges['ui'] = (lines[0]['sim'], lines[-1]['sim'])
    if (folder / 'poses.jsonl').exists():
        sims, _ = load_poses(folder / 'poses.jsonl')
        if sims:
            ranges['poses'] = (sims[0], sims[-1])
    return ranges


def _placements(track, grave) -> list[float]:
    out = []
    for (t0, p0), (t1, p1) in zip(track, track[1:]):
        if math.dist(p0, p1) > TELEPORT_M and not in_graveyard(p1, grave):
            out.append(t1)
    return out


def _release(track, start: float, end: float) -> float | None:
    moving_since = None
    for (t0, p0), (t1, p1) in zip(track, track[1:]):
        if t1 <= start or t0 >= end:
            continue
        step = math.dist(p0, p1)
        flat = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        fast = t1 > t0 and step <= TELEPORT_M and flat / (t1 - t0) >= RELEASE_MPS
        if fast:
            moving_since = t0 if moving_since is None else moving_since
            if t1 - moving_since >= RELEASE_HOLD_S - 1e-6:
                return round(moving_since, 3)
        else:
            moving_since = None
    return None


def _first(events, pattern, start, end, drone=None, among=None):
    for sim, _, e in events:
        m = pattern.match(e['text'])
        if (m and start <= sim < end and (drone is None or int(m.group(1)) == drone)
                and (among is None or int(m.group(1)) in among)):
            return sim, int(m.group(1))
    return None


def _taken_out(track, start: float, end: float, grave) -> float | None:
    """The last sim before the hostile jumps into the graveyard within [start, end)."""
    for (t0, p0), (t1, p1) in zip(track, track[1:]):
        if start <= t0 < end and not in_graveyard(p0, grave) and in_graveyard(p1, grave):
            return t0
    return None


def _defender_close(sims, rows, defenders, target, at: float) -> bool:
    for sim, row in zip(sims, rows):
        if at - HIT_WINDOW_S <= sim <= at and target in row:
            h = row[target][:3]
            if any(d in row and math.dist(row[d][:3], h) <= HIT_M for d in defenders):
                return True
    return False


def _sightings_by_hostile(events, ui, sims, rows, hostiles, defenders, rel, end) -> dict:
    """{hostile index (1-based): (sim, defender id) of its first sighting}. A sighting belongs
    to the hostile nearest the defender's detection estimate at that moment (the UI state),
    or nearest the defender itself when the state has no estimate."""
    first = {}
    for sim, _, e in events:
        m = SIGHTING.match(e['text'])
        if not m or not rel <= sim < end or int(m.group(1)) not in defenders:
            continue
        drone = int(m.group(1))
        view = next((d for d in ui.at(sim)['state'].get('drones', []) if d.get('id') == drone), {})
        est = view.get('detection')
        at = ((est['east'], est['north'], est['altitude']) if est else
              (view.get('east'), view.get('north'), view.get('altitude')))
        if at[0] is None:
            continue
        near = [(math.dist(at, p[0]), k) for k, name in enumerate(hostiles, 1)
                if (p := pose_at(sims, rows, name, sim)) is not None]
        if near:
            first.setdefault(min(near)[1], (sim, drone))
    return first


def _per_hostile(m: dict, facts: dict, info, sims, rows, events, ui, grave, rel, end) -> None:
    """Two hostiles at once: hit_k, first_sighting_k and their facts; `hit` only when every
    hostile is down (the last one), `first_sighting` the earliest."""
    defenders = _ids(info.defenders)
    seen = _sightings_by_hostile(events, ui, sims, rows, info.hostiles, defenders, rel, end)
    m.pop('hit', None)
    hits = []
    for k, name in enumerate(info.hostiles, 1):
        track = [(s, tuple(r[name][:3])) for s, r in zip(sims, rows) if name in r]
        out = _taken_out(track, rel, end, grave)
        if out is not None:
            out = _hit_at(sims, rows, info.defenders, name, out)
        attacker = None
        if out is not None:
            m[f'hit_{k}'] = round(out, 3)
            hits.append(out)
            h = pose_at(sims, rows, name, out)[0]
            near = [(math.dist(pose_at(sims, rows, d, out)[0], h), d) for d in info.defenders
                    if pose_at(sims, rows, d, out) is not None]
            attacker = int(min(near)[1].rsplit('_', 1)[1]) if near else None
        sight = seen.get(k)
        if sight is not None:
            m[f'first_sighting_{k}'] = sight[0]
        who = attacker or (sight[1] if sight else None)
        rng = None
        if sight is not None and who is not None:
            a = pose_at(sims, rows, next(d for d in info.defenders if d.endswith(f'_{who}')), sight[0])
            h = pose_at(sims, rows, name, sight[0])
            rng = round(math.dist(a[0], h[0]), 1) if a and h else None
        facts[f'attacker_{k}'] = attacker
        facts[f'sighting_range_m_{k}'] = rng
        facts[f'sighting_to_hit_s_{k}'] = (round(out - sight[0], 2)
                                           if out is not None and sight is not None else None)
    if len(hits) == len(info.hostiles):
        m['hit'] = round(max(hits), 3)
    firsts = [m[f'first_sighting_{k}'] for k in range(1, len(info.hostiles) + 1)
              if f'first_sighting_{k}' in m]
    if firsts:
        m['first_sighting'] = min(firsts)
    facts['hits'] = f'{len(hits)}/{len(info.hostiles)}'


def _hit_at(sims, rows, defenders, target, out: float) -> float | None:
    """When a hostile taken out at `out` was hit: `out` itself when a defender was that close
    just before, else the closest contact within CONTACT_LOOKBACK_S before it (the hostile was
    taken out late, e.g. after a collision while the birds were removed), else None (a miss:
    Gazebo takes the hostile out after every run)."""
    if _defender_close(sims, rows, defenders, target, out):
        return out
    contact = None  # (distance, sim) of the closest contact
    for sim, row in zip(sims, rows):
        if out - CONTACT_LOOKBACK_S <= sim <= out and target in row:
            h = row[target][:3]
            near = [math.dist(row[d][:3], h) for d in defenders if d in row]
            if near and min(near) <= HIT_M and (contact is None or min(near) < contact[0]):
                contact = (min(near), sim)
    return None if contact is None else contact[1]


def _ids(names: list[str]) -> set[int]:
    return {int(n.rsplit('_', 1)[1]) for n in names if n.rsplit('_', 1)[-1].isdigit()}


def find_takes(folder: Path) -> list[Take]:
    folder = Path(folder)
    info = read_run_info(folder)
    sims, rows = load_poses(folder / 'poses.jsonl')
    if not sims:
        raise ValueError(f'{folder / "poses.jsonl"} is empty: were the poses logged (posesrec)?')
    ui = ReplayUi(load_recording(folder / 'ui.jsonl'))
    events = ui.event_table()
    target = info.hostiles[0]
    if info.command[:1] == ['idle']:  # cameras only (e.g. the orbit): one take, the whole run
        ranges = source_ranges(folder)
        lo, hi = max(r[0] for r in ranges.values()), min(r[1] for r in ranges.values())
        return [Take(1, {'start': lo, 'end': hi}, None, target, {'hit': False, 'setup': info.setup},
                     folder=folder, name='overview')]
    grave = graveyard()
    track = [(s, tuple(r[target][:3])) for s, r in zip(sims, rows) if target in r]
    starts = _placements(track, grave) or [sims[0]]
    ranges = source_ranges(folder)
    lo = max(r[0] for r in ranges.values())
    hi = min(r[1] for r in ranges.values())
    defenders = _ids(info.defenders)
    armed = [s for s, _, e in events
             if (m := ARMED.match(e['text'])) and int(m.group(1)) in defenders]
    landed = [s for s, _, e in events
              if (m := LANDED.match(e['text'])) and int(m.group(1)) in defenders]

    takes = []
    for n, start in enumerate(starts):
        end = starts[n + 1] if n + 1 < len(starts) else sims[-1]
        rel = _release(track, start, end)
        if rel is None:
            continue  # the hostile never flew: e.g. put back on its spawn point at the end
        m = {'start': round(max(start, lo), 3), 'end': round(min(end, hi), 3), 'release': rel}
        took = [s for s in armed if start <= s < end] or armed[:1]
        if took:
            m['takeoff'] = took[0]
        down = [s for s in landed if start <= s < end] or landed[-1:]
        if down:
            m['landed'] = down[-1]
        # From the release: before it the hostile stands at its start point, where a defender
        # may already see it (and be sent at it), but that is not the engagement.
        seen = _first(events, SIGHTING, rel, end, among=defenders)
        if seen:
            m['first_sighting'] = seen[0]
        icpt = _first(events, INTERCEPT, rel, end, among=defenders)
        if icpt:
            m['intercept'] = icpt[0]
        attacker = (icpt or seen or (None, None))[1]
        out = _taken_out(track, rel, end, grave)
        # Gazebo takes the hostile out after every run (surveillance.py fly's finally), hit or
        # not: it was a hit only if a defender was that close just before (or made contact).
        if out is not None:
            out = _hit_at(sims, rows, info.defenders, target, out)
        if out is not None:
            m['hit'] = round(out, 3)
            if attacker is None:
                event = _first(events, HIT, start, end, among=defenders)
                attacker = event[1] if event else None
            back = _first(events, RETURN, out, end, attacker) if attacker else None
            if back:
                m['return'] = back[0]
        extra = {}
        if len(info.hostiles) > 1:
            _per_hostile(m, extra, info, sims, rows, events, ui, grave, rel, end)
        take = Take(len(takes) + 1, m, attacker, target, folder=folder,
                    name=str(len(takes) + 1), pair=info.options.get('pair') or None)
        data = TakeData(take, sims, rows, info.defenders, info.setup, folder, ui)
        take.facts = {**{name: f(data) for name, f in FACTS.items()}, **extra}
        takes.append(take)
    return takes


def take_data(folder: Path, take: Take) -> TakeData:
    """What the fact functions get, for one take of a run (plots use it too)."""
    folder = Path(folder)
    info = read_run_info(folder)
    sims, rows = load_poses(folder / 'poses.jsonl')
    return TakeData(take, sims, rows, info.defenders, info.setup, folder,
                    ReplayUi(load_recording(folder / 'ui.jsonl')))


def save_takes(folder: Path, takes: list[Take]) -> Path:
    path = Path(folder) / 'takes.json'
    path.write_text(json.dumps([{**asdict(t), 'folder': str(t.folder)} for t in takes],
                               indent=1) + '\n')
    return path


def shoot_takes(folder: Path) -> list[Take]:
    """The takes of a run folder, or of every run of a shoot folder (shoot.json order, else
    any subfolder with a run.json), named '<run id>/<k>'."""
    folder = Path(folder)
    if (folder / 'run.json').exists():
        return find_takes(folder)
    status = folder / 'shoot.json'
    ids = ([r['id'] for r in json.loads(status.read_text()).get('runs', [])] if status.exists()
           else sorted(p.name for p in folder.iterdir() if (p / 'run.json').exists()))
    takes = []
    for rid in ids:
        if not (folder / rid / 'run.json').exists():
            continue  # not recorded (yet)
        for t in find_takes(folder / rid):
            t.name = f'{rid}/{t.name}'
            takes.append(t)
    return takes


def rank(takes: list[Take]) -> list[tuple[Take, float, list[str]]]:
    """Best first: a hit, then a short time from sighting to hit, then the target kept in the
    chase camera's frame. Score = 100 for a hit - seconds to hit + 10 x the in-frame share."""
    out = []
    for t in takes:
        f, reasons = t.facts, []
        score = 0.0
        if f.get('hit'):
            score += 100
            reasons.append('hit')
        else:
            reasons.append('no hit')
        if f.get('sighting_to_hit_s') is not None:
            score -= f['sighting_to_hit_s']
            reasons.append(f'{f["sighting_to_hit_s"]:.1f} s to hit')
        if f.get('in_frame_frac') is not None:
            score += 10 * f['in_frame_frac']
            reasons.append(f'{100 * f["in_frame_frac"]:.0f} % in frame')
        out.append((t, round(score, 2), reasons))
    return sorted(out, key=lambda x: -x[1])


def describe(takes: list[Take]) -> str:
    lines = []
    for t in takes:
        rel = t.moments.get('release')
        lines.append(f'Take {t.name or t.number}: {"hit" if t.facts.get("hit") else "no hit"}, '
                     f'attacker {t.attacker or "-"}, target {t.target}')
        for name in MOMENTS:
            if name in t.moments:
                plus = f'  (release {t.moments[name] - rel:+.1f} s)' if rel is not None else ''
                lines.append(f'  {name:15s} {t.moments[name]:8.1f}{plus}')
        for name, value in t.facts.items():
            lines.append(f'  · {name}: {value}')
    hits = [t.facts['sighting_to_hit_s'] for t in takes if t.facts.get('sighting_to_hit_s')]
    lines.append(f'{len(takes)} take(s), {sum(bool(t.facts.get("hit")) for t in takes)} hit(s)'
                 + (f', median {statistics.median(hits):.1f} s from sighting to hit' if hits else ''))
    return '\n'.join(lines)


def describe_ranking(takes: list[Take]) -> str:
    lines = [f'{"rank":>4}  {"take":24s} {"setup":20s} why']
    for i, (t, _, why) in enumerate(rank(takes), 1):
        lines.append(f'{i:>4}  {t.name:24s} {str(t.facts.get("setup")):20s} {", ".join(why)}')
    return '\n'.join(lines)


def main(argv: list[str] | None = None) -> int:
    args = sys.argv[1:] if argv is None else argv
    if len(args) != 1 or args[0] in ('-h', '--help'):
        print(__doc__)
        return 0 if args[:1] in (['-h'], ['--help']) else 1
    folder = Path(args[0])
    if (folder / 'run.json').exists():
        takes = find_takes(folder)
        save_takes(folder, takes)
        print(describe(takes))
        return 0
    takes = shoot_takes(folder)
    for run in sorted({t.folder for t in takes}):
        save_takes(run, [t for t in takes if t.folder == run])
    print(describe(takes))
    print()
    print(describe_ranking(takes))
    return 0


if __name__ == '__main__':
    sys.exit(main())
