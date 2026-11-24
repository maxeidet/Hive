"""The story file: the demo film as a list of scenes tied to named moments of takes.

recording/stories/<name>.yaml (recording/README.md, "The story file"):

    title: ...            subtitle: ...        title_seconds: 3
    take: best            # default take: best = the first with a hit, or a take number
    end_card: results     # or leave it out;   end_seconds: 3
    scenes:
    - caption: Drone {attacker} intercepts     # {attacker}, {take} are filled in
      from: first_sighting - 5                 # a moment ± seconds (takes.MOMENTS)
      to: hit
      layout: grid                             # full (1 source), side (2), grid (4)
      show: [chase, yolo, map, wide]           # roles, see SOURCES
      speed: 1                                 # 8 = 8x faster, 0.5 = slow motion
      take: 2                                  # optional, this scene only
    - card: A weaving hostile                  # a text card; duration: 3
"""
import re
from dataclasses import dataclass, field
from pathlib import Path

import yaml

from swarm_backend.demo.takes import MOMENTS, Take

LAYOUTS = {'full': 1, 'side': 2, 'grid': 4, 'compare': 1, 'ghost': 1, 'zoom': 1}
PAIRED = ('compare', 'ghost')   # layouts that show both takes of a baseline/ours pair
SOURCES = ('map', 'ui', 'yolo', 'chase', 'wide')   # plus yolo:<id> and cam:<angle>
SCENE_KEYS = {'caption', 'from', 'to', 'layout', 'show', 'speed', 'take', 'insights', 'zoom',
              'freeze'}
CARD_KEYS = {'card', 'duration'}
STORY_KEYS = {'title', 'subtitle', 'title_seconds', 'take', 'end_card', 'end_seconds', 'scenes',
              'evidence'}
OURS = 'yolo-bytetrack-mpc'
_MOMENT = re.compile(r'^\s*([a-z_]+)\s*(?:([+-])\s*(\d+(?:\.\d*)?|\.\d+))?\s*$')


class StoryError(ValueError):
    pass


@dataclass
class Scene:
    index: int
    caption: str | None
    card: str | None
    take: str | None
    start_expr: str | None
    end_expr: str | None
    layout: str | None
    show: list[str]
    speed: float = 1.0
    duration: float = 3.0
    insights: list[str] | None = None   # None: off; [] : all of insights.INSIGHTS
    zoom: float = 3.0                   # layout zoom: the crop is 1/zoom of the picture
    freeze: float = 0.0                 # layout zoom: seconds held on the first locked frame


@dataclass
class Story:
    title: str | None
    subtitle: str | None
    title_seconds: float
    take: str
    end_card: str | None
    end_seconds: float
    scenes: list[Scene]
    path: Path
    evidence: str | None = None


@dataclass
class Resolved:
    scene: Scene
    take: Take | None
    start: float | None
    end: float | None
    sources: dict[str, str] = field(default_factory=dict)
    side: str | None = None   # 'ours' / 'baseline' in a paired layout


def parse_moment(expr) -> tuple[str, float]:
    m = _MOMENT.match(str(expr))
    if not m or m.group(1) not in MOMENTS:
        raise StoryError(f'{expr!r} is not "moment ± seconds"; the moments are: '
                         f'{", ".join(MOMENTS)}')
    name, sign, num = m.groups()
    offset = float(num) if num else 0.0
    return name, -offset if sign == '-' else offset


def source_key(role: str, attacker: int | None) -> str:
    """The recorded file (stem) that a role shows."""
    if role == 'map':
        return 'map'
    if role == 'ui':
        return 'screen'
    if role == 'yolo':
        return f'debug_d{attacker}'
    if role == 'chase':
        return 'cam_attacker_watch'
    if role == 'wide':
        return 'cam_arc_side'
    kind, _, arg = role.partition(':')
    if kind == 'plot':
        return f'plot_{arg}'
    return f'debug_d{arg}' if kind == 'yolo' else f'cam_{arg}'


def recorded_file(folder: Path, key: str) -> Path:
    """The file a source key is made from (the map and the graphs: the poses)."""
    return folder / ('poses.jsonl' if key == 'map' or key.startswith('plot_') else f'{key}.mkv')


def choose(role: str, t: Take, used: tuple[str, ...] = ()) -> str:
    """`ui|map`: the first alternative this take's run recorded and the scene does not show
    yet (`used`), else the last (so the build names what is missing)."""
    options = role.split('|')
    for a in options:
        if a not in used and recorded_file(t.folder, source_key(a, t.attacker)).exists():
            return a
    return options[-1]


def _chosen(s: Scene, t: Take, **changes) -> Scene:
    show: list[str] = []
    for r in s.show:
        show.append(choose(r, t, tuple(show)))
    return Scene(**{**s.__dict__, **changes, 'show': show})


def _check_role(role: str) -> bool:
    if role in SOURCES:
        return True
    kind, _, arg = role.partition(':')
    if kind == 'plot':
        from swarm_backend.demo.plots import PLOTS
        return arg in PLOTS
    return (kind == 'yolo' and arg.isdigit()) or (kind == 'cam' and bool(arg))


def _scene(i: int, d, path: Path) -> Scene:
    where = f'{path.name}, scene {i}'
    if not isinstance(d, dict):
        raise StoryError(f'{where}: a scene is a mapping (caption, from, to, layout, show)')
    if 'card' in d:
        extra = set(d) - CARD_KEYS
        if extra:
            raise StoryError(f'{where}: a card scene only takes {", ".join(sorted(CARD_KEYS))}, '
                             f'not {", ".join(sorted(extra))}')
        return Scene(i, None, str(d['card']), None, None, None, None, [],
                     duration=float(d.get('duration', 3.0)))
    extra = set(d) - SCENE_KEYS
    if extra:
        raise StoryError(f'{where}: unknown key(s) {", ".join(sorted(extra))}; allowed: '
                         f'{", ".join(sorted(SCENE_KEYS))}')
    for key in ('from', 'to', 'layout', 'show'):
        if key not in d:
            raise StoryError(f'{where}: missing "{key}"')
    layout, show = d['layout'], d['show']
    if layout not in LAYOUTS:
        raise StoryError(f'{where}: layout {layout!r}; use {", ".join(LAYOUTS)}')
    if not isinstance(show, list) or len(show) != LAYOUTS[layout]:
        raise StoryError(f'{where}: layout {layout} shows {LAYOUTS[layout]} source(s), got {show!r}')
    if layout == 'ghost' and show != ['map']:
        raise StoryError(f'{where}: a ghost scene shows [map]')
    if layout == 'zoom' and not str(show[0]).startswith('yolo'):
        raise StoryError(f'{where}: a zoom scene shows [yolo] or [yolo:<id>]')
    bad = [a for r in show for a in str(r).split('|') if not _check_role(a)]
    if bad:
        raise StoryError(f'{where}: unknown source(s) {", ".join(map(str, bad))}; use '
                         f'{", ".join(SOURCES)}, yolo:<id> or cam:<angle>')
    try:
        for key in ('from', 'to'):
            parse_moment(d[key])
    except StoryError as e:
        raise StoryError(f'{where}: {e}') from None
    speed = float(d.get('speed', 1.0))
    if speed <= 0:
        raise StoryError(f'{where}: speed must be above 0')
    ins = d.get('insights')
    if ins in (None, False):
        ins = None
    elif ins is True:
        ins = []
    else:
        from swarm_backend.demo.insights import INSIGHTS
        ins = [str(x) for x in (ins if isinstance(ins, list) else [ins])]
        bad = [x for x in ins if x not in INSIGHTS]
        if bad:
            raise StoryError(f'{where}: unknown insight(s) {", ".join(bad)}; use true or '
                             f'{", ".join(INSIGHTS)}')
    return Scene(i, d.get('caption'), None, None if d.get('take') is None else str(d['take']),
                 str(d['from']), str(d['to']), layout, [str(r) for r in show], speed,
                 insights=ins, zoom=float(d.get('zoom', 3.0)), freeze=float(d.get('freeze', 0.0)))


def load_story(path: Path) -> Story:
    path = Path(path)
    try:
        d = yaml.safe_load(path.read_text()) or {}
    except yaml.YAMLError as e:
        raise StoryError(f'{path.name}: not valid YAML: {e}') from None
    extra = set(d) - STORY_KEYS
    if extra:
        raise StoryError(f'{path.name}: unknown key(s) {", ".join(sorted(extra))}; allowed: '
                         f'{", ".join(sorted(STORY_KEYS))}')
    scenes = d.get('scenes') or []
    if not scenes:
        raise StoryError(f'{path.name}: no scenes')
    return Story(d.get('title'), d.get('subtitle'), float(d.get('title_seconds', 3.0)),
                 str(d.get('take', 'best')), d.get('end_card'), float(d.get('end_seconds', 3.0)),
                 [_scene(i + 1, s, path) for i, s in enumerate(scenes)], path,
                 None if d.get('evidence') is None else str(d['evidence']))


def pick_take(name: str, takes: list[Take]) -> Take:
    """best / bestN (the ranking, takes.rank), a take's name ('<run id>/<k>' in a shoot), or
    a take number (in a single run)."""
    from swarm_backend.demo.takes import rank
    if not takes:
        raise StoryError('the run has no takes')
    name = str(name)
    m = re.fullmatch(r'best(\d*)', name)
    if m:
        n = int(m.group(1) or 1)
        ranked = [t for t, _, _ in rank(takes)]
        hits = sum(bool(t.facts.get('hit')) for t in takes)
        if n == 1:
            return ranked[0]
        if n > hits:
            raise StoryError(f'take {name}: only {hits} take(s) ended in a hit')
        return ranked[n - 1]
    for t in takes:
        if t.name == name:
            return t
    if name.isdigit():
        for t in takes:
            if t.number == int(name) and '/' not in t.name:
                return t
    names = ', '.join(t.name or str(t.number) for t in takes)
    raise StoryError(f'take {name}: no such take; there are {names}')


def pairs(takes: list[Take]) -> dict[str, tuple[Take, Take]]:
    """pair id -> (ours, baseline), for pairs with both sides recorded (first take of each)."""
    sides: dict[str, dict[str, Take]] = {}
    for t in takes:
        if t.pair:
            side = 'ours' if t.facts.get('setup') == OURS else 'baseline'
            sides.setdefault(t.pair, {}).setdefault(side, t)
    return {p: (s['ours'], s['baseline']) for p, s in sides.items() if len(s) == 2}


def pick_pair(name: str, takes: list[Take]) -> tuple[Take, Take]:
    from swarm_backend.demo.takes import rank
    found = pairs(takes)
    if not found:
        raise StoryError('no baseline/ours pairs in this run or shoot (shoot.yaml compare:)')
    if name == 'best_pair':
        ours = [t for t, _, _ in rank([o for o, _ in found.values()])][0]
        return found[ours.pair]
    if name.startswith('pair:') and name[5:] in found:
        return found[name[5:]]
    raise StoryError(f'take {name}: use best_pair or pair:<id>; pairs here: {", ".join(found)}')


def _times(s: Scene, t: Take) -> tuple[list[float], list[str]]:
    times, missing = [], []
    for expr in (s.start_expr, s.end_expr):
        name, offset = parse_moment(expr)
        if name not in t.moments:
            missing.append(name)
        else:
            times.append(t.moments[name] + offset)
    return times, missing


def resolve(story: Story, takes: list[Take], take: str | None = None
            ) -> tuple[list[Resolved], list[str]]:
    """Every scene with its take, sim range and source files; problems as messages
    (a missing moment), never raised, so they can all be shown at once."""
    out, problems = [], []
    for s in story.scenes:
        if s.card is not None:
            out.append(Resolved(s, None, None, None))
            continue
        label = f"scene {s.index} '{s.caption or ''}'"
        if s.layout in PAIRED:
            try:
                ours, base = pick_pair(s.take or 'best_pair', takes)
            except StoryError as e:
                problems.append(f'{label}: {e}')
                continue
            caption = (s.caption or '').format(attacker=ours.attacker or '?', take=ours.name)
            label = f"scene {s.index} '{caption}'"
            times, missing = _times(s, ours)
            if missing or 'release' not in ours.moments or 'release' not in base.moments:
                problems.append(f"{label}: pair {ours.pair} has no moment "
                                + ', '.join(f"'{m}'" for m in dict.fromkeys(missing or ['release'])))
                continue
            start, end = times
            if end <= start:
                problems.append(f'{label}: ends ({end:.1f}) before it starts ({start:.1f})')
                continue
            shift = base.moments['release'] - ours.moments['release']
            for t, a, b, side in ((ours, start, end, 'ours'), (base, start + shift, end + shift, 'baseline')):
                scene = _chosen(s, t, caption=caption)
                out.append(Resolved(scene, t, a, b, {r: source_key(r, t.attacker) for r in scene.show}, side))
            continue
        try:
            t = pick_take(take or s.take or story.take, takes)
        except StoryError as e:
            problems.append(f'{label}: {e}')
            continue
        caption = (s.caption or '').format(attacker=t.attacker if t.attacker else '?', take=t.number)
        label = f"scene {s.index} '{caption}'"
        times, missing = _times(s, t)
        if missing:
            problems.append(f"{label}: take {t.name or t.number} has no moment "
                            + ', '.join(f"'{m}'" for m in dict.fromkeys(missing)))
            continue
        start, end = times
        if end <= start:
            problems.append(f'{label}: ends ({end:.1f}) before it starts ({start:.1f})')
            continue
        scene = _chosen(s, t, caption=caption)
        out.append(Resolved(scene, t, start, end,
                            {r: source_key(r, t.attacker) for r in scene.show}))
    return out, problems
