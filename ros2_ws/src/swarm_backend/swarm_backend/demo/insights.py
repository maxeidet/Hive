"""Insights: numbers from a take's facts, shown in the film at the moment they belong to.

A scene turns them on with `insights: true` (all) or `insights: [seen_at, time_to_hit]`.
Each insight is a moment, a text made from the take's facts, and when it applies; one whose
facts are missing (None) is left out. A new insight is one line in INSIGHTS.
"""
from dataclasses import dataclass

from swarm_backend.demo.takes import Take

INSIGHTS = {
    'seen_at': ('first_sighting', 'seen at {sighting_range_m:.0f} m', lambda f: True),
    'assigned': ('intercept', 'D{attacker} assigned', lambda f: True),
    'est_error': ('first_sighting', 'position error {est_error_at_sighting_m:.1f} m', lambda f: True),
    'time_to_hit': ('hit', '{sighting_to_hit_s:.1f} s from sighting to hit', lambda f: True),
    'closest': ('hit', 'closest pass {closest_pass_m:.1f} m', lambda f: True),
    'no_hit': ('end', 'no hit, closest {closest_pass_m:.1f} m', lambda f: not f.get('hit')),
}
HOLD_S = 3.0      # seconds a label stays (clip time)
NEAR_S = 1.5      # labels closer than this in clip time stack in rows


@dataclass
class Label:
    name: str
    text: str
    at: float      # clip seconds (after speed)
    until: float
    row: int = 0


def insights_for(take: Take, start: float, end: float, speed: float,
                 names: list[str] | None = None, hold_s: float = HOLD_S) -> list[Label]:
    labels = []
    length = (end - start) / speed
    for name, (moment, template, applies) in INSIGHTS.items():
        if names is not None and name not in names:
            continue
        at_sim = take.moments.get(moment)
        if at_sim is None or not (start <= at_sim < end or (moment == 'end' and at_sim >= end)):
            continue
        if not applies(take.facts):
            continue
        try:
            text = template.format(**take.facts)
        except (TypeError, ValueError, KeyError):
            continue  # a fact it needs is None or missing
        at = max(0.0, min((at_sim - start) / speed, length - 1.0))
        labels.append(Label(name, text, round(at, 3), round(min(length, at + hold_s), 3)))
    labels.sort(key=lambda lab: lab.at)
    for i, lab in enumerate(labels):
        busy = {o.row for o in labels[:i] if lab.at - o.at < NEAR_S and o.until > lab.at}
        lab.row = next(r for r in range(len(labels) + 1) if r not in busy)
    return labels
