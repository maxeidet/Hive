"""The baseline evaluation's scenarios on the surveillance arc (`mount-backend.sh
baseline-eval`; planning report, Interception > Evaluation and Drone Detection Using
Sensors > Evaluation).

The evaluation harness (intercept/scenario.py, detect/scenario.py) was written for one
interceptor on a 1000 m ring and 12 defenders on a ring. Here the same seeds, aspects,
speeds, altitudes and metrics are used, but the defenders are the surveillance drones on
their arc (surveillance: in swarm.yaml), so the result is a baseline of the system we fly:

  intercept  his make_scenario draws (the interceptor's bearing, the target's side and
             altitude) in the same order, from the same seed. The bearing picks the
             defender (the arc's slot it falls in), and the target's track is the one
             whose line of sight to that defender makes `aspect_deg` with its velocity
             when it is `detect_range` away. The track goes straight to the base, and
             starts `approach_m` further out along it, so the hostile drone is first
             seen by the real detector instead of being placed inside its range.
  detect     his make_scenario draws (the hostile's altitude, then its bearing), the
             bearing squeezed onto the arc's span (the arc only watches the threat
             side), no birds; it starts start_margin outside the arc, capped at
             `max_start_m` from the base so it starts on hive_base's ground plane.

Pure math, world NED, no ROS.
"""
import math
import random
from bisect import bisect_left
from dataclasses import dataclass

from swarm_backend.config import DetectConfig, InterceptConfig, SurveillanceConfig, Vector3
from swarm_backend.detect.camera import CameraModel
from swarm_backend.detect.detector import Detection, Stage
from swarm_backend.detect.frame import Body, Kind, make_frame
from swarm_backend.detect.metrics import match
from swarm_backend.detect.scenario import HOSTILE_ID, DetectScenario, FrameRecord, Mover
from swarm_backend.intercept.interceptor import Status
from swarm_backend.intercept.metrics import Sample
from swarm_backend.intercept.scenario import Scenario
from swarm_backend.swarm.arc import arc_points

APPROACH_M = 60.0  # the intercept track starts this much before first possible detection
MAX_START_M = 450.0  # hive_base's ground plane is 1000 x 1000 m around the base


def slots(config: SurveillanceConfig, altitude: float | None = None) -> list[Vector3]:
    """The defenders' points on the arc, left to right, world NED (z = -altitude above the
    base's ground)."""
    bx, by = config.base_position
    z = -(config.altitude if altitude is None else altitude)
    return arc_points((bx, by, z), math.radians(config.threat_heading_deg), config.arc_radius,
                      len(config.drone_ids), config.arc_deg)


def make_intercept_scenario(aspect_deg: float, speed: float, seed: int,
                            config: InterceptConfig, surveillance: SurveillanceConfig,
                            approach_m: float = APPROACH_M) -> tuple[Scenario, int]:
    """His intercept scenario on the arc, and the index of the defender it is built
    around (left to right). Raises ValueError for an aspect the arc cannot give."""
    rng = random.Random(seed)
    ring = rng.uniform(-math.pi, math.pi)  # same draws, same order as scenario.make_scenario
    side = rng.choice((-1.0, 1.0))
    target_z = -rng.uniform(config.hostile_altitude_min, config.hostile_altitude_max)

    points = slots(surveillance)
    n = len(points)
    index = min(n - 1, int((ring + math.pi) / (2 * math.pi) * n))
    interceptor = points[index]
    bx, by = surveillance.base_position
    big_r = math.hypot(interceptor[0] - bx, interceptor[1] - by)
    r = config.detect_range
    aspect = math.radians(aspect_deg)
    if r * math.sin(aspect) >= big_r:
        raise ValueError(f'No scenario with aspect {aspect_deg} deg at {r} m from the arc')
    at_base = math.asin(r * math.sin(aspect) / big_r)
    distance = r * math.cos(aspect) + math.sqrt(big_r ** 2 - (r * math.sin(aspect)) ** 2)
    start = min(distance + approach_m, MAX_START_M)
    bearing = math.atan2(interceptor[1] - by, interceptor[0] - bx) + side * at_base
    scenario = Scenario(
        aspect_deg=aspect_deg, speed=speed, seed=seed, base=(bx, by, 0.0),
        interceptor_start=interceptor,
        target_start=(bx + start * math.cos(bearing), by + start * math.sin(bearing), target_z),
        target_goal=(bx, by, target_z))
    return scenario, index


def intercept_grid(config: InterceptConfig, surveillance: SurveillanceConfig,
                   aspects=None, speeds=None, seeds=None) -> list[tuple[Scenario, int]]:
    """Every aspect x speed x seed (default: the config's, seeds 0..eval_runs-1)."""
    aspects = config.eval_aspects if aspects is None else aspects
    speeds = config.eval_speeds if speeds is None else speeds
    seeds = range(config.eval_runs) if seeds is None else seeds
    return [make_intercept_scenario(a, v, s, config, surveillance)
            for a in aspects for v in speeds for s in seeds]


def make_detect_scenario(seed: int, config: DetectConfig, surveillance: SurveillanceConfig,
                         max_start_m: float = MAX_START_M) -> DetectScenario:
    """His detection scenario with our defenders on the arc and no birds."""
    rng = random.Random(seed)
    z = -rng.uniform(config.hostile_altitude_min, config.hostile_altitude_max)
    u = rng.uniform(-math.pi, math.pi)  # his bearing draw, squeezed onto the arc's span
    heading = math.radians(surveillance.threat_heading_deg)
    bearing = heading + math.radians(surveillance.arc_deg / 2) * u / math.pi
    bx, by = surveillance.base_position
    distance = min(surveillance.arc_radius + config.start_margin, max_start_m)
    start = (bx + distance * math.cos(bearing), by + distance * math.sin(bearing), z)
    hostile = Mover(HOSTILE_ID, Kind.DRONE, start, (bx, by, z), config.hostile_speed,
                    config.target_size)
    return DetectScenario(seed=seed, base=(bx, by, 0.0),
                          perimeter_radius=surveillance.arc_radius,
                          defenders=tuple(slots(surveillance)), hostile=hostile, birds=())


# --- from what a Gazebo run recorded to his Samples and FrameRecords -------------------
#
# A run records, on Gazebo's clock:
#   truth     [TruthPoint]: every defender's body and camera pose, and the hostile drone
#   records   {defender index: [dict]}: one per image its real detector processed
#             (detect/live.py --record: stamp, width, height, boxes, confirmed tracks)
#   statuses  [(stamp, {defender index: Status})]: what each defender's attack was doing

SAMPLE_DT = 0.1  # 10 Hz, as scenario.STEP_S
MAX_TRUTH_GAP_S = 0.2  # an image or sample further than this from any truth is skipped
MAX_IMAGE_AGE_S = 0.3  # a sample uses its defender's latest image at most this old


@dataclass(frozen=True)
class TruthPoint:
    stamp: float
    bodies: dict  # defender index -> world NED position
    cameras: dict  # defender index -> camera pose (ground_truth.camera_pose)
    hostile: Vector3 | None


def nearest(items: list, stamps: list[float], t: float):
    """The item whose stamp is closest to t (stamps sorted), with its distance in time."""
    if not items:
        return None, math.inf
    i = bisect_left(stamps, t)
    best = min((j for j in (i - 1, i) if 0 <= j < len(items)), key=lambda j: abs(stamps[j] - t))
    return items[best], abs(stamps[best] - t)


def detections(record: dict, drone: int) -> list[Detection]:
    """A detector record's confirmed tracks as the harness's detection reports: each one
    counts as a drone (the baseline has no classifier; confirmed = k images in a row)."""
    return [Detection(time=record['stamp'], drone=drone, pixel=(u, v), box_px=(w, h),
                      stage=Stage.CLASSIFIED, kind=Kind.DRONE, confidence=1.0, track=track)
            for u, v, w, h, track in record['confirmed']]


def _frame(point: TruthPoint, drone: int, record: dict, t: float, hfov: float,
           target_size: float, friendlies: bool = True):
    bodies = [] if point.hostile is None else [
        Body(HOSTILE_ID, Kind.DRONE, point.hostile, target_size)]
    if friendlies:  # another defender in the picture is a drone, not a false alarm
        bodies += [Body(f'defender{i}', Kind.DRONE, p, target_size)
                   for i, p in point.bodies.items() if i != drone]
    model = CameraModel(hfov, record['width'], record['height'])
    return make_frame(model, t, drone, point.cameras[drone], bodies)


def frame_records(truth: list[TruthPoint], records: dict, scenario: DetectScenario, t0: float,
                  t_end: float, hfov: float) -> list[FrameRecord]:
    """His FrameRecords: every image any defender's detector processed during the run,
    with the truth of what was in it."""
    stamps = [p.stamp for p in truth]
    out = []
    for drone, recs in records.items():
        for record in recs:
            if not t0 <= record['stamp'] <= t_end:
                continue
            point, gap = nearest(truth, stamps, record['stamp'])
            if point is None or gap > MAX_TRUTH_GAP_S or drone not in point.cameras \
                    or point.hostile is None:
                continue
            t = record['stamp'] - t0
            frame = _frame(point, drone, record, t, hfov, scenario.hostile.size)
            bx, by, _ = scenario.base
            out.append(FrameRecord(
                t=t, frame=frame, detections=tuple(detections(record, drone)),
                hostile_distance=math.dist(frame.camera, point.hostile),
                hostile_outside=math.hypot(point.hostile[0] - bx, point.hostile[1] - by)
                - scenario.perimeter_radius))
    return sorted(out, key=lambda r: (r.t, r.frame.drone))


def intercept_samples(truth: list[TruthPoint], records: dict, statuses: list, drone: int,
                      t0: float, t_end: float, hfov: float, target_size: float,
                      match_radius_px: float) -> list[Sample]:
    """His Samples at 10 Hz for the defender `drone` (the one scored as the interceptor):
    true positions; in_view = the hostile drone projects into its camera; detected = its
    detector's latest image (at most MAX_IMAGE_AGE_S old) has a confirmed track within
    match_radius_px of the hostile; status = what its attack was doing."""
    stamps = [p.stamp for p in truth]
    recs = sorted(records.get(drone, []), key=lambda r: r['stamp'])
    rec_stamps = [r['stamp'] for r in recs]
    status_stamps = [s for s, _ in statuses]
    samples = []
    k = 0
    while t0 + k * SAMPLE_DT <= t_end + 1e-9:
        t = t0 + k * SAMPLE_DT
        k += 1
        point, gap = nearest(truth, stamps, t)
        if point is None or gap > MAX_TRUTH_GAP_S or point.hostile is None \
                or drone not in point.bodies:
            continue
        in_view = detected = False
        i = bisect_left(rec_stamps, t + 1e-9) - 1  # the latest image at or before t
        if i >= 0 and t - rec_stamps[i] <= MAX_IMAGE_AGE_S:
            record = recs[i]
            image_point, image_gap = nearest(truth, stamps, record['stamp'])
            if image_point is not None and image_gap <= MAX_TRUTH_GAP_S \
                    and drone in image_point.cameras and image_point.hostile is not None:
                frame = _frame(image_point, drone, record, t, hfov, target_size,
                               friendlies=False)
                hostile = [o for o in frame.objects if o.id == HOSTILE_ID]
                in_view = bool(hostile)
                found = match(detections(record, drone), hostile, match_radius_px)
                detected = any(o is not None for o in found)
        state, _ = nearest(statuses, status_stamps, t)
        status = Status.IDLE if state is None else state[1].get(drone, Status.IDLE)
        samples.append(Sample(t=t - t0, interceptor=point.bodies[drone], target=point.hostile,
                              in_view=in_view, detected=detected, status=status))
    return samples
