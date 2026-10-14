"""The detection baseline on the scenarios of the planning report (Drone Detection
Using Sensors > Evaluation). `pytest -s` prints the report's metrics.

- Scenario sweep in the fast simulation: the static ring's sector scan with the
  PixelModelDetector stand-in, eval_runs seeds. Skipped until those exist.
- Labelled Gazebo images: the dark-object baseline (and YOLO26 if HIVE_YOLO_WEIGHTS is
  set) on the test split. Skipped unless HIVE_DETECT_DATASET points at the dataset
  and the detectors exist.

The asserts are the report's assumptions, so they are the acceptance test.
"""
import os
import random

import pytest

from swarm_backend.config import DetectConfig
from swarm_backend.detect.camera import CameraModel
from swarm_backend.detect.dataset import (GazeboImageDataset, evaluate_dataset, split_by_run,
                                          summarize_dataset)
from swarm_backend.detect.detector import DarkObjectDetector, PixelModelDetector, Yolo26Detector
from swarm_backend.detect.metrics import detection_by_range, score_run, summarize
from swarm_backend.detect.scan import SectorScan
from swarm_backend.detect.scenario import run_scenario, scenario_grid

from sim import DetectFastSim

CONFIG = DetectConfig()
NOT_YET = 'detection baseline not implemented yet ({})'


def pixel_model(scenario, drone, config):
    return PixelModelDetector(config, random.Random(scenario.seed * 1000 + drone))


def sector_scan(scenario, config):
    return SectorScan(config, scenario.base)


def run(scenario):
    records = run_scenario(scenario, DetectFastSim(scenario, CONFIG), pixel_model, sector_scan,
                           CONFIG)
    return score_run(records, scenario, CONFIG)


@pytest.fixture(scope='module')
def results():
    grid = scenario_grid(CONFIG)
    try:
        first = run(grid[0])
    except NotImplementedError:
        pytest.skip(NOT_YET.format('PixelModelDetector, SectorScan'))
    results = [first] + [run(s) for s in grid[1:]]
    print('\n' + summarize(results, CONFIG, 'Baseline: sector scan, pixel-model detector'))
    return results


def test_every_run_is_scored(results):
    assert len(results) == CONFIG.eval_runs
    for r in results:
        assert r.missed == (r.first_detection_time is None)


def test_no_detection_beyond_the_detection_range(results):
    # The report's assumption: found from p_min pixels, so no further than this.
    camera = CameraModel(CONFIG.camera_fov_deg, CONFIG.image_width, CONFIG.image_height)
    max_range = camera.focal_px * CONFIG.target_size / CONFIG.detect_px
    for r in results:
        assert all(distance <= max_range * 1.1 for distance, hit in r.hostile_views if hit)


def test_detection_gets_less_likely_with_range(results):
    shares = [hit / seen for _, _, seen, hit in detection_by_range(results, CONFIG.range_bins)
              if seen]
    assert shares == sorted(shares, reverse=True)


def test_same_seed_same_result(results):
    assert run(scenario_grid(CONFIG)[0]) == results[0]


# --- labelled images ---------------------------------------------------------------

@pytest.fixture(scope='module')
def test_images():
    root = os.environ.get('HIVE_DETECT_DATASET')
    if not root:
        pytest.skip('set HIVE_DETECT_DATASET to the labelled Gazebo images')
    try:
        dataset = GazeboImageDataset(root)
    except NotImplementedError:
        pytest.skip(NOT_YET.format('GazeboImageDataset'))
    return split_by_run(dataset)[2]


def detectors():
    yield 'dark object', lambda: DarkObjectDetector(CONFIG)
    weights = os.environ.get('HIVE_YOLO_WEIGHTS')
    if weights:
        yield 'YOLO26', lambda: Yolo26Detector(CONFIG, weights)


@pytest.mark.parametrize('name, make', list(detectors()))
def test_recall_by_size_on_held_out_images(test_images, name, make):
    try:
        detector = make()
    except NotImplementedError:
        pytest.skip(NOT_YET.format(name))
    metrics = evaluate_dataset(test_images, detector, CONFIG)
    print('\n' + summarize_dataset(metrics, f'{name} on {len(test_images)} test images'))
    for lo, _, drones, found, _ in metrics.size_bins:
        if lo >= CONFIG.detect_px and drones:
            assert found / drones >= 0.5, f'{name} finds under half the drones of {lo:g}+ px'
