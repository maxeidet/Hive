# Demo scenarios Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Birds and two simultaneous hostiles in the 3-defender scenario, an examiner shoot of four baseline/ours pairs, and a film of at most 5 minutes cut from it.

**Architecture:** Birds hook into the existing single-hostile run (`SurveillanceWorld.fly`) through two no-op hooks the Gazebo backend fills. Two hostiles get their own run, `fly_many`, next to `fly`, so the single-hostile run, its referee and its scoring (`arc_eval`, the 36-case evaluation) stay as they are. The film tools learn per-hostile moments; birds become green in the camera clips and on the map.

**Tech Stack:** Python 3.12 in the `hive-px4-vnc` image (ROS 2 Jazzy, gz-transport, OpenCV, PyYAML, ffmpeg), bash + tmux on the host, pytest.

**Spec:** `hive-vault/recording/Demo scenarios (design).md` (approved by Oskar, 24 Nov 2026).

## Global Constraints

- Unit tests run in the image: `cd ~/Desktop/Hive && docker run --rm -v "$PWD":/workspace -w /workspace/ros2_ws/src/swarm_backend hive-px4-vnc bash -c 'source /opt/ros/jazzy/setup.bash; python3 -m pytest -q tests/<file>'`. The full suite (`tests/`) must pass at the end of every task (713 passed, 7 skipped on 24 Nov).
- Single-hostile runs unchanged: same results rows, same summary, same 36-case evaluation. Old `intercept.jsonl` files still load.
- Drone 5 is only spawned by the two-hostile config, never by `scenarios/surveillance.yaml`.
- One seed per scenario: `seeds: 0`. The examiner film is at most 300 s.
- Birds: 8 per run in the shoot; names start with `bird_` (`runinfo.BIRDS_PREFIX`); green in the films: BGR `(80, 200, 80)`.
- Never edit `recording/record.sh` or `docker/start-stellar.sh` while a recording runs (bash reads them as it goes). Check `tmux ls` first.
- Commits on `feat-demo`, ending with `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`.

## Review Focus

1. **A run interrupted (Ctrl-C, a failure) while birds fly:** the bird process is stopped and every bird removed, or the next run in a kept sim starts with leftovers. Test: Task 2, `test_birds_stopped_when_the_run_fails`.
2. **Birds that appear mid-run:** the map today only looks for birds in the first pose row (`map_video.py`, `st.rows[:1]`), so birds spawned later are never drawn. Test: Task 3, `test_map_draws_birds_that_appear_later`.
3. **One hostile hit, the other not:** the run goes on until the second is down or its track ends; the results say `hits 1/2`, and the takes still have `hit_1` but no `hit_2`, and `hit` is missing (not "the first hit"). Tests: Task 6, `test_one_of_two_hit`; Task 8, `test_takes_one_of_two_hit`.
4. **Both hostiles near one defender:** the base station must still send two defenders, not one twice. Test: Task 6, `test_two_defenders_sent`.
5. **Old single-hostile results and runs through the new code:** rows without `hostiles` still summarise as before; a single-hostile recording still gives the same moments. Tests: Task 7, `test_old_rows_summarise_unchanged`; Task 8, `test_single_hostile_moments_unchanged` (the existing takes tests must pass untouched).

---

## Step 1: birds

### Task 1: Birds flown by a helper with a `bird_` prefix

**Files:**
- Create: `ros2_ws/src/swarm_backend/swarm_backend/scenarios/bird_flight.py`
- Modify: `ros2_ws/src/swarm_backend/swarm_backend/intercept/inbound.py` (`plan_birds`)
- Test: `ros2_ws/src/swarm_backend/tests/test_bird_flight.py`, `tests/test_inbound.py`

**Interfaces:**
- Produces: `plan_birds(..., prefix: str = 'scn_bird_')` (new keyword, names `f'{prefix}{i}'`; the default keeps scenario 2 as it is).
- Produces: `class BirdFlight` in `bird_flight.py`: `__init__(self, world: str, log_path: str = '/tmp/birds.log')`, `start(self, plans: list[BirdPlan], t0: float) -> None` (starts `python3 -m swarm_backend.birds fly` with `{'world', 't0', 'birds': [asdict(b)]}` on stdin, as `tests/scenario2_test.py` `start_birds` does), `stop(self) -> None` (terminate, wait 60 s, kill; a no-op when not started or already stopped), `running: bool` property.

- [ ] **Step 1: Write the failing tests** `test_inbound.py::test_plan_birds_prefix`: `plan_birds(3, [(100, 0, -40)], (0, 0), ['dove'], 4.0, random.Random(0), prefix='bird_')` names are `['bird_0', 'bird_1', 'bird_2']`; without `prefix` they start with `scn_bird_`. `test_bird_flight.py::test_start_writes_the_plan` (monkeypatch `subprocess.Popen` with a fake that records `args` and stdin): the command ends with `['-m', 'swarm_backend.birds', 'fly']` and the JSON has `world`, `t0` and 3 birds with `start`, `end`, `speed`, `kind`, `name`. `test_stop_twice_is_harmless`: `stop()` before `start()` and twice after it raises nothing and terminates once.
- [ ] **Step 2:** Run them; expect failures (`unexpected keyword argument 'prefix'`, `ModuleNotFoundError: bird_flight`).
- [ ] **Step 3:** Implement. Lift `start_birds`/`remove_birds` from `tests/scenario2_test.py` into `BirdFlight` and make scenario 2 use it (its behaviour unchanged).
- [ ] **Step 4:** Run `tests/test_bird_flight.py tests/test_inbound.py tests/test_scenario2*.py`; expect PASS.
- [ ] **Step 5:** Commit: `Birds: BirdFlight helper, plan_birds name prefix`.

### Task 2: `--birds N` in the 3-defender runs

**Files:**
- Modify: `swarm_backend/scenarios/surveillance.py` (`fly`, two hooks), `swarm_backend/scenarios/gazebo_surveillance.py`, `swarm_backend/scenarios/sim_surveillance.py`, `swarm_backend/scenarios/baseline_eval.py` (parser, `runs`, `common_fields`)
- Test: `tests/test_surveillance_birds.py`

**Interfaces:**
- Consumes: `BirdFlight`, `plan_birds(prefix='bird_')` (Task 1).
- Produces: `SurveillanceWorld.start_extras(self, start: Vector3, goal: Vector3, speed: float) -> None` and `stop_extras(self) -> None`: no-ops in the base. `fly()` calls `start_extras` after `place_hostile` and `wait_sim(SETTLE_S)`, right before the hostile sets off, and `stop_extras` first thing in its `finally`. The world has `birds: int = 0` and `bird_seed: int | None = None` attributes that `baseline_eval` sets.
- Produces: `GazeboSurveillance.start_extras`: when `self.birds > 0`, plans `self.birds` birds with `plan_birds(self.birds, [start], (0.0, 0.0), list(KINDS), speed, random.Random(self.bird_seed), prefix='bird_')` and starts them with `t0 = self.clock()`. `stop_extras` stops them.
- Produces: `SimSurveillance`: logs `--birds ignored in the fast simulation (no camera images)` once when `birds > 0`.
- Produces: `baseline_eval` options `--birds N` (int, default 0) and `--bird-seed S` (default: the run's seed); results rows get `birds` and `bird_seed` only when `birds > 0` (old rows unchanged).

- [ ] **Step 1: Write the failing tests** (a `SimSurveillance` subclass that records hook calls): `test_hooks_called_around_the_flight`: one `fly()` calls `start_extras(start, goal, speed)` once, before the hostile moves, and `stop_extras()` once after. `test_birds_stopped_when_the_run_fails`: a subclass whose `pause()` raises `RuntimeError` on the 3rd call: `stop_extras()` was still called. `test_birds_option_in_rows`: `baseline_eval.main(['intercept', '--sim', '--seeds', '0', '--birds', '8', '--out', tmp])` writes a row with `birds == 8` and `bird_seed == 0`; without `--birds` the row has neither key.
- [ ] **Step 2:** Run; expect failures.
- [ ] **Step 3:** Implement as in Interfaces. `trial.sh` and `compare_paths.sh` pass unknown options on to `baseline_eval`, so nothing changes there (check `trial.sh --help` text mentions `--birds N`; add one line).
- [ ] **Step 4:** Run `tests/test_surveillance_birds.py` and the full suite; expect PASS.
- [ ] **Step 5:** Commit: `Birds in the 3-defender runs: --birds N`.

### Task 3: Birds green in the films

**Files:**
- Modify: `swarm_backend/ui/camera_video.py` (`label_lines`, `COLOURS`, the overlay's model filter), `swarm_backend/demo/map_video.py` (bird names and colour), `ros2_ws/src/swarm_backend/swarm_backend/demo/fake_run.py` (`make_run(..., birds_late: bool = False)`: a `bird_7` that appears in the pose rows only after the release)
- Test: `tests/test_camera_video.py`, `tests/test_demo_map.py` (or the existing map test file)

**Interfaces:**
- Produces: `label_lines('bird_3', None, 'x500_4') == (['bird'], 'bird')`; `COLOURS['bird'] == (80, 200, 80)`. The overlay labels models whose name starts with `x500` or with the run's birds prefix (`run.json` `birds_prefix`, default `bird_`), checked before `drone_id`, so `bird_1` is never "D1 defender".
- Produces: the map draws every name with the birds prefix found in **any** pose row, as a green dot `(80, 200, 80)`.

- [ ] **Step 1: Write the failing tests**: `test_bird_label`: the `label_lines` line above. `test_overlay_labels_birds_green` (a 1280x720 black frame, a camera at the origin looking at `bird_1` 20 m ahead): `draw_overlay` puts pixels of `(80, 200, 80)` near the image centre. `test_map_draws_birds_that_appear_later`: `make_run(..., birds_late=True)`, a map frame after the release has green pixels at `bird_7`'s projected position.
- [ ] **Step 2:** Run; expect failures (`KeyError: 'bird'`, no green pixels).
- [ ] **Step 3:** Implement as in Interfaces.
- [ ] **Step 4:** Run `tests/test_camera_video.py tests/test_demo_*.py`; expect PASS.
- [ ] **Step 5:** Commit: `Birds green in the camera clips and on the map`.

### Task 4: Birds on Stellar (one pair)

- [ ] **Step 1:** `tmux ls` (no `hive-rec`, no shoot running). Add to `recording/shoots/quick-check.yaml` an entry `birds`: `run: scenarios/trial.sh 1 --birds 8`, `compare: [baseline, yolo-bytetrack-mpc]`, `seeds: 0`; run only it: a shoot file `recording/shoots/check-birds.yaml` with that one entry, `recording/shoot.sh recording/shoots/check-birds.yaml --name check-birds` in tmux.
- [ ] **Step 2:** Check: `logs/scenario.log` shows `8 birds spawned` and `8 birds removed` per run; `poses.jsonl` has `bird_` rows; `recording/video.sh final --story compare --preview check-birds` shows green birds in the chase clip and on the map. Note the result of each side (hit or not, distracted or not).
- [ ] **Step 3:** Commit the shoot file: `Shoot: birds check`.

## Step 2: two hostiles at once

### Task 5: The two-hostile config and spawning

**Files:**
- Create: `ros2_ws/src/swarm_backend/scenarios/surveillance2.yaml` (a copy of `surveillance.yaml` plus drone 5, `gz_x500`, `spawn_pose: [-5.0, 4.0, 0.0]`, and `attack.second_target_id: 5`)
- Modify: `swarm_backend/config.py` (`AttackConfig.second_target_id: int | None = None`, validated: not a defender, not `target_id`), `scenarios/lib.sh` (`surveillance_hostile` prints `target_id` and, when set, `second_target_id`, space-separated)
- Test: `tests/test_config.py` (or the file that tests `load_config`)

**Interfaces:**
- Produces: `config.attack.second_target_id` (None in every existing config). `SurveillanceWorld.hostile_ids: tuple[int, ...]` = `(target_id,)` or `(target_id, second_target_id)`.

- [ ] **Step 1: Write the failing tests**: `test_second_target_loads`: `load_config(scenarios/surveillance2.yaml).attack.second_target_id == 5` and drone 5 is in `drones`; `surveillance.yaml` gives `None`. `test_second_target_must_differ`: a config with `second_target_id` equal to `target_id` or to a defender raises `ValueError`.
- [ ] **Step 2:** Run; expect failures.
- [ ] **Step 3:** Implement. Keep `surveillance2.yaml` in sync by a test (`test_surveillance2_matches`): every key except `drones` and `attack.second_target_id` equals `surveillance.yaml`'s.
- [ ] **Step 4:** Run the config tests and the full suite; expect PASS.
- [ ] **Step 5:** Commit: `Two-hostile config: surveillance2.yaml, drone 5`.

### Task 6: `fly_many`: several hostiles in one run (both backends)

**Files:**
- Modify: `swarm_backend/scenarios/surveillance.py`, `swarm_backend/scenarios/gazebo_surveillance.py`, `swarm_backend/scenarios/sim_surveillance.py`
- Test: `tests/test_fly_many.py`

**Interfaces:**
- Consumes: `hostile_ids` (Task 5).
- Produces (surveillance.py):
  - `@dataclass Leg: hostile_id: int; start: Vector3; goal: Vector3; speed: float; path: list[Vector3] | None = None`
  - `@dataclass HostileResult: hostile_id: int; killer: int | None; hit_time: float | None  # s since release; closest: float  # m, any defender; closest_by: int | None`
  - `@dataclass MultiFlight: t0: float; t_end: float; results: list[HostileResult]; engaged: dict[int, tuple[float, int]]  # hostile -> (t, defender) of the first ENGAGE whose order targets it, or the first ENGAGE after its release when the order carries no target`
  - `SurveillanceWorld.fly_many(self, legs: list[Leg], timeout: float = math.inf) -> MultiFlight`: all legs placed (one after the other), `wait_patrol`, `wait_sim(SETTLE_S)`, `start_extras(legs[0].start, legs[0].goal, legs[0].speed)`, all released together; a `ClosestApproach` per (defender, hostile); a hit removes that hostile only and calls `report_kill(killer)`; ends when every hostile is down, every track is done, or at `timeout`; `finally`: `stop_extras`, every remaining hostile removed.
- Produces (backends): `place_hostile(start, goal, speed, hostile_id: int | None = None)` and `remove_hostile(hostile_id: int | None = None)`; `None` means `target_id` (so `fly()` is unchanged). Gazebo: per-hostile `PX4Drone`, `Px4Process`, `SwarmManager`, pose and graveyard; drone 4's graveyard stays `GRAVEYARD_ENU = (450.0, -450.0)`, drone 5's is `(450.0, -430.0)` (inside `takes.GRAVEYARD_M` of the configured `graveyard_enu`). `_on_truth` feeds `observe_truth_many(stamp, bodies, {hostile_id: position})`. The hostile reset at the end (`stop_and_land`) resets every hostile to its spawn point. Sim: a `SimDrone` per hostile; each defender's fake detector observes the nearest present hostile.

- [ ] **Step 1: Write the failing tests** (SimSurveillance with `surveillance2.yaml`, `--detector fake`): `test_two_hostiles_both_hit`: legs from drone 1's and drone 3's sectors (bearings −40° and +40° from the threat heading, 300 m out, 4 m/s): both results have a `killer` and a `hit_time`. `test_two_defenders_sent`: the two killers differ. `test_one_of_two_hit` (the second leg at 30 m/s with `timeout` 60): one result hit, the other `killer is None` with its `closest`, and the run ended at the track's end or the timeout, not at the first hit. `test_fly_unchanged`: `fly()` on `surveillance.yaml` gives the same `Flight.killer` and `hit_time` for seed 0 as before (record them from the current code first).
- [ ] **Step 2:** Run; expect failures.
- [ ] **Step 3:** Implement as in Interfaces. Refactor the Gazebo backend's hostile fields into one small class per hostile (`_Hostile`: id, name, drone, px4, manager, pose, graveyard) with `self._hostiles: dict[int, _Hostile]`; `self.hostile`, `self.px4`, `self.hostile_manager` stay as aliases of `target_id`'s for the single-hostile code.
- [ ] **Step 4:** Run `tests/test_fly_many.py`, the surveillance and baseline_eval tests and the full suite; expect PASS.
- [ ] **Step 5:** Commit: `fly_many: several hostiles in one run`.

### Task 7: `--hostiles 2` in baseline_eval

**Files:**
- Modify: `swarm_backend/scenarios/baseline_eval.py` (parser, `jobs`, `runs`, `summary_text`), `scenarios/trial.sh` (help line)
- Test: `tests/test_baseline_eval_hostiles.py`

**Interfaces:**
- Consumes: `fly_many`, `Leg`, `MultiFlight` (Task 6).
- Produces: `--hostiles N` (1 default, 2 allowed; 2 needs a config with `second_target_id`, else `SystemExit` naming `scenarios/surveillance2.yaml`). With 2: per seed, hostile 1 starts in drone 1's sector and hostile 2 in drone 3's, each at a bearing from `random.Random(seed)` inside its sector, same speed, altitude and path options; results go to `<out>/intercept_2.jsonl` (never mixed with single-hostile rows). Row fields: the common ones plus `hostiles: [{'id', 'hit', 'killer', 'hit_time_s', 'closest_m', 'start_bearing_deg'}]` and `outcome: 'hits N/2'`.
- Produces: `summary_text` for rows with `hostiles`: one line per run (`seed 0: hits 2/2, drones 1 and 3, 14.2 s / 17.9 s`) and a total (`hostiles hit 3/4`). Rows without `hostiles` go through the current code untouched.

- [ ] **Step 1: Write the failing tests**: `test_hostiles_2_rows`: `main(['intercept', '--sim', '--config', <surveillance2.yaml>, '--hostiles', '2', '--seeds', '0', '--out', tmp])` writes one row in `intercept_2.jsonl` with two `hostiles` entries whose `start_bearing_deg` are on opposite sides of 0. `test_hostiles_2_needs_the_config`: with `surveillance.yaml` it exits, the message names `surveillance2.yaml`. `test_old_rows_summarise_unchanged`: `summary_text` on a copy of a single-hostile `intercept.jsonl` (written by the current code into a fixture first) gives the same text as before the change.
- [ ] **Step 2:** Run; expect failures.
- [ ] **Step 3:** Implement as in Interfaces. `trial.sh` passes `--hostiles` through; it waits for drones `$defenders $(surveillance_hostile)`, which now includes 5 under `surveillance2.yaml`.
- [ ] **Step 4:** Run the new tests and the full suite; expect PASS.
- [ ] **Step 5:** Commit: `baseline_eval --hostiles 2`.

### Task 8: Two hostiles in the film tools

**Files:**
- Modify: `swarm_backend/demo/takes.py`, `swarm_backend/demo/story.py` (moment names), `swarm_backend/demo/map_video.py`, `swarm_backend/ui/camera_video.py` (the `@attacker` choice), `swarm_backend/demo/fake_run.py` (`make_run(..., hostiles: int = 1)`)
- Test: `tests/test_demo_takes.py`, `tests/test_demo_story.py`

**Interfaces:**
- Produces: `MOMENTS` gains `first_sighting_1`, `first_sighting_2`, `hit_1`, `hit_2` (per hostile, in `run.json` `hostiles` order). In a two-hostile take `hit` = the later of `hit_1`/`hit_2` and only when both exist; `first_sighting` = the earlier sighting. A take's facts gain `hits` (`'2/2'`) and per-hostile `attacker_1`, `attacker_2`, `sighting_range_m_1`, `sighting_range_m_2`, `sighting_to_hit_s_1`, `sighting_to_hit_s_2`. A sighting belongs to the hostile whose true position at that sim time is nearest the sighting's position (UI detection estimate); release = the first hostile's release.
- Produces: the map draws every hostile in `run.json` `hostiles` (red trail, hit tick each). The live/replay `@attacker` is the defender nearest any present hostile (`recording_config.json` gains `"hostiles": ["x500_4", "x500_5"]`; `"hostile"` stays for old runs).

- [ ] **Step 1: Write the failing tests**: `test_two_hostile_moments`: `make_run(tmp, hostiles=2)`: one take with `hit_1`, `hit_2`, `hit == max(hit_1, hit_2)`, facts `hits == '2/2'` and `attacker_1 != attacker_2`. `test_takes_one_of_two_hit`: `make_run(tmp, hostiles=2, hit=[True, False])`: `hit_1` present, `hit_2` and `hit` absent, `hits == '1/2'`. `test_single_hostile_moments_unchanged`: the existing takes tests pass untouched, and a single-hostile take has none of the `_1`/`_2` moments. `test_story_accepts_hit_2`: a scene `from: hit_2 - 3`, `to: hit_2` resolves.
- [ ] **Step 2:** Run; expect failures.
- [ ] **Step 3:** Implement as in Interfaces.
- [ ] **Step 4:** Run `tests/test_demo_*.py tests/test_camera_video.py` and the full suite; expect PASS.
- [ ] **Step 5:** Commit: `Film tools: two hostiles`.

### Task 9: Two hostiles on Stellar (one pair)

- [ ] **Step 1:** `tmux ls`; `recording/shoots/check-two.yaml`: `name: two-hostiles`, `run: scenarios/trial.sh 1 --hostiles 2`, `config: /workspace/ros2_ws/src/swarm_backend/scenarios/surveillance2.yaml`, `compare: [baseline, yolo-bytetrack-mpc]`, `seeds: 0`. `recording/shoot.sh recording/shoots/check-two.yaml --name check-two` in tmux.
- [ ] **Step 2:** Check: both hostiles fly at once, two different defenders engage, the run stops after both are down (quick end does not trigger on the first one: `Defenders back to their slots` is only logged at the end); `video.sh moments check-two` shows `hit_1`, `hit_2`; `final --story compare --preview check-two` cuts.
- [ ] **Step 3:** Commit: `Shoot: two-hostile check`.

## Step 3: the examiner shoot and film

### Task 10: `examiner.yaml` shoot and story

**Files:**
- Create: `recording/shoots/examiner.yaml` (the design's shoot: `overview`, `head-on`, `zigzag`, `birds` with `--birds 8`, `two-hostiles` with `--hostiles 2` and `config:` `surveillance2.yaml`; all `seeds: 0`), `recording/stories/examiner.yaml`
- Test: `tests/test_demo_examiner.py`

**Interfaces:**
- Produces: `stories/examiner.yaml`: title card (3 s); overview (`cam:base_orbit`, 8 s, `take: overview/1`); per scenario a card (2 s), a `compare` scene of the pair (`take: pair:<entry>-0`, from `release + 5` to `hit + 2`, `speed: 2`), and for head-on and two-hostiles a `ghost` scene (same range, `speed: 2`); for birds a `side` scene `[chase, yolo]` of ours around `first_sighting`; for two hostiles a `grid` `[chase, ui|map, "wide|map|plot:range", yolo]` from `first_sighting - 3` to `hit + 2`; the `compare` end card with `evidence: data/baseline_eval`.
- The pair names follow `shoot.py`'s naming (`<name>-<seed>`); check them with `shoot.sh --dry-run`.

- [ ] **Step 1: Write the failing test** `test_examiner_story_fits_5_min`: on a made-up shoot (`fake_run` with the four pairs and the overview, one-hostile runs except `two-hostiles`), `final.build(..., preview=True)` of `stories/examiner.yaml` writes a film of at most 300 s, and `resolve` reports no missing moments.
- [ ] **Step 2:** Run; expect failure (no story).
- [ ] **Step 3:** Write both files; shorten `speed`/ranges until the test passes with at least 20 s to spare.
- [ ] **Step 4:** `recording/shoot.sh recording/shoots/examiner.yaml --dry-run`: 9 runs, the `two-hostiles` runs with `SWARM_CONFIG=.../surveillance2.yaml`.
- [ ] **Step 5:** Commit: `Examiner shoot and story`.

### Task 11: The shoot and the film

- [ ] **Step 1:** Machine check (`/etc/stellar/booking.json`, `nvidia-smi`, `df -h /`: at least 10 GB free), `tmux ls`. Then `tmux new -d -s shoot 'recording/shoot.sh recording/shoots/examiner.yaml --name examiner'` (about 1.5 h).
- [ ] **Step 2:** When done: `recording/video.sh moments examiner`; every pair has both sides; note each side's result.
- [ ] **Step 3:** `recording/video.sh final --story examiner --preview examiner`, look at frames of each scene (the pair in sync at `release`, green birds, both hits); fix the story if a scene is off; then `recording/video.sh final --story examiner --stills examiner`. Check `ffprobe` duration ≤ 300 s.
- [ ] **Step 4:** Record the results in `hive-vault/detection/Manual tests to do.md` (Done: the shoot, each pair's results, the film's length and path) and commit: `Examiner shoot: results`.
