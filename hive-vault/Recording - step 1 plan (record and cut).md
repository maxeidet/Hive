# Recording step 1 (record and cut): implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** One command records a run without clicking. A story file then cuts any run into a finished demo film, with no sim times typed and no code edited.

**Architecture:** New pure-Python package `swarm_backend/demo/` (no ROS imports except the pose logger): `runinfo` (who is who in a run), `takes` (takes, moments, facts), `story` (the YAML story and its moment expressions), `map_video` (the tactical map as a video source) and `final` (cuts the sources it needs through the existing cut CLIs, builds one clip per scene with ffmpeg, then concatenates them). `recording/record.sh` is a bash orchestrator on the host that drives the existing recorders in a tmux session. A synthetic run folder (`demo/fake_run.py`) stands in for real Stellar data in every test.

**Tech Stack:** Python 3.12 in the `hive-px4-vnc` container (PyYAML, OpenCV, numpy, ffmpeg are in the image), bash + tmux on the host, pytest.

**Spec:** `hive-vault/Recording - smoother demo videos (design).md` (approved 23 Nov 2026). This plan is the design's step 1 ("Record and cut"). Steps 2-4 (shoot.sh, comparison, insights, extras) get their own plans. The design's "`record.sh` is a one-entry shoot" is met from the other side: step 2's `shoot.sh` calls `record.sh` once per entry (`--name`, `--keep-sim`), so both go through the same code.

## Global Constraints

- Every cut and every film is on sim time: frame i of a clip that starts at S shows sim time `S + i / 25`.
- Film output: H.264, 1920x1080, 25 fps, `yuv420p`, `-movflags +faststart`. Preview: 960x540, `-preset ultrafast`, `-crf 28`.
- No Chromium or any browser inside the container. The UI film is optional (`--screen`, off by default), and the `map` source is the default overview.
- Nothing is installed on the host: Python runs in the container (`docker exec px4-gazebo-vnc ...` or `video.sh`'s temporary container). The host side is bash, tmux and a desktop browser.
- Don't change the detection or attack code, or the topics `/px4_N/detection/target_position` and `target_offset`.
- Claude never flies the sim. Live checks go to `hive-vault/detection/Manual tests to do.md`.
- **No commits and no pushes.** Oskar commits. Every task ends with "leave the changes uncommitted".
- After editing any `.sh` file, run `chmod +x` on it.
- Unit tests: `docker exec px4-gazebo-vnc bash -c "cd /workspace/ros2_ws/src/swarm_backend && python3 -m pytest -q tests/test_demo_*.py"`. The full suite (`tests/test_*.py`) must still pass at the end.

## Review Focus

1. **A take without a hit** (the hostile reached the base): moments after `hit` are missing. `final` stops and names the scene and the missing moment. With `--skip-missing` it builds the rest. Test: Task 5, `test_missing_moment_names_scene`. Task 7, `test_skip_missing_builds_the_rest`.
2. **A source that starts later than the scene:** for example `camrec` was started 10 s after takeoff. The debug and camera cuts already hold the first frame; the map must do the same. The screen cut refuses, so `final` must report that clearly instead of failing deep in ffmpeg. Tests: Task 6, `test_map_before_first_pose_holds`. Task 7, `test_screen_too_late_is_reported`.
3. **Moment expressions as people type them:** `hit-2.5`, `takeoff +12`, ` first_sighting - 5 `, `hit`, `release + 0.5`, and the unknown `hti - 2`, which gets an error naming the valid moments. Test: Task 5, `test_moment_expressions`.
4. **A recording with a single run where the hostile is never moved into place again,** so there are no teleports: that's one take, starting at the recording's start. Test: Task 3, `test_single_take_without_teleport`.
5. **Ctrl-C during `record.sh`:** the stop order still runs, and the tmux session and the temporary containers are gone afterwards. Test: Task 9, `test_dry_run_ctrl_c_order`, which checks the printed plan for the INT path. The real behaviour is a manual test.

---

## File Structure

| File | Responsibility |
|---|---|
| `ros2_ws/src/swarm_backend/swarm_backend/demo/__init__.py` | empty |
| `.../demo/fake_run.py` | writes a synthetic run folder (ui.jsonl, poses.jsonl, run.json, clips and their .json) for tests and dry runs |
| `.../demo/runinfo.py` | `run.json`: write from config + command, read the roles |
| `.../demo/takes.py` | takes, moments and facts from a run folder; `python3 -m swarm_backend.demo.takes RUN` prints them |
| `.../demo/poselog.py` | ROS-free CLI wrapper + the always-on pose logger (`mount-backend.sh posesrec`) |
| `.../demo/story.py` | load and validate a story YAML, parse moment expressions, resolve scenes against takes |
| `.../demo/map_video.py` | the `map` source: draws the tactical map from poses + ui.jsonl to an mp4 on sim time |
| `.../demo/final.py` | builds a film: cuts the needed sources (cached), one clip per scene, cards, concat, preview |
| `ros2_ws/src/swarm_backend/tests/test_demo_*.py` | tests per module |
| `recording/record.sh` | the host orchestrator (tmux, start order, readiness, stop order, health, `--dry-run`) |
| `recording/stories/default.yaml` | the 18 Nov cut as a story |
| `recording/final_cut.py` | **deleted** (replaced by `demo/final.py` and the story file) |
| `recording/video.sh` | new `moments` and `map`; `final` calls `demo.final` |
| `recording/screen.py` | `record --fullscreen` (the root window's size, no click) |
| `recording/recording_config.json` | `map` settings, `pose_log` prefixes |
| `docker/start-stellar.sh` | `RECORD_RUN` env: the run folder name to record into |
| `ros2_ws/src/swarm_backend/mount-backend.sh` | `posesrec` subcommand; header lines |
| `ros2_ws/src/swarm_backend/swarm_backend/ui/camera_video.py` | `record --no-pose-log` |
| `recording/README.md`, `hive-vault/Recording demo videos.md`, `hive-vault/detection/Manual tests to do.md` | docs |

Run folder (unchanged where it exists, new files marked ★):

```
data/recordings/<run>/
  run.json ★  ui.jsonl  poses.jsonl  gz/  screen.mkv/.json (only --screen)
  debug_d<ID>.mkv/.json  cam_<angle>.mkv/.json  takes.json ★  logs/ ★ (one log per part)
  cache/ ★ (cut sources)  scenes/<story>/NN_<slug>.mp4 ★  final_<story>.mp4 ★
```

---

### Task 1: Synthetic run folder

**Files:**
- Create: `swarm_backend/demo/__init__.py`, `swarm_backend/demo/fake_run.py`
- Test: `tests/test_demo_fake_run.py`

**Interfaces:**
- Produces: `make_run(folder: Path, *, takes: int = 1, hit: list[bool] | None = None, teleport: bool = True, videos: bool = True, cam_start_delay: float = 0.0, screen: bool = False) -> dict`. It writes the folder and returns `{'release': [...], 'sighting': [...], 'hit': [...], 'attacker': [...]}`, the true sim times it placed (one per take; `None` for no hit), for the tests to compare against.
- CLI: `python3 -m swarm_backend.demo.fake_run DIR [--takes N]`.

The layout it writes, which every later task relies on, matches the real files:
- `ui.jsonl`: Recorder lines `{"sim","wall","state":{"drones":[...],"events":[...]}}`. The drones are ids 1-3 (defenders) and 4 (hostile, `"hostile": true`). Each has `north`, `east`, `altitude`, `status`, `armed`, `gimbal_heading_deg` and `detection`. The events use the real texts from `ui/model.py`: `Drone N armed`, `Drone N sees the target`, `Drone N searching → intercept`, `Drone N hit the target`, `Drone N back to searching`, `Drone N landed and disarmed`. `wall` = `1763460000 + sim / 0.5` (the sim at half speed).
- `poses.jsonl`: `{"sim", "poses": {name: [x, y, z, qx, qy, qz, qw]}}` at 25 Hz, Gazebo ENU. The names are `x500_gimbal_1..3`, `x500_4` and `bird_1`. For each take the hostile is teleported (>100 m jump) to its start, waits 3 s, flies at 4 m/s toward the base, and on a hit jumps to the graveyard `[450, -450, z]`.
- `run.json`: as Task 2 writes it, with config `scenarios/surveillance.yaml` values inlined.
- With `videos`: `debug_d<attacker>.mkv/.json`, `cam_attacker_watch.mkv/.json`, `cam_arc_side.mkv/.json` from `ffmpeg -f lavfi testsrc2` with the sim time drawn in (`drawtext`). Their `.json` carries `start`, `end` and `fps` (25), and for cameras `width`, `height`, `hfov`, `model` and `angle`, in the same shape as `DroneVideo` writes. `cam_start_delay` shifts the cameras' `start`. With `screen`: `screen.mkv` + `screen.json`, the film stamped on the wall clock (`-output_ts_offset` = first wall time).

- [ ] **Step 1: Write the failing test** `test_make_run_layout`: `make_run(tmp, takes=2)` creates `ui.jsonl`, `poses.jsonl`, `run.json`, `debug_d3.mkv`, `cam_arc_side.json`, and the returned dict has 2 entries per key. `load_recording(tmp / 'ui.jsonl')` (from `ui/replay.py`) loads it without errors, and its events include `Drone 3 hit the target`.
- [ ] **Step 2:** Run it and check it fails (`ModuleNotFoundError: swarm_backend.demo`).
- [ ] **Step 3:** Implement `make_run`. The attacker is drone 3 in take 1 and drone 2 in take 2, so tests notice a hard-coded attacker.
- [ ] **Step 4:** Run the test and check it passes. `ffprobe` on `debug_d3.mkv` gives 25 fps.
- [ ] **Step 5:** Leave uncommitted.

### Task 2: `run.json`, the run describes itself

**Files:**
- Create: `swarm_backend/demo/runinfo.py`
- Test: `tests/test_demo_runinfo.py`

**Interfaces:**
- Produces:
  - `write_run_info(folder: Path, config_path: Path, command: list[str], options: dict) -> dict`
  - `read_run_info(folder: Path) -> RunInfo`
  - `@dataclass RunInfo: command: list[str]; config: dict; setup: str | None; seeds: list[int] | None; defenders: list[str]; hostiles: list[str]; birds_prefix: str; options: dict`
  - CLI: `python3 -m swarm_backend.demo.runinfo write RUN --config CFG --options JSON -- CMD...`
- Gazebo names: `<model without "gz_">_<id>`, with model defaulting to `gz_x500`. That's the same rule as `save_images.drone_names`. The defenders are `surveillance.drone_ids`, the hostiles `[attack.target_id]` (a `scenario2.hostile_ids` list wins if present), and `birds_prefix` is `"bird_"`.
- `setup`: the value after `--setup` in `command`, else `common.DEFAULT_SETUP`. `seeds`: the `--seeds` value parsed with `baseline_eval.parse_seeds` (`"0-2"` gives `[0, 1, 2]`), else None.
- The config YAML is copied into `run.json` (`config`), so a later change to the yaml doesn't change the run.

- [ ] **Step 1: Write the failing tests:**
  - `test_roles_from_surveillance_yaml`: with `scenarios/surveillance.yaml`, defenders are `['x500_gimbal_1', 'x500_gimbal_2', 'x500_gimbal_3']` and hostiles `['x500_4']`.
  - `test_roles_scenario2`: with `scenarios/scenario2.yaml`, hostiles are `['x500_2', 'x500_3']`.
  - `test_setup_and_seeds_from_command`: `['scenarios/trial.sh', '3', '--setup', 'baseline', '--seeds', '0-2']` gives `setup == 'baseline'` and `seeds == [0, 1, 2]`.
  - `test_round_trip`: write, then read, gives equal roles.
- [ ] **Step 2:** Run them and check they fail.
- [ ] **Step 3:** Implement. Don't import the ROS parts: `scenarios/common.py` and `baseline_eval.parse_seeds` must import without rclpy; if they don't, copy the two tiny bits and note it in a comment.
- [ ] **Step 4:** Run the tests and check they pass.
- [ ] **Step 5:** Leave uncommitted.

### Task 3: Takes, moments and facts

**Files:**
- Create: `swarm_backend/demo/takes.py`
- Test: `tests/test_demo_takes.py`
- Modify: `recording/video.sh` (new `moments [RUN]`: `docker exec -w $PKG $C python3 -m swarm_backend.demo.takes /workspace/$RUNS/$RUN`)

**Interfaces:**
- Consumes: `read_run_info` (Task 2), `ui.replay.load_recording` / `ReplayUi`, `ui.camera_video.load_poses` / `pose_at`.
- Produces:
  - `MOMENTS = ('start', 'release', 'takeoff', 'first_sighting', 'intercept', 'hit', 'return', 'landed', 'end')`
  - `@dataclass Take: number: int; moments: dict[str, float]; attacker: int | None; target: str; facts: dict[str, float | bool | int | None]`
  - `find_takes(folder: Path) -> list[Take]`
  - `save_takes(folder: Path, takes: list[Take]) -> Path`, which writes `takes.json`
  - `source_ranges(folder: Path) -> dict[str, tuple[float, float]]`, the sim range per recorded file (from each `.json`, `ui.jsonl`, `poses.jsonl`)
  - CLI prints one block per take: number, each moment as `name  sim  (T+ from release)`, then the facts.

Rules (from design sections 3 and 3b):
- **Take boundaries:** a teleport of the hostile is a jump of more than 50 m between consecutive pose rows, not into the graveyard. Graveyard = within 30 m of `recording_config.json`'s `graveyard_enu` (x, y). Take k runs from the k-th teleport until the next one; the last runs to the end. With no teleport there's one take from the first pose.
- `release` = the first time after the take starts when the hostile has moved more than 0.5 m/s for 1 s.
- `takeoff` / `landed` = the first `armed` / last `landed and disarmed` event of any defender within the take, else the recording's first or last (the defenders stay up between runs).
- `first_sighting` / `intercept` / `hit` / `return` = the first matching event within the take (texts as in Task 1). `hit` is cross-checked: the hostile must be in the graveyard within 2 s after it, else it's dropped with a warning on stderr.
- `attacker` = the drone of the `hit` event, else of the first `intercept`.
- `start` / `end` = the take's boundaries clipped to the overlap of all recorded sources (`source_ranges`).
- **Facts:** `hit` (bool), `attacker`, `sighting_range_m` (true 3-D distance attacker to hostile at `first_sighting`, from poses), `release_to_sighting_s`, `sighting_to_hit_s`, `closest_pass_m` (minimum attacker-hostile distance from `release` to `hit` or the take's end, never counting graveyard samples), `setup` (from run info). Each fact is one function `fact_<name>(take_data) -> value` registered in a `FACTS` dict, so steps 2 and 3 add more by adding a function.

- [ ] **Step 1: Write the failing tests** against `make_run`:
  - `test_two_takes_found`: `make_run(tmp, takes=2)` gives 2 takes. Each `release` is within 0.1 s of the true one, and `hit` and `first_sighting` within one UI sample (0.1 s).
  - `test_attacker_per_take`: the attackers are `[3, 2]`.
  - `test_take_without_hit`: `make_run(tmp, takes=2, hit=[True, False])`. Take 2 has no `hit`, no `return`, `facts['hit'] is False`, and `sighting_to_hit_s is None`.
  - `test_single_take_without_teleport` (Review Focus 4): `make_run(tmp, takes=1, teleport=False)` gives one take with `moments['start']` equal to the first pose's sim, and `release` found.
  - `test_facts_values`: the `closest_pass_m` of a hit take is below 1.6, and `sighting_range_m` is within 1 m of the distance computed from the poses at the true sighting.
  - `test_takes_json_written`: `save_takes` writes valid JSON with the same numbers.
- [ ] **Step 2:** Run them and check they fail.
- [ ] **Step 3:** Implement. Pose lookups use `camera_video.pose_at`. Events come from `ReplayUi.event_table()`.
- [ ] **Step 4:** Run the tests and check they pass. Then `recording/video.sh moments` on a folder from `python3 -m swarm_backend.demo.fake_run data/recordings/fake-1 --takes 2` prints two blocks. Remove the fake folder afterwards.
- [ ] **Step 5:** Leave uncommitted.

### Task 4: Poses always logged

Takes and the map need `poses.jsonl`, but today only `camrec` writes it, and only for `x500*` and `rec_cam_*`.

**Files:**
- Create: `swarm_backend/demo/poselog.py`
- Modify: `swarm_backend/ui/camera_video.py` (`record --no-pose-log`: don't open `poses.jsonl`. Without the flag, behaviour is unchanged)
- Modify: `mount-backend.sh` (`posesrec) run "python3 -m swarm_backend.demo.poselog $*" ;;` plus a header line)
- Modify: `recording/recording_config.json` (`"pose_log": {"prefixes": ["x500", "bird_", "rec_cam_"], "hz": 25}`)
- Test: `tests/test_demo_poselog.py`

**Interfaces:**
- Consumes: `camera_video.PoseFeed` (its log format), `debug_video.run_folder('auto')`, `read_run_info` (the roles add exact names).
- Produces:
  - `keep_names(prefixes: list[str], roles: list[str]) -> Callable[[str], bool]`
  - CLI `python3 -m swarm_backend.demo.poselog [--out auto|DIR]`, which prints `Logging poses of <world> to <file>` once subscribed and runs until Ctrl-C.

- [ ] **Step 1: Write the failing test** `test_keep_names`: keeps `x500_4`, `bird_3`, `rec_cam_arc_side` and a role name `standard_vtol_5`, and drops `base_link` and `ground_plane`.
- [ ] **Step 2:** Run it and check it fails.
- [ ] **Step 3:** Implement. `poselog` creates `PoseFeed(world, folder / 'poses.jsonl', keep_names(...))` with `world` from `fake_detection.find_scene`'s first value, or the `gz topic -l` world, as `operator_ui.start_recording` does.
- [ ] **Step 4:** Run the test and check it passes. `test_camera_video.py` still passes.
- [ ] **Step 5:** Leave uncommitted. Add to Manual tests: `posesrec` during a run writes `bird_*` rows when birds fly.

### Task 5: The story file

**Files:**
- Create: `swarm_backend/demo/story.py`
- Create: `recording/stories/default.yaml` (the 18 Nov cut, written in moments; design section 5, with `map` instead of `ui`)
- Test: `tests/test_demo_story.py`

**Interfaces:**
- Consumes: `Take`, `MOMENTS` (Task 3).
- Produces:
  - `LAYOUTS = {'full': 1, 'side': 2, 'grid': 4}`, the number of sources each needs
  - `SOURCES = ('map', 'ui', 'yolo', 'chase', 'wide')`, plus the patterns `yolo:<id>` and `cam:<angle>`
  - `@dataclass Scene: index: int; caption: str | None; card: str | None; take: str; start_expr: str; end_expr: str; layout: str; show: list[str]; speed: float`
  - `@dataclass Story: title: str | None; subtitle: str | None; title_seconds: float; take: str; end_card: str | None; end_seconds: float; scenes: list[Scene]; path: Path` (`title_seconds` and `end_seconds` default to 3)
  - `load_story(path: Path) -> Story`, which raises `StoryError` with the file, the scene number and what's wrong
  - `parse_moment(expr: str) -> tuple[str, float]`
  - `pick_take(name: str, takes: list[Take]) -> Take`, where `best` = the first take with `facts['hit']`, else take 1. `<n>` = take number n (`weave/1` style names come in step 2's shoot)
  - `@dataclass Resolved: scene: Scene; take: Take; start: float; end: float; sources: dict[str, str]`. `sources` maps a role to a recorded file key: `map` gives `'map'`, `yolo` gives `'debug_d<attacker>'`, `chase` gives `'cam_attacker_watch'`, `wide` gives `'cam_arc_side'`, `yolo:2` gives `'debug_d2'`, `cam:base_tripod` gives `'cam_base_tripod'`, and `ui` gives `'screen'`
  - `resolve(story: Story, takes: list[Take]) -> tuple[list[Resolved], list[str]]`, which returns the resolved scenes and the problems (`"scene 4 'The hit': take 1 has no moment 'hit'"`) without raising
- Captions may use `{attacker}` and `{take}`, filled in by `resolve`. `speed` defaults to 1. A scene with `card:` has no take or sources and lasts `duration` seconds (default 3).

- [ ] **Step 1: Write the failing tests:**
  - `test_moment_expressions` (Review Focus 3): `parse_moment('hit-2.5') == ('hit', -2.5)`, `parse_moment('takeoff +12') == ('takeoff', 12.0)`, `parse_moment(' first_sighting - 5 ') == ('first_sighting', -5.0)` and `parse_moment('hit') == ('hit', 0.0)`. `parse_moment('hti - 2')` raises `StoryError` whose message contains `first_sighting`.
  - `test_default_story_loads`: `load_story(recording/stories/default.yaml)` has a title, `take == 'best'`, and every scene's layout source count matches `LAYOUTS`.
  - `test_wrong_source_count`: a `grid` scene with 3 sources raises `StoryError` naming the scene.
  - `test_resolve_ranges`: on `make_run(takes=1)` takes, the scene `from: hit - 2.5` / `to: hit + 0.3` resolves to `(hit - 2.5, hit + 0.3)`, and `yolo` maps to `debug_d3`.
  - `test_missing_moment_names_scene` (Review Focus 1): on a take without a hit, the problems list has exactly one entry per scene that uses `hit`, each naming the scene number and caption.
  - `test_best_take`: with `hit=[False, True]`, `best` picks take 2.
  - `test_caption_placeholders`: `Drone {attacker} intercepts` becomes `Drone 3 intercepts`.
- [ ] **Step 2:** Run them and check they fail.
- [ ] **Step 3:** Implement with `yaml.safe_load`. Reject unknown keys in a scene with a `StoryError` listing the allowed ones.
- [ ] **Step 4:** Run the tests and check they pass.
- [ ] **Step 5:** Leave uncommitted.

### Task 6: The tactical map source

**Files:**
- Create: `swarm_backend/demo/map_video.py`
- Modify: `recording/recording_config.json` (`"map": {"half_size_m": 260, "centre_enu": [0, 110], "trail_s": 10, "width": 1920, "height": 1080}`)
- Modify: `recording/video.sh` (new `map --start S --duration D [RUN]`)
- Test: `tests/test_demo_map.py`

**Interfaces:**
- Consumes: `read_run_info`, `load_poses` / `pose_at`, `ReplayUi`, `find_takes`.
- Produces:
  - `render_frame(state: MapState, sim: float) -> numpy.ndarray` (H x W x 3, BGR)
  - `@dataclass MapState`, built once by `map_state(folder: Path, cfg: dict) -> MapState`, holding poses, ui, roles, the arc and the takes
  - `write_map(folder: Path, start: float, duration: float, out: Path, size=(1920, 1080), fps=25.0) -> Path`
  - CLI `python3 -m swarm_backend.demo.map_video RUN --start S --duration D [--out F]`, default out `map_<S>.mp4`

What it draws, per frame at sim `S + i/25`:
- a dark background in the UI's colours (`--bg` `#0a1113`, friends `#5cc8ff`, hostile `#ff5d5d`, grey `#8a9499`);
- the base square;
- the search arc from the run's config: `surveillance.base_position`, `threat_heading_deg`, `arc_deg`, `arc_radius`. The config is NED, so north becomes the screen's up;
- range rings every 50 m;
- each defender's id and a trail of the last `trail_s`;
- its camera cone, 60° wide and 40 m long, from `gimbal_heading_deg` in the UI state at that sim;
- the hostile in red once any defender has `detection`, grey before that, with a trail;
- birds as small grey dots;
- the take's moments as ticks on a timeline bar along the bottom (the whole recording's range), with a cursor at the current sim;
- the sim time in the top-left corner.

Graveyard samples are not drawn: the hostile disappears at the hit. Before the first pose, the first pose's picture is held (Review Focus 2).

- [ ] **Step 1: Write the failing tests:**
  - `test_frame_shape_and_colours`: `render_frame` on a `make_run` state is `(1080, 1920, 3)`, and the pixel at the projected base position is not the background colour.
  - `test_hostile_hidden_in_graveyard`: right after the true hit, no pixel within 20 px of the graveyard's projected spot (if it's on screen), and none at the hostile's last pre-hit position, has the hostile colour.
  - `test_map_before_first_pose_holds` (Review Focus 2): a frame 5 s before the first pose equals the frame at the first pose, ignoring the clock text region.
  - `test_write_map_duration`: `write_map(start, 4.0)` gives an mp4 of 100 frames at 25 fps (ffprobe `nb_frames`).
- [ ] **Step 2:** Run them and check they fail.
- [ ] **Step 3:** Implement with OpenCV drawing. Encode by piping raw BGR to ffmpeg, as `camera_video.overlay` does (`libx264`, `crf 18`, `yuv420p`).
- [ ] **Step 4:** Run the tests and check they pass. Look at one frame as PNG (`cv2.imwrite`) yourself, and report what it shows in the task summary.
- [ ] **Step 5:** Leave uncommitted.

### Task 7: `final`, the film builder

**Files:**
- Create: `swarm_backend/demo/final.py`
- Delete: `recording/final_cut.py`
- Modify: `recording/video.sh`. `final [--story NAME] [--preview] [--skip-missing] [--take T] [RUN]` becomes `docker exec -w $PKG $C python3 -m swarm_backend.demo.final /workspace/$RUNS/$RUN --story /workspace/recording/stories/${NAME:-default}.yaml ...`. Teach `split_run` the booleans `--preview` and `--skip-missing`.
- Test: `tests/test_demo_final.py`

**Interfaces:**
- Consumes: `load_story`, `resolve`, `Resolved` (Task 5), `find_takes`, `save_takes`, `source_ranges` (Task 3), `write_map` (Task 6). The existing CLIs: `python3 -m swarm_backend.ui.debug_video cut RUN --id N --start S --duration D --out F`, `python3 -m swarm_backend.ui.camera_video overlay RUN --angle A --start S --duration D --out F`, and `python3 /workspace/recording/screen.py cut RUN --start S --duration D --out F`.
- Produces:
  - `needed_cuts(resolved: list[Resolved]) -> dict[str, tuple[float, float]]`: per source key, the union range over all scenes, padded by 1 s on each side
  - `cut_source(folder: Path, key: str, start: float, end: float, preview: bool) -> Path`: cached as `cache/<key>_<start:.2f>_<end:.2f>.mp4` (preview: `cache/preview/...`), and reused if the file exists and is newer than its source
  - `scene_clip(r: Resolved, cuts: dict[str, tuple[Path, float]], out: Path, preview: bool) -> Path`: one scene, with the layout, labels, caption, sim range and fades as in today's `final_cut.py` (`panel`, `scene_filter`, `text`, `FADE = 0.4`, the DejaVu fonts). Move those helpers here.
  - `card_clip(title: str, subtitle: str | None, seconds: float, out: Path, preview: bool) -> Path`: white text on `#0a1113`
  - `end_card_lines(takes_used: list[Take]) -> list[str]`, for example `["2 of 2 runs ended in a hit", "3.1 s from first sighting to hit (median)", "first seen at 74 m (median)"]`
  - `build(folder: Path, story_path: Path, preview: bool = False, skip_missing: bool = False, take: str | None = None) -> Path`
- **Output:** `scenes/<story>/NN_<slug>.mp4`, one per scene and card, numbered in film order, with `slug` = the caption lower-cased with non-alphanumerics replaced by `-` and cut to 40 characters. Then `final_<story>.mp4`, or `preview_<story>.mp4`, by concatenating them with the concat demuxer (`-c copy`, all clips encoded identically), then a remux with `-movflags +faststart`. `--take T` overrides the story's default take.
- It prints a table: scene number, caption, take, sim from-to, length in the film, and the total length.
- **Missing things:** if `resolve` reports problems, or a scene's source key has no recorded file (no `cam_arc_side.mkv`), or a cut fails (screen started too late, from `screen.py cut`'s message), it collects them all. Without `--skip-missing` it prints them and exits 1 before encoding any scene. With it, it builds the other scenes and prints the skipped ones at the end.
- `takes.json` is (re)written on every build.

- [ ] **Step 1: Write the failing tests** on `make_run` folders, with a small test story written in the test:
  - `test_needed_cuts_union`: two scenes using `chase` over `[10, 20]` and `[15, 30]` need `cam_attacker_watch` over `[9, 31]`.
  - `test_build_preview_lengths`: a 3-scene story (`full` map 4 s, `side` chase+yolo 2 s at speed 0.5, `grid` 3 s) with `title_seconds: 2` and no end card. `build(preview=True)` writes `preview_<story>.mp4` at 960x540, 25 fps, length `2 + 4 + 4 + 3` s ± 0.1. `scenes/<story>/` has 4 files starting `01_` to `04_`.
  - `test_full_output_format`: `build()` on a 1-scene story gives 1920x1080, h264, yuv420p, and `ffprobe` shows `moov` before `mdat` (faststart).
  - `test_skip_missing_builds_the_rest` (Review Focus 1): on a take without a hit, the story with a `hit` scene exits with `SystemExit(1)` and the message names the scene. With `skip_missing=True` it writes the film without that scene.
  - `test_screen_too_late_is_reported` (Review Focus 2): `make_run(screen=True)` with a `ui` scene starting before the film began. `build` exits 1 with a message containing `starts` and the scene's number, and no ffmpeg traceback.
  - `test_cache_reused`: a second `build` cuts nothing. Mock `subprocess.run` for the cut CLIs and count the calls.
  - `test_end_card_lines`: two hit takes with `sighting_to_hit_s` 3.0 and 3.2 give the line `3.1 s from first sighting to hit (median)`.
- [ ] **Step 2:** Run them and check they fail.
- [ ] **Step 3:** Implement. Speeds and fades as in `final_cut.py`. Preview passes `preview=True` to every encode (scale to 960x540, `ultrafast`, `crf 28`).
- [ ] **Step 4:** Run the tests and check they pass. Then `recording/video.sh final --preview fake-1` on a fake folder (Task 3 step 4), with the default story. It builds, and its table shows every scene. Remove the fake folder afterwards.
- [ ] **Step 5:** Leave uncommitted.

### Task 8: Recording without clicking: run folder name and full-screen film

**Files:**
- Modify: `docker/start-stellar.sh`. With `--record`, the run name comes from `RECORD_RUN` if set (it may contain `/`), else `$(date +%Y%m%d-%H%M%S)`. Document it in the header comment.
- Modify: `recording/screen.py`. `record --fullscreen` films the whole root window: size from `xwininfo -root`, origin `+0+0`, no click.
- Modify: `recording/video.sh`. `screen` passes `--fullscreen` through (teach `split_run` the boolean).
- Test: `tests/test_screen_recording.py` (extend)

**Interfaces:**
- Produces: `root_region(xwininfo_root_text: str) -> tuple[int, int, int, int]`, which returns `(0, 0, w, h)` with w and h even.

- [ ] **Step 1: Write the failing test** `test_root_region`: real `xwininfo -root` text with `Width: 1921` and `Height: 1080` gives `(0, 0, 1920, 1080)`.
- [ ] **Step 2:** Run it and check it fails.
- [ ] **Step 3:** Implement, reusing `parse_xwininfo` and `even`.
- [ ] **Step 4:** Run the test and check it passes. `bash -n docker/start-stellar.sh` passes, and `chmod +x` it.
- [ ] **Step 5:** Leave uncommitted.

### Task 9: `record.sh`, one command

**Files:**
- Create: `recording/record.sh` (then `chmod +x`)
- Test: `tests/test_demo_record_dryrun.py`, which runs the script with `--dry-run` through `subprocess` (bash is in the container)

**Interfaces:**
- Consumes: `start-stellar.sh` with `RECORD_RUN` (Task 8), `mount-backend.sh ui --record auto`, `posesrec` (Task 4), `debugrec IDS`, `camrec ANGLES --no-pose-log` (Task 4), `video.sh screen --fullscreen` (Task 8), `demo.runinfo write` (Task 2), `demo.takes` (Task 3).
- Produces: `recording/record.sh [OPTIONS] SCENARIO [ARGS...]` with these options:
  - `--name NAME` (run folder name, default the date and time)
  - `--screen`, `--watch`
  - `--cameras A,B` (default `attacker_watch,arc_side`), `--no-cameras`, `--no-yolo`
  - `--export DIR`
  - `--dry-run`
  - `--keep-sim` (accepted and documented for step 2's shoot; in step 1 it only skips the final `shutdown`)

Behaviour, in this order (design section 1):
1. **Refuse to start** if `px4-gazebo-vnc` is running (as `record_tuning.sh` does), or a tmux session `hive-rec` exists.
2. Create the session `tmux new -d -s hive-rec`. Each part gets its own window and logs to `data/recordings/<name>/logs/<part>.log` through `2>&1 | tee`.
3. **Start each part in order**, waiting for its readiness line in its log before starting the next:

| Part | Command | Ready when | Timeout |
|---|---|---|---|
| `sim` | `RECORD_RUN=<name> docker/start-stellar.sh --record` + `--headless` unless `--watch` | log has `Recording this run to`, then `px4-uxrce_dds_client --instance 0 status` shows `, connected` (as `record_tuning.sh` `start_sim`) | 600 s |
| `runinfo` | `docker exec ... python3 -m swarm_backend.demo.runinfo write ...` with `SWARM_CONFIG` (default `scenarios/surveillance.yaml`) | exit 0 | 30 s |
| `ui` | `SWARM_CONFIG=... mount-backend.sh ui --record /workspace/data/recordings/<name>` | `Recording the UI on` | 60 s |
| `poses` | `mount-backend.sh posesrec --out /workspace/data/recordings/<name>` | `Logging poses of` | 60 s |
| `browser` (only `--screen`) | first of `firefox --kiosk`, `chromium-browser --kiosk`, `google-chrome --kiosk` found on the host, on `http://localhost:8090` | `xdotool`-free: wait 5 s, then check the process is alive | 30 s |
| `screen` (only `--screen`) | `recording/video.sh screen --fullscreen <name>` | `screen.mkv` exists and grows over 2 s | 30 s |
| `yolo` (unless `--no-yolo`) | `mount-backend.sh debugrec <defender ids from run.json> --out /workspace/data/recordings/<name>` | `Recording the debug images` | 60 s |
| `cams` (unless `--no-cameras`) | `mount-backend.sh camrec <angles> --no-pose-log --out ...` | `first image at sim time` once per angle | 180 s |
| `scenario` | `cd ros2_ws/src/swarm_backend && <SCENARIO> <ARGS>` | runs to its end | none |

4. **Watchdog:** every 5 s while the scenario runs, check that every started part's tmux window still has a live process. If one is dead, say `<part> stopped (log: <file>)` and go to the stop order with exit 1.
5. **Stop order** at the scenario's end, on a failure or on INT/TERM (`trap`): scenario (C-c in its window, wait up to 300 s for the defenders to land), then cams, yolo, screen, browser, poses and ui (C-c each, wait 30 s, then `kill`), then sim (`shutdown` typed into its window, wait 120 s for the container to go, then `docker rm -f`). Then `tmux kill-session -t hive-rec`.
6. **Health check:** these files must be there and not empty: `gz/` over 20 KB, `ui.jsonl`, `poses.jsonl`, `run.json`, a `debug_d<id>.mkv` per defender (unless `--no-yolo`), a `cam_<angle>.mkv` per angle (unless `--no-cameras`), and `screen.mkv` (with `--screen`). Print a ✓ or ✗ line for each. Exit 1 on any ✗.
7. **At the end:** `docker run` the takes CLI on the folder (through `video.sh moments <name>`), print the folder and the command `recording/video.sh final --preview <name>`. With `--export DIR`, `rsync -a --exclude gz --exclude cache` the folder to `DIR/<name>/`.

`--dry-run` prints each action as one line, `START <part>: <command>` / `WAIT <part>: <condition>` / `STOP <part>: <how>` / `CHECK <file>`, for both the normal path and the INT path (`ON INT:` header), then exits 0 without running anything.

- [ ] **Step 1: Write the failing tests:**
  - `test_dry_run_start_order`: with `record.sh --dry-run scenarios/trial.sh 1`, the `START` parts come in the order `sim, runinfo, ui, poses, yolo, cams, scenario` (no browser or screen), and the sim command contains `--headless` and `RECORD_RUN=`.
  - `test_dry_run_screen_and_watch`: with `--screen --watch`, `browser` and `screen` come between `poses` and `yolo`, and the sim command has no `--headless`.
  - `test_dry_run_options`: with `--no-cameras --no-yolo` there's no `cams` or `yolo` start or check. With `--cameras base_tripod`, the cams command has `base_tripod --no-pose-log`.
  - `test_dry_run_ctrl_c_order` (Review Focus 5): the `ON INT:` block's `STOP` parts are `scenario, cams, yolo, poses, ui, sim` in that order, followed by `tmux kill-session`.
- [ ] **Step 2:** Run them and check they fail.
- [ ] **Step 3:** Implement in bash. Every action goes through one function `act <verb> <part> <text> <command...>`, which prints in dry-run mode and runs otherwise, so the dry run is the real plan. The scenario's ids for `debugrec` come from `run.json` (`jq`-free: `python3 -c` inside the container).
- [ ] **Step 4:** Run the tests and check they pass. `bash -n recording/record.sh` passes.
- [ ] **Step 5:** Leave uncommitted.

### Task 10: Docs and the manual test

**Files:**
- Modify: `recording/README.md`. The quick start becomes `record.sh` → `video.sh moments` → `video.sh final --preview` → `final`. Add a story-file reference (keys, layouts, sources, moments), and remove the six-terminal section and `final_cut.py`. Keep the manual commands as "doing it by hand".
- Modify: `recording/video.sh` header (subcommands `moments`, `map`, the new `final`) and `mount-backend.sh` header (`posesrec`).
- Modify: `hive-vault/Recording demo videos.md` (how it works now, with a link to the design and this plan).
- Modify: `hive-vault/detection/Manual tests to do.md`. Add **"record.sh and final on Stellar"**:
  1. `recording/record.sh --name demo-test scenarios/trial.sh 1`, then `recording/video.sh final --preview demo-test`, then the same with `--screen --watch`.
  2. **Pass:** it gets ready without any clicking, stops by itself, every health line is ✓, `moments` shows `release`, `first_sighting` and `hit` with plausible times, and in the preview the hit in `chase` and `yolo` happens at the map's hit tick.
  3. **Note:** the time to get ready per part, and the sim speed, against the 18 Nov 0.34x.
- [ ] **Step 1:** Write the docs.
- [ ] **Step 2:** Run the full unit suite (`tests/test_*.py`). Everything passes, and the count is higher than 594 + 7 skipped.
- [ ] **Step 3:** `git status` shows only this plan's files changed, plus Oskar's own unrelated `training/` files. Leave uncommitted, and list the files for Oskar.
