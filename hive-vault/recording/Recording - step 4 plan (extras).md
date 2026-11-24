# Recording step 4 (extras): implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The finishing touches for the demo film:
- a zoom on the YOLO box at the detection;
- synced graphs;
- an orbit shot around the base;
- subtitles;
- stills (PDF) and short GIFs for the report.

**Architecture:** Each extra is independent and plugs into the step-1 pieces:
- the zoom is a layout in `final`, fed by the detectors' own box records, which `record.sh` copies into the run folder;
- graphs are a new source kind drawn offline like the map;
- the orbit is a new camera kind in `camera_video.py`'s director, recorded by a camera-only shoot entry;
- subtitles, stills and GIFs are outputs of `final`.

**Tech Stack:** Python 3.12 in the container (OpenCV, numpy, matplotlib, Pillow, ffmpeg), bash, pytest.

**Spec:** `hive-vault/recording/Recording - smoother demo videos (design).md`, sections 5 (`zoom`), 10 (graphs, orbit, subtitles, stills and GIFs) and step 4 of "Order of the work". **Builds on steps 1 to 3** (`demo.final`, `demo.story`, `demo.takes`, `demo.map_video`, `demo.insights`, `demo.fake_run`, `recording/record.sh`, `recording/shoot.sh`, `ui/camera_video.py`). **Written before any of them was tested on Stellar** (Oskar, 23 Nov). Where the built code differs, follow the code and ledger a ruling. The five tasks are independent; do them in any order.

## Global Constraints

- Everything on sim time; film and preview formats as in step 1.
- Don't change the detection or attack code. The detectors' `--record` output (`detect/live.py` `result_record`) is only read: the scenarios already start the detectors with `--record /tmp/detect_<id>.jsonl` (`gazebo_surveillance.py` `record_path`, set by `baseline_eval`, so by `trial.sh`), once per scenario run.
- No browser in the container, nothing on the host, no commits or pushes, and `chmod +x` after editing a `.sh`. Tests as in step 1; the whole suite must pass at the end.
- The orbit camera is a new **kind** in `recording_config.json`'s `replay_angles` and `camera_video.Director`; the existing angles must behave exactly as before (`tests/test_camera_video.py` stays green).

## Review Focus

1. **Zoom on a take whose attacker never confirmed a box:** the scene can't zoom, so it stops before building ("scene N: drone D confirmed no box between ...") unless `--skip-missing`. Test: Task 1, `test_zoom_without_box`.
2. **The box at the image edge:** the crop stays inside the picture (shifted, not padded with black), at the requested zoom. Test: Task 1, `test_crop_at_edge`.
3. **A detect record whose image size differs from the debug video's** (the debug image scaled): boxes are scaled by the size ratio. Test: Task 1, `test_box_scaled_to_video`.
4. **Subtitles when a scene is sped up or slowed down:** caption and insight times follow the clip's time, not the sim time. Test: Task 4, `test_srt_times_with_speed`.
5. **A GIF that is too big for a README:** the GIF is capped at 480 px wide and 8 s, and the command says how big it got. Test: Task 5, `test_gif_size_capped`.

---

## File Structure

| File | Responsibility |
|---|---|
| `recording/record.sh` | + copy `/tmp/detect_<id>.jsonl` from the container into the run folder (`detect_d<id>.jsonl`) before the sim stops |
| `swarm_backend/demo/boxes.py` | read `detect_d<id>.jsonl`: the attacker's confirmed target box per sim time, scaled to the debug video |
| `swarm_backend/demo/final.py` | + layout `zoom` (crop around the box, `freeze`); + SRT; + `--burn-subtitles`; + `--stills` |
| `swarm_backend/demo/story.py` | + `zoom` (show `[yolo]`, keys `zoom: 3`, `freeze: 1.5`); + source `plot:<name>` |
| `swarm_backend/demo/plots.py` | graphs on sim time as a video source (`range`, `base_distance`, `est_error`) |
| `swarm_backend/ui/camera_video.py` | + director kind `orbit` |
| `recording/recording_config.json` | + angle `base_orbit` |
| `recording/shoot.sh` / `swarm_backend/demo/shoot.py` | + `run: idle SECONDS` (cameras only, no scenario) |
| `swarm_backend/demo/stills.py` | PDF of one frame per scene + caption; GIF of a scene |
| tests: `test_demo_boxes.py`, `test_demo_zoom.py`, `test_demo_plots.py`, `test_camera_video.py` (extend), `test_demo_subtitles.py`, `test_demo_stills.py` | |

---

### Task 1: Zoom on the YOLO box

**Files:** Modify `recording/record.sh`, `swarm_backend/demo/story.py` and `swarm_backend/demo/final.py`. Create `swarm_backend/demo/boxes.py`. Test `tests/test_demo_boxes.py` and `tests/test_demo_zoom.py`.

**Interfaces:**
- `record.sh`, in its stop order right after `scenario` and before `cams`: for each defender id, `docker cp px4-gazebo-vnc:/tmp/detect_<id>.jsonl <run>/detect_d<id>.jsonl` (missing files are skipped silently). Extend the dry-run: `STOP detect-records: copy /tmp/detect_<id>.jsonl`. First check, in `gazebo_surveillance.py`, that the scenario does not delete the file on exit; if it does, copy it in the scenario's window before Ctrl-C and ledger the ruling.
- `boxes.py`:
  - `load_boxes(folder: Path, drone: int) -> list[Box]`, with `@dataclass Box: sim: float; u: float; v: float; w: float; h: float; track: int`. It holds the confirmed box of the record's `target` track per image (`confirmed` entries are `[u, v, w, h, id]`, `u, v` the box centre). Boxes are scaled by `debug_d<id>.json` `width` / record `width`, and the same for height (Review Focus 3).
  - `box_at(boxes, sim, max_gap=0.5) -> Box | None`
  - `first_box(boxes, start, end) -> Box | None`
- Story: `layout: zoom`, `show: [yolo]` (or `yolo:<id>`), `zoom: 3` (default 3, the crop is 1/zoom of the frame), and `freeze: 1.5` (seconds of the first confirmed box's frame, held at the start of the scene).
- `final.zoom_clip(r, cut, out, preview)`: the crop follows the box centre, smoothed with a 0.3 s moving average. It's clamped inside the image (Review Focus 2) and scaled to the output size. Labelled "Drone D camera, zoomed 3x", with a thin `HOSTILE`-coloured rectangle around the box. Built from frames with OpenCV and piped to ffmpeg, as `write_map` does. `freeze`: the first `freeze × 25` frames repeat the frame at `first_box`, with a caption pill "YOLO + tracker: locked".
- No box in the scene's range: the problem `scene N: drone D confirmed no box between S and E` (Review Focus 1).

- [ ] **Step 1: Write the failing tests:**
  - `test_load_boxes_target_only`: a hand-made `detect_d3.jsonl` with two confirmed tracks and `target` 7 gives only track 7's boxes.
  - `test_box_scaled_to_video` (Review Focus 3): record `width` 1280 with debug width 640 halves u and w.
  - `test_crop_at_edge` (Review Focus 2): a box at u = 5 gives crop x0 = 0 and the full crop width.
  - `test_zoom_without_box` (Review Focus 1): no confirmed box, so `build` exits 1 naming the scene. With `skip_missing` the rest builds.
  - `test_zoom_scene_builds`: `make_run`, extended to write `detect_d3.jsonl` with a box moving across the frame, gives a zoom scene with `freeze: 1` of a 2 s range, which builds a 3 s preview. The first 25 frames are identical (frame grabs at 0.1 s and 0.9 s are equal).
  - `test_dry_run_copies_detect_records`: in the record dry run, the `detect-records` stop comes right after `scenario`.
- [ ] **Step 2:** Run them and check they fail.
- [ ] **Step 3:** Implement it.
- [ ] **Step 4:** Run the tests and check they pass. Look at a zoomed frame as a PNG and describe it in the ledger.
- [ ] **Step 5:** Leave uncommitted.

### Task 2: Synced graphs

**Files:** Create `swarm_backend/demo/plots.py`. Modify `swarm_backend/demo/story.py` (source `plot:<name>`) and `swarm_backend/demo/final.py` (`cut_source` for `plot:*`). Test `tests/test_demo_plots.py`.

**Interfaces:**
- `PLOTS: dict[str, Callable[[TakeData], tuple[list[float], list[float], str, str]]]` gives (sims, values, title, unit) for:
  - `range`: the attacker's true distance to the target;
  - `base_distance`: the target's distance to the base;
  - `est_error`: the step-3 samples.
- `write_plot(folder, take, name, start, end, out, size)`: one frame per 1/25 s. The whole take's curve is drawn faint, the part up to the current sim bold, with a moving dot and the current value. The take's moments are vertical ticks. matplotlib renders the static background once (Agg canvas to a numpy array), and OpenCV draws the moving part per frame, so a 60 s clip takes seconds, not minutes.
- Story: `show: [chase, "plot:range"]`. The source key is `plot_<name>_take<k>`, cut by `write_plot`.

- [ ] **Step 1: Write the failing tests:**
  - `test_range_plot_values`: on `make_run`, the minimum of `range` ≈ `closest_pass_m` ± 0.2.
  - `test_plot_frames`: 2 s give 50 frames, and the frame at the hit has the dot further right than the frame at the sighting.
  - `test_plot_scene_builds`: a side scene `[map, "plot:range"]` builds a preview.
- [ ] **Step 2:** Run them and check they fail.
- [ ] **Step 3:** Implement it.
- [ ] **Step 4:** Run the tests and check they pass.
- [ ] **Step 5:** Leave uncommitted.

### Task 3: The orbit shot

**Files:** Modify `swarm_backend/ui/camera_video.py` (`Director` kind `orbit`), `recording/recording_config.json` (`"base_orbit": {"orbit": [0.0, 0.0], "radius": 220.0, "height": 70.0, "period_s": 60.0, "look_at_point": [0.0, 100.0, 30.0]}`), and `swarm_backend/demo/shoot.py` plus `recording/shoot.sh` (`run: idle 60` = record with no scenario, `sleep 60` in the scenario window). Test `tests/test_camera_video.py` (extend) and `tests/test_demo_shoot.py` (extend).

**Interfaces:**
- `Director.goal` for a spec with `orbit: [x, y]`: the camera at `(x + r cos(2π t / T), y + r sin(2π t / T), height)` looking at `look_at_point`, with `t` = sim time since the camera's first goal. No drone needed, so it also works with nobody flying. Smoothing as for the other kinds.
- `recording/shoots/demo.yaml` gets an entry `- name: overview, run: idle 60, options: [--cameras, base_orbit, --no-yolo]`.
- A story shows it as source `cam:base_orbit`. An idle run has no hostile flight, so `find_takes` would give no take; add the rule: a folder whose `run.json` command is `idle` gives one pseudo take, with moments `start` and `end` only, named `<run id>/overview`. Stories then use `take: overview-1/overview`, `from: start`, `to: start + 20`.

- [ ] **Step 1: Write the failing tests:**
  - `test_orbit_goal_circles`: goals at t = 0, T/4 and T/2 lie on the circle, 90° apart, at `height`, each looking at the point.
  - `test_existing_angles_unchanged`: `test_camera_video.py` passes unchanged.
  - `test_idle_entry`: `run: idle 60` expands to a command recognised as idle, and `shoot.sh --dry-run` shows the scenario part as `sleep 60`.
  - `test_idle_pseudo_take`: a `make_run` folder with `run.json` command `['idle', '60']` gives one take named `.../overview` with moments `start` and `end` only.
- [ ] **Step 2:** Run them and check they fail.
- [ ] **Step 3:** Implement it.
- [ ] **Step 4:** Run the tests and check they pass.
- [ ] **Step 5:** Leave uncommitted. Add to Manual tests: one `idle 60` run with `base_orbit`; the camera circles smoothly and the base stays in the frame.

### Task 4: Subtitles

**Files:** Modify `swarm_backend/demo/final.py`. Test `tests/test_demo_subtitles.py`.

**Interfaces:**
- `subtitles(entries: list[tuple[float, float, str]]) -> str`: SRT text, with times `HH:MM:SS,mmm` and numbered from 1.
- `final.build` collects `(film start, film end, text)` for:
  - every title, card and end card (their text);
  - every scene's caption (the scene's span in the film);
  - every insight label (step 3: `at` and `until`, offset by the scene's start in the film).

  It writes `final_<story>.srt` (or `preview_<story>.srt`) next to the film. Times are film times, after speed and fades (Review Focus 4).
- `--burn-subtitles`: the film is encoded once more with the `subtitles=` filter (DejaVu font, font size 28 at 1080), as `final_<story>_subtitled.mp4`.

- [ ] **Step 1: Write the failing tests:**
  - `test_srt_format`: two entries give a valid SRT (index lines, `-->`, a blank line between them).
  - `test_srt_times_with_speed` (Review Focus 4): a title of 2 s, then a scene of 1 sim second at speed 0.5, gives a caption from 00:00:02,000 to 00:00:04,000.
  - `test_burn_subtitles`: `--burn-subtitles` writes the subtitled film, the same length, and a frame differs from the plain one in the lower part.
- [ ] **Step 2:** Run them and check they fail.
- [ ] **Step 3:** Implement it.
- [ ] **Step 4:** Run the tests and check they pass.
- [ ] **Step 5:** Leave uncommitted.

### Task 5: Stills and GIFs

**Files:** Create `swarm_backend/demo/stills.py`. Modify `swarm_backend/demo/final.py` (`--stills`) and `recording/video.sh` (`final --stills`, `gif SCENE_CLIP`). Test `tests/test_demo_stills.py`.

**Interfaces:**
- `stills_pdf(scene_clips: list[Path], captions: list[str], out: Path) -> Path`: one A4 landscape page per scene, the frame from the middle of its clip, with the caption under it (Pillow, `save(..., save_all=True)`). Writes `stills_<story>.pdf`.
- `gif(clip: Path, out: Path, start: float = 0.0, seconds: float = 6.0, width: int = 480, fps: int = 12) -> tuple[Path, int]`: ffmpeg `palettegen` / `paletteuse`, capped at 480 px wide and 8 s (Review Focus 5). Returns the path and its size in bytes, which the command prints.
- `final --stills` writes the PDF, and a GIF of each scene whose caption contains "hit" (case-insensitive) as `gif_<NN>_<slug>.gif`. `video.sh gif scenes/<story>/04_the-hit.mp4 [--start S --seconds N]` makes one by hand.

- [ ] **Step 1: Write the failing tests:**
  - `test_pdf_pages`: 3 scene clips give a PDF of 3 pages (count with Pillow, or `/Type /Page` in the bytes).
  - `test_gif_size_capped` (Review Focus 5): asking for 20 s at 1920 px gives a GIF of at most 8 s and 480 px wide. The returned size matches the file.
  - `test_final_stills`: `build(..., stills=True)` on a 2-scene story with a "The hit" scene writes `stills_<story>.pdf` and one GIF.
- [ ] **Step 2:** Run them and check they fail.
- [ ] **Step 3:** Implement it.
- [ ] **Step 4:** Run the tests and check they pass, then the whole suite.
- [ ] **Step 5:** Leave uncommitted.

### Task 6: Docs and the manual test

**Files:**
- `recording/README.md`: the zoom layout, graphs, the orbit and idle entries, subtitles, stills and GIFs.
- `recording/stories/default.yaml`: a zoom scene at the detection with `freeze: 1.5`, and a graph next to the chase camera in the intercept.
- `hive-vault/recording/Recording demo videos.md`: all four steps done.
- `hive-vault/detection/Manual tests to do.md`.

- [ ] **Step 1:** Write the docs and stories.
- [ ] **Step 2:** Add the manual test **"Extras on a real shoot"**:
  1. `recording/shoot.sh recording/shoots/demo.yaml` (with the overview entry).
  2. `recording/video.sh final --stills --burn-subtitles <shoot>`.

  **Pass:**
  - the zoom shows the hostile in the box at the detection, frozen for 1.5 s;
  - the graph's dot meets its minimum at the hit;
  - the orbit circles the base smoothly;
  - the subtitles match the captions;
  - the PDF has one page per scene;
  - the hit GIF is under 5 MB.

  **Note:** how far the box is off the hostile in the zoom, if at all.
- [ ] **Step 3:** Run the whole suite. It must pass.
- [ ] **Step 4:** Leave uncommitted, and list the files.
