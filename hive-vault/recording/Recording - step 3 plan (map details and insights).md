# Recording step 3 (map details and insights): implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The tactical map shows truth vs. what the system believes, and a ghost overlay of the baseline. The film shows the right numbers at the right moment.

**Architecture:** New facts (estimation error) in `demo/takes.py`. The map (`demo/map_video.py`) draws the estimate next to the truth, and the attacker's commanded velocity and aim point. A `ghost` layout renders two takes of a pair on one map, synced on `release`. A table of insights (`demo/insights.py`) turns facts into short labels with the moment they belong to, and `final` draws them in the scenes that ask for them.

**Tech Stack:** Python 3.12 in the container (OpenCV, numpy, ffmpeg), pytest.

**Spec:** `hive-vault/recording/Recording - smoother demo videos (design).md`, sections 7 (ghost), 8 (map) and 9 (insights). **Builds on steps 1 and 2**: `demo.map_video.MapState / map_state / render_frame / project / write_map`, `demo.takes.Take` (with `folder`, `name`) / `FACTS` / `shoot_takes`, `demo.story` (`compare`, `pick_take`, `Resolved`), `demo.final` (`scene_clip`, `compare_clip`, `cut_source`), and `demo.fake_run.make_run`. **Written before steps 1 and 2 were tested on Stellar** (Oskar, 23 Nov). Where the built code differs from what's named here, follow the code and ledger a ruling.

## Global Constraints

- Everything on sim time. Film and preview formats as in step 1.
- The map is drawn offline from `poses.jsonl` and `ui.jsonl`. No new recorder, so the sim is not slowed down.
- Map colours are the operator UI's: `BG`, `FRIEND`, `HOSTILE`, `GREY`, `AREA` in `map_video.py`. The baseline ghost is `GREY` at 60 %.
- No browser in the container, nothing on the host, no changes to the detection or attack code. No commits or pushes. Tests as in step 1; the whole suite must pass at the end.
- In `ui.jsonl` a defender's `detection` is `{'north', 'east', 'altitude'}` (world NED metres, its estimate of the target), and `attack` is `{'target': {'north', 'east', 'altitude'} | None, 'velocity': {'north', 'east', 'up'} | None, 'range_m', 'impact_s', ...}` (`ui/model.py` `attack_view`). The map is Gazebo ENU: x = east, y = north.

## Review Focus

1. **No detection at all in a take** (the hostile flew past unseen): the estimate and error are absent without errors, the error facts are None, and their insights don't appear. Test: Task 1, `test_error_facts_without_detection`.
2. **Ghost with takes of different lengths** (the baseline's run lasts longer): the ghost frame past the baseline's end holds its last state, and the ours side keeps moving. Test: Task 3, `test_ghost_past_one_end`.
3. **Two insights at nearly the same moment** (sighting and intercept 0.3 s apart): both stay readable, stacked rather than drawn on top of each other. Test: Task 4, `test_insights_stack`.
4. **A detection estimate far off the map** (a bad range estimate of 2 km): the error line is clipped at the map's edge, with no OpenCV overflow and no line across the whole frame. Test: Task 2, `test_estimate_off_map`.
5. **The estimate drawn after the hit** (stale detections linger 2 s in the UI): nothing is drawn once the target is in the graveyard. Test: Task 2, `test_no_estimate_after_hit`.

---

## File Structure

| File | Responsibility |
|---|---|
| `swarm_backend/demo/takes.py` | + facts `est_error_at_sighting_m`, `est_error_median_m` |
| `swarm_backend/demo/map_video.py` | + estimate vs truth, error label, commanded velocity and aim point; + `render_ghost` |
| `swarm_backend/demo/insights.py` | the insight table: fact to label at a moment; `insights_for(take, start, end, speed)` |
| `swarm_backend/demo/story.py` | layout `ghost` (needs a pair, `show: [map]`); scene key `insights: true` |
| `swarm_backend/demo/final.py` | the ghost scene; insight labels drawn in scene and compare clips |
| `recording/stories/compare.yaml`, `default.yaml` | use `ghost` and `insights` |
| tests: `test_demo_takes.py`, `test_demo_map.py`, `test_demo_insights.py`, `test_demo_ghost.py` (extend or new) | |

---

### Task 1: Estimation error facts

**Files:** Modify `swarm_backend/demo/takes.py` and `swarm_backend/demo/fake_run.py` (its detection is already the truth plus (+1.5 N, −1.0 E): the error is 1.80 m). Test `tests/test_demo_takes.py`.

**Interfaces:**
- Produces: `est_error_samples(d: TakeData) -> list[tuple[float, float]]`, (sim, metres) for every `ui.jsonl` line from `first_sighting` to `hit` or the end where the attacker's view has a `detection`: the 3-D distance between the estimate (converted to ENU) and the target's true pose at that sim. Facts `est_error_at_sighting_m` (the first sample) and `est_error_median_m`, both None without samples.
- `TakeData` gets the `ReplayUi` (it's loaded already in `find_takes`).

- [ ] **Step 1: Write the failing tests:**
  - `test_error_facts`: on `make_run`, `est_error_median_m` ≈ 1.80 ± 0.05.
  - `test_error_facts_without_detection` (Review Focus 1): `make_run` with a new `detect=False` option, so no detections: both facts are None, and the other facts are unchanged.
- [ ] **Step 2:** Run them and check they fail.
- [ ] **Step 3:** Implement it (and add `detect` to `fake_run`).
- [ ] **Step 4:** Run the tests and check they pass.
- [ ] **Step 5:** Leave uncommitted.

### Task 2: Truth vs. belief on the map

**Files:** Modify `swarm_backend/demo/map_video.py`. Test `tests/test_demo_map.py`.

**Interfaces:**
- Produces, drawn in `render_frame` for every defender view with a `detection` at the frame's sim, and only while the target is not in the graveyard:
  - the estimate as a hollow `HOSTILE` circle (radius 9 px at 1080);
  - a thin dashed line from the defender to the estimate (what the camera sees);
  - a thin solid `GREY` line from the estimate to the true hostile, labelled `err 1.8 m`;
  - for an engaged defender with `attack.velocity`, an arrow 2 s ahead of the commanded velocity (as the UI's `VELOCITY_LEAD_S`), and the `attack.target` as a small `AREA` cross (where the attack thinks it is aiming).
- Helper `clip_line(p0, p1, w, h) -> tuple | None` (Cohen–Sutherland, or `cv2.clipLine`) is used for every line, so off-screen points never overflow (Review Focus 4).

- [ ] **Step 1: Write the failing tests:**
  - `test_estimate_drawn`: at a sim between sighting and hit, a hollow `HOSTILE` circle edge is found within 3 px of the projected estimate. There's none at the same place in a `detect=False` run.
  - `test_estimate_off_map` (Review Focus 4): a `ui.jsonl` edited so one detection is 2000 m north. `render_frame` doesn't raise, and no pixel row is fully coloured.
  - `test_no_estimate_after_hit` (Review Focus 5): 1 s after the hit, nothing of the `HOSTILE` colour is within 30 px of the last estimate.
  - `test_velocity_arrow`: for an engaged defender with a velocity in `ui.jsonl` (extend `fake_run`: the attacker's view gets `attack: {'velocity': ..., 'target': ...}` during intercept), pixels of the `FRIEND` colour lie along the arrow's direction 2 s ahead.
- [ ] **Step 2:** Run them and check they fail.
- [ ] **Step 3:** Implement it.
- [ ] **Step 4:** Run the tests and check they pass. Write one frame to a PNG and look at it yourself, then describe it in the ledger.
- [ ] **Step 5:** Leave uncommitted.

### Task 3: The ghost overlay

**Files:** Modify `swarm_backend/demo/map_video.py`, `swarm_backend/demo/story.py` and `swarm_backend/demo/final.py`. Test `tests/test_demo_ghost.py`.

**Interfaces:**
- Consumes: pairs and `compare` resolving (step 2), `map_state`, `render_frame`.
- Produces:
  - `render_ghost(ours: MapState, base: MapState, t_ours: float, t_base: float) -> np.ndarray`: ours drawn as in `render_frame`. Then the baseline's defenders, trails and hostile are drawn on top in `GREY` at 60 %, with a small "baseline" tag at its attacker. The timeline shows ours. Each side holds its last pose past its own end (Review Focus 2).
  - `write_ghost(ours_folder, base_folder, ours_start, base_start, duration, out, size) -> Path`.
  - Story `layout: ghost`, `show: [map]`, `take: pair:<id>` or `best_pair`. It resolves like `compare`: two `Resolved` synced on `release`.
  - `final` builds a ghost scene as one full-frame clip from `write_ghost`, cached under the ours folder's `cache/ghost_<pair>_...mp4`, with the caption, the sim range of ours, and a small legend "blue: ours, grey: baseline".

- [ ] **Step 1: Write the failing tests:**
  - `test_ghost_has_both`: at a sim where both sides' attackers are away from their slots, `FRIEND` pixels are near ours's attacker, and grey pixels are near the baseline's attacker position (`make_run` with different `ATTACKERS` order per folder, through a new `attackers` option).
  - `test_ghost_past_one_end` (Review Focus 2): with a baseline 10 s shorter, frames after its end still show its last positions, and ours keeps moving.
  - `test_ghost_scene_builds`: a story with one ghost scene of 3 s builds a preview of 3 s ± 0.15.
- [ ] **Step 2:** Run them and check they fail.
- [ ] **Step 3:** Implement it.
- [ ] **Step 4:** Run the tests and check they pass. Look at one ghost frame as a PNG and describe it in the ledger.
- [ ] **Step 5:** Leave uncommitted.

### Task 4: Insights, numbers at the right moment

**Files:** Create `swarm_backend/demo/insights.py`. Modify `swarm_backend/demo/story.py` (scene key `insights: true | [names]`) and `swarm_backend/demo/final.py`. Test `tests/test_demo_insights.py`.

**Interfaces:**
- Consumes: `Take.facts` and `Take.moments`.
- Produces:
  - `INSIGHTS: dict[str, tuple[str, str, Callable[[dict], bool]]]`, name → (moment, template using facts, when to show). It holds:
    - `seen_at`: `first_sighting`, `"seen at {sighting_range_m:.0f} m"`
    - `assigned`: `intercept`, `"D{attacker} assigned"`
    - `est_error`: `first_sighting`, `"position error {est_error_at_sighting_m:.1f} m"`
    - `time_to_hit`: `hit`, `"{sighting_to_hit_s:.1f} s from sighting to hit"`
    - `closest`: `hit`, `"closest pass {closest_pass_m:.1f} m"`
    - `no_hit`: `end`, `"no hit, closest {closest_pass_m:.1f} m"`, shown only when `hit` is False
    
    An insight whose facts are None is left out (Review Focus 1).
  - `@dataclass Label: text: str; at: float; until: float; row: int` (times in the scene clip's seconds, after `speed`)
  - `insights_for(take: Take, start: float, end: float, speed: float, names: list[str] | None = None, hold_s: float = 3.0) -> list[Label]`: the labels whose moment falls in [start, end), each shown for `hold_s` (clip time). Overlapping labels get increasing `row` (Review Focus 3).
  - `final.scene_clip` / `compare_clip` draw the labels as `drawtext` with `enable='between(t,at,until)'`, top-left under the first panel's label, `row` × 1.4 line heights down, with a small fade (alpha expression). In compare, each side gets its own labels.

- [ ] **Step 1: Write the failing tests:**
  - `test_labels_in_range`: on a `make_run` take, the scene from `first_sighting - 1` to `hit + 1` at speed 1 gives `seen_at`, `assigned`, `time_to_hit` and `closest`, at the right clip times ± 0.05.
  - `test_speed_scales_times`: the same at speed 0.5 doubles `at`.
  - `test_insights_stack` (Review Focus 3): sighting and intercept 0.3 s apart give rows 0 and 1.
  - `test_missing_fact_left_out`: with `sighting_range_m` None, there's no `seen_at`.
  - `test_scene_with_insights_builds`: a story scene with `insights: true` builds a preview of the right length, and an ffmpeg frame grab at the label's time differs from the same scene built without insights in the label's region.
- [ ] **Step 2:** Run them and check they fail.
- [ ] **Step 3:** Implement it.
- [ ] **Step 4:** Run the tests and check they pass, then the whole suite.
- [ ] **Step 5:** Leave uncommitted.

### Task 5: Stories, docs and the manual test

**Files:** `recording/stories/default.yaml` (`insights: true` on the detection and hit scenes), `recording/stories/compare.yaml` (a ghost scene after the side-by-side, insights on), `recording/README.md` (map legend, `ghost`, `insights`), `hive-vault/recording/Recording demo videos.md`, and `hive-vault/detection/Manual tests to do.md`.

- [ ] **Step 1:** Write the stories and docs.
- [ ] **Step 2:** Add the manual test **"Map details and insights on a real shoot"**: `recording/video.sh final --story compare --preview <shoot>` on a step-2 shoot.

  **Pass:** the estimate circle follows the hostile with a plausible error (a few metres at 50 m); the ghost shows the baseline in grey at the same time after release; the labels appear at the sighting and the hit and are readable on the projector size (1920x1080 at about 3 m). **Note:** any label that is wrong or badly placed.
- [ ] **Step 3:** Run the whole suite. It must pass.
- [ ] **Step 4:** Leave uncommitted, and list the files.
