# Recording step 2 (comparison and shoot): implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fly many runs in one go from a shoot file, with baseline and ours on the same runs, pre-screened in the fast sim and ranked. Show the pair side by side in the film, with the results and the earlier evaluation's numbers on the end card.

**Architecture:** `demo/shoot.py` expands a shoot YAML into concrete runs, pre-screens random entries with the fast sim (`baseline_eval intercept --sim`, computing the same facts as `demo/takes.py`), and keeps the shoot's status. `recording/shoot.sh` drives `recording/record.sh` once per run, reusing the sim between runs with the same config. Takes become addressable across a shoot (`<run>/<k>`, `best`, `best2`), are ranked from their facts, and a story's `compare` layout puts the two takes of a pair side by side, synced on `release`.

**Tech Stack:** Python 3.12 in the `hive-px4-vnc` container (PyYAML, OpenCV, numpy, ffmpeg, matplotlib), bash + tmux on the host, pytest.

**Spec:** `hive-vault/recording/Recording - smoother demo videos (design).md`, sections 2, 4 and 7 (step 2 in "Order of the work"). **Builds on step 1** (`hive-vault/recording/Recording - step 1 plan (record and cut).md`, built 23 Nov, uncommitted until Oskar commits). Its interfaces are used as they are in the code: `demo.runinfo.write_run_info / read_run_info / RunInfo`, `demo.takes.find_takes / Take / FACTS / source_ranges`, `demo.story.load_story / resolve / Resolved / pick_take / StoryError`, `demo.final.build / cut_source / scene_clip / card_clip / end_card_lines`, `demo.fake_run.make_run`, and `recording/record.sh`.

**Written before step 1 was tested on Stellar** (Oskar's choice, 23 Nov). Before starting, read the "record.sh and final on Stellar" result in `hive-vault/detection/Manual tests to do.md`. If it changed any step-1 interface, follow the code, not this plan, and ledger a ruling.

## Global Constraints

- Everything on sim time: frame i of a clip that starts at S shows `S + i / 25`. Film: H.264, 1920x1080, 25 fps, yuv420p, `+faststart`. Preview: 960x540, `ultrafast`, `crf 28`.
- No browser in the container. Nothing installed on the host (bash, tmux, docker only).
- Don't change the detection or attack code or the topics between them. `scenarios/*.sh` and `swarm_backend/scenarios/*.py` are used as they are, through their command-line options.
- Claude never flies the sim. The fast sim (`--sim`, no Gazebo) may be run in tests: it takes well under a second per run.
- No commits, no pushes; Oskar commits. `chmod +x` after editing a `.sh`.
- Tests: `docker exec px4-gazebo-vnc bash -c "cd /workspace/ros2_ws/src/swarm_backend && python3 -m pytest -q tests/test_demo_*.py"`. The whole suite (`tests/test_*.py`) must pass at the end.
- The facts carry the same names everywhere: `hit`, `attacker`, `sighting_range_m`, `release_to_sighting_s`, `sighting_to_hit_s`, `closest_pass_m`, `setup`, plus `in_frame_frac` from this plan.

## Review Focus

1. **A pair where one side has no take, or no hit** (the baseline never saw the hostile): the `compare` scene still builds. The side without the moment shows its run from the same time after `release`, and its result reads "no hit". Test: Task 6, `test_compare_side_without_hit`.
2. **A shoot stopped half-way** (Ctrl-C, Stellar reboot): `shoot.sh` again carries on with the runs not done, and doesn't redo or overwrite done ones. Test: Task 3, `test_resume_skips_done`.
3. **Pre-screening that finds nothing wanted in N seeds:** say so per entry, and fly the best N anyway, ranked by how many `want:` items they meet, rather than flying nothing. Test: Task 2, `test_screen_nothing_matches`.
4. **Two runs in a row with different configs** (`SWARM_CONFIG`): the sim is restarted between them, never reused. Test: Task 3, `test_plan_restarts_sim_on_config_change`.
5. **`take: best2` when only one take has a hit:** a clear `StoryError` naming how many takes hit. Test: Task 4, `test_best_n_too_few`.

---

## File Structure

| File | Responsibility |
|---|---|
| `swarm_backend/demo/shoot.py` | shoot YAML: load, expand into runs, pairs, seeds; status file; CLI `plan` / `status` / `mark` |
| `swarm_backend/demo/prescreen.py` | the fast-sim pre-screen: run seeds per setup, facts from sim rows, `want:` predicates |
| `swarm_backend/demo/takes.py` | + `fact_in_frame_frac`, + `shoot_takes(folder)`, + `rank(takes)` |
| `swarm_backend/demo/story.py` | `take:` names across a shoot (`best`, `bestN`, `<run>/<k>`, `pair:<entry>`); `compare` layout |
| `swarm_backend/demo/final.py` | builds from a run folder or a shoot folder; the compare scene; `end_card: compare` + `evidence:` |
| `swarm_backend/demo/evidence.py` | hit rate, median time to hit and detection range per setup from `baseline_eval` folders |
| `recording/shoot.sh` | host: plan, pre-screen, `record.sh` per run, resume, the end table |
| `recording/record.sh` | + `--reuse-sim` (start the parts in the running sim's session) |
| `recording/shoots/demo.yaml` | an example shoot |
| `recording/video.sh` | `moments` on a shoot folder prints the ranking |
| tests: `test_demo_shoot.py`, `test_demo_prescreen.py`, `test_demo_rank.py`, `test_demo_compare.py`, `test_demo_evidence.py`, `test_demo_record_dryrun.py` (extend) | |

Shoot folder: `data/recordings/<shoot>/shoot.json` (the plan and each run's status), `data/recordings/<shoot>/<run>/` (one normal run folder per run, as in step 1), `data/recordings/<shoot>/prescreen/` (the sim's rows).

---

### Task 1: The shoot file and its plan

**Files:** Create `swarm_backend/demo/shoot.py`, `recording/shoots/demo.yaml`. Test `tests/test_demo_shoot.py`.

**Interfaces:**
- Produces:
  - `@dataclass Entry: name: str; run: list[str]; repeat: int = 1; compare: list[str] | None = None; seeds: str | None = None; want: list[str] = []; config: str | None = None; options: list[str] = []`
  - `@dataclass Run: id: str; entry: str; command: list[str]; config: str; setup: str | None; seeds: list[int] | None; pair: str | None; options: list[str]`
  - `load_shoot(path: Path) -> list[Entry]`, which raises `ShootError` naming the entry and the key
  - `expand(entries: list[Entry], rng_seed: int = 0, prescreened: dict[str, list[int]] | None = None) -> list[Run]`
  - `same_sim(a: Run, b: Run) -> bool`, true when the configs are equal
  - CLI `python3 -m swarm_backend.demo.shoot plan SHOOT.yaml [--prescreened F]` prints one tab-separated line per run: `id`, `keep_sim` (1/0: the next run has the same config), `config`, then the command.

Rules:
- `run:` is a string, split with `shlex`. `config:` defaults to `scenarios/surveillance.yaml` (a path inside the container). `options:` holds extra `record.sh` options, for example `[--cameras, attacker_watch]`.
- `repeat: N` gives runs `<name>-1` … `<name>-N`.
- `compare: [baseline, yolo-bytetrack-mpc]` gives one run per setup, with ids `<name>-<n>-baseline` and `<name>-<n>-ours`, `--setup <s>` appended to the command, and the same `pair` id `<name>-<n>`. The setup names are checked against `scenarios.common.SETUPS`.
- `seeds: random` gives each repeat a seed from `random.Random(rng_seed)` in 0-999, appended as `--seeds <s>`, unless `prescreened[entry]` gives the seeds. `seeds: 7` or `seeds: 0-2` is passed as it is.

- [ ] **Step 1: Write the failing tests:**
  - `test_expand_repeat_and_ids`: `repeat: 2` gives ids `head-on-1` and `head-on-2`.
  - `test_compare_pairs`: `compare: [baseline, yolo-bytetrack-mpc]` with `repeat: 1` gives 2 runs with the same `pair`, setups `baseline` and `yolo-bytetrack-mpc`, ids ending `-baseline` and `-ours`, and both commands containing the same `--seeds`.
  - `test_random_seeds_repeatable`: the same `rng_seed` gives the same seeds, and `prescreened={'random': [12, 40]}` gives exactly 12 and 40.
  - `test_unknown_key_and_setup`: `repat: 2` raises naming `repat`, and `compare: [best]` raises naming `best`.
  - `test_plan_restarts_sim_on_config_change` (Review Focus 4): two runs with different `config` print `keep_sim` 0 on the first line; two with the same config print 1 on the first and 0 on the last.
- [ ] **Step 2:** Run them and check they fail.
- [ ] **Step 3:** Implement it. In `demo.yaml`, write three entries: a head-on pair (`scenarios/trial.sh 1 --aspects 0 --speeds 4`, `compare`), a weave pair (`--path weave`, `compare`), and a random zigzag (`seeds: random`, `repeat: 3`, `want: [ours_hits, baseline_misses]`, `compare`).
- [ ] **Step 4:** Run the tests and check they pass.
- [ ] **Step 5:** Leave uncommitted.

### Task 2: Pre-screening in the fast sim

**Files:** Create `swarm_backend/demo/prescreen.py`. Test `tests/test_demo_prescreen.py`.

**Interfaces:**
- Consumes: `Entry` (Task 1), the fast sim CLI `python3 -m swarm_backend.scenarios baseline_eval intercept --sim --config CFG --setup S --seeds A-B --out DIR [the entry's other baseline_eval args]`, which writes `DIR/intercept.jsonl` with one row per seed (`seed`, `outcome`, `closest_miss`, `time_to_hit`, `detect_distance`, `detected_by`, `killer`, `setup`).
- Produces:
  - `sim_facts(row: dict) -> dict`: the step-1 fact names from a sim row. `hit` = `outcome == 'hit'`, `sighting_to_hit_s` = `time_to_hit`, `sighting_range_m` = `detect_distance`, `closest_pass_m` = `closest_miss`, `attacker` = `killer`, else `detected_by`.
  - `WANTS: dict[str, Callable[[dict, dict | None], bool]]`, taking (ours facts, baseline facts or None). It holds `ours_hits`, `ours_misses`, `baseline_hits`, `baseline_misses`, `ours_sees_earlier` (ours `sighting_range_m` greater than the baseline's by at least 10 m), and `fast_hit` (ours `sighting_to_hit_s` below 5).
  - `prescreen(entry: Entry, n_candidates: int, keep: int, out: Path) -> tuple[list[int], str]`: the chosen seeds and a one-line report.
  - CLI `python3 -m swarm_backend.demo.prescreen SHOOT.yaml --candidates N --out DIR`, which writes `DIR/prescreened.json` (`{entry: [seeds]}`) and prints one report line per entry.
- The scenario arguments passed to the sim are the entry's `run` arguments after the script name, minus `--seeds` and the leading run count `N`. `trial.sh N ARGS` runs `baseline_eval intercept --aspects 0 --speeds 4 ARGS` (see `scenarios/trial.sh`), so pre-screening uses `--aspects 0 --speeds 4` plus ARGS. Only entries with `seeds: random` and a `want:` are pre-screened. Candidate seeds are `random.Random(rng_seed)`-drawn from 0-999, and the same for both setups.
- **When nothing matches** (Review Focus 3), keep the `keep` seeds that meet the most `want:` items, and report `"<entry>: 0 of N candidates match <want>; flying the closest: ..."`.

- [ ] **Step 1: Write the failing tests:**
  - `test_sim_facts_names`: a row from a real sim run (inline JSON copied from a `--sim` run) gives exactly the step-1 fact keys used.
  - `test_wants`: on hand-made facts, `ours_hits` and `baseline_misses` are True for (hit, miss) and False for (miss, hit).
  - `test_screen_nothing_matches` (Review Focus 3): with fake rows where nothing matches, it still returns `keep` seeds and the report contains `0 of`.
  - `test_prescreen_runs_the_sim`: integration, really runs the fast sim with 3 candidates and `keep=1`. It returns 1 seed from the candidates and writes both setups' `intercept.jsonl`. Mark it `@pytest.mark.slow` if it takes more than 10 s.
- [ ] **Step 2:** Run them and check they fail.
- [ ] **Step 3:** Implement it, with `subprocess.run([sys.executable, '-m', 'swarm_backend.scenarios', 'baseline_eval', ...], cwd=PKG)`.
- [ ] **Step 4:** Run the tests and check they pass. Note in the ledger how long the 3-candidate run took.
- [ ] **Step 5:** Leave uncommitted.

### Task 3: `shoot.sh` and `record.sh --reuse-sim`

**Files:** Create `recording/shoot.sh`. Modify `recording/record.sh` and `swarm_backend/demo/shoot.py` (status: `mark`, `status`). Test `tests/test_demo_shoot.py` (extend) and `tests/test_demo_record_dryrun.py` (extend).

**Interfaces:**
- Consumes: `shoot plan` (Task 1), `prescreen` (Task 2), and `record.sh` (step 1: `--name`, `--keep-sim`, `--dry-run`, options).
- Produces:
  - `recording/shoot.sh SHOOT.yaml [--name NAME] [--prescreen N] [--dry-run] [-- RECORD_OPTIONS]`. The default name is the YAML's stem plus the date.
  - `record.sh --reuse-sim`: requires a running `px4-gazebo-vnc` and the tmux session `hive-rec` with a `sim` window. It skips starting the sim and its waits, and starts the other parts as windows in that session. Its stop order leaves `sim` and the session running, like `--keep-sim`.
  - `python3 -m swarm_backend.demo.shoot mark SHOOT_DIR RUN_ID done|failed` and `status SHOOT_DIR`, which maintain `shoot.json` (`{"runs": [{"id", "status", "command", ...}], "created": ..., "yaml": ...}`).

Behaviour of `shoot.sh`, in a temporary container (as `video.sh`'s `container`, no sim needed for planning):
1. With `--prescreen N`: run the `prescreen` CLI, and save to `<shoot>/prescreen/`.
2. Run `shoot plan`. Write `shoot.json` if it isn't there yet. If it's there, it's a **resume** (Review Focus 2): runs with status `done` are skipped. If the YAML changed since (compare its stored text), stop and ask for `--name` of a new shoot.
3. For each run not done: `record.sh --name <shoot>/<id> [--keep-sim if keep_sim] [--reuse-sim if the previous run kept the sim] RECORD_OPTIONS <entry options> -- <command>` with `SWARM_CONFIG=<config>`. Then `shoot mark` done (exit 0) or failed (any other). A failed run is logged and the shoot goes on. If the previous run failed, the next run starts its own sim (stop any leftover sim and session first, as `record_tuning.sh` `stop_sim` does).
4. At the end, stop the sim if it is still kept. Print a table (id, status, takes, hits), then `video.sh moments <shoot>` (the ranking, Task 4).
- Ruling to carry: in a reused sim, `start-stellar.sh --record` already chose the `gz/` log folder, which is the first run's. The Gazebo log of later runs sits in the first run's `gz/`. Only manual replays use it; say so in the README.

- [ ] **Step 1: Write the failing tests:**
  - `test_resume_skips_done` (Review Focus 2): with `shoot.json` showing run 1 done, `shoot.sh --dry-run` prints `SKIP <run1>` and `RECORD <run2>`.
  - `test_dry_run_reuse_flags`: three runs with the same config print `--keep-sim` on the first two and `--reuse-sim` on the last two.
  - `test_record_reuse_sim_dry_run` (in the record dry-run tests): `record.sh --dry-run --reuse-sim scenarios/trial.sh 1` has no `START sim` line, no `STOP sim` line, and no `STOP session` line.
  - `test_mark_and_status`: `mark` then `status` prints the run as done.
- [ ] **Step 2:** Run them and check they fail.
- [ ] **Step 3:** Implement it. `shoot.sh --dry-run` prints `PRESCREEN ...`, `SKIP <id>` and `RECORD <id>: <record.sh command>` lines, and runs nothing.
- [ ] **Step 4:** Run the tests and check they pass. `bash -n` both scripts, then `chmod +x`.
- [ ] **Step 5:** Leave uncommitted.

### Task 4: Takes across a shoot, ranking

**Files:** Modify `swarm_backend/demo/takes.py`, `swarm_backend/demo/story.py` and `recording/video.sh` (`moments` on a shoot folder). Test `tests/test_demo_rank.py`.

**Interfaces:**
- Consumes: `Take`, `FACTS`, `find_takes` (step 1), `camera_video.project`, `load_poses`, `pose_at`.
- Produces:
  - `Take` gains `folder: Path | None = None` and `name: str = ''` (`<run id>/<k>`, or `<k>` in a single run). `find_takes` sets both.
  - `fact_in_frame_frac(d) -> float | None`: the fraction of 0.2 s samples from `first_sighting` to `hit` (or the take's end) where the target's true position projects inside the `cam_attacker_watch` image. It uses `rec_cam_attacker_watch`'s pose from `poses.jsonl`, the `hfov`, `width` and `height` from `cam_attacker_watch.json`, and `camera_video.project`. It's None without that camera. Registered in `FACTS`.
  - `shoot_takes(folder: Path) -> list[Take]`: every take of every run folder under a shoot folder (any subfolder with `run.json`), in `shoot.json` order. For a plain run folder it returns `find_takes`.
  - `rank(takes) -> list[tuple[Take, float, list[str]]]`: best first, with a score and its reasons. Hit before no hit, then a shorter `sighting_to_hit_s`, then a higher `in_frame_frac`. Missing facts count as worst.
  - `story.pick_take(name, takes)` accepts `best` (rank 1), `bestN` (rank N, a `StoryError` "only K takes hit" when N exceeds the hits, Review Focus 5), `<run id>/<k>`, or `<k>` in a single run.
  - `video.sh moments <shoot>` prints the ranking table: rank, take name, setup, hit, time to hit, in-frame percentage, reasons.
- `Resolved` gains nothing: the take carries its folder, and `final` uses `r.take.folder` for its cuts (Task 6).

- [ ] **Step 1: Write the failing tests** on a shoot folder built from `make_run` into `shoot/a-1`, `shoot/a-2` and so on, with `shoot.json` written by the test:
  - `test_shoot_takes_names`: two runs of 2 takes give names `a-1/1`, `a-1/2`, `a-2/1`, `a-2/2`, each with its folder.
  - `test_rank_order`: with `hit=[False, True]` in one run, the hit take ranks first, and its reasons mention `hit`.
  - `test_best_n_too_few` (Review Focus 5): `pick_take('best2', ...)` with one hit raises `StoryError` containing `only 1`.
  - `test_in_frame_frac`: in `make_run`, the camera stays where it is. Add a `rec_cam_attacker_watch` pose to `fake_run`, fixed and looking at the hostile's path, so the fraction is between 0.5 and 1. Change `fake_run` for this and keep its existing tests green.
- [ ] **Step 2:** Run them and check they fail.
- [ ] **Step 3:** Implement it.
- [ ] **Step 4:** Run the tests and check they pass, plus all `test_demo_*`.
- [ ] **Step 5:** Leave uncommitted.

### Task 5: The evaluation numbers

**Files:** Create `swarm_backend/demo/evidence.py`. Test `tests/test_demo_evidence.py`.

**Interfaces:**
- Consumes: `baseline_eval.load_runs(path)`, if it's importable without ROS (else read the JSON lines directly), and the folders `data/baseline_eval/` (baseline) and `data/baseline_eval_<setup>/` (each other setup), each with `intercept.jsonl`.
- Produces:
  - `evidence(root: Path) -> dict[str, dict]`: per setup, `{'runs', 'hits', 'hit_rate', 'median_time_to_hit_s', 'median_detect_m'}`.
  - `evidence_lines(ev) -> list[str]`, for example `"Gazebo evaluation, 36 runs each: baseline 12/36 hits, ours 29/36"` and `"median time to hit: baseline 6.2 s, ours 3.4 s"`.
- `ours` = `yolo-bytetrack-mpc`. A missing folder for a setup gives that setup `None`, and the lines say "no evaluation runs" for it.

- [ ] **Step 1: Write the failing tests:** `test_evidence_counts` (two temporary folders of hand-made rows: hits, runs and medians right) and `test_missing_setup`.
- [ ] **Step 2:** Run them and check they fail.
- [ ] **Step 3:** Implement it.
- [ ] **Step 4:** Run the tests and check they pass.
- [ ] **Step 5:** Leave uncommitted.

### Task 6: The `compare` layout and the compare end card

**Files:** Modify `swarm_backend/demo/story.py`, `swarm_backend/demo/final.py` and `recording/stories/` (add `compare.yaml`). Test `tests/test_demo_compare.py`.

**Interfaces:**
- Consumes: `Run.pair` through `run.json` (`write_run_info` gets `pair`, written by `record.sh` from a new `--pair` option that `shoot.sh` passes), `shoot_takes`, `rank`, `evidence_lines` (Task 5), and `final`'s scene encoding.
- Produces:
  - Story `layout: compare` with `show: [ROLE]` (one source, shown for both sides) and `take: pair:<pair id>` or `take: best_pair`, the pair whose "ours" take ranks best. `from` and `to` are moments of the **ours** take, ± seconds.
  - `story.resolve` gives a compare scene as **two** `Resolved` (ours, then baseline) with the same `scene`. The baseline side's `start` and `end` = its own `release` + (the ours range − ours `release`). That's sync on release: both sides show the same time after the hostile set off.
  - `final.compare_clip(ours: Resolved, base: Resolved, cuts, out, preview) -> Path`: side by side, ours on the right, labelled "Baseline" / "Ours" with the setup. Each side shows its result (`hit 3.1 s after sighting`, or `no hit, closest 6.0 m`) from its own `hit` moment on, or from the scene's end when there's no hit (drawtext `enable='gte(t,X)'`).
  - `end_card: compare` in a story: a card with `end_card_lines` per setup over all pairs used, and `evidence_lines` if the story has `evidence: <path>`. Bars are drawn with matplotlib to a PNG and overlaid on the card.
  - `final.build` gets the run folder or the shoot folder. Cuts are per `take.folder`, and the cache key includes the folder.

- [ ] **Step 1: Write the failing tests** on a shoot folder with a pair of runs from `make_run`, `run.json` setups baseline and ours, and a shared `pair`:
  - `test_compare_synced_on_release`: the baseline side's `start` − its `release` equals the ours side's `start` − its `release`.
  - `test_compare_side_without_hit` (Review Focus 1): the baseline made with `hit=[False]`. The scene still resolves, the film builds, and the baseline result text is `no hit`.
  - `test_compare_preview_builds`: a story with one compare scene of 4 s and `end_card: compare` gives a preview of 4 + 3 s ± 0.15, at 960x540.
  - `test_best_pair`: `take: best_pair` picks the pair whose ours take ranks first.
- [ ] **Step 2:** Run them and check they fail.
- [ ] **Step 3:** Implement it. `recording/stories/compare.yaml`: title, the default story's first two scenes from `best`, then compare scenes for `best_pair` (detection to hit with `chase`, then `map`), the hit at half speed (`compare` with `yolo`), and `end_card: compare` with `evidence: data/baseline_eval`.
- [ ] **Step 4:** Run the tests and check they pass, then the whole suite.
- [ ] **Step 5:** Leave uncommitted.

### Task 7: Docs and the manual test

**Files:** `recording/README.md` (a "Many runs: shoot.sh" section, the compare layout and `take:` names in the story reference, the ranking, and the `gz/` note from Task 3), `hive-vault/recording/Recording demo videos.md` (step 2 done), and `hive-vault/detection/Manual tests to do.md`.

- [ ] **Step 1:** Write the docs.
- [ ] **Step 2:** Add the manual test **"shoot.sh with a compare pair on Stellar"**:
  1. `recording/shoot.sh recording/shoots/demo.yaml --prescreen 30`, unattended in tmux.
  2. `recording/video.sh moments <shoot>`.
  3. `recording/video.sh final --story compare --preview <shoot>`.

  **Pass:** the runs record one after another without restarting the sim between same-config runs; a Ctrl-C and a restart of `shoot.sh` carries on; the ranking puts a hit first; in the compare scene both sides show the hostile setting off at the same moment. **Note:** the pre-screen's hit predictions against the Gazebo outcomes (how many of the "ours hits, baseline misses" seeds really did).
- [ ] **Step 3:** Run the whole suite. It must pass.
- [ ] **Step 4:** Leave uncommitted, and list the files.
