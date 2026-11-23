# Training data and YOLO training

How we collect labelled training data in Gazebo and train the YOLO detector. This note records what is known and decided as of 17 Oct 2026; alternative approaches are discussed afterwards and added here. Background: [[Detection implementation plan]] (steps 6–7), [[Progress 2026-10-13]], [[Manual - scenario 1, images and prediction]].

## Goal

A labelled, run-split dataset of Gazebo camera images with drones, birds and empty sky, and YOLO26 weights trained on it. Together they replace the `yolo` placeholder in `detect/detectors.py`, so that `./mount-backend.sh detect 2 --detector yolo` and `predict yolo` run the trained model. The rest of the pipeline (tracking, direction, distance, the two published topics) stays the same whichever detector runs, so baseline and YOLO are scored by the same code.

What the report asks of the detector:

| Part     | Requirement                                                                     |
| -------- | ------------------------------------------------------------------------------- |
| Find     | Flag small objects from about 4 px across                                       |
| Classify | Drone or bird, from about 20 px across                                          |
| Training | Gazebo images with automatic labels (drones, birds, empty sky; 20–120 m altitude; split by run), fine-tune COCO weights to 2 classes |
| Scoring  | Missed drones, P(detect) vs range, birds flagged, background false alarms, recall vs pixel size, CPU inference time |

## What the camera sees

The baseline camera is the `x500_gimbal` camera: 1280×720, 114.6° horizontal FOV (2.0 rad), 30 Hz, no distortion, so `f ≈ 411 px`. A target of width `s` at distance `d` is `411 · s / d` px wide.

| Target width in image | Drone (0.5 m) at | Bird (0.45 m wingspan) at |
| --------------------- | ---------------- | ------------------------- |
| 2 px                  | 103 m            | 92 m                      |
| 4 px (find)           | 51 m             | 46 m                      |
| 20 px (classify)      | 10 m             | 9 m                       |
| 100 px                | 2 m              | 1.8 m                     |

The planned narrow camera (step 5, not built: 20° at 1920×1080, so `f ≈ 5440 px`) sees the same sizes about 13× further away: 4 px at about 680 m, 20 px at about 136 m. Which camera the data is collected with, or both, is still open (see below).

## What exists to build on

| Piece | Where | What it gives, and what it lacks |
|---|---|---|
| `snap` / `snap(id, count)` | `save_images.py`, `mount-backend.sh snap`, `shell.py` | Saves PNGs from a drone's camera at chosen sim-time intervals (`--every 0` = every frame), with a `.json` of the truth next to each |
| Truth `.json` | `ground_truth.py` | Camera position and axes, and for each other **drone**: position, distance, direction and the **centre pixel** (`null` out of view). No box size, no occlusion check, **no birds** |
| PX4 camera orientation | `px4_camera.py`, saved as `px4_camera` in the `.json` | What a real system would know (0.35° median error against the truth) |
| `predict` | `detect/predict.py`, `mount-backend.sh predict` | Runs a detector on saved images, matches to the truth, writes a CSV and plots. `yolo` stops with a message |
| Detector interface | `detect/detectors.py` | `Detection(x, y, width, height, score, label)`. The YOLO wrapper plugs in here |
| Bird models | `feat-birds`: `birds.py`, `worlds/birds/`, `scenarios/birds.sh` | Dove, mourning dove and a flock of five, scaled to about 0.45 m wingspan, static and collision-free. `spawn`, `move`, `wander` and `remove` through Gazebo's `create` / `set_pose` / `remove` services. Size and orientation still need the manual check in [[Manual tests to do]]. Shown without wing animation |
| Saved images | `data/Manualimages/` (scenario 1, ~10 images), `data/detect_frames/` | Small test sets, not training data. `data/` is not tracked by git |

Licences of the bird meshes: dove and mourning dove are CC BY 4.0 (credit needed wherever images made with them are shared, e.g. the report). The flock is Sketchfab Standard: use in the project is fine, but the file must not be redistributed.

## How the data is collected

Three ways, all writing the same format (`data/gen/<run>/images|labels|meta`, `objects.csv`), so runs of any of them can be mixed in one `data.yaml` (`./mount-backend.sh gendata --yaml --val R --test R`, split by run). Decided 17 Oct: keep both generators, mostly live for realism, the bare camera for quick volume.

| | Bare-camera generator (`gendata`) | Live generator (`genlive`) | Flown sequences (`snap`) |
|---|---|---|---|
| Who takes the photo | A bare camera with the gimbal camera's settings, teleported | PX4 flies the gimbal drone; its own gimbal camera | The gimbal drone in a flown scenario |
| Looks like flight | Roughly: own body added as an unlabelled x500 around the camera | Exactly: own legs and propellers, body tilt, gimbal lag and wobble | Exactly, plus real target motion |
| Camera poses | Anything: 20–120 m, any heading, pitch −15° to +35° | What the drone flies: hovers at `--heights`, a new heading and gimbal pitch per image | What the scenario flies |
| Targets | Static labelled x500s placed at random pixels and sizes | The same, around the drone's real camera pose | The PX4 drones of the scenario (and birds) |
| Speed | ~0.3 images/s on the Mac | Slower: PX4 runs, the drone turns and climbs | Camera rate, but few distinct images |
| Labels | Gazebo's visible boxes | `--labels visible` (x500_gimbal_labels model) or `--labels projected` | Projected boxes |

**Bare-camera generator**: `./mount-backend.sh gendata --run r01 --count 500` (`swarm_backend/datagen/gen_data.py`, built and tried 17 Oct: 20 images, sizes spread well). A bare camera with the x500_gimbal camera's settings (115°, 1280×720) and its visible-box sensor, static labelled x500s, random camera pose, random sun from high to just above the horizon (via Gazebo's `light_config`; in `hive_base` the change hardly shows: the sky is a fixed picture and the ambient light is strong), 0–3 drones placed at random pixels at the distance that gives the drawn size (2–100 px, weighted towards 4 and 20 px), about 15 % empty images, and an unlabelled x500 where the camera drone's own body would be (`--no-own-body` leaves it out). Birds: 0–2 per image by default (see Birds below). Night is left out (a visible-light camera sees nothing; real systems use thermal).

**Drone variety** (built 18 Oct, not run in the sim yet): both generators place drones of several kinds and colours instead of only the black x500, so the model learns "small drone" and not "x500". `swarm_backend/datagen/target_models.py` builds look-only copies of PX4's own drone models (only their visuals, in one static link: no sensors, plugins or physics) with the Label system: `x500` (0.66 m), `px4vision` (0.47 m), `omniquad` (0.6 m), `quadtailsitter` (0.7 m) and `standard_vtol` (a 2.1 m fixed-wing VTOL), picked with weights 35/20/15/15/15 %. Half of them (`--recolour 0.5`) are repainted in a plain colour (black, dark grey, grey, white, red, olive). Each run spawns `--variants` (default 8) such drones and picks 1–3 of them per image; `--kinds` limits the kinds. The distance for a drawn size uses each kind's own width; `meta/*.json` lists which variant was placed where. `genlive --labels projected` places only unpainted x500s (the projected box size is the x500's). All five copies pass Gazebo's SDF check; whether the repainting also covers textured meshes is in [[Manual tests to do]].

**Birds** (18 Oct): four new flying birds from Poly Pizza (CC BY 3.0, credits in `worlds/birds/README.md`): `seagull` and `gull` (same low-poly shape, 0.7 and 0.55 m long, wings raised), `eagle` (gliding, 1.9 m span) and `soaring_bird` (0.9 m span). Their orientation in `birds.py` was set from top, front and side views of the meshes, not yet from a render. Together with the dove they are `birds.FLYING`, which both generators place by default: 0–2 birds per image (`--birds`, `--bird-kinds`), each at the distance its own span gives for the drawn size, labelled class 1. The mourning dove (thin sliver) and the flock stay out.

**Live generator** (built 17 Oct, not run yet): the real gimbal drone takes the photos.

```
./mount-backend.sh spawn gimbal_labels                 # once: the camera drone, id N
./mount-backend.sh genlive N --run L01 --count 200     # Gazebo's visible boxes
./mount-backend.sh genlive 2 --run L02 --labels projected   # stock gz_x500_gimbal
```

- `swarm_backend/datagen/gen_live.py`: takes off, climbs to each of `--heights` (default 25, 50, 80, 120 m) in turn and hovers; per image it turns 20–70° to a new heading, aims the gimbal at a random pitch (`--pitch`, default −15° to +35° from the horizon) and a small yaw off the nose, waits `--settle` sim seconds, reads the camera's true pose from Gazebo, places the targets around it (the same choices as `gendata`), and saves the next image with its labels. It lands at the end or on Ctrl-C.
- `worlds/models/x500_gimbal_labels/`: PX4's x500_gimbal plus two bounding-box cameras (visible and full boxes) fixed to the gimbal's `camera_link` with the camera sensor's own pose and lens, so they turn with the gimbal and see exactly what the camera sees. They run at 10 Hz, every third camera image, to keep rendering cheap; the generator picks an image that has boxes from the same moment. The model flies with PX4's gimbal airframe (4019): `mount-backend.sh` reads the number from the `airframe` file in the model folder.
- `--labels projected` works with the stock drone: boxes projected from a box around each placed model (`ground_truth.HALF_SIZE`, centred 0.17 m above the x500's feet) with the camera's true pose at the image's time. Looser, and blind to trees in the way.

**Flown sequences** (planned): flown scenarios recorded at the camera rate (`snap --every 0`) with the hostile drone under PX4 and birds on `wander` paths. Each object keeps a track id across frames. These give real target motion, hovering vs moving targets, and the data needed to tune and score the tracker (k-in-a-row now, ByteTrack later). `snap` already saves projected boxes for every drone and bird (`yololabels` turns them into label files).

Train / val / test are split **by run** (a run = one generator seed or one flown sequence), never by frame: neighbouring frames are near-duplicates and would leak.

## Labels

- **Source, preferred:** Gazebo's bounding-box camera (`boundingbox_camera` sensor, 2D *visible* boxes) with the `Label` system on each drone and bird model. Visible boxes account for trees in the way and match what is in the pixels. It must share pose, resolution and FOV with the RGB camera.
- **Test:** `./mount-backend.sh boxtest` (`swarm_backend/datagen/box_test.py`, 17 Oct) tries the bounding-box camera with labelled models at 3–150 m; see [[Manual tests to do]].
- **Fallback (built 17 Oct):** project a box around each model into the image with the pinhole math in `ground_truth.py` (`HALF_SIZE` per class, `corners`, `project_box`). No occlusion handling. `snap` now saves these boxes for every other drone and every bird (models named `bird...`) under `objects` in each `.json`, and `./mount-backend.sh yololabels DIR` (`yolo_labels.py`) writes YOLO label files from them, leaving out objects under 2 px or mostly outside the image (listed in `labels/ignored.csv`). `HALF_SIZE` is an estimate until `boxtest` compares it with Gazebo's full boxes.
- **Classes:** `drone` and `bird`. Empty images have no boxes.
- **Ignore:** objects under about 2 px or fully hidden are flagged and not counted as misses.
- **Format:** YOLO `.txt` per image (class, centre x, centre y, width, height, normalised), plus a per-image `.json` that keeps everything else: run id, track id per object, distance, visibility (visible box ÷ full box), moving or hovering (from the true velocity), camera pose and PX4's orientation. Pixel size bins and P(detect) vs range are computed from this `.json`.
- The truth `.json` that `snap` saves has boxes, birds and model names (= track ids) since 17 Oct; visibility and moving/hovering are still to add.

## Rendering settings

Checked 17 Oct with `boxtest`:

- **Far clip plane** of the x500_gimbal camera: 15 000 m, so targets are never cut off by distance.
- **Thin parts vanish.** Gazebo draws nothing thinner than a pixel: at 60 m the x500 is two dark dots (the motors), at 100 m it is gone completely, where a real camera would show a faint 1–2 px smudge. `--aa 8` (anti-aliasing) made no difference, probably because Docker on the Mac renders in software. So with the 115° camera, results beyond about 60 m are limited by the renderer, not the detector; the camera's design range is about 51 m (4 px), so this is accepted for now. Fix if needed: render at 2× (2560×1440) and shrink to 1280×720 by averaging (about 4× the rendering, and the flying camera must do the same). The narrow camera makes it moot.
  - **Not LOD or distance culling** (checked 20 Oct on the saved box tests): at 100 m the eagle (1.9 m span, 0.23 m thick) is still drawn (6 px wide, box found) while the x500 at the same distance is gone, and the eagle disappears at 150 m although it would still be about 5 px wide. What vanishes is whatever is thinner than about a pixel in one direction, at any distance: the renderer samples one point per pixel, so a part that misses the pixel centres is not drawn, and a pixel that is hit takes the colour of that one part (the "few pixels of the wrong colour"). Distance culling would remove a big and a small object at the same distance alike; a level-of-detail swap would show a coarser but whole silhouette, not just the motors. The camera's far clip is 1000 m on the generator's camera and 15 000 m on the gimbal camera, so nothing is clipped. So there is no LOD or cull setting to change; the fixes are anti-aliasing (to try again on Stellar's GPU, where MSAA may work) or supersampling (render larger and average down).
- **Bounding-box camera boxes** stop at about 2–3 px (box at 60 m, none at 100 m for the drone), which matches ignoring objects under 2 px.
- **Gazebo crash:** creating a bounding-box camera again under a name used before in the same Gazebo session crashes Gazebo (Ogre "node definition ... already exists"). `boxtest` and `gendata` give the camera a new name every run; if it still crashes, restart Gazebo before each run.
- Still not set: camera noise (Gaussian noise model in the SDF).

## Training

On Stellar's GPU (decided 17 Oct; Colab as an alternative), with Ultralytics. Built 17 Oct, not run yet: `training/train_yolo.py` (fine-tunes, then scores the best weights on the test runs: drone recall per pixel-size bin and false alarms per image, matching by centre), `training/train-stellar.sh` (Ultralytics' Docker image with the GPU) and `training/README.md` (Stellar and Colab steps). First small set: three runs of 30 images (r01 train, r02 val, r03 test, ~5 min, ~60 MB); 20 images take about 13 MB and the generator makes ~0.3 images/s on the Mac. The plan:

1. Start from `yolo26n.pt` and `yolo26s.pt` with `nc=2` (`drone`, `bird`). Ultralytics rebuilds the class head when `nc` differs and keeps the rest of the weights.
2. Stage 1 on Seraphim (real images), stage 2 on Gazebo plus about 20 % Seraphim.
3. **Same scale in training and at run time.** YOLO26n runs on enlarged crops around baseline candidates on the drone, so it is trained on crops of that size (e.g. 320 or 640 px). YOLO26s runs on full images at the base station. On full frames, the default 640 px input halves a 1280×720 image, so a 4 px drone becomes 2 px.
4. Limit augmentations that shrink objects further (mosaic, random down-scaling).
5. Early stopping on val. Choose the confidence threshold `τ` on val so that almost no drones are missed (the k-in-a-row tracker removes noise afterwards).
6. Report everything on held-out Gazebo runs, per pixel-size bin (e.g. under 10, 10–32, over 32 px), not only overall mAP.
7. Export to ONNX for CPU inference in the container. The weights are not committed to git, because git keeps every retrained version forever. Instead (agreed 17 Oct):
   - each trained model is attached to a **GitHub Release** of the repo (e.g. tag `yolo-v1`), which is stored next to the repo but outside git, so `git clone` / `git pull` never download it;
   - a small **model card** is committed (e.g. `models/yolo26n-v1.md`): dataset version and runs, code commit, image size, epochs, val/test numbers per pixel-size bin, `τ`, release tag and the file's sha256;
   - a script (e.g. `./mount-backend.sh get-model yolo26n-v1`) runs `gh release download` into a gitignored `models/` and checks the sha256; `--detector yolo` loads from there. Needs `gh auth login`, since the repo is private;
   - upload with `gh release create yolo-v1 models/yolo26n.onnx --notes-file models/yolo26n-v1.md`. The tag also ties the model to the exact code it was trained with.

## Evaluation

- Same seeded scenarios for baseline and YOLO, scored by one script: missed drones, P(detect) vs range, birds flagged as drones, background false alarms, recall vs pixel size, CPU inference time at the real crop size.
- Matching by box centre within a few pixels, not IoU (IoU is harsh on tiny boxes: a 1 px shift on a 5×5 box drops IoU to about 0.67).
- Flown sequences are also scored as tracks (time to confirm, lost locks).

## Known issues

- **Boxes one pixel short (fixed 18 Oct):** Gazebo's box corners are the first and last pixel of the object, both inclusive, so the right and bottom edges are one pixel further than the numbers say. Before the fix (`box_test.box_dict`), every label was a pixel too narrow and a pixel too short, which showed as parts of drones outside their boxes in the enlarged cut-outs. Runs generated before 18 Oct 18:10 (r01–r03, test01, own01) have the old labels: generate them again.
- **Own drone seen as a drone** (17 Oct, first 3-epoch model at confidence 0.01 on the scenario 1 images): parts of the camera drone itself (legs, propellers) that show in the gimbal camera's image are detected as a drone. The generator's bare camera never shows them, so the model has never learned they are background. Remedies: (1) **built 17 Oct, on by default:** in `gen_data.py`, put an unlabelled x500 where the camera drone's body is relative to the gimbal camera, so its parts appear as negatives in the training images; (2) mask the image area where the own body can show (it depends on the gimbal pitch) in the live detector, next to the existing sky mask; (3) a fixed spot in the image that does not move while the gimbal turns is not a target, so the tracker can reject it. Real training (more images and epochs, confidence 0.25) also lowers it, but propellers look drone-like, so (1) is needed anyway.

## Open questions

- [ ] Which camera to collect for: the 115° baseline camera, the narrow camera (step 5), or both?
- [ ] Bounding-box camera with the `Label` system: does it work with our meshes, including the `.glb` birds, and the drone models from PX4?
- [x] Where training runs: Stellar's GPU (Colab if easy), decided 17 Oct.
- [ ] Seraphim: access and licence.
- [ ] YOLO runtime in the container: `ultralytics` + CPU PyTorch, or ONNX Runtime only.
- [ ] Crop size used at run time, which fixes the training crop size.
- [ ] How many images per run and in total; how many runs for val and test.
- [ ] Do birds need wing animation, or is a still pose enough at these sizes?
- [ ] Drone variety: only the x500, or other drone models and colours too?

## Alternative model approaches

Not decided; to be discussed and the choice recorded here. The current plan is **A**. All of them plug in behind the same `Detection` interface in `detectors.py`, so they can be compared by the same `predict` and scenario scoring. Tile and pixel numbers are for our 1280×720 camera unless stated.

### A. Candidate crops (current plan)

A better first stage than the baseline is planned in [[Candidate detector v2 (blob2)]].

The baseline (`candidates.py`) finds dark objects against the sky; YOLO26n classifies an enlarged crop around each one at native resolution.

- **Good:** cheap (only a few crops per image), full resolution exactly where something is, reuses what is built. The candidate stage already finds targets of a few pixels.
- **Bad:** YOLO can never find what the baseline missed, e.g. a drone against trees or ground, or a faint one. Recall is capped by the baseline.
- **Data:** train on crops of the run-time size, cut around both true objects and baseline false alarms (hard negatives).
- **Built 17–18 Oct:** `./mount-backend.sh makecrops` (`swarm_backend/datagen/make_crops.py`) runs the baseline on every generated image (`data/gen/`) and cuts a square around each candidate, centred on the candidate (so as off-centre as real candidates are), resized to a fixed size. Two ways to size it (`--mode`, decided 18 Oct with scaled as the default):

  | Mode | Cut-out | Target in the cut-out | Defaults | For |
  |---|---|---|---|---|
  | `scaled` (default) | `--context` × the candidate's size, at least `--min-crop` px | About 1/context of it whatever the distance (from 4 px up; 2–3 px targets get more room): one scale to learn; far targets enlarged most (4 px drone: 12 px cut → 128 px, 11×) | context 3, min 12 px (was 24: then 4–8 px drones filled only 0.13–0.3, measured 18 Oct), out 128 px | YOLO or a classifier on candidates; cheaper (small input) |
  | `fixed` | Always `--crop` px of the image (twice the candidate's size if bigger) | Varies with distance (4 px drone → 16 px at 64 → 256), more surroundings kept | 64 px cut, out 256 px | Comparison, and when the candidate's size is unreliable |

  Scaled relies on the candidate's size being about right: when the baseline only catches part of a big drone (its motors), the cut-out is too tight; `--min-crop` and `--context` give margin. Cut-outs that cut through a drone or bird (part of it inside, under half) are left out, since as unlabelled negatives they would teach that a piece of a drone is background. Positives keep their box (moved into the cut-out); the baseline's false alarms are negatives (at most `--negatives` per image); `--missed` also cuts around drones the baseline missed. Output `data/crops/<run>/images|labels`, `crops.csv`, `settings.json` (the mode) and `data.yaml` with the same split by run. Train with `train_yolo.py --data data/crops/data.yaml --imgsz 128` (scaled) or `--imgsz 256` (fixed), and compare the two on the same test runs.
- **First run, 18 Oct** (118 generated images, offline): baseline candidate recall per drone size 2–4 px 0.61, 4–8 px 0.74, 8–16 px 0.61, 16–32 px 0.76, 32–64 px 0.32, 64–128 px 0.00 (its 40 px size limit). That is the ceiling for the two stages today, and the case for [[Candidate detector v2 (blob2)]].
- Generation is unchanged: crops, tiles and full frames are all cut from the same labelled full images, so this choice can be made late. The generated images have no horizon mask (the live detector only searches the sky), so `makecrops` sees more false alarms below the horizon than flight would.

### B. SAHI-style tiling (Slicing Aided Hyper Inference)

Cut the whole frame into overlapping tiles at native resolution, run YOLO on each, shift the boxes back to full-image coordinates and merge duplicates at tile borders (NMS, or merging overlapping boxes). Optionally one extra low-resolution pass over the full frame for large objects.

- Tiles of 640 px with 128 px overlap: **6 tiles** for 1280×720 (3 × 2), 8 for the narrow camera's 1920×1080 (4 × 2), about 1.6× the frame's pixels because of the overlap.
- **Good:** no dependency on the baseline, so it also finds drones against trees and ground; a model trained at 640 px can be reused; memory per pass is fixed.
- **Bad:** inference time grows with the number of tiles, roughly 6× one 640 pass on the drone's CPU (to be measured); each tile loses the context of the rest of the frame.
- **Data:** cut the training images into tiles the same way (sliced fine-tuning), so training and run-time scale match. Most tiles are empty sky: keep some, subsample the rest.
- **Variants:** tile only regions of interest (around baseline candidates or the tracker's predicted position, which is close to A), or run a full tiled scan every few frames and crops in between (see F).

### C. Full frame at native resolution, with or without a P2 head

Run YOLO once on the whole frame at `imgsz=1280` instead of the default 640 (which halves the image, so a 4 px drone becomes 2 px). Optionally add a P2 level (stride 4) to the network: by default YOLO predicts at strides 8, 16 and 32, so a 4 px drone covers half a cell at the finest level. Ultralytics has P2 model configs for earlier YOLOs (e.g. `yolov8-p2.yaml`); whether there is one for YOLO26 is to be checked. The stride-32 level can be dropped, since targets are at most about 100 px.

- **Good:** one pass, no merging, no dependency on the baseline. The P2 head is the standard fix for tiny objects.
- **Bad:** about 4× the compute of a 640 pass, plus the P2 head (4× the locations of stride 8). A P2 model does not start from a fully pretrained head. Probably too slow for the drone's CPU; a fit for the base station (YOLO26s).
- **Data:** full images at 1280; limit mosaic and down-scaling augmentation.

### D. Motion cues

At 4 px a drone and a bird both look like a dark dot; motion separates them from the background.

- **Frame differencing:** warp the previous image onto the current one and subtract; what remains moves on its own. The camera only rotates on the gimbal and we know the rotation from PX4, so the warp is `K R K⁻¹` from the two orientations: no ORB + RANSAC needed. Very cheap and finds movers of 2–3 px. Blind to a hovering drone; trees and clouds give noise.
- **Use:** as an extra candidate source next to the dark-object baseline (helps A), or as a fourth input channel to YOLO (the difference image next to RGB).
- **Data:** needs the flown sequences, labelled per frame with track ids, and hovering drones included so the model does not learn "drone = moving thing". Augment every frame of a window identically, or the augmentation creates fake motion.

### E. Multi-frame networks and track-before-detect

- **Multi-frame input:** stack 3–5 registered frames as input channels so the network learns motion and appearance together; only the first layer changes. More expressive than a difference channel, more data and compute.
- **Track-before-detect:** for targets too faint for any single image, add up evidence along possible trajectories over many frames before declaring a detection (radar and infrared technique).
- Both are probably beyond this project's scope; noted in case range at 2–4 px becomes the main limit.

### F. Combined pipeline

Each frame: crops around baseline candidates, motion blobs and the tracker's predicted positions (A + D), YOLO on those crops, then ByteTrack and k-in-a-row confirmation; every N frames, a full tiled scan (B) to catch new or hovering drones that the other sources miss. Most frames cost only a few crops.

### Compared

| | Finds what the baseline misses | Cost per image (drone CPU) | Training data | Fits |
|---|---|---|---|---|
| A. Candidate crops | No | Lowest (a few crops) | Crops, incl. baseline false alarms | Drone |
| B. Tiling | Yes | ~6 × 640 passes | Tiles | Drone if fast enough, base station |
| C. Full frame (+P2) | Yes | ~4 × a 640 pass, more with P2 | Full images at 1280 | Base station |
| D. Motion cues | Moving targets only | Very low | Sequences | Extra candidates for A |
| E. Multi-frame / TBD | Yes | High | Sequences | Probably out of scope |
| F. Combined | Yes (periodic scan) | Low on most frames | Crops + tiles + sequences | Drone |

Whatever is chosen, both kinds of data (placed stills and flown sequences) are useful: crops and tiles are cut from the same labelled full images, so the generator does not need to decide this up front.
