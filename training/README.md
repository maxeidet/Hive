# Training YOLO on the Gazebo images

`train_yolo.py` fine-tunes YOLO26 (COCO weights, 2 classes: drone, bird) on the images that `./mount-backend.sh gendata` writes to `data/gen/`, then scores the best weights on the test runs by pixel size (`size_recall.csv`). Background: `hive-vault/detection/Training data and YOLO training.md`.

It does not run in the simulation container (no PyTorch there). Use Stellar's GPU, or Colab.

## 1. Make the data (in the simulation)

Two generators write the same format, so their runs can be mixed:

```bash
cd ros2_ws/src/swarm_backend
# bare camera, fast:
for r in r01 r02 r03; do ./mount-backend.sh gendata --run $r --count 30; done
# the real gimbal drone takes the photos (spawn it once; it gets an id N):
./mount-backend.sh spawn gimbal_labels
./mount-backend.sh genlive N --run L01 --count 30
# data.yaml over all runs, split by run:
./mount-backend.sh gendata --yaml --val r02 --test r03
```

Three bare-camera runs of 30 images are a small test set: about 5 minutes and 60 MB. Train, val and test are whole runs, never mixed. For a real set use more and longer runs (several thousand images). Details and the differences between the generators: the vault note "Training data and YOLO training".

`data/` is not in git. On Stellar, the easiest is to generate the data on the Stellar computer itself (the simulation runs there too). Otherwise zip it (`cd data && zip -r gen.zip gen`) and move it over.

### Cut-outs for YOLO as a second stage

To train YOLO only on cut-outs around the baseline's candidates (how it runs on the drone in the plan's approach A), make them from the same runs, then train on them at their own size:

```bash
./mount-backend.sh makecrops                      # data/gen -> data/crops, same split
python3 train_yolo.py --data <...>/crops/data.yaml --imgsz 128 --device 0
```

Two kinds of cut-outs (`--mode`): `scaled` (default) cuts 3× the candidate's size (at least 12 px) and resizes to 128 px, so the target is always about a third of the cut-out; `fixed` always cuts 64 px and resizes to 256 px, so the target's size varies with distance (train with `--imgsz 256`). Make the fixed ones into another folder to compare (`--mode fixed --out /workspace/data/crops_fixed`). The size bins in `size_recall.csv` are then in cut-out pixels.

## 2a. Train on Stellar

```bash
training/train-stellar.sh --epochs 3     # quick try first
training/train-stellar.sh                # 50 epochs, early stopping
```

It runs Ultralytics' Docker image with the GPU and writes to `training/runs/`. See the script for options.

## 2b. Train on Colab

1. Zip the data on the Mac: `cd data && zip -r gen.zip gen`.
2. Open https://colab.research.google.com, File → Upload notebook → `training/colab_train.ipynb`.
3. Runtime → Change runtime type → GPU.
4. Run the cells in order. The first one shows an upload button: pick `gen.zip` and `training/train_yolo.py`. The last one downloads `yolo_results.zip` with `weights/best.pt`, `weights/best.onnx`, `size_recall.csv` and the plots.

## 2c. Real drone-vs-bird images first, then Gazebo (Colab)

`colab_dvb_then_gazebo.ipynb`: stage 1 trains on ~21 000 real images ([Drone vs Bird, Mendeley](https://data.mendeley.com/datasets/6ghdz52pd7/2), CC BY 4.0, downloaded straight into Colab and put in our class order), stage 2 fine-tunes that model on `gen.zip` (or `crops.zip`) with part of the network frozen and a lower learning rate, and an optional run trains the same stage 2 from plain COCO weights for comparison. The settings (`MODEL`, `STAGE2_FREEZE`, `STAGE2_LR0`, epochs, full images or cut-outs) are at the top of the notebook and explained at its end. `train_yolo.py` takes the same options: `--freeze N`, `--lr0`, `--patience`, `--no-score`.

## 3. Use the weights

Put the files in `models/<name>/` in the repo (e.g. `models/yolo26n-test/best.onnx`; `models/` is not in git). Then, in `ros2_ws/src/swarm_backend/`:

```bash
./mount-backend.sh predict yolo                 # on the saved images in data/Manualimages
./mount-backend.sh detect 2 --detector yolo     # live on drone 2's camera
```

Both pick the newest `models/*/*.onnx`; `--weights /workspace/models/<name>/best.onnx` picks one, `--conf` sets the lowest confidence (default 0.25), `--yolo-birds` also returns birds. The model runs on the CPU through `swarm_backend/detect/yolo_onnx.py`: ONNX Runtime if it is installed, else OpenCV (4.6 in the container, which may be too old for YOLO26; `opset=12` above helps, and adding `onnxruntime` to the container is the fix if not).

## Notes

- `--imgsz 1280` (default) trains on full images at their own size, so a 4 px drone stays 4 px. The planned on-drone version runs on crops around candidates; that comes later.
- Augmentation: `scale=0.2` instead of 0.5, so tiny drones are not shrunk away; stronger brightness changes, since the generator's sun barely shows.
- Weights are not committed. Publish a model as a GitHub Release with a model card (see the vault note).
