#!/usr/bin/env bash
# Train YOLO26 on Stellar's GPU in Ultralytics' own Docker image (PyTorch + CUDA), separate
# from the simulation container. Run from anywhere in the repo, on a Stellar computer:
#
#   training/train-stellar.sh                       train on data/gen/data.yaml, score on test
#   training/train-stellar.sh --epochs 3            quick try
#   training/train-stellar.sh --model yolo26s.pt    any train_yolo.py option
#
# Results land in training/runs/<name>/ (weights, curves, size_recall.csv). Check
# `nvidia-smi` first that nobody else is using the GPU (Stellar rules). The image is
# several GB the first time. IMAGE=... picks another Ultralytics image; it must know
# YOLO26 (Ultralytics 8.4 or later).
set -e
REPO="$(cd "$(dirname "$0")/.." && pwd)"
IMAGE="${IMAGE:-ultralytics/ultralytics:latest}"
nvidia-smi || echo "No nvidia-smi: is this a GPU computer?"
docker run --rm --device nvidia.com/gpu=all --ipc=host \
  -v "$REPO":/workspace -w /workspace/training \
  "$IMAGE" python3 train_yolo.py --data /workspace/data/gen/data.yaml --device 0 "$@"
