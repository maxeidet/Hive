"""Run a YOLO model exported to ONNX (training/README.md) on an image. No PyTorch.

Uses ONNX Runtime when it is installed, else OpenCV's dnn module. Reads both kinds of
output Ultralytics exports:
  - end-to-end (YOLO26's default, no NMS needed): (1, N, 6) rows of x1, y1, x2, y2,
    confidence, class, in the network's input pixels;
  - classic (YOLOv8/11, or YOLO26 exported without its end-to-end head): (1, 4 + classes, N)
    columns of centre x, centre y, width, height and one score per class; NMS is done here.
The image is letterboxed to the network's square input the way Ultralytics does (scaled
to fit, padded with grey 114), and boxes are mapped back to the original image.
"""
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

PAD = 114


@dataclass(frozen=True)
class Box:
    x1: float
    y1: float
    x2: float
    y2: float
    score: float
    cls: int


def letterbox(rgb: np.ndarray, size: int) -> tuple[np.ndarray, float, tuple[int, int]]:
    """The image scaled to fit `size` x `size` and padded: (1, 3, size, size) float32 in
    0-1, the scale, and the (left, top) padding in px."""
    h, w = rgb.shape[:2]
    scale = min(size / w, size / h)
    nw, nh = round(w * scale), round(h * scale)
    resized = rgb if (nw, nh) == (w, h) else cv2.resize(rgb, (nw, nh), interpolation=cv2.INTER_LINEAR)
    left, top = (size - nw) // 2, (size - nh) // 2
    canvas = np.full((size, size, 3), PAD, np.uint8)
    canvas[top:top + nh, left:left + nw] = resized
    blob = canvas.astype(np.float32).transpose(2, 0, 1)[None] / 255.0
    return blob, scale, (left, top)


def decode(output: np.ndarray, conf: float, iou: float = 0.5) -> list[Box]:
    """Boxes (input pixels) above `conf` from either output layout (see the module doc)."""
    out = np.asarray(output)
    if out.ndim == 3:
        out = out[0]
    if out.ndim != 2:
        raise ValueError(f'Unexpected YOLO output shape {np.asarray(output).shape}')
    if out.shape[1] == 6 and out.shape[0] != 6:  # end-to-end: rows of x1 y1 x2 y2 conf cls
        return [Box(*map(float, r[:4]), float(r[4]), int(r[5])) for r in out if r[4] >= conf]
    rows = out.T if out.shape[0] < out.shape[1] else out  # classic: (4 + nc, N) -> (N, 4 + nc)
    scores = rows[:, 4:]
    cls = scores.argmax(axis=1)
    best = scores[np.arange(len(rows)), cls]
    keep = best >= conf
    rows, cls, best = rows[keep], cls[keep], best[keep]
    if not len(rows):
        return []
    cx, cy, w, h = rows[:, 0], rows[:, 1], rows[:, 2], rows[:, 3]
    xywh = np.stack([cx - w / 2, cy - h / 2, w, h], axis=1)
    picked = cv2.dnn.NMSBoxesBatched(xywh.tolist(), best.tolist(), cls.tolist(), conf, iou) \
        if hasattr(cv2.dnn, 'NMSBoxesBatched') else cv2.dnn.NMSBoxes(xywh.tolist(), best.tolist(), conf, iou)
    return [Box(float(xywh[i, 0]), float(xywh[i, 1]), float(xywh[i, 0] + xywh[i, 2]),
                float(xywh[i, 1] + xywh[i, 3]), float(best[i]), int(cls[i]))
            for i in np.array(picked).reshape(-1)]


def unletterbox(boxes: list[Box], scale: float, pad: tuple[int, int], width: int,
                height: int) -> list[Box]:
    """Boxes from the network's input back to the original image, cut to its edges."""
    left, top = pad
    out = []
    for b in boxes:
        x1 = min(max((b.x1 - left) / scale, 0.0), width)
        x2 = min(max((b.x2 - left) / scale, 0.0), width)
        y1 = min(max((b.y1 - top) / scale, 0.0), height)
        y2 = min(max((b.y2 - top) / scale, 0.0), height)
        if x2 > x1 and y2 > y1:
            out.append(Box(x1, y1, x2, y2, b.score, b.cls))
    return out


class YoloOnnx:
    """A loaded model: call it with an RGB image to get Boxes in that image's pixels."""

    def __init__(self, path: Path, imgsz: int | None = None):
        self.path = Path(path)
        if not self.path.exists():
            raise FileNotFoundError(self.path)
        try:
            import onnxruntime
            providers = ['CPUExecutionProvider']
            if 'CUDAExecutionProvider' in onnxruntime.get_available_providers():
                # onnxruntime-gpu (the Docker image on x86): the CUDA and cuDNN libraries
                # come from the nvidia-* pip packages, which preload_dlls() loads.
                if hasattr(onnxruntime, 'preload_dlls'):
                    onnxruntime.preload_dlls()
                providers.insert(0, 'CUDAExecutionProvider')
            self._session = onnxruntime.InferenceSession(str(self.path), providers=providers)
            self._input = self._session.get_inputs()[0]
            shape = self._input.shape
            used = self._session.get_providers()[0]  # CPU when CUDA could not start
            self.backend = 'onnxruntime ' + ('GPU' if used == 'CUDAExecutionProvider' else 'CPU')
        except ImportError:
            self._session = None
            self._net = cv2.dnn.readNetFromONNX(str(self.path))
            shape = None
            self.backend = f'opencv {cv2.__version__}'
        static = shape[-1] if shape and isinstance(shape[-1], int) else None
        self.imgsz = imgsz or static or 1280

    def run(self, blob: np.ndarray) -> np.ndarray:
        if self._session is not None:
            return self._session.run(None, {self._input.name: blob})[0]
        self._net.setInput(blob)
        return self._net.forward()

    def __call__(self, rgb: np.ndarray, conf: float = 0.25) -> list[Box]:
        blob, scale, pad = letterbox(rgb, self.imgsz)
        boxes = decode(self.run(blob), conf)
        return unletterbox(boxes, scale, pad, rgb.shape[1], rgb.shape[0])
