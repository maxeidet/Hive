import numpy as np
import pytest

from swarm_backend.detect import yolo_onnx


def test_letterbox_pads_a_wide_image_top_and_bottom():
    rgb = np.zeros((720, 1280, 3), np.uint8)
    blob, scale, pad = yolo_onnx.letterbox(rgb, 1280)
    assert blob.shape == (1, 3, 1280, 1280) and blob.dtype == np.float32
    assert scale == 1.0 and pad == (0, 280)
    assert blob[0, 0, 0, 0] == pytest.approx(114 / 255)  # padding
    assert blob[0, 0, 280, 0] == 0.0  # the image


def test_letterbox_scales_down_to_fit():
    _, scale, pad = yolo_onnx.letterbox(np.zeros((720, 1280, 3), np.uint8), 640)
    assert scale == 0.5 and pad == (0, 140)


def test_decode_end_to_end_rows():
    out = np.zeros((1, 300, 6), np.float32)
    out[0, 0] = [100, 300, 110, 306, 0.9, 0]
    out[0, 1] = [500, 500, 520, 510, 0.1, 1]  # below the threshold
    boxes = yolo_onnx.decode(out, conf=0.25)
    assert boxes == [yolo_onnx.Box(100, 300, 110, 306, pytest.approx(0.9), 0)]


def test_decode_classic_columns_with_nms():
    n = 8400
    out = np.zeros((1, 6, n), np.float32)  # 4 box values + 2 class scores per column
    out[0, :, 0] = [105, 303, 10, 6, 0.8, 0.1]   # a drone
    out[0, :, 1] = [106, 303, 10, 6, 0.6, 0.1]   # the same drone again: removed by NMS
    out[0, :, 2] = [600, 400, 20, 10, 0.05, 0.7]  # a bird
    boxes = sorted(yolo_onnx.decode(out, conf=0.25), key=lambda b: b.cls)
    assert [b.cls for b in boxes] == [0, 1]
    assert (boxes[0].x1, boxes[0].y1, boxes[0].x2, boxes[0].y2) == pytest.approx((100, 300, 110, 306))
    assert boxes[0].score == pytest.approx(0.8)


def test_unletterbox_maps_back_and_cuts_to_the_image():
    boxes = [yolo_onnx.Box(100, 380, 110, 386, 0.9, 0),   # inside the picture (pad top 280)
             yolo_onnx.Box(10, 10, 20, 20, 0.9, 0)]       # in the padding only
    back = yolo_onnx.unletterbox(boxes, 1.0, (0, 280), 1280, 720)
    assert back == [yolo_onnx.Box(100, 100, 110, 106, 0.9, 0)]


def test_model_that_is_missing_raises():
    with pytest.raises(FileNotFoundError):
        yolo_onnx.YoloOnnx('/nonexistent/best.onnx')
