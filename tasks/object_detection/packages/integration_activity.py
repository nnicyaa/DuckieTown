from typing import Tuple

MODEL_PATH = "tasks/object_detection/models/best.onnx"


def NUMBER_FRAMES_SKIPPED() -> int:
    return 1


def filter_by_classes(pred_class: int) -> bool:
    # stop only for duckies
    # ignore signs.
    return pred_class in [0, 1, 2]


def filter_by_scores(score: float) -> bool:
    return score >= 0.55


def filter_by_bboxes(bbox: Tuple[int, int, int, int]) -> bool:
    xmin, ymin, xmax, ymax = bbox

    width = xmax - xmin
    height = ymax - ymin

    if width <= 5 or height <= 5:
        return False

    area = width * height

    # ignore tiny far-away detections.
    if area < 700:
        return False

    # opposing-lane filter:
    # keep detections mostly in our/right lane and center.
    cx = (xmin + xmax) / 2
    if cx < 416 * 0.35:
        return False

    return True