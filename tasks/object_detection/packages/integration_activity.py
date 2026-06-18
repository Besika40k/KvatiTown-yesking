from typing import Tuple

# Path to the trained model weights (.onnx file).
# Relative paths resolve from the project root.
MODEL_PATH = "tasks/object_detection/models/best.onnx"


def NUMBER_FRAMES_SKIPPED() -> int:
    # Inference runs every N+1 frames (2 = every third frame).
    return 2


def filter_by_classes(pred_class: int) -> bool:
    # Accept only duckie (0), truck (1), sign (2).
    return pred_class in [0, 1, 2]


def filter_by_scores(score: float) -> bool:
    # Discard detections below 50 % confidence.
    return score >= 0.5


def filter_by_bboxes(bbox: Tuple[int, int, int, int]) -> bool:
    # Discard bounding boxes smaller than 800 px² (too distant / noise).
    xmin, ymin, xmax, ymax = bbox
    w = xmax - xmin
    h = ymax - ymin
    area = w * h
    if area < 800:
        return False
    return True
