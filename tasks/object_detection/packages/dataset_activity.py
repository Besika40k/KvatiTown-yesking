import json
from typing import List

# Classes the model is trained to detect.
# The index here is the class ID written into YOLO label files.
CLASSES = ['duckie', 'truck', 'sign']

# Images are resized to this square size before training.
IMAGE_SIZE = 416


def convert_labelme_json(json_path: str, img_w: int, img_h: int) -> List[str]:
    with open(json_path) as f:
        data = json.load(f)

    lines = []
    for shape in data["shapes"]:
        label = shape["label"]
        if label not in CLASSES:
            continue

        cls_id = CLASSES.index(label)
        (x1, y1), (x2, y2) = shape["points"]
        xmin, xmax = min(x1, x2), max(x1, x2)
        ymin, ymax = min(y1, y2), max(y1, y2)

        xmin_s = xmin * IMAGE_SIZE / img_w
        xmax_s = xmax * IMAGE_SIZE / img_w
        ymin_s = ymin * IMAGE_SIZE / img_h
        ymax_s = ymax * IMAGE_SIZE / img_h

        cx = (xmin_s + xmax_s) / 2 / IMAGE_SIZE
        cy = (ymin_s + ymax_s) / 2 / IMAGE_SIZE
        w  = (xmax_s - xmin_s) / IMAGE_SIZE
        h  = (ymax_s - ymin_s) / IMAGE_SIZE

        lines.append(f"{cls_id} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}")

    return lines
