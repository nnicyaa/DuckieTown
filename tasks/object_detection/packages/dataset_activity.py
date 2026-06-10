import json
from typing import List

CLASSES = ['duckie', 'truck', 'sign']
IMAGE_SIZE = 416


def convert_labelme_json(json_path: str, img_w: int, img_h: int) -> List[str]:
    if json_path is None:
        return []

    with open(json_path, "r") as f:
        data = json.load(f)

    lines = []

    for shape in data.get("shapes", []):
        label = shape.get("label", "").strip().lower()
        if label not in CLASSES:
            continue

        cls_id = CLASSES.index(label)
        points = shape.get("points", [])
        if not points:
            continue

        xs = [p[0] for p in points]
        ys = [p[1] for p in points]

        xmin = max(0, min(xs))
        xmax = min(img_w, max(xs))
        ymin = max(0, min(ys))
        ymax = min(img_h, max(ys))

        if xmax <= xmin or ymax <= ymin:
            continue

        x_center = ((xmin + xmax) / 2) / img_w
        y_center = ((ymin + ymax) / 2) / img_h
        width = (xmax - xmin) / img_w
        height = (ymax - ymin) / img_h

        lines.append(
            f"{cls_id} {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}"
        )

    return lines