from typing import List, Tuple

Detection = Tuple[Tuple[int, int, int, int], float, int]

class_names = {0: 'duckie', 1: 'truck', 2: 'sign'}

LANE_FOLLOWING = "lane_following"
OBSTACLE_PRESENT = "obstacle_present"

_state = LANE_FOLLOWING
_seen_frames = 0
_clear_frames = 0


def should_stop(detections: List[Detection], img_size: int) -> Tuple[bool, str]:
    global _state, _seen_frames, _clear_frames

    obstacle_found = False
    reason = ""

    for bbox, score, cls_id in detections:

        # allow class 2 (sign) to pass through too!
        if cls_id not in [0, 1, 2]:
            continue

        xmin, ymin, xmax, ymax = bbox

        w = xmax - xmin
        h = ymax - ymin

        area = w * h

        cx = (xmin + xmax) / 2
        cy = (ymin + ymax) / 2

        # Ignore detections too far left (opposing lane)
        if cx < img_size * 0.35:
            continue

        # Ignore high/far-away detections
        if cy < img_size * 0.45:
            continue


        # CONDITION FOR DUCKIES AND TRUCKS (Classes 0 and 1)
        if cls_id in [0, 1]:
            if area > img_size * img_size * 0.012 or ymax > img_size * 0.60:
                obstacle_found = True
                reason = f"Obstacle: {class_names.get(cls_id, cls_id)} {score:.2f}"
                break


        # CONDITION FOR STOP SIGNS (Class 2)
        # Stop signs are on the side, so they might have different area limits

        elif cls_id == 2:
            # If the sign is big enough (meaning we are close enough to it) -> STOP!
            if area > 800 or ymax > img_size * 0.50:
                obstacle_found = True
                reason = f"Sign Detected: {class_names.get(cls_id, cls_id)} {score:.2f}"
                break

    # Obstacle currently visible
    if obstacle_found:
        _seen_frames += 1
        _clear_frames = 0

        # Require obstacle to persist briefly before stopping
        if _seen_frames >= 2:
            _state = OBSTACLE_PRESENT
            return True, reason

        return False, "Slowing/checking obstacle"

    # If already stopped, stay stopped for a while even if detection flickers
    if _state == OBSTACLE_PRESENT:
        _clear_frames += 1

        # Hold stop longer to avoid collisions from flickering detections
        if _clear_frames < 20:
            return True, "Waiting for obstacle to clear"

    # Reset state
    _state = LANE_FOLLOWING
    _seen_frames = 0
    _clear_frames = 0

    return False, ""