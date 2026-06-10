import time
from typing import List, Tuple

Detection = Tuple[Tuple[int, int, int, int], float, int]

class_names = {0: 'duckie', 1: 'truck', 2: 'sign'}

# --- STATE MACHINE CONSTANTS ---
LANE_FOLLOWING = "lane_following"
OBSTACLE_PRESENT = "obstacle_present"
INITIATE_PASS = "initiate_pass"

# Explicit Multi-Stage State Registries
PASSING_CLEARANCE = "passing_clearance"
PASSING_CURVE_MATCH = "passing_curve_match"
RETURN_RIGHT = "return_right"

# --- GLOBAL TRACKING VARIABLES ---
_state = LANE_FOLLOWING
_seen_frames = 0
_clear_frames = 0
_cooldown_until = 0.0

# Passing Timers
_state_start_time = 0.0
_last_bbox_width = 0.0
_estimated_target_speed = 0.0


def check_passing_safety(detections: List[Detection], img_size: int, current_lane_omega: float) -> bool:
    """
    Returns True ONLY if the road is straight and the oncoming lane is empty.
    """
    # 1. Curvature Gate: If turning hard, safety stop instead of passing
    if abs(current_lane_omega) > 0.15:
        print("[PassingSafety] Curvature check failed. Curve detected.")
        return False

    # 2. Oncoming Traffic Gate with Depth Filtering
    for bbox, score, cls_id in detections:
        if cls_id not in [0, 1]:  # Only look for vehicles
            continue

        xmin, ymin, xmax, ymax = bbox
        cx = (xmin + xmax) / 2

        # Check if an object is in the oncoming left lane zone (0% to 40% horizontal screen width)
        if cx < img_size * 0.40:
            # If the object is high up on the horizon, ignore it as noise
            if ymax < (img_size * 0.48):
                continue

            area = (xmax - xmin) * (ymax - ymin)
            if area > 600:
                print(f"[PassingSafety] TRUE Oncoming hazard (Class {cls_id}) at ymax {ymax:.1f}. Aborting pass.")
                return False

    return True


def estimate_obstacle_speed(w: int) -> float:
    """
    Measures the relative bounding box size changes to guess if the obstacle is moving.
    """
    global _last_bbox_width
    if _last_bbox_width == 0.0:
        _last_bbox_width = w
        return 0.0

    delta_w = w - _last_bbox_width
    _last_bbox_width = w

    if delta_w < 1.5:
        return 0.05
    return 0.0


def should_stop(detections: List[Detection], img_size: int, current_lane_omega: float = 0.0) -> Tuple[bool, str, float, float]:
    global _state, _seen_frames, _clear_frames, _state_start_time, _cooldown_until

    current_time = time.time()
    obstacle_found = False
    reason = ""

    # -----------------------------------------------------------
    # GATED OVERRIDE: COOLDOWN SHIELD
    # -----------------------------------------------------------
    if current_time < _cooldown_until:
        return False, "Passing Cooldown Active: Stabilizing Lane Path", -1.0, -1.0

    # -----------------------------------------------------------
    # RUNTIME MANEUVER OVERRIDES (Multi-Stage Curve Passing)
    # -----------------------------------------------------------
    # STAGE 1: Swerve cleanly out into the left lane
    if _state == INITIATE_PASS:
        elapsed = current_time - _state_start_time
        if elapsed < 0.55:
            return True, "Passing Step 1: Angling Left", 0.16, 0.52
        else:
            _state = "align_left"
            _state_start_time = current_time
            return True, "Passing Step 2: Straightening onto left side", 0.16, -0.45

    # STAGE 2: Align parallel to the road corridor
    if _state == "align_left":
        elapsed = current_time - _state_start_time
        if elapsed < 0.40:
            return True, "Passing Step 2: Aligning Parallel", 0.16, -0.45
        else:
            _state = PASSING_CLEARANCE
            _state_start_time = current_time

    # STAGE 3A: CLEARANCE PHASE - Drive straight past the duckie profile blind to the curve view
    if _state == PASSING_CLEARANCE:
        elapsed = current_time - _state_start_time
        if elapsed < 0.70:
            return True, "Passing Step 3A: Clearing duckie profile blindly", 0.16, 0.0
        else:
            _state = PASSING_CURVE_MATCH
            _state_start_time = current_time

    # STAGE 3B: CONTOUR PHASE - Trace the curve of the road from the left lane safely
    if _state == PASSING_CURVE_MATCH:
        elapsed = current_time - _state_start_time
        if elapsed < 1.0:
            if current_lane_omega > 0.02:
                dynamic_omega = current_lane_omega + 0.05
            else:
                dynamic_omega = 0.22
            return True, "Passing Step 3B: Dynamically tracing road curve", 0.15, dynamic_omega
        else:
            _state = RETURN_RIGHT
            _state_start_time = current_time
            return True, "Passing Step 4: Cutting back to home lane", 0.14, -0.55

    # STAGE 4: Return cleanly into the home lane
    if _state == RETURN_RIGHT:
        elapsed = current_time - _state_start_time
        if elapsed < 0.60:
            return True, "Passing Step 4: Angling Right", 0.14, -0.55
        else:
            print("[FSM] S-Curve Complete! Handing full control to Lane Agent with Cooldown Shield.")
            _state = LANE_FOLLOWING
            _seen_frames = 0
            _clear_frames = 0
            _cooldown_until = current_time + 2.5
            return False, "Maneuver Completed", -1.0, -1.0

    # -----------------------------------------------------------
    # SPATIAL LANE FILTERING
    # -----------------------------------------------------------
    for bbox, score, cls_id in detections:
        if cls_id not in [0, 1, 2]:
            continue

        xmin, ymin, xmax, ymax = bbox
        w, h = (xmax - xmin), (ymax - ymin)
        area = w * h
        cx = (xmin + xmax) / 2
        cy = (ymin + ymax) / 2

        if _state == LANE_FOLLOWING and cx < img_size * 0.40:
            continue

        if cls_id in [0, 1]:
            if cx > img_size * 0.75:
                continue
            if area > img_size * img_size * 0.012 or ymax > img_size * 0.60:
                obstacle_found = True
                _estimated_target_speed = estimate_obstacle_speed(w)
                reason = f"Obstacle: {class_names.get(cls_id, cls_id)} ahead."
                break

        elif cls_id == 2:
            if cx > img_size * 0.90:
                continue
            if area > 1200 or ymax > img_size * 0.50:
                obstacle_found = True
                reason = "Stop Sign obeyed."
                break

    # -----------------------------------------------------------
    # FINITE STATE MACHINE TRANSITIONS
    # -----------------------------------------------------------
    if obstacle_found:
        _seen_frames += 1
        _clear_frames = 0

        if _seen_frames >= 2:
            if cls_id in [0, 1] and check_passing_safety(detections, img_size, current_lane_omega):
                print("[FSM] Road is safe. Initiating Passing Sequence.")
                _state = INITIATE_PASS
                _state_start_time = current_time
                return True, "Initiating Overtake", 0.16, 0.45
            else:
                _state = OBSTACLE_PRESENT
                return True, reason, 0.0, 0.0

        return False, "Slowing down / assessing path safety", -1.0, -1.0

    if _state == OBSTACLE_PRESENT:
        _clear_frames += 1
        if _clear_frames < 60:
            return True, "Waiting for obstacle profile to clear", 0.0, 0.0

    _state = LANE_FOLLOWING
    _seen_frames = 0
    _clear_frames = 0
    return False, "", -1.0, -1.0


def reset_fsm():
    """Resets all tracking states completely for a fresh simulation run."""
    global _state, _seen_frames, _clear_frames, _state_start_time, _cooldown_until
    _state = LANE_FOLLOWING
    _seen_frames = 0
    _clear_frames = 0
    _state_start_time = 0.0
    _cooldown_until = 0.0
    print("[FSM] State Machine variables cleared successfully.")