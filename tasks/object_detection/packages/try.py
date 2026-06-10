




def should_stop(detections: List[Detection], img_size: int, current_lane_omega: float = 0.0) -> Tuple[bool, str, float, float]:
    global _state, _seen_frames, _clear_frames, _state_start_time, _cooldown_until

    current_time = time.time()
    obstacle_found = False
    reason = ""

    # -----------------------------------------------------------
    # GATED OVERRIDE: COOLDOWN SHIELD
    # -----------------------------------------------------------
    # If a cooldown is active, force the robot to follow the lane smoothly
    # without evaluating threats or executing transitions.
    if current_time < _cooldown_until:
        return False, "Passing Cooldown Active: Stabilizing Lane Path", -1.0, -1.0

    # -----------------------------------------------------------
    # RUNTIME MANEUVER OVERRIDES (Geometric 4-Stage S-Curve)
    # -----------------------------------------------------------
    # STAGE 1: Swerve Left out of the lane
    if _state == INITIATE_PASS:
        elapsed = current_time - _state_start_time
        if elapsed < 0.55:
            return True, "Passing Step 1: Angling Left", 0.16, 0.52
        else:
            _state = "align_left"
            _state_start_time = current_time
            return True, "Passing Step 2: Straightening onto left side", 0.16, -0.45

    # STAGE 2: Counter-steer Right to align parallel to the road
    if _state == "align_left":
        elapsed = current_time - _state_start_time
        if elapsed < 0.40:
            return True, "Passing Step 2: Aligning Parallel", 0.16, -0.45
        else:
            _state = PASSING_DRIVE
            _state_start_time = current_time

    # STAGE 3: Drive past the duckie while tracing the right-hand curve
    if _state == PASSING_DRIVE:
        elapsed = current_time - _state_start_time
        # Increased passing time to 1.8s to ensure it fully clears the duckie's bumper
        if elapsed < 1.8:
            if current_lane_omega > 0.05:
                dynamic_omega = current_lane_omega
            else:
                dynamic_omega = 0.22
            return True, "Passing Step 3: Driving past obstacle profile", 0.16, dynamic_omega
        else:
            _state = "return_right"
            _state_start_time = current_time
            return True, "Passing Step 4: Cutting back to home lane", 0.14, -0.52

    # STAGE 4: Swerve Right to return to the home lane
    if _state == "return_right":
        elapsed = current_time - _state_start_time
        if elapsed < 0.65:  # Extended slightly to ensure a complete lane return
            return True, "Passing Step 4: Angling Right", 0.14, -0.52
        else:
            print("[FSM] S-Curve Complete! Handing full control to Lane Agent with Cooldown Shield.")
            _state = LANE_FOLLOWING
            _seen_frames = 0
            _clear_frames = 0
            # Set the cooldown period directly where the state transitions occur
            _cooldown_until = current_time + 2.0
            return False, "Maneuver Completed", -1.0, -1.0

    # -----------------------------------------------------------
    # STANDARD HAZARD DETECTION SCANNER (Runs only if no overrides or cooldowns are active)
    # -----------------------------------------------------------

    # -----------------------------------------------------------
    # SPATIAL LANE FILTERING (Unchanged)
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
    # FINITE STATE MACHINE TRANSITIONS (Unchanged)
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