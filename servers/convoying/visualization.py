import cv2
import numpy as np


def create_convoying_visualization(
    image_bgr: np.ndarray,
    lane_debug_info: dict,
    target,
    command,
    lane_left: float,
    lane_right: float,
    detections,
) -> np.ndarray:
    display_w = 360
    h, w = image_bgr.shape[:2]
    display_h = int(h * display_w / w)

    camera_panel = cv2.resize(image_bgr, (display_w, display_h))

    _draw_detections(camera_panel, detections, w, h, display_w, display_h)
    _draw_target(camera_panel, target, w, h, display_w, display_h)
    _draw_marker(camera_panel, target, w, h, display_w, display_h)

    lane_panel = _make_lane_panel(lane_debug_info, display_w, display_h)
    info_panel = _make_info_panel(
        width=display_w * 2,
        target=target,
        command=command,
        lane_left=lane_left,
        lane_right=lane_right,
    )

    top = np.hstack([camera_panel, lane_panel])
    return np.vstack([top, info_panel])


def _draw_detections(panel, detections, orig_w, orig_h, display_w, display_h):
    if not detections:
        return

    sx = display_w / float(orig_w)
    sy = display_h / float(orig_h)

    for bbox, score, class_id in detections:
        x1, y1, x2, y2 = bbox

        dx1 = int(x1 * sx)
        dy1 = int(y1 * sy)
        dx2 = int(x2 * sx)
        dy2 = int(y2 * sy)

        color = (255, 100, 100)
        if class_id == 1:
            color = (0, 255, 0)

        cv2.rectangle(panel, (dx1, dy1), (dx2, dy2), color, 1)
        cv2.putText(
            panel,
            f"id:{class_id} {score:.2f}",
            (dx1, max(15, dy1 - 4)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            color,
            1,
        )


def _draw_target(panel, target, orig_w, orig_h, display_w, display_h):
    if target is None or not target.found or target.bbox is None:
        cv2.putText(
            panel,
            "TARGET LOST",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2,
        )
        return

    sx = display_w / float(orig_w)
    sy = display_h / float(orig_h)

    x1, y1, x2, y2 = target.bbox
    dx1 = int(x1 * sx)
    dy1 = int(y1 * sy)
    dx2 = int(x2 * sx)
    dy2 = int(y2 * sy)

    cv2.rectangle(panel, (dx1, dy1), (dx2, dy2), (0, 255, 255), 3)

    if target.center_x is not None and target.center_y is not None:
        cx = int(target.center_x * sx)
        cy = int(target.center_y * sy)
        cv2.circle(panel, (cx, cy), 5, (0, 255, 255), -1)

    cv2.putText(
        panel,
        f"TARGET {target.distance_state}",
        (10, 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 255),
        2,
    )


def _draw_marker(panel, target, orig_w, orig_h, display_w, display_h):
    """
    Draws the marker bracket's own bounding box (distinct from the whole
    truck bbox drawn by _draw_target) and the detected dot count. Drawn in
    magenta to be visually distinct from the cyan truck box and green/red
    YOLO detection boxes.
    """
    if target is None or not getattr(target, "marker_bbox", None):
        return

    sx = display_w / float(orig_w)
    sy = display_h / float(orig_h)

    x1, y1, x2, y2 = target.marker_bbox
    dx1 = int(x1 * sx)
    dy1 = int(y1 * sy)
    dx2 = int(x2 * sx)
    dy2 = int(y2 * sy)

    cv2.rectangle(panel, (dx1, dy1), (dx2, dy2), (255, 0, 255), 2)

    if target.marker_center_x is not None and target.marker_center_y is not None:
        mx = int(target.marker_center_x * sx)
        my = int(target.marker_center_y * sy)
        cv2.drawMarker(
            panel,
            (mx, my),
            (255, 0, 255),
            markerType=cv2.MARKER_CROSS,
            markerSize=10,
            thickness=2,
        )

    dot_count = getattr(target, "dot_count", 0)
    cv2.putText(
        panel,
        f"dots:{dot_count}",
        (dx1, min(display_h - 5, dy2 + 14)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (255, 0, 255),
        1,
    )


def _make_lane_panel(lane_debug_info, display_w, display_h):
    if not lane_debug_info or 'lane_mask' not in lane_debug_info:
        panel = np.zeros((display_h, display_w, 3), dtype=np.uint8)
        cv2.putText(
            panel,
            "No lane debug",
            (20, display_h // 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (100, 100, 100),
            2,
        )
        return panel

    lane_mask = lane_debug_info.get('lane_mask')
    lane_panel = cv2.applyColorMap(lane_mask, cv2.COLORMAP_HOT)
    lane_panel = cv2.resize(lane_panel, (display_w, display_h))

    cv2.putText(
        lane_panel,
        "Lane Mask",
        (10, 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 0),
        2,
    )

    return lane_panel


def _make_info_panel(width, target, command, lane_left, lane_right):
    height = 170
    panel = np.zeros((height, width, 3), dtype=np.uint8)
    font = cv2.FONT_HERSHEY_SIMPLEX

    if target is None:
        target_text = "Target: none"
        distance_text = "Distance: none"
        reason_text = "Reason: no target object"
        dot_text = "Dots: 0"
    else:
        target_text = f"Target found: {target.found}"
        distance_text = f"Distance: {target.distance_state}"
        reason_text = f"Target reason: {target.reason}"
        dot_text = f"Dots: {getattr(target, 'dot_count', 0)}"

    if command is None:
        state_text = "State: none"
        command_text = "Command: none"
        speed_text = f"Lane L/R: {lane_left:.3f} / {lane_right:.3f}"
        multiplier_text = "Multiplier: none"
    else:
        state_text = f"State: {getattr(command, 'state', '—')}"
        command_text = f"Move: {command.should_move} | {command.reason}"
        speed_text = (
            f"Lane L/R: {lane_left:.3f} / {lane_right:.3f}    "
            f"Final L/R: {command.left_speed:.3f} / {command.right_speed:.3f}"
        )
        multiplier_text = f"Multiplier: {command.speed_multiplier:.2f}"

    lines = [
        state_text,
        target_text,
        distance_text,
        dot_text,
        reason_text,
        command_text,
        speed_text,
        multiplier_text,
    ]

    y = 20
    for line in lines:
        cv2.putText(panel, line, (10, y), font, 0.42, (220, 220, 220), 1)
        y += 20

    return panel