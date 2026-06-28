from dataclasses import dataclass
from typing import Optional, Tuple

BBox = Tuple[int, int, int, int]
Point = Tuple[float, float]

FAR = "FAR"
GOOD = "GOOD"
CLOSE = "CLOSE"
TOO_CLOSE = "TOO_CLOSE"
LOST = "LOST"

# Explicit follower states (state machine, see ConvoyController).
SEARCH = "SEARCH"
FOLLOW_LEFT = "FOLLOW_LEFT"
FOLLOW_CENTER = "FOLLOW_CENTER"
FOLLOW_RIGHT = "FOLLOW_RIGHT"
STOPPED = "STOPPED"
LOST_TARGET = "LOST_TARGET"
TOO_CLOSE_STATE = "TOO_CLOSE_STATE"  # distinct name from distance_state's TOO_CLOSE to avoid clashing


@dataclass
class TargetInfo:
    found: bool
    bbox: Optional[BBox]
    center_x: Optional[float]
    center_y: Optional[float]
    bottom_y: Optional[int]
    area: int
    score: float
    class_id: Optional[int]
    distance_state: str
    reason: str
    # Marker-bracket fields (see MarkerBracketDetector). dot_count is the
    # number of accepted holes found within the YOLO truck bbox this frame;
    # 0 if the truck itself wasn't found, or the bracket wasn't trusted
    # (below minimum_detected_dots). marker_bbox/marker_center are the
    # bracket cluster's own bounding box/centroid, distinct from bbox/
    # center_x/center_y above which describe the whole YOLO truck box --
    # kept separate so callers can choose which signal to steer/measure
    # distance from.
    dot_count: int = 0
    marker_bbox: Optional[BBox] = None
    marker_center_x: Optional[float] = None
    marker_center_y: Optional[float] = None
    marker_dot_centers: Tuple[Point, ...] = ()


@dataclass
class ConvoyCommand:
    should_move: bool
    left_speed: float
    right_speed: float
    speed_multiplier: float
    reason: str
    # Current follower state machine state, exposed for visualization/debugging.
    state: str = FOLLOW_CENTER
