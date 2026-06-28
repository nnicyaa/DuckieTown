from dataclasses import dataclass
from typing import Optional, Tuple

BBox = Tuple[int, int, int, int]

FAR = "FAR"
GOOD = "GOOD"
CLOSE = "CLOSE"
TOO_CLOSE = "TOO_CLOSE"
LOST = "LOST"


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

    # NEW: smoothed rate of change of bottom_y, in pixels/frame.
    # Positive  = leader getting closer (bottom_y increasing, bbox growing).
    # Negative  = leader pulling away (bottom_y decreasing, bbox shrinking).
    # Zero/None = no rate available yet (first frame after acquiring target,
    #             or target was just reacquired after being lost).
    # Defaulted so any existing code constructing TargetInfo without this
    # field keeps working unchanged.
    approach_rate: Optional[float] = 0.0


@dataclass
class ConvoyCommand:
    should_move: bool
    left_speed: float
    right_speed: float
    speed_multiplier: float
    reason: str