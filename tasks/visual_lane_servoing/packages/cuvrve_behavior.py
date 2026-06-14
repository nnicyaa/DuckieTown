from typing import List, Tuple
import numpy as np


def detect_curve(yellow_xs: List[int], white_xs: List[int], curve_threshold: int = 40) -> Tuple[bool, float]:
    """
    Measures road curvature based on the lateral shift of lane pixel coordinates.
    xs[0] is closest to the robot (near), xs[-1] is farther ahead (far).

    Returns:
        is_curving (bool): True if curvature exceeds the threshold.
        curvature_intensity (float): A normalized metric representing turn severity.
    """
    shifts = []

    if len(yellow_xs) > 5:
        # Measure structural delta between foreground and background tracking lines
        yellow_shift = int(yellow_xs[-1]) - int(yellow_xs[0])
        shifts.append(yellow_shift)

    if len(white_xs) > 5:
        white_shift = int(white_xs[-1]) - int(white_xs[0])
        shifts.append(white_shift)

    if not shifts:
        return False, 0.0

    avg_shift = float(np.mean(shifts))

    if abs(avg_shift) > curve_threshold:
        # Normalize intensity score
        intensity = min(1.0, abs(avg_shift) / 200.0)
        return True, intensity

    return False, 0.0