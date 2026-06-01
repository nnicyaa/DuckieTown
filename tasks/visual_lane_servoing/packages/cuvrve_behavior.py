from typing import List, Tuple
import numpy as np


def detect_curve(yellow_xs: List[int], white_xs: List[int], curve_threshold: int = 350) -> Tuple[bool, int]:
    shift = None

    if len(yellow_xs) >= 2:
        shift = yellow_xs[-1] - yellow_xs[0]
    elif len(white_xs) >= 2:
        shift = white_xs[-1] - white_xs[0]

    if shift is None or abs(shift) <= curve_threshold:
        return False, 0

    return True, (1 if shift > 0 else -1)