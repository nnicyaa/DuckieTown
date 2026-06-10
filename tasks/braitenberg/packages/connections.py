from typing import Tuple
import numpy as np


def get_motor_left_matrix(shape: Tuple[int, int]) -> np.ndarray:
    """Left motor weight matrix: highest at bottom-left, decreasing toward top-right."""
    h, w = shape[:2]

    # y goes from 0 (top) to 1 (bottom)
    y = np.linspace(0, 1, h)
    # x goes from 1 (left) to 0 (right) to make the left side the highest
    x = np.linspace(1, 0, w)

    X, Y = np.meshgrid(x, y)

    # multiply the grids: Bottom-left becomes 1.0 * 1.0 = 1.0 (Highest)
    # top-right becomes 0.0 * 0.0 = 0.0 (Lowest)
    return Y * X


def get_motor_right_matrix(shape: Tuple[int, int]) -> np.ndarray:
    """Right motor weight matrix: highest at bottom-right, decreasing toward top-left."""
    h, w = shape[:2]

    # y goes from 0 (top) to 1 (bottom)
    y = np.linspace(0, 1, h)
    # x goes from 0 (left) to 1 (right) to make the right side the highest
    x = np.linspace(0, 1, w)

    X, Y = np.meshgrid(x, y)

    # multiply the grids: Bottom-right becomes 1.0 * 1.0 = 1.0 (Highest)
    # top-left becomes 0.0 * 0.0 = 0.0 (Lowest)
    return Y * X
