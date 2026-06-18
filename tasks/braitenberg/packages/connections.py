from typing import Tuple
import numpy as np


def get_motor_left_matrix(shape: Tuple[int, int]) -> np.ndarray:
    """Left motor weight matrix: highest at bottom-left, decreasing toward top-right."""
    h, w = shape

    y = np.linspace(0.0, 1.0, h).reshape(h, 1)   # top -> bottom
    x = np.linspace(0.0, 1.0, w).reshape(1, w)   # left -> right

    vertical = y                                     # weight grows toward bottom
    horizontal = 1.0 - x                             # weight grows toward left

    weights = vertical * horizontal                  # combine: max at bottom-left
    return weights.astype(np.float32)


def get_motor_right_matrix(shape: Tuple[int, int]) -> np.ndarray:
    """Right motor weight matrix: highest at bottom-right, decreasing toward top-left."""
    h, w = shape

    y = np.linspace(0.0, 1.0, h).reshape(h, 1)   # top -> bottom
    x = np.linspace(0.0, 1.0, w).reshape(1, w)   # left -> right

    vertical = y                                   # weight grows toward bottom
    horizontal = x                                 # weight grows toward right

    weights = vertical * horizontal                # combine: max at bottom-right
    return weights.astype(np.float32)