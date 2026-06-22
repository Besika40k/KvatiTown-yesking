from typing import Tuple
import numpy as np


def get_motor_left_matrix(shape: Tuple[int, int]) -> np.ndarray:
    h, w = shape

    # left side strong
    horizontal = np.linspace(1, 0, w).reshape(1, w)
    vertical = np.linspace(0.5, 1.0, h).reshape(h, 1)

    return horizontal * vertical


def get_motor_right_matrix(shape: Tuple[int, int]) -> np.ndarray:
    h, w = shape

    # right side strong
    horizontal = np.linspace(0, 1, w).reshape(1, w)
    vertical = np.linspace(0.5, 1.0, h).reshape(h, 1)

    return horizontal * vertical