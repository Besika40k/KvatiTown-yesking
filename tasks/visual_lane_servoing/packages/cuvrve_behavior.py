from typing import List, Tuple
import numpy as np


# ============================================================================
# CURVE DETECTION — slice shift analysis
# ============================================================================


def detect_curve(
    yellow_xs: List[int],
    white_xs:  List[int],
    shift_threshold: int = 350,
) -> Tuple[bool, int]:
    """
    xs[0] is nearest to the robot; xs[-1] is farthest ahead.
    Returns (is_curve, direction) where direction: +1 = right, -1 = left, 0 = none.
    """
    shift = None

    if len(yellow_xs) >= 2:
        shift = yellow_xs[-1] - yellow_xs[0]
    elif len(white_xs) >= 2:
        shift = white_xs[-1] - white_xs[0]

    if shift is None or abs(shift) < shift_threshold:
        return False, 0

    direction = 1 if shift > 0 else -1
    return True, direction
