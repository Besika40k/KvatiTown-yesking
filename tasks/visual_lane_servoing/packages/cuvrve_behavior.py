from typing import List, Tuple
import numpy as np


def detect_curve(
    yellow_xs: List[int],
    white_xs:  List[int],
    curve_threshold: int = 350,
) -> Tuple[bool, int]:
    # xs[0] is closest to the robot, xs[-1] farthest ahead.
    # If a line shifts by > curve_threshold pixels between near and far,
    # the road is curving. The sign tells direction:
    #   +1 = right curve, -1 = left curve, 0 = straight.
    shift = None

    if len(yellow_xs) >= 2:
        shift = yellow_xs[-1] - yellow_xs[0]
    elif len(white_xs) >= 2:
        shift = white_xs[-1] - white_xs[0]

    if shift is None or abs(shift) < curve_threshold:
        return False, 0

    direction = 1 if shift > 0 else -1
    return True, direction