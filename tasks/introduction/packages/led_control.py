import colorsys
from typing import Dict, List


def set_turning_leds(direction: str) -> Dict[int, List[float]]:
    """Set LEDs to indicate turning direction."""

    direction = direction.lower().strip()

    # Colors
    YELLOW = list(colorsys.hsv_to_rgb(0.15, 1.0, 1.0))
    WHITE = [1.0, 1.0, 1.0]
    RED = [1.0, 0.0, 0.0]
    OFF = [0.0, 0.0, 0.0]

    # LED indices (correct mapping)
    FRONT_LEFT = 0
    FRONT_RIGHT = 2
    BACK_LEFT = 4
    BACK_RIGHT = 3

    # Initialize all LEDs to OFF
    leds = {
        FRONT_LEFT: OFF.copy(),
        FRONT_RIGHT: OFF.copy(),
        BACK_LEFT: OFF.copy(),
        BACK_RIGHT: OFF.copy(),
    }

    if direction == "left":
        leds[FRONT_LEFT] = YELLOW.copy()
        leds[BACK_LEFT] = YELLOW.copy()

    elif direction == "right":
        leds[FRONT_RIGHT] = YELLOW.copy()
        leds[BACK_RIGHT] = YELLOW.copy()

    elif direction == "forward":
        leds[FRONT_LEFT] = WHITE.copy()
        leds[FRONT_RIGHT] = WHITE.copy()

    elif direction == "stop":
        leds[BACK_LEFT] = RED.copy()
        leds[BACK_RIGHT] = RED.copy()
    return leds