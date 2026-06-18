import colorsys
from typing import List


def set_turning_leds(direction: str) -> dict:
    """Set LEDs to indicate turning direction.

    LED layout:
        FRONT
        [0]     [2]

        [4]     [3]
        BACK

    Supported directions:
        - left
        - right
        - forward
        - stop

    Returns:
        dict: Mapping of LED index to [R, G, B] values in range [0.0, 1.0]
    """
    yellow = [1.0, 1.0, 0.0]
    white = [1.0, 1.0, 1.0]
    red = [1.0, 0.0, 0.0]
    off = [0.0, 0.0, 0.0]

    direction = direction.lower().strip()

    if direction == "left":
        return {
            0: yellow,            # front-left on
            2: off,               # front-right off
            3: off,               # back-right off
            4: yellow,            # back-left on
        }

    elif direction == "right":
        return {
            0: off,               # front-left off
            2: yellow,            # front-right on
            3: yellow,            # back-right on
            4: off,               # back-left off
        }

    elif direction == "forward":
        return {
            0: white,             # all LEDs white
            2: white,
            3: white,
            4: white,
        }

    elif direction == "stop":
        return {
            0: red,               # all LEDs red
            2: red,
            3: red,
            4: red,
        }

    raise ValueError("direction must be one of: 'left', 'right', 'forward', 'stop'")