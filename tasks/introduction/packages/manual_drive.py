from typing import Dict, Tuple
import logging
logger = logging.getLogger(__name__)

SPEED = 1
TURN = 0.5


def get_motor_speeds(keys_pressed: Dict[str, bool]) -> Tuple[float, float]:
    left = 0.0
    right = 0.0

    if keys_pressed.get("w") or keys_pressed.get("up"):
        left += SPEED
        right += SPEED

    if keys_pressed.get("s") or keys_pressed.get("down"):
        left -= SPEED
        right -= SPEED

    if keys_pressed.get("a") or keys_pressed.get("left"):
        left -= TURN                       # rotate left: reduce left, boost right
        right += TURN

    if keys_pressed.get("d") or keys_pressed.get("right"):
        left += TURN                       # rotate right: boost left, reduce right
        right -= TURN

    return left, right