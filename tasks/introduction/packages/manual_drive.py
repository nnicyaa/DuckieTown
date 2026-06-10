from typing import Dict, Tuple
import logging
logger = logging.getLogger(__name__)

SPEED = 1
TURN = 0.8


def get_motor_speeds(keys_pressed: Dict[str, bool]) -> Tuple[float, float]:
    left_speed = 0.0
    right_speed = 0.0

    # to handle forward and backward movement
    if keys_pressed.get('up', False):
        left_speed += SPEED
        right_speed += SPEED
    if keys_pressed.get('down', False):
        left_speed -= SPEED
        right_speed -= SPEED

    # to handle turning (robot turns toward the slower wheel)
    if keys_pressed.get('left', False):
        left_speed -= TURN
        right_speed += TURN
    if keys_pressed.get('right', False):
        left_speed += TURN
        right_speed -= TURN

    # speed is limited so we must clip it so combinations like up+right don't exceed 1.0
    left_speed = max(-1.0, min(1.0, left_speed))
    right_speed = max(-1.0, min(1.0, right_speed))
    return left_speed, right_speed
