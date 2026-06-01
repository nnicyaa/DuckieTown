from typing import Dict, Tuple
import logging

logger = logging.getLogger(__name__)

def get_motor_speeds(keys_pressed: Dict[str, bool]) -> Tuple[float, float]:
    left = 0.0
    right = 0.0

    if keys_pressed.get("up"):
        left += 1.5
        right += 1.5

    if keys_pressed.get("down"):
        left -= 1.5
        right -= 1.5

    if keys_pressed.get("left"):
        left -= 0.7
        right += 0.7

    if keys_pressed.get("right"):
        left += 0.7
        right -= 0.7

    logger.info(f"keys={keys_pressed}, left={left}, right={right}")
    return left, right