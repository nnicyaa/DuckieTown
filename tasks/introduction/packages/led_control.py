from typing import Dict, List


def set_turning_leds(direction: str) -> Dict[int, List[float]]:
    """Return LED colors for turning signals."""
    direction = direction.lower().strip()

    OFF = [0.0, 0.0, 0.0]
    YELLOW = [1.0, 1.0, 0.0]
    WHITE = [1.0, 1.0, 1.0]
    RED = [1.0, 0.0, 0.0]

    leds = {
        0: OFF.copy(),
        2: OFF.copy(),
        3: OFF.copy(),
        4: OFF.copy(),
    }

    if direction == "left":
        leds[0] = YELLOW.copy()
        leds[4] = YELLOW.copy()
    elif direction == "right":
        leds[2] = YELLOW.copy()
        leds[3] = YELLOW.copy()
    elif direction == "forward":
        leds[0] = WHITE.copy()
        leds[2] = WHITE.copy()
    elif direction == "stop":
        leds[4] = RED.copy()
        leds[3] = RED.copy()

    return leds