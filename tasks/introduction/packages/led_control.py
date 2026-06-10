import colorsys
from typing import List


def set_turning_leds(direction: str) -> dict:
    """Set LEDs to indicate turning direction."""
    yellow = [1.0, 1.0, 0.0]
    white = [1.0, 1.0, 1.0]
    red = [1.0, 0.0, 0.0]
    off = [0.0, 0.0, 0.0]

    # default state: all 4 LEDs off
    leds = {
        0: off,  # front-left
        2: off,  # front-right
        3: off,  # back-left
        4: off  # back-right
    }

    # Catch 'forward' or 'up'
    if direction == 'left':
        leds[0] = yellow.copy()
        leds[4] = yellow.copy()

    # Catch 'backward', 'down', or 'back'
    elif direction == 'stop':
        leds[4] = red.copy()
        leds[3] = red.copy()

    elif direction == 'right':
        leds[2] = yellow.copy()
        leds[3] = yellow.copy()

    elif direction == 'forward':
        leds[0] = white.copy()
        leds[2] =  white.copy()

    return leds
