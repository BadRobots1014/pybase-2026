"""
A collection of methods for converting between native Falcon 500 units and standard units, like metres.
"""

import math

FALCON_CPR = 2048
DEGREES_PER_ROTATION = 360
RADS_PER_ROTATION = 2 * math.pi


def metres_to_rotations(distance: float, circumference: float) -> float:
    return distance / circumference


def rotations_to_metres(rotations: float, circumference: float) -> float:
    return rotations * circumference


def inches_to_meters(inches: float) -> float:
    return inches * 0.0254
