from math import cos, sin
import numpy as np

from shared.types import Position

def calc_vector(
    theta: float,
    length: float
) -> np.array:
    return np.array([
        cos(theta) * length,
        sin(theta) * length
    ])

def get_euclidean_distance(
    start: Position,
    end: Position
) ->  float:
    return ((start.y - end.y)**2 + (start.x - end.x)**2)**0.5