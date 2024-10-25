from math import cos, dist, sin
import numpy as np

from shared.position import Position

def polar_to_vector(
    degree: float,
    magnitude: float
) -> np.array:
    return np.array([
        magnitude * cos(degree),
        magnitude * sin(degree)
    ])

def distance_euclidean(
    x1: Position,
    x2: Position
) ->  float:
    return dist((x1.x, x1.y), (x2.x, x2.y))