from math import cos, sin
import numpy as np

from shared.types import Position

def calc_vector(
    theta: float,
    length: float
) -> np.array:
    """Calculate a vector of specified length, pointing in direction theta radians

    Args:
        theta (float) : Direction of vector
        length (float) : Magnitude of vector
    
    Returns:
        np.array : Column vector. Typically to calculate the vector going from robot's bottom left to top left corner.
    """
    return np.array([
        cos(theta) * length,
        sin(theta) * length
    ])

def get_euclidean_distance(
    start: Position,
    end: Position
) ->  float:
    """
    Calculate the euclidean distance between two points
    """
    return ((start.y - end.y)**2 + (start.x - end.x)**2)**0.5