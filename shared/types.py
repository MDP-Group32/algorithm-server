from math import pi
import numpy as np
from typing import Tuple

from shared.constants import (
    GRID_COORD, 
    GRID_THETA
)

class Position:

    def __init__(
        self,
        x: float,
        y: float,
        theta: float
    ):
        self.x = x
        self.y = y
        self.theta = theta


    def alignToGrid(
        self,
    ) -> "Position":
        '''
        The function adjusts the position's coordinates and angle to the nearest specified grid resolution.

        Coordinates are adjusted to the nearest multiple of GRID_COORD (5 units in this case).
        Angle is adjusted to the nearest multiple of GRID_THETA (15 degrees in this case).
        '''
        x = int(round(self.x / GRID_COORD) * GRID_COORD)
        y = int(round(self.y / GRID_COORD) * GRID_COORD)
        theta = round(self.theta % (2*pi) / pi * 180 / GRID_THETA) * GRID_THETA / 180 * pi

        return Position(x, y, theta)
    

    def clone(self) -> "Position":
       return Position(self.x, self.y, self.theta)
       

    def add(
        self, 
        vec: np.array
    ):
        """
        Adds the given vector to the position's coordinates.
        """
        self.x += vec[0]
        self.y += vec[1]


    def to_tuple(self) -> Tuple[float, float, float]:
        return self.x, self.y, self.theta


    def __str__(self) -> str:
        return f'(x:{self.x:6.2f}, y:{self.y:6.2f}, θ:{self.theta:6.2f})'