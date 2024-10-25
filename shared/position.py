from math import pi
import numpy as np

from shared.constants import (
    GRID_XY, 
    GRID_ANGLE
)

class Position:

    def __init__(
        self,
        x: float,
        y: float,
        theta: float
    ):
        self.theta = theta
        self.y = y
        self.x = x

    def deep_copy(self):
       return Position(self.x, self.y, self.theta)
       
    def position_as_tuple(self):
        return self.x, self.y, self.theta

    def vector_sum(
        self, 
        vec: np.array
    ):
        self.x += vec[0]
        self.y += vec[1]

    def get_grid_position(
        self,
    ):
        theta = round(self.theta % (2*pi) / pi * 180 / GRID_ANGLE) * GRID_ANGLE / 180 * pi
        y = int(round(self.y / GRID_XY) * GRID_XY)
        x = int(round(self.x / GRID_XY) * GRID_XY)

        return Position(x, y, theta)