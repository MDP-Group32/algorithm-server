from enum import Enum


class Bearing(Enum):
    NORTH = 1
    SOUTH = 2
    EAST = 3
    WEST = 4


class Action(Enum):
    FORWARD = 1
    BACKWARD = 2
    FORWARD_LEFT = 3
    FORWARD_RIGHT = 4
    BACKWARD_LEFT = 5
    BACKWARD_RIGHT = 6
