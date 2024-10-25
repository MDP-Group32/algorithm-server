from math import pi

from shared.constants import (
    OBSTACLE_SIZE,
    CAMERA_OFFSET,
    ROBOT_THEORETICAL_HEIGHT,
    ROBOT_THEORETICAL_WIDTH
)
from shared.enums import Bearing
from shared.position import Position


class Obstacle:
    #NOTE: For obstacle, the (x,y) position is always bottom left (agnostic of the direction it is facing)
    def __init__(
        self,
        x: float,
        y: float,
        facing: Bearing
    ):
        self.x = x
        self.y = y
        self.facing = facing
        self.middle = (x+(OBSTACLE_SIZE/2), y+(OBSTACLE_SIZE/2))


    def position_to_capture_image(self) -> Position:
        x = self.x #original obstacle x
        y = self.y #original obstacle y
        theta = None

        offset = (ROBOT_THEORETICAL_WIDTH - OBSTACLE_SIZE) / 2

        adjustments = {
            Bearing.NORTH: (offset + OBSTACLE_SIZE, CAMERA_OFFSET + OBSTACLE_SIZE + ROBOT_THEORETICAL_HEIGHT, -pi / 2),
            Bearing.SOUTH: (-offset, -(CAMERA_OFFSET + ROBOT_THEORETICAL_HEIGHT), pi / 2),
            Bearing.EAST: (CAMERA_OFFSET + OBSTACLE_SIZE + ROBOT_THEORETICAL_WIDTH, -offset, pi),
            Bearing.WEST: (-(CAMERA_OFFSET + ROBOT_THEORETICAL_WIDTH), offset + OBSTACLE_SIZE, 0)
        }
        dx, dy, theta = adjustments[self.facing]
        interaction_position = Position(x + dx, y + dy, theta)
        return interaction_position
