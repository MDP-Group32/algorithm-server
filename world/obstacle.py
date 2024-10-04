from math import pi

from shared.constants import (
    OBSTACLE_WIDTH,
    ROBOT_MIN_CAMERA_DIST,
    ROBOT_HEIGHT,
    ROBOT_WIDTH
)
from shared.enums import Direction
from shared.types import Position


class Obstacle:
    #NOTE: For obstacle, the (x,y) position is always bottom left (agnostic of the direction it is facing)
    def __init__(
        self,
        x: float,
        y: float,
        facing: Direction
    ):
        self.x = x
        self.y = y
        self.facing = facing
        self.middle = (x+(OBSTACLE_WIDTH/2), y+(OBSTACLE_WIDTH/2))


    def get_interaction_position(self) -> Position:
        '''
        Returns the (x,y) position of the robot to interact with the obstacle. (E.g. to take a photo)
        Refer to IPAD Notability for Diagram explaining this calculation
        '''
        x = self.x #original obstacle x
        y = self.y #original obstacle y
        theta = None

        offset = (ROBOT_WIDTH - OBSTACLE_WIDTH) / 2

        adjustments = {
            Direction.NORTH: (offset + OBSTACLE_WIDTH, ROBOT_MIN_CAMERA_DIST + OBSTACLE_WIDTH + ROBOT_HEIGHT, -pi / 2),
            Direction.SOUTH: (-offset, -(ROBOT_MIN_CAMERA_DIST + ROBOT_HEIGHT), pi / 2),
            Direction.EAST: (ROBOT_MIN_CAMERA_DIST + OBSTACLE_WIDTH + ROBOT_WIDTH, -offset, pi),
            Direction.WEST: (-(ROBOT_MIN_CAMERA_DIST + ROBOT_WIDTH), offset + OBSTACLE_WIDTH, 0)
        }
        print('self.x', str(self.x), 'self.y', str(self.y), 'self.facing', str(self.facing))
        dx, dy, theta = adjustments[self.facing]
        interaction_position = Position(x + dx, y + dy, theta)
        print('interaction_position:', interaction_position)
        return interaction_position
