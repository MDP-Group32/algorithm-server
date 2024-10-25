from math import pi
import numpy as np
from typing import List

from world.obstacle import Obstacle
from shared.constants import (
    MAP_HEIGHT,
    MAP_WIDTH,
    OBSTACLE_SIZE,
    ROBOT_THEORETICAL_HEIGHT,
    ROBOT_THEORETICAL_WIDTH,
    EDGE_ERR,
    MOVE_1_BOUND,
    MOVE_2_BOUND,
    MOVE_3_BOUND,
    MOVE_4_BOUND,
    MOVE_5_BOUND,
    MOVE_6_BOUND,
    MOVE_7_BOUND,
    MOVE_8_BOUND,
    FORWARD_DISTANCE,
    BACKWARD_DISTANCE,
    ALLOWANCE
)
from shared.enums import Action
from shared.position import Position
from shared.enums import Action
from shared.vector import polar_to_vector


class World:
    bounds = {
        Action.FORWARD: [
            ALLOWANCE, 
            ALLOWANCE + ROBOT_THEORETICAL_WIDTH,
            ALLOWANCE + FORWARD_DISTANCE + ROBOT_THEORETICAL_HEIGHT,
            ALLOWANCE
        ],
        Action.BACKWARD: [
            ALLOWANCE,
            ALLOWANCE + ROBOT_THEORETICAL_WIDTH,
            ALLOWANCE + ROBOT_THEORETICAL_HEIGHT,
            ALLOWANCE + BACKWARD_DISTANCE
        ],
        Action.FORWARD_LEFT: MOVE_1_BOUND + MOVE_2_BOUND,
        Action.FORWARD_RIGHT: MOVE_3_BOUND + MOVE_4_BOUND,
        Action.BACKWARD_LEFT: MOVE_5_BOUND + MOVE_6_BOUND,
        Action.BACKWARD_RIGHT: MOVE_7_BOUND + MOVE_8_BOUND
    }


    def __init__(self, obstacles: List["Obstacle"]):
        self.obstacles = obstacles


    def inside_world(self, x: float, y: float) -> bool:
        return (
            -EDGE_ERR <= x <= MAP_WIDTH + EDGE_ERR
            and -EDGE_ERR <= y <= MAP_HEIGHT + EDGE_ERR
        )
    
    def is_valid_path(self, pos: Position, obstacles: List["Obstacle"]) -> bool:
        # Robot
        r_origin = np.array([pos.x, pos.y]) # Bottom left of robot coordinates
        r_vec_up = polar_to_vector(pos.theta, ROBOT_THEORETICAL_HEIGHT) # Top left of robot coordinates
        r_vec_right = polar_to_vector( # Bottom right of robot coordinates
            pos.theta - pi / 2, ROBOT_THEORETICAL_WIDTH
        )

        # Check if Robot is within the bound of the map
        if not (
            self.inside_world(*r_origin) # Bottom left
            and self.inside_world(*(r_origin + r_vec_up)) # Top left
            and self.inside_world(*(r_origin + r_vec_right)) # Bottom right
            and self.inside_world(*(r_origin + r_vec_right + r_vec_up)) # Top right
        ):
            return False

        # Robot Corners
        r_corners = [
            r_origin,  # btm left
            r_origin + r_vec_right,  # btm right
            r_origin + r_vec_up,  # top left
            r_origin + r_vec_up + r_vec_right,  # top right
        ]

        # For every obstacle, check if any of the 4 obstacle corners lies within the robot
        for obs in obstacles:
            # Obstacle x and y bounds
            o_btm = obs.y + EDGE_ERR
            o_left = obs.x + EDGE_ERR
            o_top = obs.y + OBSTACLE_SIZE - EDGE_ERR
            o_right = obs.x + OBSTACLE_SIZE - EDGE_ERR

            # Return False if Robot 4 corners' (x, y) is inside the obstacle (x, y) boundary
            for cx, cy in r_corners:
                if o_left <= cx <= o_right and o_btm <= cy <= o_top:
                    return False        
        return True
    
    def images(
        self,
        position,
        movements
    ):
        v_t = polar_to_vector(position.theta, 1)
        v_r = polar_to_vector(position.theta - pi/2, 1)

        bounds = self.bounds[movements]

        top_left = np.array([position.x, position.y]) + v_t * bounds[2] - v_r * bounds[0]
        bottom_right = np.array([position.x, position.y]) - v_t * bounds[3] + v_r * bounds[1]

        x_min, x_max = sorted([bottom_right[0], top_left[0]])
        y_min, y_max = sorted([bottom_right[1], top_left[1]])

        return [
            obstacle 
            for obstacle in self.obstacles 
            if x_min < obstacle.middle[0] < x_max and y_min < obstacle.middle[1] < y_max
    ]