from math import pi
import numpy as np
from typing import List

from world.obstacle import Obstacle
from shared.constants import (
    MAP_HEIGHT,
    MAP_WIDTH,
    OBSTACLE_WIDTH,
    ROBOT_HEIGHT,
    ROBOT_WIDTH,
    EDGE_ERR,
    FL_X_BOUND,
    FL_Y_BOUND,
    FR_X_BOUND,
    FR_Y_BOUND,
    BL_X_BOUND,
    BL_Y_BOUND,
    BR_X_BOUND,
    BR_Y_BOUND,
    DIST_FW,
    DIST_BW,
    BUFFER
)
from shared.enums import Movement
from shared.types import Position
from shared.enums import Movement
from shared.utils import calc_vector


class World:
    bounds = {
        Movement.FWD: [
            BUFFER, 
            BUFFER + ROBOT_WIDTH,
            BUFFER + DIST_FW + ROBOT_HEIGHT,
            BUFFER
        ],
        Movement.BWD: [
            BUFFER,
            BUFFER + ROBOT_WIDTH,
            BUFFER + ROBOT_HEIGHT,
            BUFFER + DIST_BW
        ],
        Movement.FWD_LEFT: FL_X_BOUND + FL_Y_BOUND,
        Movement.FWD_RIGHT: FR_X_BOUND + FR_Y_BOUND,
        Movement.BWD_LEFT: BL_X_BOUND + BL_Y_BOUND,
        Movement.BWD_RIGHT: BR_X_BOUND + BR_Y_BOUND
    }


    def __init__(self, obstacles: List["Obstacle"]):
        self.obstacles = obstacles


    def is_within_map_boundary(self, x: float, y: float) -> bool:
        return (
            -EDGE_ERR <= x <= MAP_WIDTH + EDGE_ERR
            and -EDGE_ERR <= y <= MAP_HEIGHT + EDGE_ERR
        )
    
    def is_valid_path(self, pos: Position, obstacles: List["Obstacle"]) -> bool:
        # Robot
        r_origin = np.array([pos.x, pos.y]) # Bottom left of robot coordinates
        r_vec_up = calc_vector(pos.theta, ROBOT_HEIGHT) # Top left of robot coordinates
        r_vec_right = calc_vector( # Bottom right of robot coordinates
            pos.theta - pi / 2, ROBOT_WIDTH
        )

        # Check if Robot is within the bound of the map
        if not (
            self.is_within_map_boundary(*r_origin) # Bottom left
            and self.is_within_map_boundary(*(r_origin + r_vec_up)) # Top left
            and self.is_within_map_boundary(*(r_origin + r_vec_right)) # Bottom right
            and self.is_within_map_boundary(*(r_origin + r_vec_right + r_vec_up)) # Top right
        ):
            return False

        # Robot Segments / Edges
        r_segments = [
            (r_origin, r_origin + r_vec_up),  # left edge (bottom left, top left)
            (r_origin, r_origin + r_vec_right),  # btm edge (bottom left, bottom right)
            (r_origin + r_vec_up, r_origin + r_vec_up + r_vec_right),  # top edge (top left, top right)
            (r_origin + r_vec_right, r_origin + r_vec_right + r_vec_up),  # right edge (bottom right, top right)
        ]

        # Robot Corners
        r_corners = [
            r_origin,  # btm left
            r_origin + r_vec_right,  # btm right
            r_origin + r_vec_up,  # top left
            r_origin + r_vec_up + r_vec_right,  # top right
        ]

        # For every obstacle, check if any of the 4 obstacle corners lies within the robot
        EXTRA_VIRTUAL_BOUNDARY = 0 # To increase the virtual boundary of the obstacle (in cm)
        # EXTRA_VIRTUAL_BOUNDARY = 5 # To increase the virtual boundary of the obstacle (in cm)
        for obs in obstacles:
            # Obstacle x and y bounds
            o_btm = obs.y + EDGE_ERR - EXTRA_VIRTUAL_BOUNDARY
            o_left = obs.x + EDGE_ERR - EXTRA_VIRTUAL_BOUNDARY
            o_top = obs.y + OBSTACLE_WIDTH - EDGE_ERR + EXTRA_VIRTUAL_BOUNDARY
            o_right = obs.x + OBSTACLE_WIDTH - EDGE_ERR + EXTRA_VIRTUAL_BOUNDARY

            # Return False if Robot 4 corners' (x, y) is inside the obstacle (x, y) boundary
            for cx, cy in r_corners:
                if o_left <= cx <= o_right and o_btm <= cy <= o_top:
                    return False
            for o_x, o_y in (
                (o_left, o_btm),  # obstacle's btm left corner
                (o_left, o_top),  # obstacle's top left corner
                (o_right, o_top),  # obstacle's top right corner
                (o_right, o_btm),  # obstacle's btm right corner
            ):
                crosses = 0
                for (st_x, st_y), (end_x, end_y) in r_segments:
                    if (st_y > o_y and end_y > o_y) or (st_y < o_y and end_y < o_y):
                        continue
                    
                    if (end_x - st_x) == 0:
                        intersect_x = st_x
                    else:
                        m = (end_y - st_y) / (end_x - st_x)
                        if m == 0:
                            intersect_x = min(st_x, end_x)
                        else:
                            intersect_x = st_x + (o_y - st_y) / m
                    crosses += o_x <= intersect_x
                if crosses == 1:
                    return False
        
        return True
    
    def priority_obs(
        self,
        pos: "Position",
        move: "Movement"
    ) -> List["Obstacle"]:
        v_t = calc_vector(pos.theta, 1)
        v_r = calc_vector(pos.theta - pi/2, 1)

        bounds = self.bounds[move]
        st = np.array([pos.x, pos.y])
        tl = st + v_t*bounds[2] - v_r*bounds[0]
        br = st - v_t*bounds[3] + v_r*bounds[1]

        x_bounds = sorted([br[0], tl[0]])
        y_bounds = sorted([br[1], tl[1]])

        return list(filter(lambda o:x_bounds[0]<o.middle[0]<x_bounds[1] and y_bounds[0]<o.middle[1]<y_bounds[1], self.obstacles))