from math import pi
import numpy as np

from world.world import World
from shared.constants import (
    BACKWARD_LEFT_RADIUS,
    BACKWARD_RIGHT_RADIUS,
    FORWARD_LEFT_RADIUS,
    FORWARD_RIGHT_RADIUS,
    FORWARD_DISTANCE
)
from shared.enums import Action
from shared.position import Position
from shared.vector import polar_to_vector

def has_collision(
    start,
    movement,
    world
):
    obstacles = world.images(start, movement)
    
    # Define movement-specific radius configurations
    radius_configs = {
        Action.FORWARD_LEFT: FORWARD_LEFT_RADIUS,
        Action.BACKWARD_LEFT: BACKWARD_LEFT_RADIUS,
        Action.FORWARD_RIGHT: FORWARD_RIGHT_RADIUS,
        Action.BACKWARD_RIGHT: BACKWARD_RIGHT_RADIUS
    }
    
    # Handle straight movements separately
    if movement in [Action.FORWARD, Action.BACKWARD]:
        vector = polar_to_vector(start.theta, FORWARD_DISTANCE)
        if movement == Action.BACKWARD:
            vector *= -1
        new_position = start.deep_copy().vector_sum(vector)
        return not world.is_valid_path(new_position, obstacles)
    
    # Handle diagonal movements
    radius_config = radius_configs[movement]
    start_vector = np.array([start.x, start.y])
    v_u = polar_to_vector(start.theta - pi/2, 1)
    v_r = polar_to_vector(start.theta, 1)
    
    for wp in radius_config:
        position = Position(
            *(start_vector + wp[0]*v_u + wp[1]*v_r), 
            (start.theta + wp[2]) % (2*pi)
        )
        if not world.is_valid_path(position, obstacles):
            return True
    
    return False