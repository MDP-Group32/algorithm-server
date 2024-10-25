from math import pi
from shared.constants import (
    BACKWARD_LEFT_DISPLACEMENT,
    BACKWARD_RIGHT_DISPLACEMENT,
    BACKWARD_DISTANCE,
    FORWARD_LEFT_DISPLACEMENT,
    FORWARD_RIGHT_DISPLACEMENT,
    FORWARD_DISTANCE
)
from shared.vector import polar_to_vector
from math import pi

DISTANCES = {
    "forward": FORWARD_DISTANCE,
    "backward": -BACKWARD_DISTANCE,
    "forward_left": FORWARD_LEFT_DISPLACEMENT,
    "forward_right": FORWARD_RIGHT_DISPLACEMENT,
    "backward_left": BACKWARD_LEFT_DISPLACEMENT,
    "backward_right": BACKWARD_RIGHT_DISPLACEMENT
}

ROTATIONS = {
    "left": pi / 2,
    "right": -pi / 2,
}

def move(pos, direction, is_forward=True):
    distance = DISTANCES[direction]
    if isinstance(distance, tuple):
        horizontal_vector = polar_to_vector(pos.theta - pi / 2, distance[0])
        vertical_vector = polar_to_vector(pos.theta, distance[1])
        move_vector = horizontal_vector + vertical_vector
    else:
        move_vector = polar_to_vector(pos.theta, distance)
    
    new_pos = pos.deep_copy()
    new_pos.vector_sum(move_vector)
    
    if "left" in direction:
        new_pos.theta += ROTATIONS["left"] if is_forward else ROTATIONS["right"]
    elif "right" in direction:
        new_pos.theta += ROTATIONS["right"] if is_forward else ROTATIONS["left"]
    
    return new_pos

def move_forward(pos):
    return move(pos, "forward")

def move_backward(pos):
    return move(pos, "backward", is_forward=False)

def move_forward_left(pos):
    return move(pos, "forward_left")

def move_forward_right(pos):
    return move(pos, "forward_right")

def move_backward_left(pos):
    return move(pos, "backward_left", is_forward=False)

def move_backward_right(pos):
    return move(pos, "backward_right", is_forward=False)
