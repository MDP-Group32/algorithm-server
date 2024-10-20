from math import pi
from shared.constants import (
    DIST_BL,
    DIST_BR,
    DIST_BW,
    DIST_FL,
    DIST_FR,
    DIST_FW
)
from shared.utils import calc_vector
from shared.types import Position


def move_forward(pos: "Position") -> "Position":
    new = pos.clone()
    new.add(calc_vector(pos.theta, DIST_FW))
    return new

def move_backward(pos: "Position") -> "Position":
    new = pos.clone()
    new.add(calc_vector(pos.theta, -DIST_BW))
    return new


def move_forward_left(pos: "Position") -> "Position":
    horizontal_vector = calc_vector(pos.theta-pi/2, DIST_FL[0])
    vertical_vector = calc_vector(pos.theta, DIST_FL[1])
    new = pos.clone()
    new.add(vertical_vector + horizontal_vector)
    new.theta = pos.theta + pi/2 # face left rotate 90 degrees
    return new


def move_forward_right(pos: "Position") -> "Position":
    horizontal_vector = calc_vector(pos.theta-pi/2, DIST_FR[0])
    vertical_vector = calc_vector(pos.theta, DIST_FR[1])
    new = pos.clone()
    new.add(vertical_vector + horizontal_vector)
    new.theta = pos.theta - pi/2
    return new


def move_backward_left(pos: "Position") -> "Position":
    horizontal_vector = calc_vector(pos.theta-pi/2, DIST_BL[0])
    vertical_vector = calc_vector(pos.theta, DIST_BL[1])
    new = pos.clone()
    new.add(vertical_vector + horizontal_vector)
    new.theta = pos.theta - pi/2 # will end up facing right "reverse"
    return new


def move_backward_right(pos: "Position") -> "Position":
    horizontal_vector = calc_vector(pos.theta-pi/2, DIST_BR[0])
    vertical_vector = calc_vector(pos.theta, DIST_BR[1])
    new = pos.clone()
    new.add(vertical_vector + horizontal_vector)
    new.theta = pos.theta + pi/2 # will end up facing left "reverse"
    return new
