from math import pi
import numpy as np

from world.world import World
from shared.constants import (
    WPS_BL,
    WPS_BR,
    WPS_FL,
    WPS_FR,
    DIST_FW
)
from shared.enums import Movement
from shared.types import Position
from shared.utils import calc_vector

def has_collision(
    start: Position,
    movement: Movement,
    world: World
) -> bool:

    obs = world.priority_obs(start, movement)

    if movement == Movement.FWD_LEFT:
        wps = WPS_FL
    elif movement == Movement.BWD_LEFT:
        wps = WPS_BL
    elif movement == Movement.FWD_RIGHT:
        wps = WPS_FR
    elif movement == Movement.BWD_RIGHT:
        wps = WPS_BR
    else:
        # fwd or bwd
        v = calc_vector(start.theta, DIST_FW)
        new = start.clone()
        if movement == Movement.BWD:
            v *= -1
        new.add(v)
        return not world.is_valid_path(new, obs)
    start_vector = np.array([start.x, start.y])
    v_u = calc_vector(start.theta - pi/2, 1)
    v_r = calc_vector(start.theta, 1)
    
    for wp in wps:
        pos = Position(*(start_vector + wp[0]*v_u + wp[1]*v_r), (start.theta + wp[2]) % (2*pi))
        if not world.is_valid_path(pos, obs):
            return True
    return False