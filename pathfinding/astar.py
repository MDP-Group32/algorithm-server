from collections import deque
import heapq
from math import pi
import numpy as np
from typing import List, Optional

from world.world import World
from shared.constants import (
    BACKWARD_LEFT_DISPLACEMENT, 
    BACKWARD_RIGHT_DISPLACEMENT, 
    BACKWARD_DISTANCE, 
    FORWARD_LEFT_DISPLACEMENT, 
    FORWARD_RIGHT_DISPLACEMENT, 
    FORWARD_DISTANCE,
    PENALIZE_STOP_MOTION_FACTOR,
    ANGLE_ALLOWANCE,
    X_ALLOWANCE,
    Y_ALLOWANCE,
)
from shared.enums import Action
from shared.position import Position
from shared.vector import polar_to_vector, distance_euclidean
from pathfinding.path_validation import has_collision
from stm.exploration import (
    move_backward,
    move_backward_left, 
    move_backward_right, 
    move_forward, 
    move_forward_left, 
    move_forward_right
)



class Node:

    def __init__(
        self,
        pos: Position,
        c_pos: Position,
        g: float,
        h: float,
        parent: Optional["Node"] = None,
        v: Optional[int] = 1,
        s: Optional[int] = 0,
        d: Optional[float] = 0
    ):
        self.pos = pos
        self.c_pos = c_pos # pos in continuous scale
        self.g = g # st to curr (cost to reach current node) => g(n)
        self.h = h # curr to end (cost from current node to end) => h(n)
        self.f = g + h # total cost of path through n to goal => f(n) = g(n) + h(n)

        self.v = v # prev motion's vert direction, -1: bwd, 1: fwd to get to current node
        self.s = s # prev motion's steering direction, -1: left, 0: straight, 1: right to get to current node
        self.d = d # dist of prev motion to get to current node

        self.parent = parent


    def deep_copy(self):
        return Node(self.pos, self.c_pos, self.g, self.h, self.parent, self.v, self.s, self.d)
    

    def __eq__(
        self,
        node: "Node"
    ) -> bool:
        return self.pos.x == node.pos.x and \
            self.pos.y == node.pos.y and \
            self.pos.theta == node.pos.theta


    def __lt__(
        self, 
        node: "Node"
    ) -> bool:
        return self.f < node.f
    
class AStar:

    def __init__(
        self, 
        world: "World"
    ):
        # self.moves: (v, s, d, Action, movement function)
        self.moves = (
            ( 1,  0, FORWARD_DISTANCE,    Action.FORWARD,       move_forward),
            ( 1, -1, FORWARD_LEFT_DISPLACEMENT[2], Action.FORWARD_LEFT,  move_forward_left),
            ( 1,  1, FORWARD_RIGHT_DISPLACEMENT[2], Action.FORWARD_RIGHT, move_forward_right),
            (-1,  0, BACKWARD_DISTANCE,    Action.BACKWARD,       move_backward),
            (-1, -1, BACKWARD_LEFT_DISPLACEMENT[2], Action.BACKWARD_LEFT,  move_backward_left),
            (-1,  1, BACKWARD_RIGHT_DISPLACEMENT[2], Action.BACKWARD_RIGHT, move_backward_right),
        )
        self.map = world
        self.end = None
        self.x_bounds = None
        self.y_bounds = None

    def is_goal_node(
        self,
        pos
    ):
        if self.x_bounds[0] <= pos.x <= self.x_bounds[1] and \
           self.y_bounds[0] <= pos.y <= self.y_bounds[1] and \
           abs(self.end.theta - pos.theta) % (2*pi) <= ANGLE_ALLOWANCE:
            return True
        return False

    def explore(
        self,
        node, 
    ):
        st = node.c_pos

        for v, s, d, mv, move_func in self.moves:
            nxt_pos = move_func(st)
            nxt_pos_tup = nxt_pos.get_grid_position().position_as_tuple()
            
            # self.closed represents nodes already explored
            if nxt_pos_tup in self.closed or has_collision(st, mv, self.map):
                continue

            penalty = 0
            if v != node.v or s != node.s:
                penalty = PENALIZE_STOP_MOTION_FACTOR # penalty for changing direction

            neighbor = Node(nxt_pos.get_grid_position(), nxt_pos, node.g + penalty + d, distance_euclidean(nxt_pos, self.end), node, v, s, d)            

            if neighbor.f < self.open_h.get(nxt_pos_tup, -1):
                for i, br in enumerate(self.open):
                    if br == neighbor:
                        br.f = -1
                        heapq._siftdown(self.open, 0, i)
                        heapq.heappop(self.open)
                        break

            heapq.heappush(self.open, neighbor)
            self.open_h[nxt_pos_tup] = neighbor.f

    def allowable_errors(self):
        vv = polar_to_vector(self.end.theta, 1)
        vh = polar_to_vector(self.end.theta - pi/2, 1)
        end = np.array([self.end.x, self.end.y])
        _TR = end + vh * X_ALLOWANCE[1] + vv * Y_ALLOWANCE[0]
        _BL = end - vh * X_ALLOWANCE[0] - vv * Y_ALLOWANCE[1]
        
        self.x_bounds = sorted([_TR[0], _BL[0]])
        self.y_bounds = sorted([_TR[1], _BL[1]])

        self.x_bounds = [max(0, self.x_bounds[0]), min(self.x_bounds[1], 195)]
        self.y_bounds = [max(0, self.y_bounds[0]), min(self.y_bounds[1], 195)]

    def search(
        self,
        st,
        end,
    ) -> List["Node"]:
        self.end = end
        self.open = [Node(st.get_grid_position(), st, 0, 0)]
        self.open_h = {} # keep track of unique cells that are in open set
        self.closed = [] # contains nodes already explored
        self.allowable_errors()

        while self.open:

            node = heapq.heappop(self.open)
            tup = node.pos.position_as_tuple()

            if self.is_goal_node(node.c_pos):
                return self.get_full_path(node)

            self.closed.append(tup) # add current node to explored nodes (already visited)
            self.explore(node) # generate successor nodes
        return [] # return empty array if goal is not reachable


    def get_full_path(
        self,
        last
    ):
        res = deque()

        while last:
            res.appendleft(last)
            last = last.parent
        return list(res)