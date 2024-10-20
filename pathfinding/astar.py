from collections import deque
import heapq
from math import pi
import numpy as np
from typing import List, Optional

from world.world import World
from shared.constants import (
    DIST_BL, 
    DIST_BR, 
    DIST_BW, 
    DIST_FL, 
    DIST_FR, 
    DIST_FW,
    PENALTY_STOP,
    MAX_THETA_ERR,
    MAX_X_ERR,
    MAX_Y_ERR,
)
from shared.enums import Movement
from shared.types import Position
from shared.utils import calc_vector, get_euclidean_distance
from pathfinding.path_validation import has_collision
from stm.move import (
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


    def clone(self) -> "Node":
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
        # custom comparator for heapq
        return self.f < node.f
    

    def __str__(self) -> str:
        return f'Node(x:{self.c_pos.x:6.2f}, y:{self.c_pos.y:6.2f}, θ:{self.c_pos.theta:6.2f}, g:{self.g:6.2f}, h:{self.h:6.2f}, f:{self.f:6.2f}), v:{self.v}, s:{self.s}'


class AStar:

    def __init__(
        self, 
        mp: "World"
    ):
        # self.moves: (v, s, d, Movement, movement function)
        self.moves = (
            ( 1,  0, DIST_FW,    Movement.FWD,       move_forward),
            ( 1, -1, DIST_FL[2], Movement.FWD_LEFT,  move_forward_left),
            ( 1,  1, DIST_FR[2], Movement.FWD_RIGHT, move_forward_right),
            (-1,  0, DIST_BW,    Movement.BWD,       move_backward),
            (-1, -1, DIST_BL[2], Movement.BWD_LEFT,  move_backward_left),
            (-1,  1, DIST_BR[2], Movement.BWD_RIGHT, move_backward_right),
        )
        self.map = mp
        self.end = None
        self.x_bounds = None
        self.y_bounds = None

    def _goal(
        self,
        pos: "Position"
    ) -> bool:
        if self.x_bounds[0] <= pos.x <= self.x_bounds[1] and \
           self.y_bounds[0] <= pos.y <= self.y_bounds[1] and \
           abs(self.end.theta - pos.theta) % (2*pi) <= MAX_THETA_ERR:
            return True
        return False

    def _expand(
        self,
        node: "Node", 
    ):
        st = node.c_pos

        for v, s, d, mv, move_func in self.moves:
            nxt_pos = move_func(st)
            nxt_pos_tup = nxt_pos.alignToGrid().to_tuple()
            
            # self.closed represents nodes already explored
            if nxt_pos_tup in self.closed or has_collision(st, mv, self.map):
                continue

            penalty = 0
            if v != node.v or s != node.s:
                penalty = PENALTY_STOP # penalty for changing direction

            nxt_node = Node(nxt_pos.alignToGrid(), nxt_pos, node.g + penalty + d, get_euclidean_distance(nxt_pos, self.end), node, v, s, d)            

            # there is a shorter way to reach a node that is already in open set
            if nxt_node.f < self.open_h.get(nxt_pos_tup, -1):
                for i, br in enumerate(self.open):
                    if br == nxt_node:
                        # replace br with cell
                        br.f = -1
                        heapq._siftdown(self.open, 0, i)
                        heapq.heappop(self.open)
                        break

            heapq.heappush(self.open, nxt_node)
            self.open_h[nxt_pos_tup] = nxt_node.f

    def _set_bounds(self):
        vv = calc_vector(self.end.theta, 1)
        vh = calc_vector(self.end.theta - pi/2, 1)
        end = np.array([self.end.x, self.end.y])
        _TR = end + vh * MAX_X_ERR[1] + vv * MAX_Y_ERR[0]
        _BL = end - vh * MAX_X_ERR[0] - vv * MAX_Y_ERR[1]
        
        self.x_bounds = sorted([_TR[0], _BL[0]])
        self.y_bounds = sorted([_TR[1], _BL[1]])

        self.x_bounds = [max(0, self.x_bounds[0]), min(self.x_bounds[1], 195)]
        self.y_bounds = [max(0, self.y_bounds[0]), min(self.y_bounds[1], 195)]

    def search(
        self,
        st: "Position",
        end: "Position",
    ) -> List["Node"]:
        end_node = Node(end, end, 0, 0)
        self.end = end
        self.open = [Node(st.alignToGrid(), st, 0, 0)]
        self.open_h = {} # keep track of unique cells that are in open set
        self.closed = [] # contains nodes already explored
        self._set_bounds()

        while self.open:

            node = heapq.heappop(self.open)
            tup = node.pos.to_tuple()

            if self._goal(node.c_pos):
                return self._reconstruct(node)

            self.closed.append(tup) # add current node to explored nodes (already visited)
            self._expand(node) # generate successor nodes
        return [] # return empty array if goal is not reachable


    def _reconstruct(
        self,
        last: "Node"
    ) -> List["Node"]:
        res = deque()

        while last:
            res.appendleft(last)
            last = last.parent
        return list(res)