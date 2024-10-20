import heapq
import itertools
import multiprocessing as mp
import time
from typing import List
from enum import Enum

from world.world import World
from shared.types import Position
from pathfinding.astar import AStar


MAX_ASTAR_F_COST = 100000

def _permutate(n: int) -> List[List[int]]:
    return [list(p) for p in itertools.permutations(range(n)) if p[0] == 0]


class AlgoType(Enum):
    EXHAUSTIVE_ASTAR = "Exhaustive Astar"

class SearchProcess(mp.Process):
    def __init__(
        self,
        pos: List["Position"],
        astar: AStar,
        todo: mp.Queue,
        done: mp.Queue,
        i:int,
        algo_type = AlgoType
    ):
        super().__init__()
        self.astar = astar
        self.pos = pos
        self.todo = todo
        self.done = done
        self.i = i
        self.algo_type = algo_type


    def _search(
        self,
        st: int,
        end: int
    ) -> float:
        match (self.algo_type):
            case AlgoType.EXHAUSTIVE_ASTAR:
                path = self.astar.search(self.pos[st], self.pos[end])
                return path[-1].f if path else MAX_ASTAR_F_COST
            case _:
                raise Exception("Invalid AlgoType")
    
    def run(self):
        while 1:
            try:
                st, end = self.todo.get()
                self.done.put((st, end, self._search(st, end)))
            except:
                return

class HamiltonianSearch:
    def __init__(
        self,
        world: "World", 
        src: "Position",
        algo_type: AlgoType,
        n: int = 8 # Number of child processes to run concurrently
    ):
        self.astar = AStar(world)
        self.src = src
        # node the robot is at currently + the nodes that must be visited (position to take photo)
        self.pos = [src] + [o.get_interaction_position() for o in world.obstacles]
        self.n = n
        self.algo_type = algo_type

    
    def search(self, top_n: int = 3):
        st = time.time()
        n = len(self.pos) # number of nodes (5 obstacles + 1 robot starting position)
        m = int(n*n - n - (n-1)) # total paths to calc from pt to pt (excluding from pt_a to pt_a and from pt_a to 0)
        perms = _permutate(n) # all permutations of visiting order of obstacles eg [[0, 1, 2, 3, 4, 5], [0, 1, 3, 2, 4, 5], ...]
        edges = [[0 for _ in range(n)] for _ in range(n)]
        todo = mp.Queue()
        done = mp.Queue()

        for r in range(n):
            for c in range(1, n):
                if r != c:
                    todo.put((r, c))
        for i in range(self.n): # here self.n refers to the number of child processes to run concurrently
            #self.pos = interaction position coordinates
            p = SearchProcess(self.pos, self.astar, todo, done, i, self.algo_type)
            p.daemon = True
            p.start()

        while m:
            r, c, f = done.get()
            edges[r][c] = f
            m -= 1

        # get shortest path, i.e., lowest cost among all permutations
        st2 = time.time()
        h = []
        for i, perm in enumerate(perms):
            cost = sum([edges[perm[i]][perm[i+1]] for i in range(n-1)])
            heapq.heappush(h, (cost, perm))

        loc_mn_path = []
        loc_mn_f = float('inf')
        min_perm = []
        for _ in range(min(top_n, len(h))):

            path = []
            prev = self.pos[0]
            cost, perm = heapq.heappop(h)
            f = 0

            for i in range(1, n):
                segment = self.astar.search(prev, self.pos[perm[i]])

                if segment:
                    path.append(segment)
                    prev = segment[-1].c_pos
                    f += segment[-1].f
                else:
                    f += MAX_ASTAR_F_COST

                if f > loc_mn_f:
                    break

            if f < loc_mn_f:
                loc_mn_f = f
                loc_mn_path = path
                min_perm = perm

            if f < MAX_ASTAR_F_COST:
                return perm, path
            
        return min_perm, loc_mn_path
    
