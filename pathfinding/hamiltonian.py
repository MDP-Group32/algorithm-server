import heapq
import itertools
import logging
import multiprocessing as mp
import time
from typing import List
from enum import Enum

from world.world import World
from shared.types import Position
from shared.utils import get_euclidean_distance
from pathfinding.astar import AStar


MAX_ASTAR_F_COST = 99999

logger = logging.getLogger('HAMILTONIAN PATH')

def _permutate(n: int) -> List[List[int]]:
    """
    This function generates all permutations of numbers from 0 to n-1, 
    where 0 is always the first element, and returns them as a list of lists.
    """
    return [list(p) for p in itertools.permutations(range(n)) if p[0] == 0]


class AlgoType(Enum):
    """Enumeration for possible algorithms to be used for `HamiltonianSearch`"""
    EXHAUSTIVE_ASTAR = "Exhaustive Astar"
    EUCLIDEAN = "Euclidean"

class SearchProcess(mp.Process):
    """A Process (similar to a Thread) used for multiprocessing to speed up algorithm computation time"""
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
        logger.info(f'Spawning P{i}')


    def _search(
        self,
        st: int,
        end: int
    ) -> float:
        """Search According to the `AlgoType`
        @returns:
            Exhaustive Astar: Astar 'f(n)' cost
            Euclidean: Euclidean Distance
        """
        logger.info(f'P{self.i} start search {st, end}')

        match (self.algo_type):
            case AlgoType.EXHAUSTIVE_ASTAR:
                path = self.astar.search(self.pos[st], self.pos[end])
                return path[-1].f if path else MAX_ASTAR_F_COST
            case AlgoType.EUCLIDEAN:
                return get_euclidean_distance(self.pos[st], self.pos[end])
            case _:
                raise Exception("Invalid AlgoType")
    
    def run(self):
        while 1:
            try:
                st, end = self.todo.get()
                self.done.put((st, end, self._search(st, end)))
            except:
                logger.info(f'P{self.i} finished')
                return

class HamiltonianSearch:
    """
    Uses `Astar` (If AlgoType.EXHAUSIVE_ASTAR) to do an exhaustive search on all possible permutations of order of obstacles to visit 
    and finds the lowest cost permutation and its associated paths.

    Uses Multiprocessing (parameter `n` which defaults to 8) to lower computation time.

    Params:

        `world`: World object
        `src`: Position object of the source/starting position
        `n` = 8: Number of child processes to run concurrently

    Main Method: `search()`
        
        Returns:
            min_perm`: lowest cost order of visiting all the obstacles starting from starting location;
            `loc_mn_path`: An array of a path Array of `Node` where each inner path Array is the path from one location to another;
    """

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
        """
        Returns 
          `min_perm`: lowest cost order of visiting all the obstacles starting from starting location; 
          `loc_mn_path`: An array of a path Array of `Node` where each inner path Array is the path from one location to another
        """
        print("----- Start Hamiltonian Search -----")

        st = time.time()
        n = len(self.pos) # number of nodes (5 obstacles + 1 robot starting position)
        print(f'Number of nodes: {n}')
        m = int(n*n - n - (n-1)) # total paths to calc from pt to pt (excluding from pt_a to pt_a and from pt_a to 0)
        print(f'Total paths to calculate: {m}')
        perms = _permutate(n) # all permutations of visiting order of obstacles eg [[0, 1, 2, 3, 4, 5], [0, 1, 3, 2, 4, 5], ...]
        edges = [[0 for _ in range(n)] for _ in range(n)]
        todo = mp.Queue() # (r, c)
        done = mp.Queue() # (r, c, astar f cost)

        for r in range(n):
            for c in range(1, n):
                if r != c:
                    todo.put((r, c))

        # Create multiple Threads/Process and starts them
        for node in self.pos:
            print("Self.pos: ", str(node.x), str(node.y), str(node.theta))
        for i in range(self.n): # here self.n refers to the number of child processes to run concurrently
            #self.pos = interaction position coordinates
            p = SearchProcess(self.pos, self.astar, todo, done, i, self.algo_type)
            p.daemon = True
            p.start()

        while m:
            r, c, f = done.get()
            edges[r][c] = f
            logger.info(f'{r} -> {c} ({f})')
            m -= 1
 
        logger.info(f'Adj list completed in {time.time()-st} s')
        print(f'Adj list completed in {time.time()-st} s')

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
            logger.info(f'Calculating path for {perm}')

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
                print("f < MAX_ASTAR_F_COST")
                print(f'Time (pathfinding) {time.time()-st2} s')
                print(f'Total runtime {time.time()-st} s')
                return perm, path
        
        print(f'Time (pathfinding) {time.time()-st2} s')
        print(f'Total runtime {time.time()-st} s')
        return min_perm, loc_mn_path