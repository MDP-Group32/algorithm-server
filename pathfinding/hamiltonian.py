import heapq
import itertools
import multiprocessing as mp
import time
from typing import List, Tuple
from enum import Enum

from world.world import World
from shared.position import Position
from pathfinding.astar import AStar


MAX_ASTAR_F_COST = 100000

def _permutate(n: int) -> List[List[int]]:
    return [list(p) for p in itertools.permutations(range(n)) if p[0] == 0]

class SearchProcess(mp.Process):
    """AStar search process."""

    def __init__(
        self,
        positions,
        astar,
        task_queue,
        result_queue,
        process_id
    ):
        super().__init__()
        self.positions = positions
        self.astar = astar
        self.task_queue = task_queue
        self.result_queue = result_queue
        self.process_id = process_id

    def _astar_search(self, start_index: int, end_index: int) -> float:
        path = self.astar.search(self.positions[start_index], self.positions[end_index])
        return path[-1].f if path else MAX_ASTAR_F_COST

    def run(self):
        while True:
            try:
                start_index, end_index = self.task_queue.get()
                result = (start_index, end_index, self._astar_search(start_index, end_index))
                self.result_queue.put(result)
            except Exception as e:
                return

class HamiltonianSearch:
    def __init__(
        self,
        world: "World", 
        src: "Position",
        n: int = 8 # Number of child processes to run concurrently
    ):
        self.astar = AStar(world)
        self.src = src
        # node the robot is at currently + the nodes that must be visited (position to take photo)
        self.pos = [src] + [o.position_to_capture_image() for o in world.obstacles]
        self.n = n

    
    def search(self, top_n: int = 3) -> Tuple[List[int], List[List["Position"]]]:
    
        # Constants
        num_nodes = len(self.pos) # number of nodes (5 obstacles + 1 robot starting position)
        num_paths = num_nodes * (num_nodes - 1) - (num_nodes - 1) # total paths to calc from pt to pt (excluding from pt_a to pt_a and from pt_a to 0)
        num_processes = self.n
        
        # Initialize data structures
        permutations = _permutate(num_nodes) # all permutations of visiting order of obstacles eg [[0, 1, 2, 3, 4, 5], [0, 1, 3, 2, 4, 5], ...]
        adjacency_matrix = [[0] * num_nodes for _ in range(num_nodes)]
        task_queue = mp.Queue()
        result_queue = mp.Queue()
        
        # Create tasks for parallel processing
        for i in range(num_nodes):
            for j in range(1, num_nodes):
                if i != j:
                    task_queue.put((i, j))
        
        # Start parallel processes
        processes = []
        for i in range(num_processes):
            p = SearchProcess(self.pos, self.astar, task_queue, result_queue, i)
            p.daemon = True
            p.start()
            processes.append(p)
        
        # Collect results from parallel processing
        while num_paths:
            r, c, f = result_queue.get()
            adjacency_matrix[r][c] = f
            num_paths -= 1
        
        # Find shortest path among all permutations
        start_time = time.time()
        heap = []
        for perm in permutations:
            cost = sum(adjacency_matrix[perm[i]][perm[i+1]] for i in range(num_nodes-1))
            heapq.heappush(heap, (cost, perm))
        
        min_cost = float('inf')
        min_path = []
        min_perm = []
        for _ in range(min(top_n, len(heap))):
            cost, perm = heapq.heappop(heap)
            path = self._reconstruct_path(perm)
            if path and path[-1].f < min_cost:
                min_cost = path[-1].f
                min_path = path
                min_perm = perm
            if path and path[-1].f < MAX_ASTAR_F_COST:
                return perm, path
        
        return min_perm, min_path


    def _reconstruct_path(self, perm: List[int]) -> List[List["Position"]]:
        path = []
        prev = self.pos[0]
        for i in range(1, len(perm)):
            segment = self.astar.search(prev, self.pos[perm[i]])
            if segment:
                path.append(segment)
                prev = segment[-1].c_pos
            else:
                return []
        return path
    
