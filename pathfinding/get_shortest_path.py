from math import pi
from pathfinding.hamiltonian import HamiltonianSearch
from shared.constants import GRID_COORD
from shared.enums import Direction
from shared.models import AlgorithmInput, AlgorithmInputMode
from shared.types import Position
from world.obstacle import Obstacle
from world.world import World

numeric_to_direction_map = {
    '1': Direction.NORTH,
    '2': Direction.SOUTH,
    '3': Direction.EAST,
    '4': Direction.WEST
}

def extract_obstacles_from_input(input_obstacles):
  """
  Helper function to convert input obstacles to `Obstacle` object accepted by the algorithm
  """
  obstacles = []

  grid_pos_to_c_pos_multiplier = GRID_COORD #5

  for obstacle in input_obstacles:
    obstacles.append(Obstacle(
      x=obstacle["x"] * grid_pos_to_c_pos_multiplier,
      y=obstacle["y"] * grid_pos_to_c_pos_multiplier,
      facing=numeric_to_direction_map[str(obstacle["d"])]
    ))

  return obstacles

def get_shortest_path(algo_input: AlgorithmInput):
  algo_server_mode = algo_input["server_mode"]

  # Obstacles
  obstacles = extract_obstacles_from_input(algo_input["value"]["obstacles"])

  # Start Position
  start_position = Position(x=0, y=0, theta=pi/2)

  # World
  world = World(obstacles=obstacles)

  # Algorithm
  algo_type = algo_input["algo_type"]
  print("Algorithm: ", algo_type)
  algo = HamiltonianSearch(world=world, src=start_position, algo_type=algo_type)

  # Algorithm Search⭐
  min_perm, paths = algo.search()

  # Results
  if algo_server_mode == AlgorithmInputMode.SIMULATOR:
    simulator_algo_output = []
    for path in paths:
        simulator_algo_output.extend(node.pos for node in path)
        
        # Position configuration to represent scanning for simulator
        # each path from a node to another node is to take photo of the obstacle
        simulator_algo_output.extend([Position(-1, -1, -2), Position(-1, -1, -1)])

    return simulator_algo_output