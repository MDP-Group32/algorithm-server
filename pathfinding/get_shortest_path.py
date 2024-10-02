from math import pi
from pathfinding.hamiltonian import HamiltonianSearch
from shared.constants import GRID_COORD
from shared.enums import Direction
from shared.models import AlgorithmInput, AlgorithmInputMode, AlgorithmOutputLiveCommand
from shared.types import Position
from stm.commands import convert_segments_to_commands, getFinalStmCommand
from world.obstacle import Obstacle
from world.world import World

numeric_to_direction_map = {
    '1': Direction.NORTH,
    '2': Direction.SOUTH,
    '3': Direction.EAST,
    '4': Direction.WEST
}

def extract_obstacles_from_input(input_obstacles, algo_server_mode):
  """
  Helper function to convert input obstacles to `Obstacle` object accepted by the algorithm
  """
  obstacles = []

  grid_pos_to_c_pos_multiplier = GRID_COORD #5

  # if algo_server_mode == AlgorithmInputMode.LIVE:
    # Live mode uses 10cm grid format (so need to *2 to align with algo's 5cm grid format)
  grid_pos_to_c_pos_multiplier *= 2
  
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
  obstacles = extract_obstacles_from_input(algo_input["value"]["obstacles"], algo_server_mode)
  for obstacle in obstacles:
    print("Obstacles: ", obstacle.x, obstacle.y, obstacle.facing)
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
  
  elif algo_server_mode == AlgorithmInputMode.LIVE:
    print("Min Perm: ", min_perm)
    print("Paths: ", paths)
    current_perm = 1
    stm_commands = []
    obstacle_orders = []

    for path in paths:
      commands = convert_segments_to_commands(path)
      print("Commands: ", commands)
      stm_commands.extend(commands)

      # Add SNAP1 command after each path (from one obstacle to another) (For Raspberry Pi Team to know when to scan the image)
      stm_commands.append([f"ST00{min_perm[current_perm]}", commands[-1][1]])
      obstacle_orders.append(int(min_perm[current_perm]))
      current_perm += 1 # Increment by current_perm to access the next obstacle_id
      
    print("STM Commands: ", stm_commands)
    algoOutputLiveCommands: list[AlgorithmOutputLiveCommand] = [] # Array of commands
    for command in stm_commands:
      print('Command[0]', command[0])
      print('Command[0] edited', getFinalStmCommand(command[0]))
      algoOutputLiveCommands.append(AlgorithmOutputLiveCommand(
        cat="control",
        # value=command[0],
        value=getFinalStmCommand(command[0]),
        end_position=command[1]
      ))
    
    # Add FIN as the last command (For Raspberry Pi Team to know that the algorithm has ended)
    algoOutputLiveCommands.append(AlgorithmOutputLiveCommand(
      cat="control",
      value="#",
      end_position=algoOutputLiveCommands[-1].end_position
    ))

    return algoOutputLiveCommands, obstacle_orders