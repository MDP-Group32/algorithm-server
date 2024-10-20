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
  obstacles = []

  grid_pos_to_c_pos_multiplier = GRID_COORD * 2  

  for obstacle in input_obstacles:
    if obstacle["x"] == 0 and 1 <= obstacle["y"] <= 18 and (obstacle["d"] == 2 or obstacle["d"] == 1):
      obstacle["x"] += 1
    elif obstacle["x"] == 19 and 1 <= obstacle["y"] <= 18 and (obstacle["d"] == 2 or obstacle["d"] == 1):
      obstacle["x"] -= 1
    elif obstacle["y"] == 0 and 1 <= obstacle["x"] <= 18 and (obstacle["d"] == 3 or obstacle["d"] == 4):
      obstacle["y"] += 1
    elif obstacle["y"] == 19 and 1 <= obstacle["x"] <= 18 and (obstacle["d"] == 3 or obstacle["d"] == 4):
      obstacle["y"] -= 1
    elif obstacle["x"] == 0 and obstacle["y"] == 19 and obstacle["d"] == 2:
      obstacle["x"] += 1
    elif obstacle["x"] == 0 and obstacle["y"] == 19 and obstacle["d"] == 3:
      obstacle["y"] -= 1
    elif obstacle["x"] == 19 and obstacle["y"] == 19 and obstacle["d"] == 2:
      obstacle["x"] -= 1
    elif obstacle["x"] == 19 and obstacle["y"] == 19 and obstacle["d"] == 4:
      obstacle["y"] -= 1
    elif obstacle["x"] == 19 and obstacle["y"] == 0 and obstacle["d"] == 4:
      obstacle["y"] += 1
    elif obstacle["x"] == 19 and obstacle["y"] == 0 and obstacle["d"] == 1:
      obstacle["x"] -= 1
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
  start_position = Position(x=0, y=0, theta=pi/2)

  world = World(obstacles=obstacles)

  algo_type = algo_input["algo_type"]
  algo = HamiltonianSearch(world=world, src=start_position, algo_type=algo_type)

  min_perm, paths = algo.search()

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
    obstacle_order_str = ''

    for path in paths:
      commands = convert_segments_to_commands(path)
      print("Commands: ", commands)
      stm_commands.extend(commands)

      # Add ST001 command after each path (from one obstacle to another) (For Raspberry Pi Team to know when to scan the image)
      stm_commands.append([f"ST00{min_perm[current_perm]}", commands[-1][1]])
      obstacle_orders.append(int(min_perm[current_perm]))
      current_perm += 1 # Increment by current_perm to access the next obstacle_id
      
    # print("STM Commands: ", stm_commands)
    algoOutputLiveCommands: list[AlgorithmOutputLiveCommand] = [] # Array of commands
    for command in stm_commands:
      final_stm_command = getFinalStmCommand(command[0])
      if final_stm_command == 'FF000':
        continue
      algoOutputLiveCommands.append(AlgorithmOutputLiveCommand(
        cat="control",
        value=final_stm_command,
        end_position=command[1]
      ))
      obstacle_order_str += final_stm_command
    
    algoOutputLiveCommands.append(AlgorithmOutputLiveCommand(
      cat="control",
      value="#####",
      end_position=algoOutputLiveCommands[-1].end_position
    ))
    obstacle_order_str += '#####'
    string_length = format_length(obstacle_order_str)
    return algoOutputLiveCommands, obstacle_orders, obstacle_order_str, string_length

def format_length(s):
    """
    Calculate the length of the input string and return it as a string, 
    prefixed with zeroes to ensure a minimum length of 4 characters.

    Args:
        s (str): The input string.

    Returns:
        str: The length of the input string as a 4-character string.
    """
    return f"{len(s):04d}"
