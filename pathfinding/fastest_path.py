from math import pi
from pathfinding.hamiltonian import HamiltonianSearch
from shared.constants import GRID_XY
from shared.enums import Bearing
from shared.models import Settings, InputSourceType
from shared.position import Position
from stm.commands import get_command_one, getFinalStmCommand
from world.obstacle import Obstacle
from world.world import World

numeric_to_direction_map = {
    '1': Bearing.NORTH,
    '2': Bearing.SOUTH,
    '3': Bearing.EAST,
    '4': Bearing.WEST
}

def extract_obstacles_from_input(input_obstacles):
  obstacles = []

  grid_pos_to_c_pos_multiplier = GRID_XY * 2  

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

def fastest_path(algo_input: Settings):

  # Obstacles
  obstacles = extract_obstacles_from_input(algo_input["value"]["obstacles"])
  source_Node = Position(x=0, y=0, theta=pi/2)

  world = World(obstacles=obstacles)

  hamiltonian_search = HamiltonianSearch(world=world, src=source_Node)

  permutate, fastest_paths = hamiltonian_search.search()

  algo_server_mode = algo_input["server_mode"]
  if algo_server_mode == InputSourceType.SIMULATOR:
    return get_simulator_fastest_path(fastest_paths)
  
  elif algo_server_mode == InputSourceType.LIVE:
    return get_live_fastest_path(permutate, fastest_paths)

def get_simulator_fastest_path(fastest_paths):
    simulator_algo_output = []
    for path in fastest_paths:
        simulator_algo_output.extend(node.pos for node in path)
        
        # Position configuration to represent scanning for simulator
        # each path from a node to another node is to take photo of the obstacle
        simulator_algo_output.extend([Position(-1, -1, -2), Position(-1, -1, -1)])

    return simulator_algo_output

def get_live_fastest_path(permutate, fastest_paths):
    current_perm = 1
    stm_commands = []
    obstacle_orders = []
    obstacle_order_str = ''

    for path in fastest_paths:
      commands = get_command_one(path)
      stm_commands.extend(commands)

      # Add ST001 command after each path (from one obstacle to another) (For Raspberry Pi Team to know when to scan the image)
      stm_commands.append([f"ST00{permutate[current_perm]}", commands[-1][1]])
      obstacle_orders.append(int(permutate[current_perm]))
      current_perm += 1 # Increment by current_perm to access the next obstacle_id
      
    for command in stm_commands:
      final_stm_command = getFinalStmCommand(command[0])
      if final_stm_command == 'FF000':
        continue
      obstacle_order_str += final_stm_command
    
    obstacle_order_str += '#####'
    string_length = format_length(obstacle_order_str)
    return obstacle_orders, obstacle_order_str, string_length

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
