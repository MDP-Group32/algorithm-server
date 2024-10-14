from enum import Enum
from typing import Optional

from pydantic import BaseModel

from pathfinding.hamiltonian import AlgoType


class AlgorithmInputMode(Enum):
  SIMULATOR = "simulator"
  LIVE = "live"

class AlgorithmInputValueObstacle(BaseModel):
  id: int # obstacle_id
  x: int # grid_format
  y: int # grid_format
  d: int # direction of obstacle; 1: North; 2: South; 3: East; 4: West

class AlgorithmInputValue(BaseModel):
  obstacles: list[AlgorithmInputValueObstacle]

class AlgorithmInput(BaseModel):
  cat: str = "obstacles"
  value: AlgorithmInputValue
  server_mode: Optional[AlgorithmInputMode] = AlgorithmInputMode.LIVE
  algo_type: Optional[AlgoType] = AlgoType.EXHAUSTIVE_ASTAR

class AlgorithmOutputSimulatorPosition(BaseModel):
  x: int # in cm
  y: int # in cm
  theta: float # in radian

class AlgorithmOutputSimulator(BaseModel):
  positions: list[AlgorithmOutputSimulatorPosition]
  runtime: str

class AlgorithmOutputLivePosition(BaseModel):
  x: int # in cm
  y: int # in cm
  d: int # Robot Face -> 1: North; 2: South; 3: East; 4: West

class AlgorithmOutputLiveCommand(BaseModel):
  cat: str = "control"
  value: str
  end_position: AlgorithmOutputLivePosition

class AlgorithmOutputLive(BaseModel):
  commands: list[AlgorithmOutputLiveCommand]
  path: list[int] # obstacle orders
  coordinates: str
  path_string: str
  string_length: str
