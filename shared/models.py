from enum import Enum
from typing import Optional

from pydantic import BaseModel

from pathfinding.hamiltonian import AlgoType


class AlgorithmInputMode(Enum):
  SIMULATOR = "simulator"
  LIVE = "live"

class AlgorithmInputValueObstacle(BaseModel):
  id: int
  x: int
  y: int
  d: int

class AlgorithmInputValue(BaseModel):
  obstacles: list[AlgorithmInputValueObstacle]

class AlgorithmInput(BaseModel):
  cat: str = "obstacles"
  value: AlgorithmInputValue
  server_mode: Optional[AlgorithmInputMode] = AlgorithmInputMode.LIVE
  algo_type: Optional[AlgoType] = AlgoType.EXHAUSTIVE_ASTAR

class AlgorithmOutputSimulatorPosition(BaseModel):
  x: int
  y: int 
  theta: float 

class AlgorithmOutputSimulator(BaseModel):
  positions: list[AlgorithmOutputSimulatorPosition]

class AlgorithmOutputLivePosition(BaseModel):
  x: int 
  y: int 
  d: int

class AlgorithmOutputLiveCommand(BaseModel):
  cat: str = "control"
  value: str
  end_position: AlgorithmOutputLivePosition

class AlgorithmOutputLive(BaseModel):
  path: list[int] # obstacle orders
  path_string: str
  string_length: str
