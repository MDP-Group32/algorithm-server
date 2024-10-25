from enum import Enum
from pydantic import BaseModel

class InputSourceType(Enum):
  SIMULATOR = "simulator"
  LIVE = "live"

class InputObs(BaseModel):
  id: int
  x: int
  y: int
  d: int

class Settings(BaseModel):
  value: list[InputObs]
  server_mode: InputSourceType = InputSourceType.LIVE

class RobotPos(BaseModel):
  x: int
  y: int 
  theta: float 

class SimulatorResponse(BaseModel):
  positions: list[RobotPos]

class LiveResponsePosition(BaseModel):
  x: int 
  y: int 
  d: int

class LiveResponse(BaseModel):
  path: list[int] # obstacle orders
  path_string: str
  string_length: str
