from shared.models import AlgorithmOutputLive, AlgorithmInput, AlgorithmInputMode, AlgorithmOutputSimulator
from pathfinding.get_shortest_path import get_shortest_path
from pathfinding.hamiltonian import AlgoType

import multiprocessing as mp
import time

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

app = FastAPI()

origins = [
    "http://localhost",
    "http://localhost:3000",
    "http://localhost:3001",
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/")
async def healthcheck():
    return { "status": "ok" }

@app.get("/algorithm/simulator/shortest_path/", tags=["Algorithm"])
async def sample_algorithm_simulator():
  # Basic Mock Data
  simulator_algorithm_input: AlgorithmInput = {
    "cat": "obstacles",
    "value": {
      "obstacles": [
        { "id": 1, "x": 15, "y": 10, "d": 4 }, # 10cm grid (x, y)
        { "id": 2, "x": 1, "y": 18, "d": 2 }, # 10cm grid (x, y)
      ],
    },
    "server_mode": AlgorithmInputMode.SIMULATOR,
    "algo_type": AlgoType.EXHAUSTIVE_ASTAR,
  }

  positions = get_shortest_path(simulator_algorithm_input)
  
  return { "positions": positions }

@app.post("/algorithm/simulator/shortest_path", response_model=AlgorithmOutputSimulator, tags=["Algorithm"])
async def algorithm_simulator(algo_input: AlgorithmInput):
  start_time = time.time()
  positions = get_shortest_path(algo_input.model_dump())

  runtime = time.time() - start_time # in seconds

  return { "positions": positions, "runtime": "{:.4f} seconds".format(runtime) }

@app.get("/algo/live/simple-test", response_model=AlgorithmOutputLive, tags=["Algorithm"])
async def algo_live_test():
  live_algo_input: AlgorithmInput = {
    "cat": "obstacles",
    "value": {
      "obstacles": [
        { "id": 1, "x": 15, "y": 10, "d": 4 }, # 10cm grid (x, y)
        { "id": 2, "x": 1, "y": 18, "d": 2 }, # 10cm grid (x, y)
      ],
    },
    "server_mode": AlgorithmInputMode.LIVE,
    "algo_type": AlgoType.EXHAUSTIVE_ASTAR,
  }
  commands, obstacle_orders, obstacle_order_str, string_length = get_shortest_path(live_algo_input)
  # print('android: ', android_final_coordinates)

  return { "commands": commands, "path": obstacle_orders, "path_string": obstacle_order_str, "string_length": string_length }

@app.post("/algo/live", response_model=AlgorithmOutputLive, tags=["Algorithm"])
async def algo_live(algo_input: AlgorithmInput):
  commands, obstacle_orders, obstacle_order_str, string_length = get_shortest_path(algo_input.model_dump())
  return { "commands": commands, "path": obstacle_orders, "path_string": obstacle_order_str, "string_length": string_length }

if __name__ == '__main__':
  mp.freeze_support()
