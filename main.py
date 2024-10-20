from shared.models import AlgorithmOutputLive, AlgorithmInput, AlgorithmOutputSimulator
from pathfinding.get_shortest_path import get_shortest_path
from pathfinding.hamiltonian import AlgoType

import multiprocessing as mp

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

@app.post("/algorithm/simulator/shortest_path", response_model=AlgorithmOutputSimulator, tags=["Algorithm"])
async def algorithm_simulator(algo_input: AlgorithmInput):
  positions = get_shortest_path(algo_input.model_dump())
  return { "positions": positions }

@app.post("/algo/live", response_model=AlgorithmOutputLive, tags=["Algorithm"])
async def algo_live(algo_input: AlgorithmInput):
  obstacle_orders, obstacle_order_str, string_length = get_shortest_path(algo_input.model_dump())
  return { "path": obstacle_orders, "path_string": obstacle_order_str, "string_length": string_length }

if __name__ == '__main__':
  mp.freeze_support()
