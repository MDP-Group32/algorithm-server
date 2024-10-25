from shared.models import LiveResponse, Settings, SimulatorResponse
from pathfinding.fastest_path import fastest_path

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


@app.post("/algo/live", response_model=LiveResponse, tags=["Algorithm"])
async def algo_live(algo_input: Settings):
  obstacle_orders, obstacle_order_str, string_length = fastest_path(algo_input.model_dump())
  return { "path": obstacle_orders, "path_string": obstacle_order_str, "string_length": string_length }

@app.post("/algorithm/simulator/shortest_path", response_model=SimulatorResponse, tags=["Algorithm"])
async def algorithm_simulator(algo_input: Settings):
  positions = fastest_path(algo_input.model_dump())
  return { "positions": positions }

if __name__ == '__main__':
  mp.freeze_support()
