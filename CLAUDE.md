# CLAUDE.md (my_PyDrivingSim)

Python/pygame simulator by prof. Gastone Pietro Rosati Papini (upstream `github.com/tonegas/PyDrivingSim`),
forked at `github.com/AleBiCi/my_PyDrivingSim`. Workspace rules are in `../../CLAUDE.md`.

## Run

- Windows: `py -3 -m venv .venv`, `. .venv\Scripts\activate`, `pip install -r requirements.txt`,
  `python simulator.py`. Start the C++ agent first.
- Pinned: `pygame==2.1.2`, `numpy==1.24.1`.

## Code map

- `simulator.py`: picks the scenario objects (`AutonomousVehicle`, `BasicTrafficLight`,
  `BasicSpeedLimit`, `GetTheCoins`, `OnlyVehicle`).
- `scenarios/main_scenarios.py`: scenario definitions (vehicle start `(0, -1, 0)`, target at `(182, -1)`,
  cones, traffic light at x = 160, coins as waypoints around cones).
- `pydrivingsim/agent.py`: builds the scenario message every agent `dt = 0.05` s (filters
  `ALgtFild`, `YawRateFild`, `SteerWhlAg` over the sub-steps), calls `client_agent_compute`, and
  applies `(RequestedAcc, RequestedSteerWhlAg)` as `(pedal, front-wheel angle)`.
  Objects are transformed to the vehicle frame here; lane info assumes a straight 4 m road.
- `pydrivingsim/vehicle.py`: state integrated with explicit Euler at `dt = 0.001` s using
  `vehicle_model/vehicle_double_track_model.py` (double track + Pacejka, `vehicle_params.py`,
  `pacejka_params.py`). State: `[0]=x, [1]=y, [2]=yaw, [3]=u, [4]=v, [5]=yaw rate, [10]=steer`.
- `agent/`: ctypes bindings and precompiled `libagent_lib` (read-only). `interfaces_python_data_structs.py`
  must stay byte-compatible with `basic_agent_ST/lib/interfaces_data_structs.h`.

## Rules for this repo

- Agent logic goes in the C++ agent, not here. Changes here are limited to scenarios, test setups
  and (if the developer decides) off-road parameters/terrain, each asked for explicitly.
- "Off-road" is not modelled yet (flat road, one Pacejka parameter set). Do not invent terrain or
  tyre parameters: ask, and base any change on `slides/01_Tire_Models` / `03_Vehicle_Dynamics` or the
  books they cite.
- The working tree shows every file as modified because of line endings only
  (`git diff --ignore-all-space` is empty). Do not commit that churn; set `core.autocrlf` or
  `.gitattributes` first if the developer agrees.
