# Off-road maps

Run a map with the agent (start the C++ agent first):

    python simulator.py --map scenarios/maps/offroad_boulder_field.json

Look at a map without the agent (vehicle still, camera on the map centre):

    python simulator.py --map scenarios/maps/offroad_boulder_field.json --preview

Without `--map` the simulator runs the original scenario (`AutonomousVehicle` + traffic light + speed limit).

## Format

One JSON file per map. Units: metres and degrees. Frame: the simulator world frame,
x to the right, y up on the screen, yaw counter-clockwise from x.

```json
{
  "name": "Boulder field",
  "background": {"tile": "imgs/offroad/terrain.png", "tile_size_m": 20.0},
  "start": {"x": 0.0, "y": 0.0, "yaw_deg": 0.0},
  "target": {"x": 60.0, "y": 8.0},
  "stop_on_collision": true,
  "obstacles": [
    {"type": "boulder", "x": 12.0, "y": 0.5, "d": 3.5}
  ]
}
```

| Key | Required | Meaning |
|---|---|---|
| `name` | no | Window title. |
| `background` | no | Omit it to keep the default background (`imgs/bg.jpeg`). |
| `background.tile` | no | Seamless tile image, repeated over the whole plane. If the file is missing, the default background is kept. |
| `background.tile_size_m` | no | Side of one tile in metres. Default 20. |
| `start.x`, `start.y` | yes | Initial vehicle position [m]. |
| `start.yaw_deg` | no | Initial heading [deg], default 0. |
| `target.x`, `target.y` | yes | Goal. The run ends when the vehicle is within 1 m of it on both axes (`Target.render`). |
| `stop_on_collision` | no | End the run on the first collision (default `true`). With `false` the hit obstacle turns red and the run continues. |
| `obstacles[]` | no | Up to **19** static obstacles (the scenario message holds 20 objects, one is the target). |
| `obstacles[].type` | yes | `rock`, `boulder`, `tree`, `bush` or `stump` (sprite `imgs/offroad/<type>.png`). |
| `obstacles[].x`, `.y` | yes | Centre [m]. |
| `obstacles[].d` | yes | Footprint diameter [m]. The footprint is a circle for the collision check and for the agent. |

The loader (`scenarios/map_scenario.py`, `load_map`) checks these rules and stops with a message
that names the file and the bad entry.

## What the agent receives

Every agent step (`dt = 0.05` s) `pydrivingsim/agent.py` adds the map objects to the scenario
message, in the vehicle frame like cones and coins:

| `ObjID` | Object | `ObjX`, `ObjY` | `ObjLen`, `ObjWidth` |
|---|---|---|---|
| 3 | target | centre, vehicle frame [m] | 0 |
| 4 | obstacle | centre, vehicle frame [m] | diameter `d` [m] |

Objects keep the same order in every message (map order, target first), so index `i` is the
same object across cycles. If the agent fills `NTrajectoryPoints` and
`TrajectoryPointIX/IY` (vehicle frame), the simulator draws them as a yellow polyline.

## Collision check

Circle of diameter `d` against the rectangle `VehicleLen x VehicleWidth`
(`vehicle.L x vehicle.Wf`, the same values sent in the message) centred on the vehicle position
and rotated by its yaw (`pydrivingsim/obstacle.py`, `Obstacle.collides`).

## Sprites

Each obstacle type loads `imgs/offroad/<type>.png` (top view, transparent background); a missing
file falls back to `imgs/error.png`, so maps run before the artwork is chosen. The sprite is scaled
so that `SPRITE_FILL` (in `pydrivingsim/obstacle.py`) of its width equals the diameter `d`.
To add an obstacle type, add the name to `OBSTACLE_TYPES` there and drop in the PNG.
