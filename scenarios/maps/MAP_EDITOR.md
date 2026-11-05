# Map editor

`scenarios/maps/map_editor.py` creates and edits off-road maps for `simulator.py --map`. You place the
obstacles, the target and the start pose on a fixed **30 m x 30 m** square and save the map as JSON (format in
[README.md](README.md)).

## Run

From the repository root, with the virtual environment active:

    python scenarios/maps/map_editor.py                      # new map, the name is asked on the first save
    python scenarios/maps/map_editor.py quarry               # new map, saved as custom_maps/map_quarry.json
    python scenarios/maps/map_editor.py scenarios/maps/custom_maps/map_quarry.json    # edit a saved map

`--vehicle <set>` picks the parameter set used to draw the start footprint (`chimera` by default, the
vehicle `MapScenario` creates; others: `polaris_utv`, `cherokee_37in`, `uaz469`).

The editor works from the repository root by itself (`imgs/...` paths, simulator packages), so it can be
started from any folder.

## Map area and border

- Every map is a 30 m x 30 m square: x from 0 to 30 m, y from -15 to 15 m (simulator world frame, x right,
  y up, yaw counter-clockwise from x). The editor saves it as `"bounds": {"x_min": 0, "y_min": -15, "size": 30}`.
- A **1 m border margin** runs along the inside of the border (dashed line). No object can go into it:
  - obstacles: the whole footprint circle stays inside the margin;
  - target: the 2 m x 2 m square in which the run ends (1 m on each axis, `Target.render`) stays inside;
  - start: the whole vehicle rectangle (`VehicleLen x VehicleWidth`), at its current yaw, stays inside.
- A click outside the border places nothing. A click inside the margin, a drag past it, a bigger diameter or a
  rotation near the edge moves the object inwards until it fits.
- The area outside the border is shaded, here and in the simulator's pre-run preview.
- A map that does not fit the square (for example the 75 m example maps in this folder) is refused at load
  with the list of objects that are outside.

Constants at the top of `map_editor.py`: `MAP_SIZE`, `X_MIN`, `Y_MIN`, `BORDER_MARGIN`.

## Saving

- Every map is saved in `scenarios/maps/custom_maps/` (created on the first save) as `map_<name>.json`.
- **Ctrl+S** saves under the current name. A new map, or a map opened from another folder or without the
  `map_` prefix, opens the name prompt instead.
- **Ctrl+Shift+S** (save as) always opens the prompt. `map_` is fixed: type the rest of the name (letters,
  digits, `_`, `-`), Enter saves, Esc cancels. An existing file with another name needs a second Enter to be
  overwritten.
- The JSON `name` (window title in the simulator) is set to `map_<name>`.
- A map is saved only if `validate_map` accepts it (max 18 obstacles, all fields valid, objects inside the
  bounds) and no object is in the border margin.

## Keys

The side panel lists every option in one column: the shortcut (bigger, lighter) followed by its tip.

| Key | Action |
|---|---|
| V | select / move tool |
| S | start pose tool: click places it, drag sets the yaw |
| T | target tool |
| 1-5 | obstacle tools: rock, boulder, tree, bush, stump |
| Left click | place with the current tool, or select (V) |
| Left drag | move the selection (V), set the yaw (S) |
| Right drag | pan |
| Wheel | zoom |
| `[` `]`, Shift+wheel | obstacle diameter -/+ 0.25 m |
| Q / E | start yaw +5 / -5 deg (Shift: 45 deg) |
| Tab | change the type of the selected obstacle |
| Del | delete the selected obstacle |
| G | snap positions to a 0.5 m grid |
| F | fit the map in the window |
| Ctrl+Z | undo |
| Ctrl+S | save |
| Ctrl+Shift+S | save as `map_<name>` |
| Esc | quit (twice with unsaved changes) |

Obstacles on the start footprint or on the target turn red: the map is still saved, but the run would
collide at t = 0 or the target could not be reached.

## Limits

- At most 18 obstacles: the scenario message holds 20 objects, slot 0 is the GPS pseudo-object (ObjID 5) and
  one slot is the target (ObjID 3).
- The agent does not receive the border: its RRT* samples in the box around start, target and obstacles plus
  `bounds_pad` (`basic_agent_ST/hlp/hlp.cc`), so a path can leave the square. The preview shows it.
