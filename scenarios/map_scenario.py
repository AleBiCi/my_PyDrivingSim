# Off-road map scenario: start pose, target and static obstacles loaded from a JSON map file.
# Format documented in scenarios/maps/README.md.

import json
import math

from pydrivingsim import World, Vehicle, Agent, Target, Obstacle, OBSTACLE_TYPES, PathOverlay
from scenarios.map_preview import run_preview

# input_data_str holds 20 objects: 1 GPS pseudo-object + 1 target + up to 18 obstacles
MAX_OBSTACLES = 18
DEFAULT_TILE_SIZE = 20.0        # [m]


def load_map(path):
    """Read and validate a map file. Returns the dict with defaults filled in."""
    with open(path) as f:
        m = json.load(f)
    return validate_map(m, path)


def validate_map(m, path):
    """Check a map dict (path = name used in the messages) and fill in the defaults. Returns m."""
    def num(d, key, where):
        if key not in d or not isinstance(d[key], (int, float)):
            raise ValueError("%s: '%s' missing or not a number in %s" % (path, key, where))
        return float(d[key])

    m.setdefault("name", path)
    # No "background.tile" -> the default background (imgs/bg.jpeg) is kept
    bg = m.setdefault("background", {})
    bg.setdefault("tile", None)
    bg.setdefault("tile_size_m", DEFAULT_TILE_SIZE)
    m.setdefault("stop_on_collision", True)

    start = m.get("start", {})
    m["start"] = {"x": num(start, "x", "start"), "y": num(start, "y", "start"),
                  "yaw_deg": float(start.get("yaw_deg", 0.0))}
    target = m.get("target", {})
    m["target"] = {"x": num(target, "x", "target"), "y": num(target, "y", "target")}

    obstacles = m.get("obstacles", [])
    if len(obstacles) > MAX_OBSTACLES:
        raise ValueError("%s: %d obstacles, max %d (the scenario message holds 20 objects incl. GPS and target)"
                         % (path, len(obstacles), MAX_OBSTACLES))
    for i, o in enumerate(obstacles):
        where = "obstacles[%d]" % i
        if o.get("type") not in OBSTACLE_TYPES:
            raise ValueError("%s: %s type must be one of %s" % (path, where, OBSTACLE_TYPES))
        o["x"], o["y"], o["d"] = num(o, "x", where), num(o, "y", where), num(o, "d", where)
        if o["d"] <= 0:
            raise ValueError("%s: %s diameter must be > 0" % (path, where))
    m["obstacles"] = obstacles

    # Optional square border (written by scenarios/maps/map_editor.py): every object must lie inside it
    if "bounds" in m:
        b = m["bounds"]
        b = m["bounds"] = {"x_min": num(b, "x_min", "bounds"), "y_min": num(b, "y_min", "bounds"),
                           "size": num(b, "size", "bounds")}
        if b["size"] <= 0:
            raise ValueError("%s: bounds size must be > 0" % path)
        x_max, y_max = b["x_min"] + b["size"], b["y_min"] + b["size"]

        def inside(x, y, r, where):
            if x - r < b["x_min"] or x + r > x_max or y - r < b["y_min"] or y + r > y_max:
                raise ValueError("%s: %s outside the bounds" % (path, where))
        inside(m["start"]["x"], m["start"]["y"], 0.0, "start")
        inside(m["target"]["x"], m["target"]["y"], 0.0, "target")
        for i, o in enumerate(obstacles):
            inside(o["x"], o["y"], o["d"] / 2, "obstacles[%d]" % i)
    return m


class MapScenario():
    def __init__(self, map_path, use_agent=True, wait_start=True):
        """use_agent = False: map preview only (no agent, no run).
        wait_start: show the pre-run preview with the agent's whole RRT* path and wait for Start."""
        self.map = load_map(map_path)
        m = self.map
        World().set_caption("PyDrivingSim - " + m["name"])
        if m["background"]["tile"]:
            World().set_background_tile(m["background"]["tile"], m["background"]["tile_size_m"])

        # Objects are drawn in creation order: map objects first, vehicle on top
        target = Target()
        target.set_pos((m["target"]["x"], m["target"]["y"]))
        obstacles = []
        for o in m["obstacles"]:
            obstacle = Obstacle(o["type"], o["d"])
            obstacle.set_pos((o["x"], o["y"]))
            obstacles.append(obstacle)
        planned = PathOverlay(color=(0, 220, 255), width=2)    # whole RRT* path from the pre-run preview
        path = PathOverlay()                                    # trajectory window sent every agent cycle

        self.vehicle = Vehicle()
        self.vehicle.set_pos_ang((m["start"]["x"], m["start"]["y"], math.radians(m["start"]["yaw_deg"])))
        target.set_object(self.vehicle)
        for obstacle in obstacles:
            obstacle.set_object(self.vehicle, m["stop_on_collision"])

        self.agent = None
        v = self.vehicle.vehicle.vehicle
        if use_agent:
            self.agent = Agent(self.vehicle)
            if wait_start:
                # The agent plans (pose estimator + RRT*) on its first cycle with the vehicle frozen
                points, status = self.agent.preview_path()
                planned.set_points(points)
                if not run_preview(m, v.L, v.Wf, points, status, True):
                    World().loop = 0
            self.agent.path_overlay = path
            self.vehicle.set_screen_here()
        else:
            run_preview(m, v.L, v.Wf, [], None, False)
            World().loop = 0

    def update(self):
        if self.agent is None:
            return
        self.agent.compute()
        action = self.agent.get_action()

        self.vehicle.set_screen_here()
        self.vehicle.control([action[0], action[1]])

    def terminate(self):
        if self.agent is not None:
            self.agent.terminate()
