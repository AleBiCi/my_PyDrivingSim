# Off-road map scenario: start pose, target and static obstacles loaded from a JSON map file.
# Format documented in scenarios/maps/README.md.

import json
import math

from pydrivingsim import World, Vehicle, Agent, Target, Obstacle, OBSTACLE_TYPES, PathOverlay

# input_data_str holds 20 objects: 1 target + up to 19 obstacles
MAX_OBSTACLES = 19
DEFAULT_TILE_SIZE = 20.0        # [m]


def load_map(path):
    """Read and validate a map file. Returns the dict with defaults filled in."""
    with open(path) as f:
        m = json.load(f)

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
        raise ValueError("%s: %d obstacles, max %d (the scenario message holds 20 objects incl. the target)"
                         % (path, len(obstacles), MAX_OBSTACLES))
    for i, o in enumerate(obstacles):
        where = "obstacles[%d]" % i
        if o.get("type") not in OBSTACLE_TYPES:
            raise ValueError("%s: %s type must be one of %s" % (path, where, OBSTACLE_TYPES))
        o["x"], o["y"], o["d"] = num(o, "x", where), num(o, "y", where), num(o, "d", where)
        if o["d"] <= 0:
            raise ValueError("%s: %s diameter must be > 0" % (path, where))
    m["obstacles"] = obstacles
    return m


class MapScenario():
    def __init__(self, map_path, use_agent=True):
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
        path = PathOverlay()

        self.vehicle = Vehicle()
        self.vehicle.set_pos_ang((m["start"]["x"], m["start"]["y"], math.radians(m["start"]["yaw_deg"])))
        target.set_object(self.vehicle)
        for obstacle in obstacles:
            obstacle.set_object(self.vehicle, m["stop_on_collision"])

        self.agent = None
        if use_agent:
            self.agent = Agent(self.vehicle)
            self.agent.path_overlay = path
            self.vehicle.set_screen_here()
        else:
            # Preview: still vehicle, camera on the centre of the map
            xs = [m["start"]["x"], m["target"]["x"]] + [o["x"] for o in m["obstacles"]]
            ys = [m["start"]["y"], m["target"]["y"]] + [o["y"] for o in m["obstacles"]]
            # screen_world_center is at x = 250 px of a 1000 px wide screen: shift by 250 px
            World().set_world_pos(((min(xs) + max(xs)) / 2 - 250 / World().scaling_factor,
                                   (min(ys) + max(ys)) / 2))

    def update(self):
        if self.agent is None:
            self.vehicle.control([0.0, 0.0])
            return
        self.agent.compute()
        action = self.agent.get_action()

        self.vehicle.set_screen_here()
        self.vehicle.control([action[0], action[1]])

    def terminate(self):
        if self.agent is not None:
            self.agent.terminate()
