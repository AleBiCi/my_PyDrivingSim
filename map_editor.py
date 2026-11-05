# Off-road map editor: place obstacles, the target and the start pose, then save the JSON map
# (format in scenarios/maps/README.md). Run from the repository root:
#
#     python map_editor.py scenarios/maps/my_map.json [--vehicle chimera]
#
# A missing file starts a new map that is saved under that name. Keys are listed in the side panel.

import argparse
import copy
import json
import math
import os

import pygame

from scenarios.map_scenario import validate_map, MAX_OBSTACLES
from pydrivingsim.obstacle import OBSTACLE_TYPES
from pydrivingsim.mapview import MapView, map_conflicts, TARGET_REACH
from vehicle_model.param_sets import load_param_set, PARAM_SETS

SCREEN = (1200, 800)
PANEL_W = 300                       # [px] side panel
PANEL_COLOR = (30, 30, 30)
TEXT_COLOR = (235, 235, 235)
DIM_COLOR = (150, 150, 150)
HI_COLOR = (255, 230, 0)
BAD_COLOR = (255, 110, 90)
ZOOM_STEP = 1.15
SNAP = 0.5                          # [m] position grid with snapping on
D_STEP = 0.25                       # [m] diameter step
D_MIN = 0.25                        # [m]
DEFAULT_D = {"rock": 2.0, "boulder": 4.0, "tree": 3.0, "bush": 2.5, "stump": 1.0}     # [m]
YAW_STEP, YAW_STEP_BIG = 5.0, 45.0  # [deg]
UNDO_DEPTH = 50

TOOLS = ["select", "start", "target"] + list(OBSTACLE_TYPES)
TOOL_KEYS = {pygame.K_v: "select", pygame.K_s: "start", pygame.K_t: "target"}
TOOL_KEYS.update({pygame.K_1 + i: kind for i, kind in enumerate(OBSTACLE_TYPES)})

HELP = [
    "V select / move    S start    T target",
    "1-%d obstacle: %s" % (len(OBSTACLE_TYPES), ", ".join(OBSTACLE_TYPES)),
    "",
    "Left click     place / select",
    "Left drag      move (start tool: set yaw)",
    "Right drag     pan        Wheel  zoom",
    "[ ]  or Shift+wheel   diameter",
    "Q E  yaw +-5 deg (Shift: 45)",
    "Tab  change obstacle type",
    "Del  delete obstacle",
    "G    snap to %.1f m grid" % SNAP,
    "F    fit map    Ctrl+Z undo",
    "Ctrl+S save    Esc quit",
]


def new_map(name):
    return {"name": name,
            "start": {"x": 0.0, "y": 0.0, "yaw_deg": 0.0},
            "target": {"x": 40.0, "y": 0.0},
            "stop_on_collision": True,
            "obstacles": []}


def map_to_json(m):
    """Same layout as the maps in scenarios/maps: one obstacle per line. Coordinates rounded to 1 cm."""
    r = lambda v: round(float(v), 2)
    head = {"name": m["name"]}
    bg = m.get("background", {})
    if bg.get("tile"):
        head["background"] = {"tile": bg["tile"], "tile_size_m": r(bg["tile_size_m"])}
    head["start"] = {"x": r(m["start"]["x"]), "y": r(m["start"]["y"]), "yaw_deg": r(m["start"]["yaw_deg"])}
    head["target"] = {"x": r(m["target"]["x"]), "y": r(m["target"]["y"])}
    head["stop_on_collision"] = bool(m["stop_on_collision"])
    text = json.dumps(head, indent=2)[:-2]
    obs = [json.dumps({"type": o["type"], "x": r(o["x"]), "y": r(o["y"]), "d": r(o["d"])}) for o in m["obstacles"]]
    text += ',\n  "obstacles": [' + (("\n    " + ",\n    ".join(obs) + "\n  ") if obs else "") + "]\n}\n"
    return text


class Editor:
    def __init__(self, path, vehicle):
        self.path = path
        if os.path.exists(path):
            with open(path) as f:
                self.m = validate_map(json.load(f), path)
            self.status = "Loaded " + path
        else:
            self.m = validate_map(new_map(os.path.splitext(os.path.basename(path))[0]), path)
            self.status = "New map: Ctrl+S saves to " + path
        vp = load_param_set(vehicle)[0].vehicle
        self.vehicle_name = vehicle
        self.screen = pygame.display.set_mode(SCREEN)
        pygame.display.set_caption("Map editor - " + path)
        self.font = pygame.font.SysFont(None, 22)
        self.small = pygame.font.SysFont(None, 19)
        self.view = MapView(self.m, vp.L, vp.Wf)
        self.map_rect = pygame.Rect(0, 0, SCREEN[0] - PANEL_W, SCREEN[1])
        self.view.fit(self.map_rect, margin=10.0)
        self.tool = "select"
        self.selected = None            # ("start",), ("target",), ("obstacle", i)
        self.snap = False
        self.undo = []
        self.dirty = False
        self.quit_armed = False
        self.drag = None                # "move", "yaw", "pan"
        self.drag_offset = (0.0, 0.0)
        self.drag_pushed = False

    # ------------------------------------------------------------- helpers
    def push_undo(self):
        self.undo.append(copy.deepcopy(self.m))
        del self.undo[:-UNDO_DEPTH]
        self.dirty = True

    def snapped(self, x, y):
        if self.snap:
            return round(x / SNAP) * SNAP, round(y / SNAP) * SNAP
        return x, y

    def item_at(self, x, y):
        """Topmost item under the world point (start, target, then obstacles last drawn first)."""
        s = self.m["start"]
        c, sn = math.cos(math.radians(s["yaw_deg"])), math.sin(math.radians(s["yaw_deg"]))
        xv = (x - s["x"]) * c + (y - s["y"]) * sn
        yv = -(x - s["x"]) * sn + (y - s["y"]) * c
        if abs(xv) <= self.view.vehicle_len / 2 and abs(yv) <= self.view.vehicle_width / 2:
            return ("start",)
        t = self.m["target"]
        if abs(x - t["x"]) <= TARGET_REACH and abs(y - t["y"]) <= TARGET_REACH:
            return ("target",)
        for i in reversed(range(len(self.m["obstacles"]))):
            o = self.m["obstacles"][i]
            if math.hypot(x - o["x"], y - o["y"]) <= max(o["d"] / 2, 0.5):
                return ("obstacle", i)
        return None

    def item_pos(self, item):
        if item[0] == "obstacle":
            o = self.m["obstacles"][item[1]]
            return o["x"], o["y"]
        return self.m[item[0]]["x"], self.m[item[0]]["y"]

    def set_item_pos(self, item, x, y):
        d = self.m["obstacles"][item[1]] if item[0] == "obstacle" else self.m[item[0]]
        d["x"], d["y"] = x, y

    def change_d(self, steps):
        if self.selected and self.selected[0] == "obstacle":
            self.push_undo()
            o = self.m["obstacles"][self.selected[1]]
            o["d"] = max(D_MIN, o["d"] + steps * D_STEP)
            self.status = "%s d = %.2f m" % (o["type"], o["d"])

    def rotate(self, deg):
        if self.selected == ("start",) or self.tool == "start":
            self.push_undo()
            s = self.m["start"]
            s["yaw_deg"] = (s["yaw_deg"] + deg + 180.0) % 360.0 - 180.0
            self.selected = ("start",)

    def save(self):
        try:
            validate_map(copy.deepcopy(self.m), self.path)
        except ValueError as e:
            self.status = "NOT SAVED: " + str(e)
            return
        d = os.path.dirname(self.path)
        if d and not os.path.isdir(d):
            os.makedirs(d)
        with open(self.path, "w", newline="\n") as f:
            f.write(map_to_json(self.m))
        self.dirty = False
        warn = map_conflicts(self.m, self.view.vehicle_len, self.view.vehicle_width)
        self.status = "Saved " + self.path + ("  (check the red items)" if warn else "")

    # ------------------------------------------------------------- events
    def on_mouse_down(self, event):
        if not self.map_rect.collidepoint(event.pos):
            return
        if event.button in (2, 3):
            self.drag = "pan"
            return
        if event.button != 1:
            return
        x, y = self.snapped(*self.view.s2w(*event.pos))
        if self.tool == "select":
            self.selected = self.item_at(*self.view.s2w(*event.pos))
            if self.selected:
                px, py = self.item_pos(self.selected)
                self.drag = "move"
                self.drag_offset = (px - x, py - y)
                self.drag_pushed = False
        elif self.tool in ("start", "target"):
            self.push_undo()
            self.set_item_pos((self.tool,), x, y)
            self.selected = (self.tool,)
            self.drag = "yaw" if self.tool == "start" else None
        else:
            if len(self.m["obstacles"]) >= MAX_OBSTACLES:
                self.status = "Max %d obstacles (GPS + target + 18 = 20 message slots)" % MAX_OBSTACLES
                return
            self.push_undo()
            self.m["obstacles"].append({"type": self.tool, "x": x, "y": y, "d": DEFAULT_D.get(self.tool, 2.0)})
            self.selected = ("obstacle", len(self.m["obstacles"]) - 1)
            self.status = "%d / %d obstacles" % (len(self.m["obstacles"]), MAX_OBSTACLES)

    def on_mouse_move(self, event):
        if self.drag == "pan":
            self.view.pan_px(*event.rel)
        elif self.drag == "move" and self.selected:
            if not self.drag_pushed:
                self.push_undo()
                self.drag_pushed = True
            x, y = self.view.s2w(*event.pos)
            self.set_item_pos(self.selected, *self.snapped(x + self.drag_offset[0], y + self.drag_offset[1]))
        elif self.drag == "yaw":
            s = self.m["start"]
            x, y = self.view.s2w(*event.pos)
            if math.hypot(x - s["x"], y - s["y"]) > 0.5:
                s["yaw_deg"] = round(math.degrees(math.atan2(y - s["y"], x - s["x"])))

    def on_key(self, event):
        ctrl = event.mod & pygame.KMOD_CTRL
        shift = event.mod & pygame.KMOD_SHIFT
        if event.key != pygame.K_ESCAPE:
            self.quit_armed = False
        if ctrl and event.key == pygame.K_s:
            self.save()
        elif ctrl and event.key == pygame.K_z:
            if self.undo:
                self.m.clear()
                self.m.update(self.undo.pop())
                self.selected = None
                self.dirty = True
                self.status = "Undo"
        elif event.key == pygame.K_ESCAPE:
            if self.dirty and not self.quit_armed:
                self.quit_armed = True
                self.status = "Unsaved changes: Esc again to quit, Ctrl+S to save"
            else:
                return False
        elif event.key in TOOL_KEYS:
            self.tool = TOOL_KEYS[event.key]
            if self.tool in ("start", "target"):
                self.selected = (self.tool,)
        elif event.key in (pygame.K_DELETE, pygame.K_BACKSPACE):
            if self.selected and self.selected[0] == "obstacle":
                self.push_undo()
                del self.m["obstacles"][self.selected[1]]
                self.selected = None
        elif event.key == pygame.K_TAB:
            if self.selected and self.selected[0] == "obstacle":
                self.push_undo()
                o = self.m["obstacles"][self.selected[1]]
                o["type"] = OBSTACLE_TYPES[(OBSTACLE_TYPES.index(o["type"]) + 1) % len(OBSTACLE_TYPES)]
        elif event.key == pygame.K_RIGHTBRACKET:
            self.change_d(+1)
        elif event.key == pygame.K_LEFTBRACKET:
            self.change_d(-1)
        elif event.key == pygame.K_q:
            self.rotate(YAW_STEP_BIG if shift else YAW_STEP)
        elif event.key == pygame.K_e:
            self.rotate(-(YAW_STEP_BIG if shift else YAW_STEP))
        elif event.key == pygame.K_g:
            self.snap = not self.snap
            self.status = "Snap " + ("on" if self.snap else "off")
        elif event.key == pygame.K_f:
            self.view.fit(self.map_rect, margin=10.0)
        return True

    # ------------------------------------------------------------- drawing
    def text(self, s, x, y, color=TEXT_COLOR, font=None):
        self.screen.blit((font or self.font).render(s, True, color), (x, y))

    def draw_panel(self, warn):
        x0 = SCREEN[0] - PANEL_W
        self.screen.fill(PANEL_COLOR, (x0, 0, PANEL_W, SCREEN[1]))
        x = x0 + 14
        y = 12
        self.text(self.m["name"] + (" *" if self.dirty else ""), x, y, HI_COLOR)
        y += 26
        self.text("vehicle footprint: %s %.2f x %.2f m" % (self.vehicle_name, self.view.vehicle_len,
                                                          self.view.vehicle_width), x, y, DIM_COLOR, self.small)
        y += 30
        for i, t in enumerate(TOOLS):
            key = "V" if t == "select" else ("S" if t == "start" else ("T" if t == "target" else str(i - 2)))
            self.text("[%s] %s" % (key, t), x + (i % 2) * 140, y + (i // 2) * 22,
                      HI_COLOR if t == self.tool else TEXT_COLOR)
        y += 22 * ((len(TOOLS) + 1) // 2) + 14
        n = len(self.m["obstacles"])
        self.text("obstacles: %d / %d    snap: %s" % (n, MAX_OBSTACLES, "on" if self.snap else "off"), x, y,
                  BAD_COLOR if n >= MAX_OBSTACLES else TEXT_COLOR)
        y += 30
        sel = self.selected
        if sel == ("start",):
            s = self.m["start"]
            lines = ["START", "x %.2f   y %.2f m" % (s["x"], s["y"]), "yaw %.1f deg" % s["yaw_deg"]]
        elif sel == ("target",):
            t = self.m["target"]
            lines = ["TARGET", "x %.2f   y %.2f m" % (t["x"], t["y"]),
                     "dist from start %.1f m" % math.hypot(t["x"] - self.m["start"]["x"], t["y"] - self.m["start"]["y"])]
        elif sel and sel[0] == "obstacle":
            o = self.m["obstacles"][sel[1]]
            lines = ["OBSTACLE %d: %s" % (sel[1], o["type"]), "x %.2f   y %.2f m" % (o["x"], o["y"]),
                     "d %.2f m" % o["d"]]
        else:
            lines = ["nothing selected"]
        for k, line in enumerate(lines):
            self.text(line, x, y, HI_COLOR if k == 0 else TEXT_COLOR)
            y += 22
        y += 10
        mx, my = pygame.mouse.get_pos()
        if self.map_rect.collidepoint(mx, my):
            wx, wy = self.view.s2w(mx, my)
            self.text("cursor x %.2f   y %.2f m" % (wx, wy), x, y, DIM_COLOR, self.small)
        y += 30
        for line in HELP:
            self.text(line, x, y, DIM_COLOR, self.small)
            y += 19
        y += 10
        if warn:
            self.text("Red: obstacle on the start footprint", x, y, BAD_COLOR, self.small)
            self.text("or on the target", x, y + 18, BAD_COLOR, self.small)
        # status line, wrapped
        words, line, y = self.status.split(" "), "", SCREEN[1] - 70
        for w in words:
            if self.small.size(line + " " + w)[0] > PANEL_W - 28:
                self.text(line, x, y, BAD_COLOR if self.status.startswith("NOT") else TEXT_COLOR, self.small)
                y += 18
                line = w
            else:
                line = (line + " " + w).strip()
        self.text(line, x, y, BAD_COLOR if self.status.startswith("NOT") else TEXT_COLOR, self.small)

    def run(self):
        clock = pygame.time.Clock()
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    if self.dirty and not self.quit_armed:
                        self.quit_armed = True
                        self.status = "Unsaved changes: close again to quit, Ctrl+S to save"
                    else:
                        return
                elif event.type == pygame.KEYDOWN:
                    if not self.on_key(event):
                        return
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    self.on_mouse_down(event)
                elif event.type == pygame.MOUSEBUTTONUP:
                    self.drag = None
                elif event.type == pygame.MOUSEMOTION:
                    self.on_mouse_move(event)
                elif event.type == pygame.MOUSEWHEEL:
                    if pygame.key.get_mods() & pygame.KMOD_SHIFT:
                        self.change_d(event.y)
                    else:
                        mx, my = pygame.mouse.get_pos()
                        if self.map_rect.collidepoint(mx, my):
                            self.view.zoom_at(mx, my, ZOOM_STEP ** event.y)
            warn = map_conflicts(self.m, self.view.vehicle_len, self.view.vehicle_width)
            self.view.draw(self.screen, selected=self.selected, warn=warn)
            self.draw_panel(warn)
            pygame.display.flip()
            clock.tick(60)


def main():
    parser = argparse.ArgumentParser(description="PyDrivingSim off-road map editor")
    parser.add_argument("map", help="map file to edit or create, e.g. scenarios/maps/my_map.json")
    parser.add_argument("--vehicle", default="chimera", choices=list(PARAM_SETS),
                        help="parameter set for the start footprint (MapScenario uses Vehicle() = chimera)")
    args = parser.parse_args()
    pygame.init()
    Editor(args.map, args.vehicle).run()
    pygame.quit()


if __name__ == "__main__":
    main()
