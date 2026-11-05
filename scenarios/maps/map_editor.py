# Off-road map editor: place obstacles, the target and the start pose on a 30 m x 30 m square map,
# then save the JSON map (format in scenarios/maps/README.md) in scenarios/maps/custom_maps/map_<name>.json.
# Run from the repository root (guide in scenarios/maps/MAP_EDITOR.md):
#
#     python scenarios/maps/map_editor.py [map]
#
# map: an existing map file to open, or a name for a new map. No argument: new map, the name is asked on save.

import os
import sys

# The editor uses the simulator packages, which load agent/ and imgs/ with relative paths: work from the
# repository root. CALLER_CWD resolves the map argument against the folder the editor was started from.
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CALLER_CWD = os.getcwd()
sys.path.insert(0, ROOT)
os.chdir(ROOT)

import argparse
import copy
import json
import math
import re

import pygame

from scenarios.map_scenario import validate_map, MAX_OBSTACLES
from pydrivingsim.obstacle import OBSTACLE_TYPES
from pydrivingsim.mapview import MapView, map_conflicts, TARGET_REACH
from vehicle_model.param_sets import load_param_set, PARAM_SETS

CUSTOM_DIR = os.path.join("scenarios", "maps", "custom_maps")     # every save goes here (relative to ROOT)
NAME_PREFIX = "map_"
NAME_CHARS = re.compile(r"[A-Za-z0-9_-]")

# Map area: fixed square, x in [X_MIN, X_MIN + MAP_SIZE], y in [Y_MIN, Y_MIN + MAP_SIZE] (saved as "bounds")
MAP_SIZE = 30.0                     # [m]
X_MIN, Y_MIN = 0.0, -MAP_SIZE / 2   # [m] start on the left edge, x axis through the middle
BORDER_MARGIN = 1.0                 # [m] free strip along the border: no object closer than this to the edge

SCREEN = (1200, 800)
PANEL_W = 320                       # [px] side panel
PANEL_COLOR = (30, 30, 30)
TEXT_COLOR = (235, 235, 235)
KEY_COLOR = (250, 250, 250)         # shortcut: lighter and bigger than its tip
TIP_COLOR = (160, 160, 160)
HEAD_COLOR = (120, 170, 255)
HI_COLOR = (255, 230, 0)
BAD_COLOR = (255, 110, 90)
MARGIN_COLOR = (250, 250, 250)
KEY_FONT, TIP_FONT = 22, 18         # [px] font sizes
KEY_COL_W = 112                     # [px] width of the shortcut column
ROW_H = 20                          # [px]
ZOOM_STEP = 1.15
SNAP = 0.5                          # [m] position grid with snapping on
D_STEP = 0.25                       # [m] diameter step
D_MIN = 0.25                        # [m]
DEFAULT_D = {"rock": 2.0, "boulder": 4.0, "tree": 3.0, "bush": 2.5, "stump": 1.0}     # [m]
YAW_STEP, YAW_STEP_BIG = 5.0, 45.0  # [deg]
UNDO_DEPTH = 50

TOOLS = [("V", "select", "select / move"), ("S", "start", "start pose"), ("T", "target", "target")]
TOOLS += [(str(i + 1), kind, kind) for i, kind in enumerate(OBSTACLE_TYPES)]
TOOL_KEYS = {pygame.K_v: "select", pygame.K_s: "start", pygame.K_t: "target"}
TOOL_KEYS.update({pygame.K_1 + i: kind for i, kind in enumerate(OBSTACLE_TYPES)})

COMMANDS = [
    ("Left click", "place / select"),
    ("Left drag", "move (start: set yaw)"),
    ("Right drag", "pan"),
    ("Wheel", "zoom"),
    ("[  ]", "diameter -/+"),
    ("Shift+wheel", "diameter"),
    ("Q  E", "yaw +-5 deg (Shift: 45)"),
    ("Tab", "obstacle type"),
    ("Del", "delete obstacle"),
    ("G", "snap to %.1f m grid" % SNAP),
    ("F", "fit map"),
    ("Ctrl+Z", "undo"),
    ("Ctrl+S", "save"),
    ("Ctrl+Shift+S", "save as map_<name>"),
    ("Esc", "quit"),
]


def new_map():
    return {"name": "new map",
            "bounds": {"x_min": X_MIN, "y_min": Y_MIN, "size": MAP_SIZE},
            "start": {"x": X_MIN + 3.0, "y": Y_MIN + MAP_SIZE / 2, "yaw_deg": 0.0},
            "target": {"x": X_MIN + MAP_SIZE - 3.0, "y": Y_MIN + MAP_SIZE / 2},
            "stop_on_collision": True,
            "obstacles": []}


def map_to_json(m):
    """Same layout as the maps in scenarios/maps: one obstacle per line. Coordinates rounded to 1 cm."""
    r = lambda v: round(float(v), 2)
    head = {"name": m["name"]}
    bg = m.get("background", {})
    if bg.get("tile"):
        head["background"] = {"tile": bg["tile"], "tile_size_m": r(bg["tile_size_m"])}
    b = m["bounds"]
    head["bounds"] = {"x_min": r(b["x_min"]), "y_min": r(b["y_min"]), "size": r(b["size"])}
    head["start"] = {"x": r(m["start"]["x"]), "y": r(m["start"]["y"]), "yaw_deg": r(m["start"]["yaw_deg"])}
    head["target"] = {"x": r(m["target"]["x"]), "y": r(m["target"]["y"])}
    head["stop_on_collision"] = bool(m["stop_on_collision"])
    text = json.dumps(head, indent=2)[:-2]
    obs = [json.dumps({"type": o["type"], "x": r(o["x"]), "y": r(o["y"]), "d": r(o["d"])}) for o in m["obstacles"]]
    text += ',\n  "obstacles": [' + (("\n    " + ",\n    ".join(obs) + "\n  ") if obs else "") + "]\n}\n"
    return text


def custom_name(stem):
    """map_<name> from a file stem or typed text; None if nothing valid is left."""
    stem = "".join(c for c in stem if NAME_CHARS.match(c))
    if stem.startswith(NAME_PREFIX):
        stem = stem[len(NAME_PREFIX):]
    return NAME_PREFIX + stem if stem else None


class Editor:
    def __init__(self, arg, vehicle):
        vp = load_param_set(vehicle)[0].vehicle
        self.vehicle_len, self.vehicle_width = vp.L, vp.Wf
        self.vehicle_name = vehicle
        self.name = None                # map_<name> used by Ctrl+S, None = ask
        if arg and os.path.isfile(arg):
            with open(arg) as f:
                self.m = validate_map(json.load(f), arg)
            self.m["bounds"] = {"x_min": X_MIN, "y_min": Y_MIN, "size": MAP_SIZE}
            bad = self.out_of_border()
            if bad:
                sys.exit("%s does not fit the %g m x %g m map (%g m border margin): %s"
                         % (arg, MAP_SIZE, MAP_SIZE, BORDER_MARGIN, ", ".join(bad)))
            stem = os.path.splitext(os.path.basename(arg))[0]
            in_custom = os.path.abspath(os.path.dirname(arg)) == os.path.abspath(CUSTOM_DIR)
            self.name = custom_name(stem) if (in_custom and stem.startswith(NAME_PREFIX)) else None
            self.status = "Loaded " + arg + ("" if self.name else ": Ctrl+S asks for a map_ name")
        else:
            self.m = validate_map(new_map(), "new map")
            if arg:
                self.name = custom_name(os.path.splitext(os.path.basename(arg))[0])
            self.status = "New %g m x %g m map" % (MAP_SIZE, MAP_SIZE)
        self.screen = pygame.display.set_mode(SCREEN)
        self.update_caption()
        self.key_font = pygame.font.SysFont(None, KEY_FONT)
        self.tip_font = pygame.font.SysFont(None, TIP_FONT)
        self.view = MapView(self.m, self.vehicle_len, self.vehicle_width)
        self.map_rect = pygame.Rect(0, 0, SCREEN[0] - PANEL_W, SCREEN[1])
        self.view.fit(self.map_rect, margin=2.0)
        self.tool = "select"
        self.selected = None            # ("start",), ("target",), ("obstacle", i)
        self.snap = False
        self.undo = []
        self.dirty = False
        self.quit_armed = False
        self.drag = None                # "move", "yaw", "pan"
        self.drag_offset = (0.0, 0.0)
        self.drag_pushed = False
        self.typing = None              # text typed after "map_" while the save-as prompt is open
        self.overwrite_armed = False

    # ------------------------------------------------------------- border
    def half_extents(self, item, d=None, yaw_deg=None):
        """Half sizes [m] of the axis-aligned box around an item, used to keep it inside the border."""
        if item[0] == "obstacle":
            r = (self.m["obstacles"][item[1]]["d"] if d is None else d) / 2
            return r, r
        if item[0] == "target":
            return TARGET_REACH, TARGET_REACH     # the run ends inside this square (Target.render)
        yaw = math.radians(self.m["start"]["yaw_deg"] if yaw_deg is None else yaw_deg)
        hl, hw = self.vehicle_len / 2, self.vehicle_width / 2
        return (hl * abs(math.cos(yaw)) + hw * abs(math.sin(yaw)),
                hl * abs(math.sin(yaw)) + hw * abs(math.cos(yaw)))

    def allowed(self, item, **kw):
        """Range of the item's centre that keeps it BORDER_MARGIN inside the border: (x_lo, x_hi, y_lo, y_hi)."""
        hx, hy = self.half_extents(item, **kw)
        return (X_MIN + BORDER_MARGIN + hx, X_MIN + MAP_SIZE - BORDER_MARGIN - hx,
                Y_MIN + BORDER_MARGIN + hy, Y_MIN + MAP_SIZE - BORDER_MARGIN - hy)

    def fits(self, item, x, y, **kw):
        x_lo, x_hi, y_lo, y_hi = self.allowed(item, **kw)
        return x_lo <= x <= x_hi and y_lo <= y <= y_hi

    def clamp(self, item, x, y):
        x_lo, x_hi, y_lo, y_hi = self.allowed(item)
        return min(max(x, x_lo), x_hi), min(max(y, y_lo), y_hi)

    def inside_border(self, x, y):
        return X_MIN <= x <= X_MIN + MAP_SIZE and Y_MIN <= y <= Y_MIN + MAP_SIZE

    def out_of_border(self):
        items = [("start",), ("target",)] + [("obstacle", i) for i in range(len(self.m["obstacles"]))]
        return ["%s %d" % i if len(i) > 1 else i[0] for i in items if not self.fits(i, *self.item_pos(i))]

    # ------------------------------------------------------------- helpers
    def update_caption(self):
        pygame.display.set_caption("Map editor - " + (self.name or "unnamed") + (" *" if getattr(self, "dirty", False) else ""))

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
        if abs(xv) <= self.vehicle_len / 2 and abs(yv) <= self.vehicle_width / 2:
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
            o = self.m["obstacles"][self.selected[1]]
            d = min(max(D_MIN, o["d"] + steps * D_STEP), MAP_SIZE - 2 * BORDER_MARGIN)
            self.push_undo()
            o["d"] = d
            # near the border the obstacle moves inwards instead of crossing the margin
            self.set_item_pos(self.selected, *self.clamp(self.selected, o["x"], o["y"]))
            self.status = "%s d = %.2f m" % (o["type"], o["d"])

    def rotate(self, deg):
        if self.selected == ("start",) or self.tool == "start":
            s = self.m["start"]
            self.selected = ("start",)
            self.push_undo()
            s["yaw_deg"] = (s["yaw_deg"] + deg + 180.0) % 360.0 - 180.0
            # near the border the vehicle moves inwards instead of crossing the margin
            self.set_item_pos(("start",), *self.clamp(("start",), s["x"], s["y"]))

    # ------------------------------------------------------------- saving
    def target_path(self, name):
        return os.path.join(CUSTOM_DIR, name + ".json")

    def save(self, name):
        try:
            validate_map(copy.deepcopy(self.m), name)
        except ValueError as e:
            self.status = "NOT SAVED: " + str(e)
            return
        bad = self.out_of_border()
        if bad:
            self.status = "NOT SAVED: inside the border margin: " + ", ".join(bad)
            return
        if not os.path.isdir(CUSTOM_DIR):
            os.makedirs(CUSTOM_DIR)
        self.m["name"] = name
        with open(self.target_path(name), "w", newline="\n") as f:
            f.write(map_to_json(self.m))
        self.name = name
        self.dirty = False
        warn = map_conflicts(self.m, self.vehicle_len, self.vehicle_width)
        self.status = "Saved " + self.target_path(name) + ("  (check the red items)" if warn else "")

    def open_save_as(self):
        self.typing = self.name[len(NAME_PREFIX):] if self.name else ""
        self.overwrite_armed = False
        self.status = "Save as: type the name, Enter to save, Esc to cancel"

    def on_typing_key(self, event):
        if event.key == pygame.K_ESCAPE:
            self.typing = None
            self.status = "Save cancelled"
        elif event.key == pygame.K_BACKSPACE:
            self.typing = self.typing[:-1]
            self.overwrite_armed = False
        elif event.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
            name = custom_name(self.typing)
            if name is None:
                self.status = "Type a name (letters, digits, _ or -)"
                return
            if os.path.exists(self.target_path(name)) and name != self.name and not self.overwrite_armed:
                self.overwrite_armed = True
                self.status = "%s.json exists: Enter again to overwrite" % name
                return
            self.typing = None
            self.save(name)

    def on_text(self, event):
        if self.typing is not None:
            self.typing += "".join(c for c in event.text if NAME_CHARS.match(c))
            self.overwrite_armed = False

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
            return
        if not self.inside_border(x, y):
            self.status = "Outside the %g m x %g m border" % (MAP_SIZE, MAP_SIZE)
            return
        if self.tool in ("start", "target"):
            self.push_undo()
            self.set_item_pos((self.tool,), *self.clamp((self.tool,), x, y))
            self.selected = (self.tool,)
            self.drag = "yaw" if self.tool == "start" else None
        else:
            if len(self.m["obstacles"]) >= MAX_OBSTACLES:
                self.status = "Max %d obstacles (GPS + target + 18 = 20 message slots)" % MAX_OBSTACLES
                return
            d = min(DEFAULT_D.get(self.tool, 2.0), MAP_SIZE - 2 * BORDER_MARGIN)
            self.push_undo()
            self.m["obstacles"].append({"type": self.tool, "x": x, "y": y, "d": d})
            item = ("obstacle", len(self.m["obstacles"]) - 1)
            self.set_item_pos(item, *self.clamp(item, x, y))
            self.selected = item
            self.status = "%d / %d obstacles" % (len(self.m["obstacles"]), MAX_OBSTACLES)

    def on_mouse_move(self, event):
        if self.drag == "pan":
            self.view.pan_px(*event.rel)
        elif self.drag == "move" and self.selected:
            if not self.drag_pushed:
                self.push_undo()
                self.drag_pushed = True
            x, y = self.view.s2w(*event.pos)
            x, y = self.snapped(x + self.drag_offset[0], y + self.drag_offset[1])
            self.set_item_pos(self.selected, *self.clamp(self.selected, x, y))
        elif self.drag == "yaw":
            s = self.m["start"]
            x, y = self.view.s2w(*event.pos)
            if math.hypot(x - s["x"], y - s["y"]) > 0.5:
                s["yaw_deg"] = round(math.degrees(math.atan2(y - s["y"], x - s["x"])))
                self.set_item_pos(("start",), *self.clamp(("start",), s["x"], s["y"]))

    def on_key(self, event):
        if self.typing is not None:
            self.on_typing_key(event)
            return True
        ctrl = event.mod & pygame.KMOD_CTRL
        shift = event.mod & pygame.KMOD_SHIFT
        if event.key != pygame.K_ESCAPE:
            self.quit_armed = False
        if ctrl and event.key == pygame.K_s:
            if shift or self.name is None:
                self.open_save_as()
            else:
                self.save(self.name)
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
            self.view.fit(self.map_rect, margin=2.0)
        return True

    # ------------------------------------------------------------- drawing
    def text(self, s, x, y, color, font):
        self.screen.blit(font.render(s, True, color), (x, y))

    def row(self, key, tip, y, key_color=KEY_COLOR, tip_color=TIP_COLOR):
        """One option: shortcut (bigger, lighter) then its tip, baselines aligned."""
        k = self.key_font.render(key, True, key_color)
        t = self.tip_font.render(tip, True, tip_color)
        x = SCREEN[0] - PANEL_W + 14
        self.screen.blit(k, (x, y))
        self.screen.blit(t, (x + KEY_COL_W, y + k.get_height() - t.get_height()))

    def draw_margin(self):
        x0, y0 = self.view.w2s(X_MIN + BORDER_MARGIN, Y_MIN + MAP_SIZE - BORDER_MARGIN)
        x1, y1 = self.view.w2s(X_MIN + MAP_SIZE - BORDER_MARGIN, Y_MIN + BORDER_MARGIN)
        rect = pygame.Rect(int(x0), int(y0), int(x1 - x0), int(y1 - y0))
        # dashed line: the objects stay inside it
        dash = 8
        for (a, b) in ((rect.topleft, rect.topright), (rect.bottomleft, rect.bottomright),
                       (rect.topleft, rect.bottomleft), (rect.topright, rect.bottomright)):
            n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / dash))
            for k in range(0, n, 2):
                p = (a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n)
                q = (a[0] + (b[0] - a[0]) * (k + 1) / n, a[1] + (b[1] - a[1]) * (k + 1) / n)
                pygame.draw.line(self.screen, MARGIN_COLOR, p, q, 1)

    def draw_panel(self, warn):
        x0 = SCREEN[0] - PANEL_W
        x = x0 + 14
        self.screen.fill(PANEL_COLOR, (x0, 0, PANEL_W, SCREEN[1]))
        y = 10
        self.text((self.name or "unnamed map") + (" *" if self.dirty else ""), x, y, HI_COLOR, self.key_font)
        y += 22
        self.text("%g x %g m, margin %g m, %s %.2f x %.2f m" % (MAP_SIZE, MAP_SIZE, BORDER_MARGIN, self.vehicle_name,
                  self.vehicle_len, self.vehicle_width), x, y, TIP_COLOR, self.tip_font)
        y += 24
        self.text("TOOLS", x, y, HEAD_COLOR, self.tip_font)
        y += 18
        for key, tool, tip in TOOLS:
            on = tool == self.tool
            self.row(key, tip, y, HI_COLOR if on else KEY_COLOR, HI_COLOR if on else TIP_COLOR)
            y += ROW_H
        y += 6
        self.text("COMMANDS", x, y, HEAD_COLOR, self.tip_font)
        y += 18
        for key, tip in COMMANDS:
            self.row(key, tip, y)
            y += ROW_H
        y += 8
        n = len(self.m["obstacles"])
        self.text("obstacles %d / %d    snap %s" % (n, MAX_OBSTACLES, "on" if self.snap else "off"), x, y,
                  BAD_COLOR if n >= MAX_OBSTACLES else TEXT_COLOR, self.tip_font)
        y += 20
        sel = self.selected
        if sel == ("start",):
            s = self.m["start"]
            line = "start  x %.2f  y %.2f  yaw %.0f deg" % (s["x"], s["y"], s["yaw_deg"])
        elif sel == ("target",):
            t = self.m["target"]
            line = "target  x %.2f  y %.2f" % (t["x"], t["y"])
        elif sel and sel[0] == "obstacle":
            o = self.m["obstacles"][sel[1]]
            line = "%s %d  x %.2f  y %.2f  d %.2f" % (o["type"], sel[1], o["x"], o["y"], o["d"])
        else:
            line = "nothing selected"
        self.text(line, x, y, HI_COLOR if sel else TIP_COLOR, self.tip_font)
        y += 20
        mx, my = pygame.mouse.get_pos()
        if self.map_rect.collidepoint(mx, my):
            wx, wy = self.view.s2w(mx, my)
            self.text("cursor  x %.2f  y %.2f" % (wx, wy), x, y, TIP_COLOR, self.tip_font)
        if warn:
            self.text("red: obstacle on start footprint / target", x, y + 18, BAD_COLOR, self.tip_font)
        # save-as prompt
        y = SCREEN[1] - 112
        if self.typing is not None:
            box = pygame.Rect(x - 4, y - 4, PANEL_W - 20, 28)
            pygame.draw.rect(self.screen, (60, 60, 60), box)
            pygame.draw.rect(self.screen, HI_COLOR, box, 2)
            cursor = "_" if (pygame.time.get_ticks() // 500) % 2 == 0 else " "
            self.text(NAME_PREFIX + self.typing + cursor + ".json", x, y, TEXT_COLOR, self.key_font)
        # status line, wrapped
        bad = self.status.startswith("NOT") or "border" in self.status or "Max" in self.status
        words, line, y = self.status.split(" "), "", SCREEN[1] - 72
        for w in words:
            if self.tip_font.size(line + " " + w)[0] > PANEL_W - 28:
                self.text(line, x, y, BAD_COLOR if bad else TEXT_COLOR, self.tip_font)
                y += 17
                line = w
            else:
                line = (line + " " + w).strip()
        self.text(line, x, y, BAD_COLOR if bad else TEXT_COLOR, self.tip_font)

    def draw(self):
        warn = map_conflicts(self.m, self.vehicle_len, self.vehicle_width)
        self.view.draw(self.screen, selected=self.selected, warn=warn)
        self.draw_margin()
        self.draw_panel(warn)
        self.update_caption()

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
                elif event.type == pygame.TEXTINPUT:
                    self.on_text(event)
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
            self.draw()
            pygame.display.flip()
            clock.tick(60)


def main():
    parser = argparse.ArgumentParser(description="PyDrivingSim off-road map editor (%g m x %g m maps)" % (MAP_SIZE, MAP_SIZE))
    parser.add_argument("map", nargs="?", help="map file to open, or name of a new map (saved as %s/map_<name>.json)"
                        % CUSTOM_DIR.replace(os.sep, "/"))
    parser.add_argument("--vehicle", default="chimera", choices=list(PARAM_SETS),
                        help="parameter set for the start footprint (MapScenario uses Vehicle() = chimera)")
    args = parser.parse_args()
    # a file is resolved against the folder the editor was started from
    path = os.path.join(CALLER_CWD, args.map) if args.map else None
    arg = os.path.abspath(path) if path and os.path.isfile(path) else args.map
    pygame.init()
    Editor(arg, args.vehicle).run()
    pygame.quit()


if __name__ == "__main__":
    main()
