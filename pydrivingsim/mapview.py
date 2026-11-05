# Top-down drawing of an off-road map (scenarios/maps/README.md) with its own zoom and pan.
# Used by the pre-run preview (pydrivingsim/previewpanel.py) and by the map editor (map_editor.py).
# It does not use World(), so it also works in a window that is not the simulator's.

import math

import pygame

from pydrivingsim.obstacle import SPRITE_FILL

BG_COLOR = (110, 100, 80)           # same fill as World._render
GRID_COLOR = (135, 125, 105)
GRID_STEP = 5.0                     # [m] grid spacing; labelled every 2 steps
PATH_COLOR = (0, 220, 255)
START_COLOR = (40, 120, 255)
TARGET_COLOR = (230, 40, 40)
FOOTPRINT_COLOR = (255, 255, 255)
WARN_COLOR = (255, 60, 60)
SELECT_COLOR = (255, 230, 0)
TARGET_REACH = 1.0                  # [m] Target.render ends the run within 1 m on both axes
MIN_SCALE, MAX_SCALE = 1.0, 80.0    # [px/m]


class MapView:
    def __init__(self, m, vehicle_len, vehicle_width):
        """m: map dict as returned by load_map; vehicle_len/width [m] = VehicleLen/VehicleWidth."""
        self.m = m
        self.vehicle_len = vehicle_len
        self.vehicle_width = vehicle_width
        self.rect = pygame.Rect(0, 0, 100, 100)     # screen area the map is drawn in
        self.scale = 10.0                           # [px/m]
        self.center = (0.0, 0.0)                    # world point at the centre of rect [m]
        self.font = pygame.font.SysFont(None, 18)
        self._sprites = {}                          # kind -> raw image (None = no file)
        self._scaled = {}                           # (kind, px) -> scaled image
        self._tile_raw = None
        self._tile_key = None
        self._tile = None

    # ---------------------------------------------------------------- coordinates
    def w2s(self, x, y):
        return (self.rect.centerx + (x - self.center[0]) * self.scale,
                self.rect.centery - (y - self.center[1]) * self.scale)

    def s2w(self, px, py):
        return (self.center[0] + (px - self.rect.centerx) / self.scale,
                self.center[1] - (py - self.rect.centery) / self.scale)

    def fit(self, rect, extra_points=(), margin=5.0):
        """Zoom and centre so that start, target, obstacles and extra_points [(x, y)] fit in rect."""
        self.rect = pygame.Rect(rect)
        m = self.m
        xs = [m["start"]["x"], m["target"]["x"]] + [p[0] for p in extra_points]
        ys = [m["start"]["y"], m["target"]["y"]] + [p[1] for p in extra_points]
        for o in m["obstacles"]:
            xs += [o["x"] - o["d"] / 2, o["x"] + o["d"] / 2]
            ys += [o["y"] - o["d"] / 2, o["y"] + o["d"] / 2]
        dx = max(max(xs) - min(xs) + 2 * margin, 1.0)
        dy = max(max(ys) - min(ys) + 2 * margin, 1.0)
        self.scale = min(max(min(self.rect.w / dx, self.rect.h / dy), MIN_SCALE), MAX_SCALE)
        self.center = ((min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2)

    def zoom_at(self, px, py, factor):
        """Zoom by factor keeping the world point under (px, py) fixed."""
        wx, wy = self.s2w(px, py)
        self.scale = min(max(self.scale * factor, MIN_SCALE), MAX_SCALE)
        nx, ny = self.s2w(px, py)
        self.center = (self.center[0] + wx - nx, self.center[1] + wy - ny)

    def pan_px(self, dx, dy):
        self.center = (self.center[0] - dx / self.scale, self.center[1] + dy / self.scale)

    # ---------------------------------------------------------------- images
    def _sprite(self, kind, px):
        if kind not in self._sprites:
            try:
                self._sprites[kind] = pygame.image.load("imgs/offroad/" + kind + ".png").convert_alpha()
            except Exception:
                self._sprites[kind] = None
        raw = self._sprites[kind]
        if raw is None or px < 2:
            return None
        key = (kind, px)
        if key not in self._scaled:
            if len(self._scaled) > 200:
                self._scaled.clear()
            w, h = raw.get_size()
            self._scaled[key] = pygame.transform.smoothscale(raw, (px, max(1, int(h * px / w))))
        return self._scaled[key]

    def _draw_background(self, surf):
        surf.fill(BG_COLOR, self.rect)
        tile = self.m.get("background", {}).get("tile")
        if not tile:
            return
        if self._tile_raw is None:
            try:
                self._tile_raw = pygame.image.load(tile).convert()
            except Exception:
                self._tile_raw = False
        if not self._tile_raw:
            return
        side = int(round(self.m["background"]["tile_size_m"] * self.scale))
        if side < 8:
            return
        if self._tile_key != side:
            self._tile = pygame.transform.smoothscale(self._tile_raw, (side, side))
            self._tile_key = side
        # anchored to the world origin like World._render
        ox, oy = self.w2s(0.0, 0.0)
        x0 = self.rect.left + (ox - self.rect.left) % side - side
        y0 = self.rect.top + (oy - self.rect.top) % side - side
        y = y0
        while y < self.rect.bottom:
            x = x0
            while x < self.rect.right:
                surf.blit(self._tile, (x, y))
                x += side
            y += side

    def _draw_grid(self, surf):
        step = GRID_STEP
        while step * self.scale < 25:
            step *= 2
        x_min, y_max = self.s2w(self.rect.left, self.rect.top)
        x_max, y_min = self.s2w(self.rect.right, self.rect.bottom)
        k = math.floor(x_min / step)
        while k * step <= x_max:
            px = self.w2s(k * step, 0)[0]
            pygame.draw.line(surf, GRID_COLOR, (px, self.rect.top), (px, self.rect.bottom), 2 if k == 0 else 1)
            if k % 2 == 0:
                surf.blit(self.font.render("%g" % (k * step), True, GRID_COLOR), (px + 2, self.rect.bottom - 16))
            k += 1
        k = math.floor(y_min / step)
        while k * step <= y_max:
            py = self.w2s(0, k * step)[1]
            pygame.draw.line(surf, GRID_COLOR, (self.rect.left, py), (self.rect.right, py), 2 if k == 0 else 1)
            if k % 2 == 0:
                surf.blit(self.font.render("%g" % (k * step), True, GRID_COLOR), (self.rect.left + 2, py + 2))
            k += 1

    # ---------------------------------------------------------------- objects
    def vehicle_corners(self, x, y, yaw):
        """World corners of the VehicleLen x VehicleWidth rectangle (the collision footprint, obstacle.py)."""
        hl, hw = self.vehicle_len / 2, self.vehicle_width / 2
        c, s = math.cos(yaw), math.sin(yaw)
        return [(x + a * c - b * s, y + a * s + b * c) for a, b in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw))]

    def draw_obstacle(self, surf, o, selected=False, warn=False):
        cx, cy = self.w2s(o["x"], o["y"])
        r = max(2, int(o["d"] / 2 * self.scale))
        img = self._sprite(o["type"], int(o["d"] / SPRITE_FILL * self.scale))
        if img is not None:
            surf.blit(img, img.get_rect(center=(int(cx), int(cy))))
        else:
            pygame.draw.circle(surf, (90, 80, 65), (int(cx), int(cy)), r)
            lbl = self.font.render(o["type"], True, FOOTPRINT_COLOR)
            if lbl.get_width() < 2 * r - 4:
                surf.blit(lbl, lbl.get_rect(center=(int(cx), int(cy))))
        color = WARN_COLOR if warn else (SELECT_COLOR if selected else FOOTPRINT_COLOR)
        pygame.draw.circle(surf, color, (int(cx), int(cy)), r, 3 if (selected or warn) else 1)

    def draw_start(self, surf, selected=False, warn=False):
        s = self.m["start"]
        yaw = math.radians(s["yaw_deg"])
        pts = [self.w2s(*p) for p in self.vehicle_corners(s["x"], s["y"], yaw)]
        color = WARN_COLOR if warn else (SELECT_COLOR if selected else START_COLOR)
        pygame.draw.polygon(surf, START_COLOR, pts)
        pygame.draw.polygon(surf, color, pts, 3 if (selected or warn) else 1)
        a = max(self.vehicle_len, 2.0)      # heading arrow [m]
        p0 = self.w2s(s["x"], s["y"])
        p1 = self.w2s(s["x"] + a * math.cos(yaw), s["y"] + a * math.sin(yaw))
        pygame.draw.line(surf, FOOTPRINT_COLOR, p0, p1, 2)
        pygame.draw.circle(surf, FOOTPRINT_COLOR, (int(p1[0]), int(p1[1])), 3)

    def draw_target(self, surf, selected=False, warn=False):
        t = self.m["target"]
        x0, y0 = self.w2s(t["x"] - TARGET_REACH, t["y"] + TARGET_REACH)
        side = 2 * TARGET_REACH * self.scale
        color = WARN_COLOR if warn else (SELECT_COLOR if selected else TARGET_COLOR)
        pygame.draw.rect(surf, color, pygame.Rect(x0, y0, side, side), 3 if (selected or warn) else 2)
        cx, cy = self.w2s(t["x"], t["y"])
        pygame.draw.circle(surf, TARGET_COLOR, (int(cx), int(cy)), max(4, int(0.3 * self.scale)))

    def draw_path(self, surf, points, color=PATH_COLOR):
        if len(points) < 2:
            return
        px = [self.w2s(x, y) for (x, y) in points]
        pygame.draw.lines(surf, color, False, px, 3)
        for p in px:
            pygame.draw.circle(surf, color, (int(p[0]), int(p[1])), 3)

    def draw(self, surf, path=(), selected=None, warn=()):
        """selected / warn entries: ("start",), ("target",) or ("obstacle", index)."""
        clip = surf.get_clip()
        surf.set_clip(self.rect)
        self._draw_background(surf)
        self._draw_grid(surf)
        for i, o in enumerate(self.m["obstacles"]):
            self.draw_obstacle(surf, o, selected == ("obstacle", i), ("obstacle", i) in warn)
        self.draw_path(surf, path)
        self.draw_target(surf, selected == ("target",), ("target",) in warn)
        self.draw_start(surf, selected == ("start",), ("start",) in warn)
        surf.set_clip(clip)


def map_conflicts(m, vehicle_len, vehicle_width):
    """Entries that make a run fail at once: obstacles touching the vehicle footprint at the start
    (collision at t = 0) or covering the target point. Returns a set of ("obstacle", i), ("start",),
    ("target",)."""
    from pydrivingsim.obstacle import circle_hits_rectangle
    warn = set()
    s, t = m["start"], m["target"]
    for i, o in enumerate(m["obstacles"]):
        if circle_hits_rectangle(o["x"], o["y"], o["d"], s["x"], s["y"], math.radians(s["yaw_deg"]),
                                 vehicle_len, vehicle_width):
            warn |= {("obstacle", i), ("start",)}
        if math.hypot(o["x"] - t["x"], o["y"] - t["y"]) < o["d"] / 2:
            warn |= {("obstacle", i), ("target",)}
    return warn
