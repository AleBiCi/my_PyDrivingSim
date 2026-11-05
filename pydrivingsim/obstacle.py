# Off-road static obstacle (rock, boulder, tree, bush, stump).
# Footprint: a circle of diameter `d` [m] centred in `pos`. This is what the agent receives
# (ObjLen = ObjWidth = d, ObjID = 4, see agent.py) and what the collision check uses.

import math
import pygame

from pydrivingsim import VirtualObject, World

# Sprites: imgs/offroad/<kind>.png (top view, transparent background), imgs/error.png if missing.
# The object outline is assumed to fill this fraction of the image width (rest = margin/shadow).
# TODO(dev): set to match the chosen sprites (1.0 if the outline touches the image border).
SPRITE_FILL = 0.8
OBSTACLE_TYPES = ("rock", "boulder", "tree", "bush", "stump")


def circle_hits_rectangle(xo, yo, d, xr, yr, yaw, length, width):
    """Circle of diameter d centred in (xo, yo) vs rectangle length x width centred in (xr, yr), rotated by yaw."""
    dx = xo - xr
    dy = yo - yr
    # circle centre in the rectangle frame
    xv = dx * math.cos(yaw) + dy * math.sin(yaw)
    yv = -dx * math.sin(yaw) + dy * math.cos(yaw)
    cx = min(max(xv, -length / 2), length / 2)
    cy = min(max(yv, -width / 2), width / 2)
    return (xv - cx) ** 2 + (yv - cy) ** 2 < (d / 2) ** 2


class ObstacleSprite(pygame.sprite.Sprite):
    def __init__(self, obstacle):
        super().__init__()
        try:
            sprite = pygame.image.load("imgs/offroad/" + obstacle.kind + ".png").convert_alpha()
        except Exception:
            sprite = pygame.image.load("imgs/error.png").convert_alpha()
        w, h = sprite.get_size()
        scale = (World().scaling_factor * obstacle.d / SPRITE_FILL) / w
        self.image_fix = [pygame.transform.smoothscale(sprite, (int(w * scale), int(h * scale)))]
        # state 1 = hit by the vehicle: same sprite with a red outline circle
        hit = self.image_fix[0].copy()
        r = int(World().scaling_factor * obstacle.d / 2)
        pygame.draw.circle(hit, (220, 30, 30), hit.get_rect().center, r, 3)
        self.image_fix.append(hit)

        self.image = self.image_fix[0]
        self.rect = self.image_fix[0].get_rect()
        self.obstacle = obstacle

    def update(self) -> None:
        self.rect.center = [
            (self.obstacle.pos[0] - World().get_world_pos()[0]) * World().scaling_factor + World().screen_world_center[0],
            (World().get_world_pos()[1] - self.obstacle.pos[1]) * World().scaling_factor + World().screen_world_center[1]
        ]
        self.image = self.image_fix[self.obstacle.state]


class Obstacle(VirtualObject):
    __metadata = {
        "dt": 0.01
    }
    def __init__(self, kind: str, d: float):
        super().__init__(self.__metadata["dt"])
        self.kind = kind
        self.d = d                  # [m] footprint diameter
        self.size = d               # same attribute name used by cones/coins in agent.py
        self.sprite = ObstacleSprite(self)
        self.group = pygame.sprite.Group()
        self.group.add(self.sprite)

        self.obj = None
        self.stop_on_collision = True
        self.state = 0
        self.pos = (0, 0)
        self.reset()

    def set_pos(self, point: tuple):
        self.pos = point

    def set_object(self, obj, stop_on_collision=True):
        """Vehicle to check collisions against."""
        self.obj = obj
        self.stop_on_collision = stop_on_collision

    def reset(self):
        self.state = 0

    def collides(self) -> bool:
        """Circle (obstacle) vs rectangle VehicleLen x VehicleWidth centred on the vehicle position."""
        v = self.obj
        return circle_hits_rectangle(self.pos[0], self.pos[1], self.d, v.state[0], v.state[1], v.state[2],
                                     v.vehicle.vehicle.L, v.vehicle.vehicle.Wf)

    def object_freq_compute(self):
        if self.obj is None or self.state == 1:
            return
        if self.collides():
            self.state = 1
            print("COLLISION with %s at (%.2f, %.2f), t = %.2f s" % (self.kind, self.pos[0], self.pos[1], World().time))
            if self.stop_on_collision:
                World().loop = 0

    def render(self):
        self.sprite.update()
        self.group.draw(World().screen)
