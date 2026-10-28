# Draws a polyline in world coordinates (e.g. the trajectory points returned by the agent).

import pygame

from pydrivingsim import VirtualObject, World


class PathOverlay(VirtualObject):
    __metadata = {
        "dt": 0.1
    }
    def __init__(self, color=(255, 230, 0), width=3):
        super().__init__(self.__metadata["dt"])
        self.color = color
        self.width = width
        self.points = []        # [(x, y)] world frame [m]

    def set_points(self, points):
        self.points = list(points)

    def render(self):
        if len(self.points) < 2:
            return
        px = [((x - World().get_world_pos()[0]) * World().scaling_factor + World().screen_world_center[0],
               (World().get_world_pos()[1] - y) * World().scaling_factor + World().screen_world_center[1])
              for (x, y) in self.points]
        pygame.draw.lines(World().screen, self.color, False, px, self.width)
        for p in px:
            pygame.draw.circle(World().screen, self.color, (int(p[0]), int(p[1])), self.width + 1)
