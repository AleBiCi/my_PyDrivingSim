# Authors : Gastone Pietro Rosati Papini
# Date    : 09/08/2022
# License : MIT

import pygame

class Singleton(type):
    _instances = {}
    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]

class World(metaclass=Singleton):
    __metadata = {
        "render_fps": 50,
        "screen_size": ( 1000, 800 ),
        "world_in_screen": 55,
        "final_time": "inf",
        "dt": 0.001
    }
    def __init__(self):
        self.loop = 1

        pygame.init()
        pygame.display.init()
        self.screen = pygame.display.set_mode( (self.__metadata["screen_size"][0], self.__metadata["screen_size"][1]) )
        self.backgorund = pygame.image.load("imgs/bg.jpeg")

        # Video information
        self.scaling_factor = self.__metadata["screen_size"][0] / self.__metadata["world_in_screen"]

        # Simulation information
        self.sim_call_freq = (1 / self.__metadata["dt"]) / self.__metadata["render_fps"]
        self.num_of_step = 0
        self.clock = pygame.time.Clock()
        self.time = 0

        # where is in the sceen the (0,0) coordinate
        self.screen_world_center = (250 , self.__metadata["screen_size"][1] / 2)
        # backgound offset
        self.bg_pos = (-1100,-1745)
        # initial world position
        self.set_world_pos((0, 0))

        # optional tiled background (off-road maps), see set_background_tile()
        self.bg_tile = None

        self.obj_list = []

    def set_world_pos(self, pos):
        self.world_pos = pos

    def get_world_pos(self):
        return self.world_pos

    def set_background_tile(self, img, tile_size_m):
        """Replace the default background with a seamless tile repeated every tile_size_m [m]."""
        try:
            tile = pygame.image.load(img).convert()
        except Exception:
            print("Background tile %s not found: keeping the default background" % img)
            return
        side = max(1, int(round(tile_size_m * self.scaling_factor)))
        self.bg_tile = pygame.transform.smoothscale(tile, (side, side))

    def set_caption(self, text):
        pygame.display.set_caption(text)

    def get_dt(self):
        return self.__metadata["dt"]

    def add(self, object):
        self.obj_list.append(object)

    def _render(self):
        # move background in the 0,0 configuration, move the backgound in the position of the moving world, center the view
        # clear the frame: off-road maps can move the view outside the default background image
        self.screen.fill((110, 100, 80))
        if self.bg_tile is None:
            screen_x_pos = self.bg_pos[0] - self.world_pos[0] * self.scaling_factor + self.__metadata["screen_size"][0] / 2
            # the minus sign there is because the graph y axis points downward.
            screen_y_pos = self.bg_pos[1] + self.world_pos[1] * self.scaling_factor + self.__metadata["screen_size"][1] / 2
            self.screen.blit(self.backgorund, (screen_x_pos,screen_y_pos))
        else:
            # tile anchored to the world origin, so it moves with the world like the objects
            side = self.bg_tile.get_width()
            x0 = (self.screen_world_center[0] - self.world_pos[0] * self.scaling_factor) % side - side
            y0 = (self.screen_world_center[1] + self.world_pos[1] * self.scaling_factor) % side - side
            y = y0
            while y < self.__metadata["screen_size"][1]:
                x = x0
                while x < self.__metadata["screen_size"][0]:
                    self.screen.blit(self.bg_tile, (x, y))
                    x += side
                y += side
        for obj in self.obj_list:
            obj.render()

        pygame.event.pump()
        self.clock.tick(self.__metadata["render_fps"])
        pygame.display.flip()


    def update(self):
        self.time += self.__metadata["dt"]
        self.num_of_step += 1
        if self.num_of_step >= self.sim_call_freq:
            self._render()
            self.num_of_step = 0

        for obj in self.obj_list:
            obj.compute()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.loop = 0

        if (not self.__metadata["final_time"]=="inf") and self.time > self.__metadata["final_time"]:
            self.loop = 0

    def exit(self):
        pygame.display.quit()
        pygame.quit()
        exit()