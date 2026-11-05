# Pre-run preview of an off-road map: the whole map fitted to the window with the agent's RRT* path,
# vehicle frozen. Start button (or Space/Enter) returns True, Esc or closing the window returns False.
# Mouse: wheel = zoom, left drag = pan; F = fit the map again.

import math

import pygame

from pydrivingsim import World
from pydrivingsim.mapview import MapView, map_conflicts

BAR_H = 64                          # [px] info bar at the top
BAR_COLOR = (30, 30, 30)
TEXT_COLOR = (235, 235, 235)
BAD_COLOR = (255, 110, 90)
BUTTON_COLOR = (40, 160, 70)
BUTTON_HOVER = (60, 190, 90)
ZOOM_STEP = 1.15

# HLP status codes, basic_agent_ST hlp/hlp.h and hlp/rrt_star.h
HLP_STATUS = {0: "OK", -1: "no path found (RRTS_NO_PATH)", -2: "start blocked (RRTS_START_BLOCKED)",
              -3: "goal blocked (RRTS_GOAL_BLOCKED)", -4: "bad input (RRTS_BAD_INPUT)",
              1: "planner not run (HLP_NOT_RUN)", 2: "no target in the message (HLP_NO_TARGET)"}


def path_length(points):
    return sum(math.hypot(x1 - x0, y1 - y0) for (x0, y0), (x1, y1) in zip(points, points[1:]))


def path_info(points, status, use_agent):
    """One line about the path and whether it is a problem."""
    if not use_agent:
        return "No agent (--preview): map only", False
    if status is None:
        return ("Agent without the preview handshake (rebuild basic_agent): %d points of its first window"
                % len(points)), True
    if status != 0:
        return "RRT*: " + HLP_STATUS.get(status, "status %d" % status), True
    return "RRT* path: %d vertices, %.1f m" % (len(points), path_length(points)), False


def run_preview(m, vehicle_len, vehicle_width, points, status, use_agent):
    """Blocking preview loop on the simulator window. Returns True when Start is pressed."""
    screen = World().screen
    sw, sh = screen.get_size()
    view = MapView(m, vehicle_len, vehicle_width)
    map_rect = pygame.Rect(0, BAR_H, sw, sh - BAR_H)
    view.fit(map_rect, points)
    warn = map_conflicts(m, vehicle_len, vehicle_width)
    font = pygame.font.SysFont(None, 24)
    small = pygame.font.SysFont(None, 20)
    button = pygame.Rect(sw - 170, 12, 150, BAR_H - 24)
    info, bad = path_info(points, status, use_agent)
    hint = ("Start / Space: run    Esc: quit    wheel: zoom    drag: pan    F: fit" if use_agent
            else "Esc: close    wheel: zoom    drag: pan    F: fit")
    if warn:
        hint = "WARNING: obstacle on the start footprint or on the target (red)    " + hint
    clock = pygame.time.Clock()
    dragging = False

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    return False
                if use_agent and event.key in (pygame.K_SPACE, pygame.K_RETURN, pygame.K_KP_ENTER):
                    return True
                if event.key == pygame.K_f:
                    view.fit(map_rect, points)
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                if use_agent and button.collidepoint(event.pos):
                    return True
                dragging = map_rect.collidepoint(event.pos)
            if event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                dragging = False
            if event.type == pygame.MOUSEMOTION and dragging:
                view.pan_px(*event.rel)
            if event.type == pygame.MOUSEWHEEL:
                mx, my = pygame.mouse.get_pos()
                view.zoom_at(mx, my, ZOOM_STEP ** event.y)

        view.draw(screen, points, warn=warn)
        screen.fill(BAR_COLOR, (0, 0, sw, BAR_H))
        screen.blit(font.render(m["name"] + "  -  " + info, True, BAD_COLOR if bad else TEXT_COLOR), (12, 10))
        screen.blit(small.render("%d obstacles    %s" % (len(m["obstacles"]), hint), True, TEXT_COLOR), (12, 38))
        if use_agent:
            hover = button.collidepoint(pygame.mouse.get_pos())
            pygame.draw.rect(screen, BUTTON_HOVER if hover else BUTTON_COLOR, button, border_radius=6)
            lbl = font.render("START", True, TEXT_COLOR)
            screen.blit(lbl, lbl.get_rect(center=button.center))
        pygame.display.flip()
        clock.tick(30)
