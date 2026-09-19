from __future__ import annotations

import pygame
from shapely.geometry import Polygon

from swarmpathfinding.core.belief import BLOCKED, BeliefMap, ObstacleRecord
from swarmpathfinding.core.geom import Vec2
from swarmpathfinding.core.results import PathResult
from swarmpathfinding.physics.bodies import Agent
from swarmpathfinding.viz.camera import Camera
from swarmpathfinding.viz.colors import (
    AGENT,
    AGENT_OUTLINE,
    FLOOR,
    GHOST,
    GOAL,
    GRID_LINE,
    LAST_SEEN_WALL,
    PATH,
    WALL,
)
from swarmpathfinding.worlds.continuous import ContinuousWorld
from swarmpathfinding.worlds.grid import GridWorld


def draw_grid(surface: pygame.Surface, camera: Camera, world: GridWorld) -> None:
    cs = world.cell_size
    origin = camera.world_to_screen(Vec2(0, world.height * cs))
    corner = camera.world_to_screen(Vec2(world.width * cs, 0))
    rect = pygame.Rect(
        origin[0],
        origin[1],
        corner[0] - origin[0],
        corner[1] - origin[1],
    )
    pygame.draw.rect(surface, FLOOR, rect)
    if camera.zoom >= 8:
        for x in range(world.width + 1):
            a = camera.world_to_screen(Vec2(x * cs, 0))
            b = camera.world_to_screen(Vec2(x * cs, world.height * cs))
            pygame.draw.line(surface, GRID_LINE, a, b, 1)
        for y in range(world.height + 1):
            a = camera.world_to_screen(Vec2(0, y * cs))
            b = camera.world_to_screen(Vec2(world.width * cs, y * cs))
            pygame.draw.line(surface, GRID_LINE, a, b, 1)
    for y in range(world.height):
        row = world.blocked[y]
        x = 0
        while x < world.width:
            if not row[x]:
                x += 1
                continue
            x1 = x
            while x1 < world.width and row[x1]:
                x1 += 1
            tl = camera.world_to_screen(Vec2(x * cs, (y + 1) * cs))
            br = camera.world_to_screen(Vec2(x1 * cs, y * cs))
            pygame.draw.rect(
                surface,
                WALL,
                pygame.Rect(tl[0], tl[1], max(1, br[0] - tl[0]), max(1, br[1] - tl[1])),
            )
            x = x1


def draw_polygons(surface: pygame.Surface, camera: Camera, world: ContinuousWorld) -> None:
    bg_tl = camera.world_to_screen(Vec2(0, world.height))
    bg_br = camera.world_to_screen(Vec2(world.width, 0))
    pygame.draw.rect(
        surface,
        FLOOR,
        pygame.Rect(bg_tl[0], bg_tl[1], bg_br[0] - bg_tl[0], bg_br[1] - bg_tl[1]),
    )
    for obs in world.obstacles:
        _fill_polygon(surface, camera, obs.polygon, WALL)


def draw_last_seen_grid(surface: pygame.Surface, camera: Camera, belief: BeliefMap, cell_size: float) -> None:
    overlay = pygame.Surface(surface.get_size(), pygame.SRCALPHA)
    for y in range(belief.height):
        for x in range(belief.width):
            if int(belief.occupancy[y, x]) != int(BLOCKED):
                continue
            tl = camera.world_to_screen(Vec2(x * cell_size, (y + 1) * cell_size))
            br = camera.world_to_screen(Vec2((x + 1) * cell_size, y * cell_size))
            pygame.draw.rect(
                overlay,
                LAST_SEEN_WALL,
                pygame.Rect(tl[0], tl[1], max(1, br[0] - tl[0]), max(1, br[1] - tl[1])),
            )
    surface.blit(overlay, (0, 0))


def draw_ghosts(surface: pygame.Surface, camera: Camera, ghosts: list[ObstacleRecord]) -> None:
    overlay = pygame.Surface(surface.get_size(), pygame.SRCALPHA)
    for ghost in ghosts:
        if len(ghost.vertices) < 3:
            continue
        pts = [camera.world_to_screen(v) for v in ghost.vertices]
        pygame.draw.polygon(overlay, GHOST, pts)
        tip = camera.world_to_screen(ghost.position + ghost.velocity)
        pygame.draw.line(overlay, (255, 220, 120), camera.world_to_screen(ghost.position), tip, 2)
    surface.blit(overlay, (0, 0))


def draw_agents(surface: pygame.Surface, camera: Camera, agents: list[Agent]) -> None:
    for agent in agents:
        center = camera.world_to_screen(agent.pos)
        r = max(3, int(agent.radius * camera.zoom))
        pygame.draw.circle(surface, AGENT, center, r)
        pygame.draw.circle(surface, AGENT_OUTLINE, center, r, 1)
        nose = camera.world_to_screen(
            Vec2(
                agent.pos.x + agent.radius * 1.6 * _cos(agent.heading),
                agent.pos.y + agent.radius * 1.6 * _sin(agent.heading),
            )
        )
        pygame.draw.line(surface, AGENT_OUTLINE, center, nose, 2)


def draw_goal(surface: pygame.Surface, camera: Camera, goal: Vec2) -> None:
    c = camera.world_to_screen(goal)
    pygame.draw.circle(surface, GOAL, c, max(5, int(0.4 * camera.zoom)))
    pygame.draw.circle(surface, (20, 20, 20), c, max(5, int(0.4 * camera.zoom)), 1)


def draw_path(surface: pygame.Surface, camera: Camera, result: PathResult) -> None:
    if len(result.path) < 2:
        return
    pts = [camera.world_to_screen(p) for p in result.path]
    pygame.draw.lines(surface, PATH, False, pts, 2)
    for pt in pts:
        pygame.draw.circle(surface, PATH, pt, 3)


def _fill_polygon(surface: pygame.Surface, camera: Camera, polygon: Polygon, color) -> None:
    pts = [camera.world_to_screen(Vec2(x, y)) for x, y in polygon.exterior.coords]
    if len(pts) >= 3:
        pygame.draw.polygon(surface, color, pts)


def _cos(a: float) -> float:
    import math

    return math.cos(a)


def _sin(a: float) -> float:
    import math

    return math.sin(a)
