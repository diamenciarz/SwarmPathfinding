from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import pygame
from shapely.geometry import Polygon
from shapely.ops import unary_union

from swarmpathfinding.core.geom import Vec2
from swarmpathfinding.physics.bodies import Agent
from swarmpathfinding.viz.camera import Camera
from swarmpathfinding.viz.colors import FOG, VISION, VISION_EDGE
from swarmpathfinding.worlds.grid import GridWorld


@dataclass(slots=True)
class FogCache:
    mask: np.ndarray | None = None
    last_positions: tuple[tuple[float, float], ...] = ()
    frame: int = 0
    skip: int = 5
    move_eps: float = 0.35
    union_polygon: Polygon | None = field(default=None, repr=False)

    def needs_rebuild(self, agents: list[Agent], frame: int) -> bool:
        positions = tuple((round(a.pos.x, 2), round(a.pos.y, 2)) for a in agents)
        if self.mask is None:
            return True
        if frame - self.frame >= self.skip:
            return True
        if len(positions) != len(self.last_positions):
            return True
        for a, b in zip(positions, self.last_positions):
            if (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 > self.move_eps * self.move_eps:
                return True
        return False


def visible_mask_grid(world: GridWorld, agents: list[Agent], require_los: bool = True) -> np.ndarray:
    visible = np.zeros((world.height, world.width), dtype=bool)
    for agent in agents:
        for cell in agent.vision.visible_cells(agent.pos, agent.heading, world, require_los=require_los):
            visible[cell.y, cell.x] = True
    return visible


def vision_union_polygon(agents: list[Agent]) -> Polygon:
    polys = [agent.vision.footprint_polygon(agent.pos, agent.heading) for agent in agents]
    if not polys:
        return Polygon()
    return unary_union(polys)


def draw_vision(surface: pygame.Surface, camera: Camera, agents: list[Agent]) -> None:
    for agent in agents:
        poly = agent.vision.footprint_polygon(agent.pos, agent.heading)
        points = [camera.world_to_screen(Vec2(x, y)) for x, y in poly.exterior.coords]
        if len(points) >= 3:
            overlay = pygame.Surface(surface.get_size(), pygame.SRCALPHA)
            pygame.draw.polygon(overlay, VISION, points)
            pygame.draw.polygon(overlay, VISION_EDGE, points, width=1)
            surface.blit(overlay, (0, 0))


def draw_fog_mask(
    surface: pygame.Surface,
    camera: Camera,
    world: GridWorld,
    visible: np.ndarray,
) -> None:
    overlay = pygame.Surface(surface.get_size(), pygame.SRCALPHA)
    cs = world.cell_size
    for y in range(world.height):
        row = visible[y]
        x = 0
        while x < world.width:
            if row[x]:
                x += 1
                continue
            x1 = x
            while x1 < world.width and not row[x1]:
                x1 += 1
            top_left = camera.world_to_screen(Vec2(x * cs, (y + 1) * cs))
            bottom_right = camera.world_to_screen(Vec2(x1 * cs, y * cs))
            rect = pygame.Rect(
                top_left[0],
                top_left[1],
                max(1, bottom_right[0] - top_left[0]),
                max(1, bottom_right[1] - top_left[1]),
            )
            overlay.fill(FOG, rect)
            x = x1
    surface.blit(overlay, (0, 0))
