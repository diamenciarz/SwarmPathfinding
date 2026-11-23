from __future__ import annotations

import math
from typing import Iterable

import numpy as np
import pygame
from shapely.geometry import Polygon

from swarmpathfinding.core.belief import BLOCKED, UNKNOWN, BeliefMap, ObstacleRecord
from swarmpathfinding.core.geom import Vec2
from swarmpathfinding.physics.bodies import Agent
from swarmpathfinding.viz.camera import Camera
from swarmpathfinding.viz.colors import (
    AGENT,
    AGENT_OUTLINE,
    FLOOR,
    FOG_REMEMBERED,
    FOG_UNKNOWN,
    GHOST,
    GHOST_EDGE,
    GOAL,
    GRID_LINE,
    PATH,
    PATH_FAINT,
    PENDING_EDIT,
    REMEMBERED_WALL,
    SEARCH,
    STALE,
    UNKNOWN_CELL,
    VISION,
    VISION_EDGE,
    WALL,
)
from swarmpathfinding.worlds.continuous import ContinuousWorld
from swarmpathfinding.worlds.grid import GridWorld


# ---------------------------------------------------------------- raster helper


def blit_cells(surface: pygame.Surface, camera: Camera, rgba: np.ndarray, cell_size: float) -> None:
    """Blit an RGBA cell raster (row 0 = world y 0) scaled to the camera.

    Only the part of the raster inside the viewport is scaled, so deep zoom stays cheap.
    """
    h, w = rgba.shape[:2]
    vp = camera.viewport
    tl = camera.screen_to_world(vp.left, vp.top)
    br = camera.screen_to_world(vp.right, vp.bottom)
    x0 = max(0, int(math.floor(tl.x / cell_size)))
    x1 = min(w, int(math.ceil(br.x / cell_size)))
    y0 = max(0, int(math.floor(br.y / cell_size)))
    y1 = min(h, int(math.ceil(tl.y / cell_size)))
    if x1 <= x0 or y1 <= y0:
        return
    sub = np.ascontiguousarray(rgba[y0:y1, x0:x1][::-1])
    img = pygame.image.frombuffer(sub.tobytes(), (x1 - x0, y1 - y0), "RGBA")
    sx0, sy0 = camera.local(Vec2(x0 * cell_size, y1 * cell_size))
    sx1, sy1 = camera.local(Vec2(x1 * cell_size, y0 * cell_size))
    if sx1 <= sx0 or sy1 <= sy0:
        return
    img = pygame.transform.scale(img, (sx1 - sx0, sy1 - sy0))
    surface.blit(img, (sx0, sy0))


def _rgba(shape: tuple[int, int], color) -> np.ndarray:
    out = np.zeros((*shape, 4), dtype=np.uint8)
    out[...] = (*color[:3], color[3] if len(color) > 3 else 255)
    return out


# ---------------------------------------------------------------- worlds


def draw_grid(surface: pygame.Surface, camera: Camera, world: GridWorld) -> None:
    """Ground-truth grid: floor and walls."""
    rgba = _rgba(world.blocked.shape, FLOOR)
    rgba[world.blocked] = (*WALL, 255)
    blit_cells(surface, camera, rgba, world.cell_size)
    draw_grid_lines(surface, camera, world)


def draw_belief_grid(
    surface: pygame.Surface,
    camera: Camera,
    world: GridWorld,
    belief: BeliefMap,
    visible: np.ndarray | None,
) -> None:
    """What the swarm remembers. Unknown cells are dark, remembered-but-not-visible
    cells are dimmed, and stale memories (the world changed since) are tinted red."""
    occ = belief.occupancy
    rgba = _rgba(occ.shape, UNKNOWN_CELL)
    rgba[occ == 0] = (*FLOOR, 255)
    rgba[occ == BLOCKED] = (*WALL, 255)
    rgba[belief.stale_cells(world.blocked)] = (*STALE, 255)
    if visible is not None:
        dim = (~visible) & (occ != UNKNOWN)
        rgba[dim, :3] = (rgba[dim, :3].astype(np.uint16) * 3 // 5).astype(np.uint8)
    blit_cells(surface, camera, rgba, world.cell_size)
    draw_grid_lines(surface, camera, world)


def draw_grid_lines(surface: pygame.Surface, camera: Camera, world: GridWorld) -> None:
    if camera.zoom < 10:
        return
    cs = world.cell_size
    for x in range(world.width + 1):
        a = camera.local(Vec2(x * cs, 0))
        b = camera.local(Vec2(x * cs, world.height * cs))
        pygame.draw.line(surface, GRID_LINE, a, b, 1)
    for y in range(world.height + 1):
        a = camera.local(Vec2(0, y * cs))
        b = camera.local(Vec2(world.width * cs, y * cs))
        pygame.draw.line(surface, GRID_LINE, a, b, 1)


def draw_polygons(surface: pygame.Surface, camera: Camera, world: ContinuousWorld) -> None:
    bg_tl = camera.local(Vec2(0, world.height))
    bg_br = camera.local(Vec2(world.width, 0))
    pygame.draw.rect(
        surface,
        FLOOR,
        pygame.Rect(bg_tl[0], bg_tl[1], bg_br[0] - bg_tl[0], bg_br[1] - bg_tl[1]),
    )
    for obs in world.obstacles:
        _fill_polygon(surface, camera, obs.polygon, WALL)


def draw_floor(surface: pygame.Surface, camera: Camera, world: ContinuousWorld) -> None:
    tl = camera.local(Vec2(0, world.height))
    br = camera.local(Vec2(world.width, 0))
    pygame.draw.rect(surface, FLOOR, pygame.Rect(tl[0], tl[1], br[0] - tl[0], br[1] - tl[1]))


def draw_remembered(surface: pygame.Surface, camera: Camera, records: Iterable[ObstacleRecord]) -> None:
    """Obstacles as the swarm last saw them, in a dimmed wall colour.

    Drawn over the fog so a remembered shape reads the same whether its cells
    were seen or not; `draw_visible_truth` then paints the in-view part bright."""
    for rec in records:
        if len(rec.vertices) >= 3:
            pygame.draw.polygon(surface, REMEMBERED_WALL, [camera.local(v) for v in rec.vertices])


def draw_visible_truth(surface: pygame.Surface, camera: Camera, world: ContinuousWorld, union) -> None:
    """Real obstacles, but only the parts inside the swarm's current vision."""
    if union is None or union.is_empty:
        return
    for obs in world.obstacles:
        if not union.intersects(obs.polygon):
            continue
        part = obs.polygon.intersection(union)
        for poly in getattr(part, "geoms", [part]):
            if poly.geom_type == "Polygon" and not poly.is_empty:
                _fill_polygon(surface, camera, poly, WALL)


def draw_fog_raster(
    surface: pygame.Surface,
    camera: Camera,
    belief: BeliefMap,
    visible: np.ndarray | None,
    cell_size: float = 1.0,
) -> None:
    """Fog over a continuous world: unknown is near-opaque, remembered is dimmed."""
    occ = belief.occupancy
    rgba = np.zeros((*occ.shape, 4), dtype=np.uint8)
    rgba[occ == UNKNOWN] = FOG_UNKNOWN
    remembered = occ != UNKNOWN
    if visible is not None:
        remembered &= ~visible
    rgba[remembered] = FOG_REMEMBERED
    blit_cells(surface, camera, rgba, cell_size)


def draw_ghosts(surface: pygame.Surface, camera: Camera, ghosts: Iterable[ObstacleRecord]) -> None:
    """Last-seen obstacles, moved along their last-seen velocity."""
    overlay = pygame.Surface(surface.get_size(), pygame.SRCALPHA)
    for ghost in ghosts:
        if len(ghost.vertices) < 3:
            continue
        pts = [camera.local(v) for v in ghost.vertices]
        pygame.draw.polygon(overlay, GHOST, pts)
        pygame.draw.polygon(overlay, GHOST_EDGE, pts, 1)
        if ghost.velocity.length_sq() > 1e-9:
            a = camera.local(ghost.position)
            b = camera.local(ghost.position + ghost.velocity)
            pygame.draw.line(overlay, GHOST_EDGE, a, b, 2)
    surface.blit(overlay, (0, 0))


def draw_search(surface: pygame.Surface, camera: Camera, closed: np.ndarray, cell_size: float) -> None:
    """Cells the planner expanded (its closed set)."""
    rgba = np.zeros((*closed.shape, 4), dtype=np.uint8)
    rgba[closed] = SEARCH
    blit_cells(surface, camera, rgba, cell_size)


def draw_vision_union(surface: pygame.Surface, camera: Camera, union) -> None:
    """The union of every agent's vision shape, drawn once (no alpha stacking)."""
    if union is None or union.is_empty:
        return
    overlay = pygame.Surface(surface.get_size(), pygame.SRCALPHA)
    polys = list(union.geoms) if hasattr(union, "geoms") else [union]
    for poly in polys:
        if poly.geom_type != "Polygon" or poly.is_empty:
            continue
        pts = [camera.local(Vec2(x, y)) for x, y in poly.exterior.coords]
        if len(pts) >= 3:
            pygame.draw.polygon(overlay, VISION, pts)
            pygame.draw.polygon(overlay, VISION_EDGE, pts, 1)
    surface.blit(overlay, (0, 0))


# ---------------------------------------------------------------- swarm


def draw_agents(surface: pygame.Surface, camera: Camera, agents: list[Agent], focus: int | None = None) -> None:
    for agent in agents:
        center = camera.local(agent.pos)
        r = max(3, int(agent.radius * camera.zoom))
        if focus is not None and agent.id == focus:
            pygame.draw.circle(surface, PATH, center, r + 3, 2)
        pygame.draw.circle(surface, AGENT, center, r)
        pygame.draw.circle(surface, AGENT_OUTLINE, center, r, 1)
        nose = camera.local(
            Vec2(
                agent.pos.x + agent.radius * 1.6 * math.cos(agent.heading),
                agent.pos.y + agent.radius * 1.6 * math.sin(agent.heading),
            )
        )
        pygame.draw.line(surface, AGENT_OUTLINE, center, nose, 2)


def draw_goal(surface: pygame.Surface, camera: Camera, goal: Vec2) -> None:
    c = camera.local(goal)
    r = max(5, int(0.4 * camera.zoom))
    pygame.draw.circle(surface, GOAL, c, r)
    pygame.draw.circle(surface, (20, 20, 20), c, r, 1)


def draw_paths(
    surface: pygame.Surface,
    camera: Camera,
    paths: Iterable[list[Vec2]],
    focus_path: list[Vec2] | None = None,
) -> None:
    """Every agent's remaining route faintly, the focus agent's route solid."""
    overlay = pygame.Surface(surface.get_size(), pygame.SRCALPHA)
    drew = False
    for path in paths:
        if len(path) >= 2:
            pygame.draw.lines(overlay, PATH_FAINT, False, [camera.local(p) for p in path], 1)
            drew = True
    if drew:
        surface.blit(overlay, (0, 0))
    if focus_path and len(focus_path) >= 2:
        pygame.draw.lines(surface, PATH, False, [camera.local(p) for p in focus_path], 2)


def draw_pending_cell(surface: pygame.Surface, camera: Camera, x: int, y: int, cell_size: float) -> None:
    tl = camera.local(Vec2(x * cell_size, (y + 1) * cell_size))
    br = camera.local(Vec2((x + 1) * cell_size, y * cell_size))
    rect = pygame.Rect(tl[0], tl[1], max(3, br[0] - tl[0]), max(3, br[1] - tl[1]))
    pygame.draw.rect(surface, PENDING_EDIT, rect.inflate(2, 2), 2)


def draw_pending_polygon(surface: pygame.Surface, camera: Camera, polygon: Polygon) -> None:
    pts = [camera.local(Vec2(x, y)) for x, y in polygon.exterior.coords]
    if len(pts) >= 3:
        pygame.draw.polygon(surface, PENDING_EDIT, pts, 2)


def _fill_polygon(surface: pygame.Surface, camera: Camera, polygon: Polygon, color) -> None:
    pts = [camera.local(Vec2(x, y)) for x, y in polygon.exterior.coords]
    if len(pts) >= 3:
        pygame.draw.polygon(surface, color, pts)
