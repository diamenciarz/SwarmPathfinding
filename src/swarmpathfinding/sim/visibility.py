"""Visibility for the live lab: which cells the swarm sees right now."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
import shapely
from shapely.ops import unary_union

from swarmpathfinding.physics.bodies import Agent
from swarmpathfinding.worlds.grid import GridWorld


@dataclass(slots=True)
class GridVisionCache:
    """Per-agent visible-cell lists, recomputed only when an agent changes cell,
    turns noticeably, or the map changes. LOS ray casts are the expensive part."""

    entries: dict[int, tuple[tuple, np.ndarray, np.ndarray]] = field(default_factory=dict)

    def clear(self) -> None:
        self.entries.clear()

    def visible_mask(self, world: GridWorld, agents: list[Agent], require_los: bool) -> np.ndarray:
        visible = np.zeros((world.height, world.width), dtype=bool)
        for agent in agents:
            cell = world.world_to_cell(agent.pos)
            heading_key = round(agent.heading / math.radians(5)) if agent.vision.name == "cone" else 0
            key = (cell, heading_key, world.generation, require_los, id(agent.vision))
            hit = self.entries.get(agent.id)
            if hit is None or hit[0] != key:
                cells = list(agent.vision.visible_cells(agent.pos, agent.heading, world, require_los=require_los))
                xs = np.fromiter((c.x for c in cells), dtype=np.int64, count=len(cells))
                ys = np.fromiter((c.y for c in cells), dtype=np.int64, count=len(cells))
                hit = (key, xs, ys)
                self.entries[agent.id] = hit
            visible[hit[2], hit[1]] = True
        return visible


def vision_union(agents: list[Agent]):
    polys = [
        agent.vision.footprint_polygon(agent.pos, agent.heading)
        for agent in agents
        if getattr(agent.vision, "radius", 0.0) > 0
    ]
    if not polys:
        return None
    return unary_union(polys)


def raster_visible(union, width: int, height: int, cell_size: float = 1.0) -> np.ndarray:
    """Cells whose centre lies inside the vision union (vectorised)."""
    if union is None or union.is_empty:
        return np.zeros((height, width), dtype=bool)
    xs = (np.arange(width) + 0.5) * cell_size
    ys = (np.arange(height) + 0.5) * cell_size
    gx, gy = np.meshgrid(xs, ys)
    return shapely.contains_xy(union, gx, gy)
