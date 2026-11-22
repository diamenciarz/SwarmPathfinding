"""Turn a BeliefMap into a boolean occupancy grid that grid search can run on.

Grid worlds: one planning cell per world cell, straight from the last-seen map.
Continuous worlds: the last-seen obstacle polygons are inflated by the agent
radius (configuration space) and rasterised at `resolution` world units.

Unknown cells count as free when `belief.freespace_unknown` is set (the usual
optimistic fog-of-war assumption) and as blocked otherwise.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import shapely
from shapely.geometry import Polygon

from swarmpathfinding.core.belief import BLOCKED, UNKNOWN, BeliefMap
from swarmpathfinding.core.geom import Cell, Vec2


@dataclass(slots=True)
class PlanGrid:
    blocked: np.ndarray  # bool [height, width]
    cell_size: float

    @property
    def width(self) -> int:
        return int(self.blocked.shape[1])

    @property
    def height(self) -> int:
        return int(self.blocked.shape[0])

    def to_cell(self, point: Vec2) -> Cell:
        x = min(max(int(math.floor(point.x / self.cell_size)), 0), self.width - 1)
        y = min(max(int(math.floor(point.y / self.cell_size)), 0), self.height - 1)
        return Cell(x, y)

    def center(self, cell: Cell) -> Vec2:
        return Vec2((cell.x + 0.5) * self.cell_size, (cell.y + 0.5) * self.cell_size)

    def segment_blocked(self, a: Vec2, b: Vec2, ignore_a: float = 0.0, ignore_b: float = 0.0) -> bool:
        """Sample the segment a->b and report whether any sample is blocked.

        Samples closer than `ignore_a` / `ignore_b` to the ends are skipped, so an
        agent hugging a wall (inside the inflated margin) does not count as blocked.
        """
        length = a.dist(b)
        n = max(2, int(math.ceil(length / (self.cell_size * 0.5))) + 1)
        t = np.linspace(0.0, 1.0, n)
        d = t * length
        keep = (d >= ignore_a) & (length - d >= ignore_b)
        if not keep.any():
            return False
        t = t[keep]
        xs = np.floor((a.x + (b.x - a.x) * t) / self.cell_size).astype(int)
        ys = np.floor((a.y + (b.y - a.y) * t) / self.cell_size).astype(int)
        inside = (xs >= 0) & (xs < self.width) & (ys >= 0) & (ys < self.height)
        if not inside.all():
            return True
        return bool(self.blocked[ys, xs].any())


def plan_grid(belief: BeliefMap, agent_radius: float = 0.35, resolution: float = 0.5) -> PlanGrid:
    """Planning grid for `belief`, cached on the belief until its generation changes."""
    key = ("plan_grid", agent_radius, resolution)
    hit = belief.derived.get(key)
    if hit is not None and hit[0] == belief.generation:
        return hit[1]
    if belief.kind == "continuous":
        grid = _continuous_grid(belief, agent_radius, resolution)
    else:
        grid = _grid_grid(belief)
    belief.derived[key] = (belief.generation, grid)
    return grid


def _grid_grid(belief: BeliefMap) -> PlanGrid:
    blocked = belief.occupancy == BLOCKED
    if not belief.freespace_unknown:
        blocked = blocked | (belief.occupancy == UNKNOWN)
    return PlanGrid(blocked=np.asarray(blocked, dtype=bool), cell_size=1.0)


def _continuous_grid(belief: BeliefMap, agent_radius: float, resolution: float) -> PlanGrid:
    # The belief raster is one cell per world unit, so its size is the world size.
    w = max(1, int(round(belief.width / resolution)))
    h = max(1, int(round(belief.height / resolution)))
    blocked = np.zeros((h, w), dtype=bool)
    if not belief.freespace_unknown:
        f = max(1, int(round(1.0 / resolution)))
        unknown = np.repeat(np.repeat(belief.occupancy == UNKNOWN, f, axis=0), f, axis=1)
        if unknown.shape == blocked.shape:
            blocked |= unknown
    for rec in belief.obstacles.values():
        if len(rec.vertices) < 3:
            continue
        poly = Polygon([(v.x, v.y) for v in rec.vertices])
        if not poly.is_valid:
            poly = poly.buffer(0)
        if agent_radius > 0:
            poly = poly.buffer(agent_radius, quad_segs=4)
        if poly.is_empty:
            continue
        minx, miny, maxx, maxy = poly.bounds
        x0 = max(0, int(math.floor(minx / resolution)))
        x1 = min(w - 1, int(math.floor(maxx / resolution)))
        y0 = max(0, int(math.floor(miny / resolution)))
        y1 = min(h - 1, int(math.floor(maxy / resolution)))
        if x1 < x0 or y1 < y0:
            continue
        xs = (np.arange(x0, x1 + 1) + 0.5) * resolution
        ys = (np.arange(y0, y1 + 1) + 0.5) * resolution
        gx, gy = np.meshgrid(xs, ys)
        inside = shapely.contains_xy(poly, gx, gy)
        blocked[y0 : y1 + 1, x0 : x1 + 1] |= inside
    return PlanGrid(blocked=blocked, cell_size=resolution)
