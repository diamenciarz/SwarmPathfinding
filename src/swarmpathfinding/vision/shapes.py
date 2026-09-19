from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterator

from shapely.geometry import Point, Polygon

from swarmpathfinding.core.geom import Cell, Vec2
from swarmpathfinding.worlds.grid import GridWorld


@dataclass(slots=True)
class CircleVision:
    radius: float
    name: str = "circle"

    def contains(self, origin: Vec2, heading: float, point: Vec2) -> bool:
        return origin.dist_sq(point) <= self.radius * self.radius

    def footprint_polygon(self, origin: Vec2, heading: float) -> Polygon:
        return Point(origin.x, origin.y).buffer(self.radius, resolution=16)

    def visible_cells(
        self,
        origin: Vec2,
        heading: float,
        world: GridWorld,
        require_los: bool = True,
    ) -> Iterator[Cell]:
        origin_cell = world.world_to_cell(origin)
        reach = int(math.ceil(self.radius / world.cell_size)) + 1
        r2 = self.radius * self.radius
        for dy in range(-reach, reach + 1):
            for dx in range(-reach, reach + 1):
                cell = Cell(origin_cell.x + dx, origin_cell.y + dy)
                if not world.in_bounds(cell):
                    continue
                center = world.cell_center(cell)
                if origin.dist_sq(center) > r2:
                    continue
                if require_los and not world.line_of_sight(origin_cell, cell):
                    continue
                yield cell


@dataclass(slots=True)
class ConeVision:
    radius: float
    half_angle: float  # radians
    name: str = "cone"

    def contains(self, origin: Vec2, heading: float, point: Vec2) -> bool:
        delta = point - origin
        if delta.length_sq() > self.radius * self.radius:
            return False
        if delta.length_sq() == 0.0:
            return True
        angle = math.atan2(delta.y, delta.x)
        diff = _angle_diff(angle, heading)
        return abs(diff) <= self.half_angle

    def footprint_polygon(self, origin: Vec2, heading: float) -> Polygon:
        steps = 12
        points = [(origin.x, origin.y)]
        start = heading - self.half_angle
        end = heading + self.half_angle
        for i in range(steps + 1):
            t = start + (end - start) * (i / steps)
            points.append(
                (
                    origin.x + math.cos(t) * self.radius,
                    origin.y + math.sin(t) * self.radius,
                )
            )
        return Polygon(points)

    def visible_cells(
        self,
        origin: Vec2,
        heading: float,
        world: GridWorld,
        require_los: bool = True,
    ) -> Iterator[Cell]:
        origin_cell = world.world_to_cell(origin)
        reach = int(math.ceil(self.radius / world.cell_size)) + 1
        for dy in range(-reach, reach + 1):
            for dx in range(-reach, reach + 1):
                cell = Cell(origin_cell.x + dx, origin_cell.y + dy)
                if not world.in_bounds(cell):
                    continue
                center = world.cell_center(cell)
                if not self.contains(origin, heading, center):
                    continue
                if require_los and not world.line_of_sight(origin_cell, cell):
                    continue
                yield cell


class NoVision:
    name = "none"
    radius = 0.0

    def contains(self, origin: Vec2, heading: float, point: Vec2) -> bool:
        return False

    def footprint_polygon(self, origin: Vec2, heading: float) -> Polygon:
        return Point(origin.x, origin.y).buffer(0.01)

    def visible_cells(
        self,
        origin: Vec2,
        heading: float,
        world: GridWorld,
        require_los: bool = True,
    ) -> Iterator[Cell]:
        return iter(())


def make_vision(kind: str, radius: float = 6.0, half_angle_deg: float = 40.0):
    if kind == "circle":
        return CircleVision(radius=radius)
    if kind == "cone":
        return ConeVision(radius=radius, half_angle=math.radians(half_angle_deg))
    return NoVision()


def _angle_diff(a: float, b: float) -> float:
    diff = (a - b + math.pi) % (2 * math.pi) - math.pi
    return diff
