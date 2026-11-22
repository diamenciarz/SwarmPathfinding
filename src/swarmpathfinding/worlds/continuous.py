from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from shapely import make_valid
from shapely.errors import GEOSException
from shapely.geometry import LineString, Point, Polygon, box
from shapely.ops import unary_union

from swarmpathfinding.core.geom import Vec2


@dataclass(slots=True)
class PolygonObstacle:
    id: str
    polygon: Polygon
    velocity: Vec2 = field(default_factory=lambda: Vec2(0.0, 0.0))

    @property
    def position(self) -> Vec2:
        x, y = self.polygon.centroid.coords[0]
        return Vec2(x, y)

    def translated(self, delta: Vec2) -> PolygonObstacle:
        moved = _translate(self.polygon, delta)
        return PolygonObstacle(id=self.id, polygon=moved, velocity=self.velocity)

    def vertices(self) -> tuple[Vec2, ...]:
        coords = list(self.polygon.exterior.coords)[:-1]
        return tuple(Vec2(x, y) for x, y in coords)


@dataclass(slots=True)
class ContinuousWorld:
    width: float
    height: float
    obstacles: list[PolygonObstacle]
    generation: int = 0
    _union: Polygon | None = field(default=None, repr=False)

    @property
    def world_size(self) -> Vec2:
        return Vec2(self.width, self.height)

    @property
    def bounds(self) -> Polygon:
        return box(0.0, 0.0, self.width, self.height)

    def obstacle_union(self) -> Polygon:
        if self._union is None:
            self._union = _union_polygons(obs.polygon for obs in self.obstacles)
        return self._union

    def occupancy_fraction(self) -> float:
        try:
            area = float(self.obstacle_union().area)
        except (GEOSException, ValueError):
            area = sum(float(obs.polygon.area) for obs in self.obstacles if not obs.polygon.is_empty)
        denom = float(self.width * self.height)
        return area / denom if denom else 0.0

    def invalidate(self) -> None:
        self._union = None
        self.generation += 1

    def blocked(self, point: Vec2, radius: float = 0.0) -> bool:
        if point.x < 0 or point.y < 0 or point.x > self.width or point.y > self.height:
            return True
        geom: Point | Polygon = Point(point.x, point.y)
        if radius > 0:
            geom = geom.buffer(radius)
        return bool(geom.intersects(self.obstacle_union()))

    def line_of_sight(self, a: Vec2, b: Vec2, radius: float = 0.0) -> bool:
        line = LineString([(a.x, a.y), (b.x, b.y)])
        if radius > 0:
            line = line.buffer(radius)
        return not bool(line.intersects(self.obstacle_union()))

    def sample_free(self, rng: np.random.Generator, radius: float = 0.0, margin: float = 1.0) -> Vec2:
        for _ in range(10_000):
            x = float(rng.uniform(margin, self.width - margin))
            y = float(rng.uniform(margin, self.height - margin))
            point = Vec2(x, y)
            if not self.blocked(point, radius=radius):
                return point
        raise RuntimeError("no free sample found")

    def add_obstacle(self, obstacle: PolygonObstacle) -> None:
        self.obstacles.append(obstacle)
        self.invalidate()

    def remove_obstacle(self, obstacle_id: str) -> bool:
        remaining = [obs for obs in self.obstacles if obs.id != obstacle_id]
        if len(remaining) == len(self.obstacles):
            return False
        self.obstacles = remaining
        self.invalidate()
        return True

    def step_movers(self, dt: float) -> None:
        moved = False
        new_list: list[PolygonObstacle] = []
        for obs in self.obstacles:
            if obs.velocity.length_sq() == 0.0:
                new_list.append(obs)
                continue
            delta = obs.velocity * dt
            nxt = obs.translated(delta)
            # bounce on world bounds
            minx, miny, maxx, maxy = nxt.polygon.bounds
            vx, vy = nxt.velocity.x, nxt.velocity.y
            if minx < 0 or maxx > self.width:
                vx = -vx
            if miny < 0 or maxy > self.height:
                vy = -vy
            if vx != obs.velocity.x or vy != obs.velocity.y:
                nxt = PolygonObstacle(obs.id, obs.polygon, Vec2(vx, vy))
            new_list.append(nxt)
            moved = True
        if moved:
            self.obstacles = new_list
            self.invalidate()


def _translate(polygon: Polygon, delta: Vec2) -> Polygon:
    coords = [(x + delta.x, y + delta.y) for x, y in polygon.exterior.coords]
    return Polygon(coords)


def as_valid_polygon(geom) -> Polygon | None:
    """Repair or reject a shapely geometry so GEOS unions cannot blow up."""
    if geom is None or geom.is_empty:
        return None
    if not geom.is_valid:
        try:
            geom = make_valid(geom)
        except (GEOSException, ValueError):
            try:
                geom = geom.buffer(0)
            except (GEOSException, ValueError):
                return None
    if geom.geom_type == "MultiPolygon":
        geoms = [g for g in geom.geoms if not g.is_empty]
        if not geoms:
            return None
        geom = max(geoms, key=lambda g: g.area)
    if geom.geom_type == "GeometryCollection":
        polys = [g for g in geom.geoms if g.geom_type in ("Polygon", "MultiPolygon")]
        if not polys:
            return None
        return as_valid_polygon(unary_union(polys))
    if geom.geom_type != "Polygon" or geom.is_empty or geom.area < 1e-8:
        return None
    if not geom.is_valid:
        try:
            geom = geom.buffer(0)
        except (GEOSException, ValueError):
            return None
        if geom.geom_type != "Polygon" or geom.is_empty:
            return None
    return geom


def _union_polygons(geoms) -> Polygon:
    cleaned = []
    for geom in geoms:
        valid = as_valid_polygon(geom)
        if valid is not None:
            cleaned.append(valid)
    if not cleaned:
        return Polygon()
    try:
        merged = unary_union(cleaned)
    except GEOSException:
        merged = Polygon()
        for geom in cleaned:
            try:
                merged = merged.union(geom.buffer(0))
            except GEOSException:
                continue
    # Keep every piece: obstacles that do not touch merge into a MultiPolygon.
    if merged.is_empty:
        return Polygon()
    if not merged.is_valid:
        try:
            merged = make_valid(merged)
        except (GEOSException, ValueError):
            return Polygon()
    return merged
