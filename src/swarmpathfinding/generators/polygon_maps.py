from __future__ import annotations

import math

import numpy as np
from shapely.affinity import rotate, translate
from shapely.errors import GEOSException
from shapely.geometry import MultiPoint, Polygon, box

from swarmpathfinding.core.geom import Vec2
from swarmpathfinding.worlds.continuous import ContinuousWorld, PolygonObstacle, as_valid_polygon


def random_polygons(
    width: float = 48.0,
    height: float = 36.0,
    occupancy: float = 0.10,
    n: int | None = None,
    rng: np.random.Generator | None = None,
    sides: int = 5,
    radius_range: tuple[float, float] = (1.2, 4.0),
) -> ContinuousWorld:
    """Keep adding non-overlapping convex k-gons until occupancy ρ is met (or n cap)."""
    rng = rng or np.random.default_rng()
    max_n = n if n is not None else 200
    world = ContinuousWorld(width=width, height=height, obstacles=[])
    bounds = box(1.0, 1.0, width - 1.0, height - 1.0)
    map_area = width * height
    occupied = 0.0
    attempts = 0
    while occupied / map_area < occupancy - 0.01 and len(world.obstacles) < max_n:
        attempts += 1
        if attempts > max_n * 60:
            break
        rlo, rhi = radius_range
        poly = _random_convex(rng, sides=sides, radius=float(rng.uniform(rlo, rhi)))
        x = float(rng.uniform(3, width - 3))
        y = float(rng.uniform(3, height - 3))
        poly = translate(poly, xoff=x, yoff=y)
        poly = rotate(poly, rng.uniform(0, 360), origin="centroid")
        poly = as_valid_polygon(poly)
        if poly is None or not bounds.contains(poly):
            continue
        if _overlaps_any(poly, world.obstacles):
            continue
        if occupied + poly.area > (occupancy + 0.03) * map_area and world.obstacles:
            continue
        world.obstacles.append(PolygonObstacle(id=f"p{len(world.obstacles)}", polygon=poly))
        occupied += float(poly.area)
    world.generation = 0
    world._union = None
    return world


def sparse_giants(
    width: float = 48.0,
    height: float = 36.0,
    occupancy: float = 0.08,
    rng: np.random.Generator | None = None,
) -> ContinuousWorld:
    return random_polygons(
        width, height, occupancy=occupancy, n=5, rng=rng, sides=6, radius_range=(4.0, 8.0)
    )


def dense_clutter(
    width: float = 48.0,
    height: float = 36.0,
    occupancy: float = 0.35,
    rng: np.random.Generator | None = None,
) -> ContinuousWorld:
    return random_polygons(
        width, height, occupancy=occupancy, n=160, rng=rng, sides=4, radius_range=(0.6, 1.6)
    )


def u_trap(
    width: float = 48.0,
    height: float = 36.0,
) -> ContinuousWorld:
    """A C-shape that eats potential fields. Start left, goal inside the pocket."""
    cx, cy = width * 0.55, height * 0.5
    wall = 1.2
    depth = 10.0
    opening = 8.0
    outer = box(cx - depth, cy - opening, cx + 2.0, cy + opening)
    inner = box(cx - depth - 1.0, cy - opening + wall, cx + 2.0 - wall, cy + opening - wall)
    poly = outer.difference(inner)
    if poly.geom_type == "MultiPolygon":
        poly = max(poly.geoms, key=lambda g: g.area)
    return ContinuousWorld(
        width=width,
        height=height,
        obstacles=[PolygonObstacle(id="utrap", polygon=poly)],
    )


def moving_polygons(
    width: float = 48.0,
    height: float = 36.0,
    n_static: int = 6,
    n_moving: int = 3,
    speed: float = 3.0,
    rng: np.random.Generator | None = None,
) -> ContinuousWorld:
    """Sparse static clutter plus constant-velocity convex movers (bounce on bounds)."""
    rng = rng or np.random.default_rng()
    world = random_polygons(width, height, occupancy=0.08, n=n_static, rng=rng, sides=5)
    bounds = box(2.0, 2.0, width - 2.0, height - 2.0)
    added = 0
    attempts = 0
    while added < n_moving and attempts < 80:
        attempts += 1
        poly = _random_convex(rng, sides=4, radius=float(rng.uniform(1.0, 2.2)))
        x = float(rng.uniform(6, width - 6))
        y = float(rng.uniform(6, height - 6))
        poly = translate(poly, xoff=x, yoff=y)
        poly = as_valid_polygon(poly)
        if poly is None or not bounds.contains(poly):
            continue
        if _overlaps_any(poly, world.obstacles):
            continue
        angle = float(rng.uniform(0, 2 * math.pi))
        vel = Vec2(math.cos(angle) * speed, math.sin(angle) * speed)
        world.obstacles.append(
            PolygonObstacle(id=f"m{added}", polygon=poly, velocity=vel)
        )
        added += 1
    world.generation = 0
    world._union = None
    return world


def pick_deletable_obstacle(world: ContinuousWorld, rng: np.random.Generator) -> str | None:
    static = [obs.id for obs in world.obstacles if obs.velocity.length_sq() == 0.0]
    if not static:
        return None
    return static[int(rng.integers(0, len(static)))]


def _overlaps_any(poly: Polygon, obstacles: list[PolygonObstacle]) -> bool:
    for existing in obstacles:
        try:
            if poly.intersects(existing.polygon):
                return True
        except GEOSException:
            return True
    return False


def _random_convex(rng: np.random.Generator, sides: int, radius: float) -> Polygon:
    """Convex hull of random disk points — always simple, never self-intersecting."""
    n = max(sides, 3) + 4
    pts = []
    for _ in range(n):
        angle = float(rng.uniform(0.0, 2.0 * math.pi))
        r = radius * math.sqrt(float(rng.uniform(0.05, 1.0)))
        pts.append((r * math.cos(angle), r * math.sin(angle)))
    hull = MultiPoint(pts).convex_hull
    valid = as_valid_polygon(hull)
    if valid is not None:
        return valid
    return Polygon(
        [
            (radius * math.cos(a), radius * math.sin(a))
            for a in np.linspace(0.0, 2.0 * math.pi, sides, endpoint=False)
        ]
    )
