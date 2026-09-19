from __future__ import annotations

from swarmpathfinding.core.geom import Vec2


def cluster_positions(positions: list[Vec2], radius: float) -> list[list[int]]:
    """Union-find clusters: i and j merge if |pi - pj| <= radius."""
    n = len(positions)
    parent = list(range(n))

    def find(i: int) -> int:
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    def union(a: int, b: int) -> None:
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[rb] = ra

    r2 = radius * radius
    for i in range(n):
        for j in range(i + 1, n):
            if positions[i].dist_sq(positions[j]) <= r2:
                union(i, j)

    buckets: dict[int, list[int]] = {}
    for i in range(n):
        buckets.setdefault(find(i), []).append(i)
    return list(buckets.values())


def centroid(positions: list[Vec2]) -> Vec2:
    if not positions:
        return Vec2(0.0, 0.0)
    sx = sum(p.x for p in positions)
    sy = sum(p.y for p in positions)
    inv = 1.0 / len(positions)
    return Vec2(sx * inv, sy * inv)
