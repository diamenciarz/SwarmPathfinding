"""Visibility graph: shortest paths around polygons by corner-hopping.

Every shortest path among polygonal obstacles bends only at obstacle corners,
so the graph's nodes are the (convex) corners of the obstacles, grown by the
agent radius, plus the start and goal. Two nodes are joined when the straight
segment between them clears every obstacle. A* over that graph gives the
exact shortest path for the known map, with very few nodes.

The graph is built lazily: a node's visible neighbours are worked out only
when A* expands it, and are cached per map version, so agents planning on the
same map share them. Grid worlds are turned into polygons by merging the
blocked cells.
"""

from __future__ import annotations

import heapq
import math
import time
from typing import Any, Sequence

import numpy as np
import shapely
import shapely.geometry.polygon
from shapely.geometry import Polygon, box
from shapely.ops import unary_union

from swarmpathfinding.core.belief import BLOCKED, BeliefMap
from swarmpathfinding.core.capabilities import adapted, designed
from swarmpathfinding.core.geom import Vec2
from swarmpathfinding.core.protocols import BasePathfinder, EmptyDebugLayer
from swarmpathfinding.core.registry import register
from swarmpathfinding.core.results import Budget, PathResult

CLEAR = 0.03  # extra clearance beyond the agent radius for collision tests
CORNER = 0.08  # corners sit this far beyond the radius, so they are just outside


class VisGeometry:
    """Obstacles, corner nodes and a cache of who-sees-whom for one map version."""

    def __init__(self, polys: list, width: float, height: float, radius: float):
        self.width, self.height = width, height
        grown = [p.buffer(radius + CLEAR, quad_segs=4) for p in polys if not p.is_empty]
        self.blocker = unary_union(grown) if grown else Polygon()
        # the tree holds the separate small polygons, not the union: intersecting a
        # line with many small shapes is far cheaper than with one huge one
        self.parts = [p for p in grown if not p.is_empty]
        shapely.prepare(self.parts)
        self.tree = shapely.STRtree(self.parts) if self.parts else None
        nodes = []
        for p in polys:
            if p.is_empty:
                continue
            ring = p.buffer(radius + CORNER, join_style="mitre", mitre_limit=1.6)
            for part in getattr(ring, "geoms", [ring]):
                nodes.extend(_convex_corners(part))
        arr = np.array(nodes, dtype=float).reshape(-1, 6)  # corner, previous corner, next corner
        if len(arr):
            pts = arr[:, :2]
            keep = (pts[:, 0] > 0) & (pts[:, 1] > 0) & (pts[:, 0] < width) & (pts[:, 1] < height)
            arr = arr[keep]
        if len(arr) and not self.blocker.is_empty:
            arr = arr[~shapely.contains_xy(self.blocker, arr[:, 0], arr[:, 1])]
        self.nodes = arr[:, :2].copy()
        self.prev = arr[:, 2:4].copy()
        self.next = arr[:, 4:6].copy()
        self.seen: dict[int, np.ndarray] = {}  # node -> visible node indices (corners only)

    def visible(self, a: np.ndarray, targets: np.ndarray, skip_a: float = 0.0, skip_b: float = 0.0) -> np.ndarray:
        """Which segments a -> targets[i] clear the obstacles. `skip_a` / `skip_b`
        ignore the first / last stretch (an agent or goal inside the clearance)."""
        if not len(targets):
            return np.zeros(0, dtype=bool)
        if self.tree is None:
            return np.ones(len(targets), dtype=bool)
        starts = np.repeat(a.reshape(1, 2), len(targets), axis=0)
        if skip_a > 0:
            d = targets - starts
            n = np.linalg.norm(d, axis=1, keepdims=True)
            starts = starts + d * np.minimum(1.0, skip_a / np.maximum(n, 1e-9))
        if skip_b > 0:
            d = starts - targets
            n = np.linalg.norm(d, axis=1, keepdims=True)
            targets = targets + d * np.minimum(1.0, skip_b / np.maximum(n, 1e-9))
        coords = np.stack([starts, targets], axis=1)
        lines = shapely.linestrings(coords)
        hit_lines, _ = self.tree.query(lines, predicate="intersects")
        ok = np.ones(len(targets), dtype=bool)
        ok[np.unique(hit_lines)] = False
        return ok

    def tangent(self, i: int | None, a: np.ndarray, others: np.ndarray, idx: np.ndarray) -> np.ndarray:
        """Bitangent test: a shortest path only leaves corner a (index i) along a
        line that keeps both of a's neighbouring corners on one side, and only
        arrives at corner j the same way. Prunes most corner pairs before any
        geometry test."""
        d = others - a
        ok = np.ones(len(others), dtype=bool)
        if i is not None:
            c1 = d[:, 0] * (self.prev[i, 1] - a[1]) - d[:, 1] * (self.prev[i, 0] - a[0])
            c2 = d[:, 0] * (self.next[i, 1] - a[1]) - d[:, 1] * (self.next[i, 0] - a[0])
            ok &= c1 * c2 >= -1e-9
        pj, nj = self.prev[idx], self.next[idx]
        e = a - others  # direction from each corner j back to a
        c1 = e[:, 0] * (pj[:, 1] - others[:, 1]) - e[:, 1] * (pj[:, 0] - others[:, 0])
        c2 = e[:, 0] * (nj[:, 1] - others[:, 1]) - e[:, 1] * (nj[:, 0] - others[:, 0])
        ok &= c1 * c2 >= -1e-9
        return ok

    def inside(self, p: np.ndarray) -> bool:
        return bool(not self.blocker.is_empty and shapely.contains_xy(self.blocker, p[0], p[1]))


def _convex_corners(poly) -> list[tuple[float, ...]]:
    """Corners where the obstacle is convex, on the outer ring and on the rings
    of any holes (free pockets inside a wall mass). After orienting (outer ring
    counter-clockwise, holes clockwise) the obstacle is always on the left while
    walking a ring, so a convex corner is a left turn on every ring."""
    poly = shapely.geometry.polygon.orient(poly, 1.0)
    out = []
    for ring in (poly.exterior, *poly.interiors):
        coords = np.asarray(ring.coords)[:-1]
        n = len(coords)
        if n < 3:
            continue
        for i in range(n):
            a, b, c = coords[i - 1], coords[i], coords[(i + 1) % n]
            cross = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])
            if cross > 1e-9:
                out.append((float(b[0]), float(b[1]), float(a[0]), float(a[1]), float(c[0]), float(c[1])))
    return out


def _belief_polygons(belief: BeliefMap) -> tuple[list, float, float]:
    if belief.kind == "continuous":
        polys = []
        for rec in belief.obstacles.values():
            if len(rec.vertices) >= 3:
                p = Polygon([(v.x, v.y) for v in rec.vertices])
                polys.append(p if p.is_valid else p.buffer(0))
        return polys, float(belief.width), float(belief.height)
    ys, xs = np.nonzero(belief.occupancy == BLOCKED)
    cells = [box(x, y, x + 1, y + 1) for x, y in zip(xs.tolist(), ys.tolist())]
    merged = unary_union(cells) if cells else Polygon()
    polys = list(getattr(merged, "geoms", [merged])) if not merged.is_empty else []
    return polys, float(belief.width), float(belief.height)


def vis_geometry(belief: BeliefMap, radius: float) -> VisGeometry:
    key = ("visgraph", radius)
    hit = belief.derived.get(key)
    if hit is not None and hit[0] == belief.generation:
        return hit[1]
    polys, w, h = _belief_polygons(belief)
    geo = VisGeometry(polys, w, h, radius)
    belief.derived[key] = (belief.generation, geo)
    return geo


@register("visibility_graph")
class VisibilityGraphPlanner(BasePathfinder):
    name = "visibility_graph"
    label = "visibility graph"
    summary = (
        "A* over obstacle corners: the nodes are the convex corners of the (radius-grown) obstacles, and two "
        "nodes are linked when they can see each other. Exact shortest paths among polygons, from a few hundred nodes."
    )
    overlays = ("corner nodes", "visibility edges A* looked at", "expanded corners")
    frees_start = False
    capabilities = {
        ("world", "continuous"): designed("shortest paths among polygons bend only at their corners"),
        ("world", "grid"): adapted("merges blocked cells into polygons first; every cell corner becomes a node"),
        ("knowledge", "fog"): adapted("only remembered polygons are obstacles; the graph is rebuilt when the map changes"),
        ("changes", "slow_edit"): adapted("rebuilds the corner set after the edit"),
        ("changes", "movers"): adapted("moving obstacles are frozen where they were last seen"),
        ("agents", "crowd"): adapted("one search per agent, but agents share the corner visibility cache"),
    }

    def __init__(self, agent_radius: float = 0.35):
        self.agent_radius = agent_radius

    def debug_layer(self):
        return EmptyDebugLayer()

    def plan(self, belief: Any, starts: Sequence[Vec2], goal: Vec2, budget: Budget | None = None) -> PathResult:
        if not starts or not isinstance(belief, BeliefMap):
            return PathResult(success=False)
        t0 = time.perf_counter()
        start = starts[0]
        radius = self.agent_radius if belief.kind == "continuous" else 0.0
        geo = vis_geometry(belief, radius)
        s = np.array([start.x, start.y])
        g = np.array([goal.x, goal.y])
        nodes = geo.nodes
        n = len(nodes)
        S, G = n, n + 1  # indices of start and goal
        allp = np.vstack([nodes, s, g]) if n else np.vstack([s, g])
        skip_s = 0.8 if geo.inside(s) else 0.0
        skip_g = 0.8 if geo.inside(g) else 0.0
        record = budget is not None and budget.record_trace
        # what the goal can see, once (segments are symmetric)
        goal_sees = self._sees(geo, None, g, skip_g)
        gcost = {S: 0.0}
        parent = {S: -1}
        heap = [(float(np.linalg.norm(g - s)), 0.0, S)]
        closed = set()
        looked: list[tuple[int, int]] = []
        expansions = 0
        found = False
        while heap:
            _, gu, u = heapq.heappop(heap)
            if u in closed:
                continue
            closed.add(u)
            expansions += 1
            if u == G:
                found = True
                break
            pu = allp[u]
            if u == S:
                vis = self._sees(geo, None, s, skip_s)
                to_goal = bool(geo.visible(s, g.reshape(1, 2), skip_s, skip_g)[0])
            else:
                vis = geo.seen.get(u)
                if vis is None:
                    vis = self._sees(geo, u, pu, 0.0)
                    vis[u] = False
                    geo.seen[u] = vis
                to_goal = bool(goal_sees[u])
            cand = list(np.nonzero(vis)[0])
            if to_goal:
                cand.append(G)
            for v in cand:
                v = int(v)
                if v in closed:
                    continue
                c = gu + float(np.linalg.norm(allp[v] - pu))
                if c < gcost.get(v, math.inf):
                    gcost[v] = c
                    parent[v] = u
                    heapq.heappush(heap, (c + float(np.linalg.norm(g - allp[v])), c, v))
                if record and len(looked) < 6000:
                    looked.append((u, v))
        extra: dict[str, Any] = {"nodes_total": n}
        if record:
            extra["vg_nodes"] = nodes.copy()
            extra["vg_edges"] = np.array([[*allp[a], *allp[b]] for a, b in looked]).reshape(-1, 4)
            extra["vg_closed"] = np.array([allp[i] for i in closed if i < n]).reshape(-1, 2)
        if not found:
            return PathResult(path=(start,), success=False, expansions=expansions, runtime_ms=(time.perf_counter() - t0) * 1000, extra=extra)
        chain = [G]
        while parent[chain[-1]] != -1:
            chain.append(parent[chain[-1]])
        chain.reverse()
        path = tuple([start] + [Vec2(float(allp[i][0]), float(allp[i][1])) for i in chain[1:-1]] + [goal])
        return PathResult(
            path=path,
            cost=sum(a.dist(b) for a, b in zip(path, path[1:])),
            success=True,
            expansions=expansions,
            runtime_ms=(time.perf_counter() - t0) * 1000,
            extra=extra,
        )

    @staticmethod
    def _sees(geo: VisGeometry, i: int | None, p: np.ndarray, skip: float) -> np.ndarray:
        """Corners visible from p that a shortest path could use."""
        n = len(geo.nodes)
        out = np.zeros(n, dtype=bool)
        if not n:
            return out
        idx = np.arange(n)
        # (a start or goal inside the clearance is a degenerate case: test every corner)
        cand = idx if skip > 0 else idx[geo.tangent(i, p, geo.nodes, idx)]
        if i is not None:
            cand = cand[cand != i]
        if len(cand):
            out[cand[geo.visible(p, geo.nodes[cand], skip)]] = True
        return out

    @staticmethod
    def draw_overlay(surface, camera, result, blocked=None) -> None:
        import pygame

        ex = result.extra
        edges = ex.get("vg_edges")
        if edges is not None and len(edges):
            layer = pygame.Surface(surface.get_size(), pygame.SRCALPHA)
            for x0, y0, x1, y1 in edges:
                pygame.draw.line(layer, (120, 200, 160, 70), camera.local(Vec2(x0, y0)), camera.local(Vec2(x1, y1)), 1)
            surface.blit(layer, (0, 0))
        nodes = ex.get("vg_nodes")
        if nodes is not None:
            for x, y in nodes:
                pygame.draw.circle(surface, (150, 160, 190), camera.local(Vec2(float(x), float(y))), 2)
        closed = ex.get("vg_closed")
        if closed is not None:
            for x, y in closed:
                pygame.draw.circle(surface, (240, 200, 90), camera.local(Vec2(float(x), float(y))), 4, 1)
