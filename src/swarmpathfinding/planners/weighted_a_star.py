from __future__ import annotations

import heapq
import math
import time
from typing import Any, Sequence

import numpy as np

from swarmpathfinding.core.belief import BeliefMap
from swarmpathfinding.core.geom import Cell, Vec2
from swarmpathfinding.core.plan_grid import plan_grid
from swarmpathfinding.core.protocols import BasePathfinder
from swarmpathfinding.core.registry import register
from swarmpathfinding.core.results import Budget, PathResult

SQRT2 = math.sqrt(2.0)
_MOVES = (
    (1, 0, 1.0),
    (-1, 0, 1.0),
    (0, 1, 1.0),
    (0, -1, 1.0),
    (1, 1, SQRT2),
    (1, -1, SQRT2),
    (-1, 1, SQRT2),
    (-1, -1, SQRT2),
)


@register("weighted_a_star")
class WeightedAStarPlanner(BasePathfinder):
    """8-connected weighted A* on the shared last-seen map.

    f = g + w * h with the octile heuristic. w = 1 is plain, optimal A*.
    w > 1 expands fewer cells and returns a path at most w times the optimum
    (no corner cutting, no re-opening of closed cells).
    Continuous worlds are searched on a rasterised configuration space
    (obstacles inflated by the agent radius), so paths follow cell centres.
    """

    name = "weighted_a_star"
    uses_weight = True

    def __init__(self, weight: float = 1.0, agent_radius: float = 0.35, resolution: float = 0.5):
        self.weight = weight
        self.agent_radius = agent_radius
        self.resolution = resolution

    def plan(
        self,
        belief: Any,
        starts: Sequence[Vec2],
        goal: Vec2,
        budget: Budget | None = None,
    ) -> PathResult:
        if not starts or not isinstance(belief, BeliefMap):
            return PathResult(success=False)
        t0 = time.perf_counter()
        w = max(1.0, budget.weight if budget is not None else self.weight)
        grid = plan_grid(belief, self.agent_radius, self.resolution)
        start = starts[0]
        s_cell = grid.to_cell(start)
        g_cell = grid.to_cell(goal)
        blocked = grid.blocked
        if belief.kind == "continuous":
            # Inflation can swallow a start/goal hugging a wall; let the search leave/enter it.
            blocked = blocked.copy()
            blocked[s_cell.y, s_cell.x] = False
            blocked[g_cell.y, g_cell.x] = False
        elif blocked[g_cell.y, g_cell.x]:
            return PathResult(path=(start,), success=False, runtime_ms=_ms(t0))
        max_exp = budget.max_expansions if budget is not None else None
        cells, expansions, closed = _search(blocked, s_cell, g_cell, w, max_exp)
        extra = {"closed": closed, "cell_size": grid.cell_size, "weight": w}
        if cells is None:
            return PathResult(
                path=(start,), success=False, expansions=expansions, runtime_ms=_ms(t0), extra=extra
            )
        pts = [start] + [grid.center(c) for c in cells[1:-1]] + [goal]
        if len(pts) > 2 and grid.segment_blocked(start, pts[1], ignore_a=grid.cell_size * 0.25):
            # An off-centre start would clip a wall corner on the way to the next
            # cell; go via the centre of the start cell first.
            pts.insert(1, grid.center(s_cell))
        cost = sum(a.dist(b) for a, b in zip(pts, pts[1:]))
        return PathResult(
            path=tuple(pts),
            cost=cost,
            success=True,
            expansions=expansions,
            runtime_ms=_ms(t0),
            extra=extra,
        )


def _ms(t0: float) -> float:
    return (time.perf_counter() - t0) * 1000.0


def _search(
    blocked: np.ndarray, start: Cell, goal: Cell, w: float, max_expansions: int | None
) -> tuple[list[Cell] | None, int, np.ndarray]:
    h, width = blocked.shape
    flat_blocked = blocked.ravel().tolist()
    n = h * width
    inf = float("inf")
    g = [inf] * n
    parent = [-1] * n
    closed = bytearray(n)
    s = start.y * width + start.x
    t = goal.y * width + goal.x
    gx, gy = goal.x, goal.y
    k = SQRT2 - 1.0

    def heur(x: int, y: int) -> float:
        dx = abs(x - gx)
        dy = abs(y - gy)
        return w * (max(dx, dy) + k * min(dx, dy))

    g[s] = 0.0
    heap = [(heur(start.x, start.y), 0.0, s)]
    expansions = 0
    found = False
    while heap:
        _, gc, cur = heapq.heappop(heap)
        if closed[cur]:
            continue
        closed[cur] = 1
        expansions += 1
        if cur == t:
            found = True
            break
        if max_expansions is not None and expansions >= max_expansions:
            break
        cy, cx = divmod(cur, width)
        for dx, dy, c in _MOVES:
            nx = cx + dx
            ny = cy + dy
            if nx < 0 or ny < 0 or nx >= width or ny >= h:
                continue
            nb = ny * width + nx
            if flat_blocked[nb] or closed[nb]:
                continue
            if dx and dy and (flat_blocked[cy * width + nx] or flat_blocked[ny * width + cx]):
                continue
            ng = gc + c
            if ng < g[nb]:
                g[nb] = ng
                parent[nb] = cur
                heapq.heappush(heap, (ng + heur(nx, ny), ng, nb))
    closed_mask = np.frombuffer(bytes(closed), dtype=np.uint8).reshape(h, width).astype(bool)
    if not found:
        return None, expansions, closed_mask
    out: list[Cell] = []
    cur = t
    while cur != -1:
        cy, cx = divmod(cur, width)
        out.append(Cell(cx, cy))
        cur = parent[cur]
    out.reverse()
    return out, expansions, closed_mask
