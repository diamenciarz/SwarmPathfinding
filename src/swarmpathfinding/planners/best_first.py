"""The A* family on an 8-connected grid: Dijkstra, A*, greedy best-first and
weighted A*. One search, different weights of g and h in f."""

from __future__ import annotations

import heapq
import math
import time
from typing import Any, Sequence

import numpy as np

from swarmpathfinding.core.belief import BeliefMap
from swarmpathfinding.core.capabilities import Fit, Rating, adapted, designed
from swarmpathfinding.core.geom import Cell, Vec2
from swarmpathfinding.core.plan_grid import plan_grid
from swarmpathfinding.core.protocols import BasePathfinder
from swarmpathfinding.core.registry import register
from swarmpathfinding.core.results import Budget, PathResult
from swarmpathfinding.core.trace import EXPAND, PUSH, SearchTrace

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


class GridBestFirst(BasePathfinder):
    """Shared plumbing for the A* family: plans on the last-seen map
    (continuous worlds on an inflated raster) with `best_first_search`.
    Subclasses only choose the weights of g and h."""

    uses_weight = False
    records_trace = True  # fills Budget.record_trace requests (step-through replay)
    # Continuous worlds: unblock the start cell when inflation swallowed it. Planners
    # that share work between agents (flow fields) keep one map and step out instead.
    frees_start = True
    overlays = ("open / closed sets", "current cell", "parent arrows", "g / h / f heatmap", "step-through replay", "cell inspector")
    capabilities = {
        ("world", "grid"): designed(),
        ("world", "continuous"): adapted("searches a 0.5-unit raster with obstacles inflated by the agent radius"),
        ("knowledge", "full"): designed(),
        ("knowledge", "fog"): adapted("unknown cells count as free; replans from scratch when a seen wall blocks the route"),
        ("changes", "static"): designed(),
        ("changes", "slow_edit"): adapted("replans from scratch after the edit"),
        ("changes", "movers"): adapted("moving obstacles are frozen where they were last seen"),
        ("agents", "single"): designed(),
        ("agents", "crowd"): adapted("one search per agent (shared by agents in the same half-cell)"),
    }

    def __init__(self, weight: float = 1.0, agent_radius: float = 0.35, resolution: float = 0.5):
        self.weight = weight
        self.agent_radius = agent_radius
        self.resolution = resolution

    def weights(self, budget: Budget | None) -> tuple[float, float]:
        """(weight of g, weight of h) in f."""
        return 1.0, 1.0

    def debug_extra(self) -> dict:
        """Planner-specific data for its debug overlay (see `draw_overlay`)."""
        extra, self._debug = getattr(self, "_debug", {}), {}
        return extra

    def search(
        self,
        blocked: np.ndarray,
        start: Cell,
        goal: Cell,
        budget: Budget | None,
        max_expansions: int | None,
        trace: SearchTrace | None,
    ) -> tuple[list[Cell] | None, int, np.ndarray]:
        """Cells of the route (consecutive cells may be far apart, the segments
        between them are free), expansions, and the closed mask."""
        gw, hw = self.weights(budget)
        return best_first_search(blocked, start, goal, gw, hw, max_expansions, trace)

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
        grid = plan_grid(belief, self.agent_radius, self.resolution)
        start = starts[0]
        s_cell = grid.to_cell(start)
        g_cell = grid.to_cell(goal)
        blocked = grid.blocked
        if belief.kind == "continuous":
            # Inflation can swallow a start/goal hugging a wall; let the search leave/enter it.
            blocked = blocked.copy()
            if self.frees_start:
                blocked[s_cell.y, s_cell.x] = False
            blocked[g_cell.y, g_cell.x] = False
        elif blocked[g_cell.y, g_cell.x]:
            return PathResult(path=(start,), success=False, runtime_ms=_ms(t0))
        max_exp = budget.max_expansions if budget is not None else None
        trace = SearchTrace(cell_size=grid.cell_size) if budget is not None and budget.record_trace else None
        cells, expansions, closed = self.search(blocked, s_cell, g_cell, budget, max_exp, trace)
        gw, hw = self.weights(budget)
        extra = {"closed": closed, "cell_size": grid.cell_size, "weight": hw, "g_weight": gw}
        extra.update(self.debug_extra())
        if trace is not None:
            extra["trace"] = trace
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




@register("dijkstra")
class DijkstraPlanner(GridBestFirst):
    """Uniform-cost search: f = g. Optimal, explores in rings around the start."""

    label = "Dijkstra"
    summary = "Expands cells in order of distance from the start (f = g, no heuristic). Optimal, but searches in every direction."
    capabilities = {**GridBestFirst.capabilities, ("agents", "single"): Rating(Fit.OK)}

    def weights(self, budget: Budget | None) -> tuple[float, float]:
        return 1.0, 0.0


@register("a_star")
class AStarPlanner(GridBestFirst):
    """A*: f = g + h with the admissible octile heuristic. Optimal."""

    label = "A*"
    summary = "Expands the cell with the lowest f = g + h (octile distance to the goal). Optimal, and aims the search at the goal."


@register("greedy_best_first")
class GreedyBestFirstPlanner(GridBestFirst):
    """Greedy best-first: f = h. Fast, not optimal, can be lured into dead ends."""

    label = "greedy best-first"
    summary = "Always expands the cell that looks closest to the goal (f = h, ignores the cost so far). Very fast in open maps, paths can be long."

    def weights(self, budget: Budget | None) -> tuple[float, float]:
        return 0.0, 1.0


@register("weighted_a_star")
class WeightedAStarPlanner(GridBestFirst):
    """f = g + w * h. w = 1 is A*; w > 1 expands fewer cells and returns a path
    at most w times the optimum."""

    uses_weight = True
    params = (("w", "heuristic weight", 1.0, 3.0),)
    label = "weighted A*"
    summary = "Best-first grid search on f = g + w*h. w = 1 is optimal A*; larger w is faster but up to w times longer."

    def weights(self, budget: Budget | None) -> tuple[float, float]:
        return 1.0, max(1.0, budget.weight if budget is not None else self.weight)


def _ms(t0: float) -> float:
    return (time.perf_counter() - t0) * 1000.0


def best_first_search(
    blocked: np.ndarray,
    start: Cell,
    goal: Cell,
    g_weight: float,
    h_weight: float,
    max_expansions: int | None,
    trace: SearchTrace | None = None,
) -> tuple[list[Cell] | None, int, np.ndarray]:
    """8-connected best-first search ordered by f = g_weight * g + h_weight * h
    (octile h, no corner cutting, closed cells are never reopened).

    (1, 0) Dijkstra, (1, 1) A*, (1, w) weighted A*, (0, 1) greedy best-first.
    Ties on f go to the larger g (deeper first), then to insertion order."""
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

    def octile(x: int, y: int) -> float:
        dx = abs(x - gx)
        dy = abs(y - gy)
        return max(dx, dy) + k * min(dx, dy)

    g[s] = 0.0
    h0 = octile(start.x, start.y)
    heap = [(h_weight * h0, 0.0, 0, s)]
    counter = 1
    if trace is not None:
        trace.add(PUSH, start.x, start.y, 0.0, heap[0][0], h0)
    expansions = 0
    found = False
    while heap:
        fc, neg_g, _, cur = heapq.heappop(heap)
        gc = g[cur]
        if closed[cur]:
            continue
        closed[cur] = 1
        expansions += 1
        if trace is not None:
            trace.add(EXPAND, cur % width, cur // width, gc, fc, octile(cur % width, cur // width), *_xy(parent[cur], width))
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
                hn = octile(nx, ny)
                fn = g_weight * ng + h_weight * hn
                heapq.heappush(heap, (fn, -ng, counter, nb))
                counter += 1
                if trace is not None:
                    trace.add(PUSH, nx, ny, ng, fn, hn, cx, cy)
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


def _xy(index: int, width: int) -> tuple[int, int]:
    if index < 0:
        return -1, -1
    return index % width, index // width
