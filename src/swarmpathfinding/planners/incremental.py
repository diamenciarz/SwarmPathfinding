"""Incremental search: LPA* and D* Lite (Koenig & Likhachev).

Both keep their search tree between calls and, when the map changes, repair
only the part of the tree the change affects, instead of searching again.

One engine, `IncrementalSearch`, computes shortest paths from a fixed
`source` towards a `target`, with keys [min(g, rhs) + h(target, s) + km,
min(g, rhs)]. g is the current cost from the source, rhs the one-step
lookahead; a cell is "inconsistent" (queued) while g != rhs.

- LPA*: source = the agent's start, target = the goal. The tree is only
  valid for that start, so when the agent has moved it starts over (this is
  the limitation D* Lite removes).
- D* Lite: source = the goal, target = the agent. The agent may move; `km`
  grows by h(old position, new position) so queued keys stay valid.

The planners keep one engine per agent (`Budget.agent_id`).
"""

from __future__ import annotations

import heapq
import math

import numpy as np

from swarmpathfinding.core.capabilities import adapted, designed
from swarmpathfinding.core.geom import Cell
from swarmpathfinding.core.registry import register
from swarmpathfinding.core.results import Budget
from swarmpathfinding.core.trace import EXPAND, PUSH, UPDATE, SearchTrace
from swarmpathfinding.planners.best_first import SQRT2, GridBestFirst, _MOVES

INF = float("inf")
K = SQRT2 - 1.0
REBUILD_FRACTION = 0.25  # more changed cells than this: start over, it is cheaper


class IncrementalSearch:
    def __init__(self, blocked: np.ndarray, source: Cell, target: Cell) -> None:
        self.h_, self.w = blocked.shape
        self.flat = blocked.ravel().tolist()
        self.blocked = blocked.copy()
        n = self.h_ * self.w
        self.g = [INF] * n
        self.rhs = [INF] * n
        self.queue: list[tuple[float, float, int, int]] = []
        self.key: dict[int, tuple[float, float]] = {}
        self.counter = 0
        self.km = 0.0
        self.source = source.y * self.w + source.x
        self.target = target.y * self.w + target.x
        self.trace: SearchTrace | None = None
        self.rhs[self.source] = 0.0
        self._insert(self.source)

    # ------------------------------------------------------------ helpers

    def _h(self, a: int, b: int) -> float:
        ay, ax = divmod(a, self.w)
        by, bx = divmod(b, self.w)
        dx, dy = abs(ax - bx), abs(ay - by)
        return max(dx, dy) + K * min(dx, dy)

    def _calc_key(self, s: int) -> tuple[float, float]:
        m = min(self.g[s], self.rhs[s])
        return (m + self._h(self.target, s) + self.km, m)

    def _insert(self, s: int) -> None:
        k = self._calc_key(s)
        self.key[s] = k
        heapq.heappush(self.queue, (k[0], k[1], self.counter, s))
        self.counter += 1
        if self.trace is not None:
            self.trace.add(PUSH, s % self.w, s // self.w, self.g[s], k[0], self._h(self.target, s), *self._best_xy(s))

    def _top_key(self) -> tuple[float, float]:
        while self.queue:
            k1, k2, _, s = self.queue[0]
            if self.key.get(s) == (k1, k2):
                return k1, k2
            heapq.heappop(self.queue)  # stale entry
        return INF, INF

    def neighbours(self, s: int):
        """(neighbour, edge cost) for every traversable 8-neighbour edge."""
        flat, w, h = self.flat, self.w, self.h_
        if flat[s]:
            return
        y, x = divmod(s, w)
        for dx, dy, c in _MOVES:
            nx, ny = x + dx, y + dy
            if not (0 <= nx < w and 0 <= ny < h):
                continue
            nb = ny * w + nx
            if flat[nb]:
                continue
            if dx and dy and (flat[y * w + nx] or flat[ny * w + x]):
                continue
            yield nb, c

    def _best_xy(self, s: int) -> tuple[int, int]:
        """The neighbour s takes its rhs from (its parent towards the source)."""
        best, best_v = -1, INF
        for nb, c in self.neighbours(s):
            v = self.g[nb] + c
            if v < best_v:
                best, best_v = nb, v
        return (best % self.w, best // self.w) if best >= 0 else (-1, -1)

    def update_vertex(self, s: int) -> None:
        if s != self.source:
            best = INF
            for nb, c in self.neighbours(s):
                v = self.g[nb] + c
                if v < best:
                    best = v
            if best != self.rhs[s]:
                self.rhs[s] = best
                if self.trace is not None:
                    self.trace.add(UPDATE, s % self.w, s // self.w, best, INF if best == INF else best, math.nan)
        self.key.pop(s, None)
        if self.g[s] != self.rhs[s]:
            self._insert(s)

    # ------------------------------------------------------------ main loop

    def compute(self, max_expansions: int | None = None) -> int:
        expansions = 0
        t = self.target
        while True:
            top = self._top_key()
            if not (top < self._calc_key(t) or self.rhs[t] != self.g[t]):
                break
            if top == (INF, INF):
                break
            k1, k2, _, u = heapq.heappop(self.queue)
            self.key.pop(u, None)
            k_new = self._calc_key(u)
            if (k1, k2) < k_new:
                self._insert(u)
                continue
            expansions += 1
            if self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                if self.trace is not None:
                    self.trace.add(EXPAND, u % self.w, u // self.w, self.g[u], k_new[0], self._h(t, u), *self._best_xy(u))
                for nb, _ in self.neighbours(u):
                    self.update_vertex(nb)
            else:
                self.g[u] = INF
                if self.trace is not None:
                    self.trace.add(EXPAND, u % self.w, u // self.w, INF, k_new[0], self._h(t, u))
                self.update_vertex(u)
                for nb, _ in self.neighbours(u):
                    self.update_vertex(nb)
            if max_expansions is not None and expansions >= max_expansions:
                break
        return expansions

    def move_target(self, target: Cell) -> None:
        """D* Lite: the agent (target) moved; keep queued keys valid via km."""
        t = target.y * self.w + target.x
        if t != self.target:
            self.km += self._h(self.target, t)
            self.target = t

    def apply_changes(self, blocked: np.ndarray) -> int:
        """Update edge costs around cells whose blocked state changed. Returns how many."""
        changed = np.argwhere(blocked != self.blocked)
        if not len(changed):
            return 0
        self.blocked = blocked.copy()
        self.flat = blocked.ravel().tolist()
        touched: set[int] = set()
        w, h = self.w, self.h_
        for y, x in changed:
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < w and 0 <= ny < h:
                        touched.add(ny * w + nx)
        for s in touched:
            if self.flat[s]:
                # a blocked cell has no edges: its costs become unknown
                self.rhs[s] = INF if s != self.source else 0.0
                self.key.pop(s, None)
                if self.g[s] != self.rhs[s]:
                    self._insert(s)
            else:
                self.update_vertex(s)
        return len(changed)

    def path_from_target(self) -> list[Cell] | None:
        """Greedy descent of g from the target to the source."""
        if self.g[self.target] == INF:
            return None
        cur = self.target
        out = [Cell(cur % self.w, cur // self.w)]
        seen = {cur}
        while cur != self.source:
            best, best_v = -1, INF
            for nb, c in self.neighbours(cur):
                v = self.g[nb] + c
                if v < best_v:
                    best, best_v = nb, v
            if best < 0 or best in seen or best_v == INF:
                return None
            cur = best
            seen.add(cur)
            out.append(Cell(cur % self.w, cur // self.w))
        return out

    def g_snapshot(self) -> np.ndarray:
        arr = np.array(self.g, dtype=np.float32).reshape(self.h_, self.w)
        arr[np.isinf(arr)] = np.nan
        return arr

    def closed_mask(self) -> np.ndarray:
        """Cells with a settled (finite, consistent) cost: the retained tree."""
        g = np.array(self.g).reshape(self.h_, self.w)
        rhs = np.array(self.rhs).reshape(self.h_, self.w)
        return np.isfinite(g) & (g == rhs)


class _IncrementalPlanner(GridBestFirst):
    """Shared per-agent bookkeeping for LPA* and D* Lite."""

    incremental = True  # the simulation passes agent ids and does not share results
    moving_target = False

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.engines: dict[object, IncrementalSearch] = {}
        self.last_reused = False
        self.last_changed = 0

    def _engine(self, blocked, start: Cell, goal: Cell, budget: Budget | None) -> tuple[IncrementalSearch, bool]:
        aid = budget.agent_id if budget is not None else None
        eng = self.engines.get(aid)
        source, target = (goal, start) if self.moving_target else (start, goal)
        fresh = (
            eng is None
            or eng.blocked.shape != blocked.shape
            or eng.source != source.y * eng.w + source.x
            or (not self.moving_target and eng.target != target.y * eng.w + target.x)
            or np.count_nonzero(blocked != eng.blocked) > REBUILD_FRACTION * blocked.size
        )
        if fresh:
            eng = IncrementalSearch(blocked, source, target)
            if aid is not None:
                self.engines[aid] = eng
        return eng, not fresh

    def search(self, blocked, start, goal, budget: Budget | None, max_expansions, trace):
        eng, reused = self._engine(blocked, start, goal, budget)
        if trace is not None:
            trace.base_g = eng.g_snapshot() if reused else None
        eng.trace = trace
        if reused:
            if self.moving_target:
                eng.move_target(start)
            self.last_changed = eng.apply_changes(blocked)
        else:
            self.last_changed = 0
        self.last_reused = reused
        expansions = eng.compute(max_expansions)
        eng.trace = None
        cells = eng.path_from_target()
        if cells is not None and not self.moving_target:
            cells.reverse()
        return cells, expansions, eng.closed_mask()

    def plan(self, belief, starts, goal, budget=None):
        result = super().plan(belief, starts, goal, budget)
        result.extra["reused"] = self.last_reused
        result.extra["changed_cells"] = self.last_changed
        return result


_INCREMENTAL_CAPS = {
    **GridBestFirst.capabilities,
    ("knowledge", "fog"): designed("repairs its tree as the swarm discovers walls; unknown cells count as free"),
    ("changes", "slow_edit"): designed("repairs only the part of the tree the edit touched"),
}


@register("lpa_star")
class LPAStarPlanner(_IncrementalPlanner):
    label = "LPA*"
    summary = (
        "Lifelong Planning A*: keeps its search between calls and repairs only cells whose cost changed. "
        "The tree belongs to one start, so it starts over once the agent has moved."
    )
    overlays = ("retained tree (faint) vs repair work", "inconsistent cells (queued)", *GridBestFirst.overlays[1:])
    capabilities = {
        **_INCREMENTAL_CAPS,
        ("knowledge", "fog"): adapted("repairs when the map changes, but restarts whenever the agent has moved since the last search"),
        ("changes", "slow_edit"): designed("repairs only the touched cells while the agent has not moved"),
        ("changes", "movers"): adapted("moving obstacles are frozen where last seen; each sighting forces a repair"),
    }


@register("d_star_lite")
class DStarLitePlanner(_IncrementalPlanner):
    label = "D* Lite"
    moving_target = True
    summary = (
        "Searches backwards from the goal and keeps the tree while the agent moves (key offset km), "
        "so discovering or editing a wall only repairs the affected cells. The classic planner for unknown maps."
    )
    overlays = LPAStarPlanner.overlays
    capabilities = {
        **_INCREMENTAL_CAPS,
        ("changes", "movers"): adapted("moving obstacles are frozen where last seen; each sighting forces a repair"),
        ("agents", "crowd"): adapted("one tree per agent; agents with a shared goal could share one (not yet)"),
    }
