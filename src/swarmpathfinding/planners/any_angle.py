"""Jump Point Search and Theta*, both built on the A* family's plumbing.

JPS (Harabor & Grastien 2011), no-corner-cutting variant: on a uniform-cost
grid it skips the long runs of symmetric cells A* would queue, and only
queues "jump points" where something forces a turn. Same optimal paths as A*,
far fewer queue operations. Its scans are recorded as SCAN events.

Theta* (Nash et al. 2007): A* where a cell may take its grandparent as parent
whenever the grandparent can see it, so paths are any-angle (not restricted to
the 8 grid directions). Each line-of-sight test is recorded as a LOS event.
"""

from __future__ import annotations

import heapq
import math

import numpy as np

from swarmpathfinding.core.capabilities import Fit, Rating, adapted
from swarmpathfinding.core.geom import Cell
from swarmpathfinding.core.registry import register
from swarmpathfinding.core.results import Budget
from swarmpathfinding.core.trace import EXPAND, LOS, PUSH, SCAN, SearchTrace
from swarmpathfinding.planners.best_first import SQRT2, GridBestFirst, _MOVES

K = SQRT2 - 1.0


def _octile(x0: int, y0: int, x1: int, y1: int) -> float:
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    return max(dx, dy) + K * min(dx, dy)


def _closed_mask(closed: bytearray, shape) -> np.ndarray:
    return np.frombuffer(bytes(closed), dtype=np.uint8).reshape(shape).astype(bool)


def _walk_back(parent: list[int], t: int, width: int) -> list[Cell]:
    out: list[Cell] = []
    cur = t
    while cur != -1:
        cy, cx = divmod(cur, width)
        out.append(Cell(cx, cy))
        cur = parent[cur]
    out.reverse()
    return out


# ---------------------------------------------------------------- JPS


def jps_search(
    blocked: np.ndarray,
    start: Cell,
    goal: Cell,
    max_expansions: int | None,
    trace: SearchTrace | None = None,
) -> tuple[list[Cell] | None, int, np.ndarray]:
    h, w = blocked.shape
    flat = blocked.ravel().tolist()
    gx, gy = goal.x, goal.y

    def free(x: int, y: int) -> bool:
        return 0 <= x < w and 0 <= y < h and not flat[y * w + x]

    def scan(x: int, y: int) -> None:
        if trace is not None:
            trace.add(SCAN, x, y, 0.0, 0.0)

    def jump_straight(x: int, y: int, dx: int, dy: int) -> tuple[int, int] | None:
        """Walk from (x, y) (already stepped onto) in a straight line."""
        while True:
            if not free(x, y):
                return None
            scan(x, y)
            if x == gx and y == gy:
                return x, y
            if dx:
                if (free(x, y - 1) and not free(x - dx, y - 1)) or (free(x, y + 1) and not free(x - dx, y + 1)):
                    return x, y
            else:
                if (free(x - 1, y) and not free(x - 1, y - dy)) or (free(x + 1, y) and not free(x + 1, y - dy)):
                    return x, y
            x += dx
            y += dy

    def jump(x: int, y: int, dx: int, dy: int) -> tuple[int, int] | None:
        if not dx or not dy:
            return jump_straight(x, y, dx, dy)
        while True:
            if not free(x, y):
                return None
            scan(x, y)
            if x == gx and y == gy:
                return x, y
            if jump_straight(x + dx, y, dx, 0) is not None or jump_straight(x, y + dy, 0, dy) is not None:
                return x, y
            # a diagonal step needs both orthogonal cells free (no corner cutting)
            if not (free(x + dx, y) and free(x, y + dy)):
                return None
            x += dx
            y += dy

    def neighbours(x: int, y: int, px: int, py: int) -> list[tuple[int, int]]:
        if px < 0:
            out = []
            for mx, my, _ in _MOVES:
                if not free(x + mx, y + my):
                    continue
                if mx and my and not (free(x + mx, y) and free(x, y + my)):
                    continue
                out.append((mx, my))
            return out
        dx = (x > px) - (x < px)
        dy = (y > py) - (y < py)
        out = []
        if dx and dy:
            if free(x, y + dy):
                out.append((0, dy))
            if free(x + dx, y):
                out.append((dx, 0))
            if free(x, y + dy) and free(x + dx, y):
                out.append((dx, dy))
        elif dx:
            nxt, up, down = free(x + dx, y), free(x, y + 1), free(x, y - 1)
            if nxt:
                out.append((dx, 0))
                if up:
                    out.append((dx, 1))
                if down:
                    out.append((dx, -1))
            if up:
                out.append((0, 1))
            if down:
                out.append((0, -1))
        else:
            nxt, right, left = free(x, y + dy), free(x + 1, y), free(x - 1, y)
            if nxt:
                out.append((0, dy))
                if right:
                    out.append((1, dy))
                if left:
                    out.append((-1, dy))
            if right:
                out.append((1, 0))
            if left:
                out.append((-1, 0))
        # diagonal moves listed by the straight rules still need both sides free
        return [(mx, my) for mx, my in out if not (mx and my) or (free(x + mx, y) and free(x, y + my))]

    n = h * w
    inf = float("inf")
    g = [inf] * n
    parent = [-1] * n
    closed = bytearray(n)
    s = start.y * w + start.x
    t = goal.y * w + goal.x
    g[s] = 0.0
    h0 = _octile(start.x, start.y, gx, gy)
    heap = [(h0, 0.0, 0, s)]
    counter = 1
    if trace is not None:
        trace.add(PUSH, start.x, start.y, 0.0, h0, h0)
    expansions = 0
    found = False
    while heap:
        _, _, _, cur = heapq.heappop(heap)
        if closed[cur]:
            continue
        closed[cur] = 1
        expansions += 1
        cy, cx = divmod(cur, w)
        gc = g[cur]
        par = parent[cur]
        ppx, ppy = (par % w, par // w) if par >= 0 else (-1, -1)
        if trace is not None:
            trace.add(EXPAND, cx, cy, gc, gc + _octile(cx, cy, gx, gy), _octile(cx, cy, gx, gy), ppx, ppy)
        if cur == t:
            found = True
            break
        if max_expansions is not None and expansions >= max_expansions:
            break
        for mx, my in neighbours(cx, cy, ppx, ppy):
            jp = jump(cx + mx, cy + my, mx, my)
            if jp is None:
                continue
            jx, jy = jp
            nb = jy * w + jx
            if closed[nb]:
                continue
            ng = gc + _octile(cx, cy, jx, jy)
            if ng < g[nb]:
                g[nb] = ng
                parent[nb] = cur
                hn = _octile(jx, jy, gx, gy)
                heapq.heappush(heap, (ng + hn, -ng, counter, nb))
                counter += 1
                if trace is not None:
                    trace.add(PUSH, jx, jy, ng, ng + hn, hn, cx, cy)
    mask = _closed_mask(closed, (h, w))
    if not found:
        return None, expansions, mask
    return _walk_back(parent, t, w), expansions, mask


# ---------------------------------------------------------------- Theta*


def line_of_sight(flat: list, w: int, h: int, x0: int, y0: int, x1: int, y1: int) -> bool:
    """Whether the segment between two cell centres crosses only free cells.
    Every cell the segment touches is checked; where it passes exactly through
    a cell corner, both cells beside the corner must be free (no squeezing)."""

    def blocked(x: int, y: int) -> bool:
        return not (0 <= x < w and 0 <= y < h) or bool(flat[y * w + x])

    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx = 1 if x1 > x0 else -1
    sy = 1 if y1 > y0 else -1
    x, y = x0, y0
    err = dx - dy
    dx2, dy2 = dx * 2, dy * 2
    steps = dx + dy
    while True:
        if blocked(x, y):
            return False
        if steps <= 0:
            return True
        if err > 0:
            x += sx
            err -= dy2
            steps -= 1
        elif err < 0:
            y += sy
            err += dx2
            steps -= 1
        else:
            if blocked(x + sx, y) or blocked(x, y + sy):
                return False
            x += sx
            y += sy
            err += dx2 - dy2
            steps -= 2


def theta_search(
    blocked: np.ndarray,
    start: Cell,
    goal: Cell,
    max_expansions: int | None,
    trace: SearchTrace | None = None,
    lazy: bool = False,
) -> tuple[list[Cell] | None, int, np.ndarray]:
    h, w = blocked.shape
    flat = blocked.ravel().tolist()
    gx, gy = goal.x, goal.y

    def euclid(x: int, y: int, x1: int, y1: int) -> float:
        return math.hypot(x1 - x, y1 - y)

    def sees(a: int, bx: int, by: int) -> bool:
        ax, ay = a % w, a // w
        ok = line_of_sight(flat, w, h, ax, ay, bx, by)
        if trace is not None:
            trace.add(LOS, bx, by, 1.0 if ok else 0.0, 0.0, float("nan"), ax, ay)
        return ok

    n = h * w
    inf = float("inf")
    g = [inf] * n
    parent = [-1] * n
    closed = bytearray(n)
    s = start.y * w + start.x
    t = goal.y * w + goal.x
    g[s] = 0.0
    parent[s] = s
    h0 = euclid(start.x, start.y, gx, gy)
    heap = [(h0, 0.0, 0, s)]
    counter = 1
    if trace is not None:
        trace.add(PUSH, start.x, start.y, 0.0, h0, h0)
    expansions = 0
    found = False
    while heap:
        _, _, _, cur = heapq.heappop(heap)
        if closed[cur]:
            continue
        cy, cx = divmod(cur, w)
        if lazy and parent[cur] != cur and not sees(parent[cur], cx, cy):
            # Lazy Theta*: the optimistic parent was not visible; take the best
            # closed neighbour instead (always visible).
            best, best_g = -1, inf
            for mx, my, c in _MOVES:
                nx, ny = cx + mx, cy + my
                if not (0 <= nx < w and 0 <= ny < h):
                    continue
                nb = ny * w + nx
                if closed[nb] and g[nb] + c < best_g:
                    if mx and my and (flat[cy * w + nx] or flat[ny * w + cx]):
                        continue
                    best, best_g = nb, g[nb] + c
            parent[cur], g[cur] = best, best_g
        closed[cur] = 1
        expansions += 1
        gc = g[cur]
        par = parent[cur]
        if trace is not None:
            hp = euclid(cx, cy, gx, gy)
            trace.add(EXPAND, cx, cy, gc, gc + hp, hp, *(divmod(par, w)[::-1] if par != cur else (-1, -1)))
        if cur == t:
            found = True
            break
        if max_expansions is not None and expansions >= max_expansions:
            break
        px, py = par % w, par // w
        for mx, my, c in _MOVES:
            nx, ny = cx + mx, cy + my
            if not (0 <= nx < w and 0 <= ny < h):
                continue
            nb = ny * w + nx
            if flat[nb] or closed[nb]:
                continue
            if mx and my and (flat[cy * w + nx] or flat[ny * w + cx]):
                continue
            # path 2: straight from the parent if the parent can see the neighbour
            if par != cur and (lazy or sees(par, nx, ny)):
                cand_parent, ng = par, g[par] + euclid(px, py, nx, ny)
            else:
                cand_parent, ng = cur, gc + c
            if ng < g[nb] - 1e-12:
                g[nb] = ng
                parent[nb] = cand_parent
                hn = euclid(nx, ny, gx, gy)
                heapq.heappush(heap, (ng + hn, -ng, counter, nb))
                counter += 1
                if trace is not None:
                    trace.add(PUSH, nx, ny, ng, ng + hn, hn, cand_parent % w, cand_parent // w)
    mask = _closed_mask(closed, (h, w))
    if not found:
        return None, expansions, mask
    parent[s] = -1
    return _walk_back(parent, t, w), expansions, mask


# ---------------------------------------------------------------- planners

_ANY_ANGLE_CAPS = {
    **GridBestFirst.capabilities,
    ("agents", "single"): Rating(Fit.DESIGNED),
}


@register("jps")
class JumpPointSearchPlanner(GridBestFirst):
    label = "JPS"
    summary = (
        "Jump Point Search: A* that skips runs of symmetric cells and only queues jump points where a wall forces a turn. "
        "Same optimal paths as A*, a fraction of the queue work on open uniform grids."
    )
    overlays = ("jump scans (light cells)", *GridBestFirst.overlays)
    capabilities = {
        **_ANY_ANGLE_CAPS,
        ("changes", "movers"): adapted("moving obstacles are frozen where they were last seen; long jumps go stale quickly"),
    }

    def search(self, blocked, start, goal, budget: Budget | None, max_expansions, trace):
        return jps_search(blocked, start, goal, max_expansions, trace)


@register("theta_star")
class ThetaStarPlanner(GridBestFirst):
    label = "Theta*"
    summary = (
        "Any-angle A*: a cell takes its grandparent as parent whenever there is line of sight, "
        "so routes are straight lines between corners instead of 8-direction zigzags."
    )
    overlays = ("line-of-sight tests (green clear, red blocked)", *GridBestFirst.overlays)
    capabilities = {
        **_ANY_ANGLE_CAPS,
        ("world", "continuous"): adapted("any-angle on a 0.5-unit raster; routes hug the inflated obstacles"),
    }

    def search(self, blocked, start, goal, budget: Budget | None, max_expansions, trace):
        return theta_search(blocked, start, goal, max_expansions, trace)


@register("lazy_theta_star")
class LazyThetaStarPlanner(GridBestFirst):
    label = "Lazy Theta*"
    summary = (
        "Theta* that assumes line of sight when queueing and checks it only when a cell is expanded, "
        "so it runs far fewer line-of-sight tests for nearly the same routes."
    )
    overlays = ThetaStarPlanner.overlays
    capabilities = ThetaStarPlanner.capabilities

    def search(self, blocked, start, goal, budget: Budget | None, max_expansions, trace):
        return theta_search(blocked, start, goal, max_expansions, trace, lazy=True)
