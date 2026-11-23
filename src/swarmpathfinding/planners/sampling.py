"""Sampling-based planners for continuous space: RRT, RRT* and BIT*.

They never look at a grid of cells to search; they throw random points into
the free space and connect them with straight segments. Collision checks use
the same inflated planning raster as every other planner (`core/plan_grid`),
so what counts as "blocked" is identical across the lab.

Runs are reproducible: the random generator is seeded from the start and goal,
so the same scenario always grows the same tree.

- RRT (LaValle 1998): grow a tree toward random points; stop at the first path.
  Fast to a feasible path, which is jagged and never improves.
- RRT* (Karaman & Frazzoli 2011): pick the cheapest parent among nearby nodes
  and rewire neighbours through the new node; the path keeps shortening as the
  sample budget is spent (asymptotically optimal).
- BIT* (Gammell et al. 2015): batches of samples searched in order of
  estimated solution cost, like A* over an implicit random graph, with lazy
  collision checks. After the first solution, new samples come only from the
  ellipse that could still improve it (informed sampling).
"""

from __future__ import annotations

import heapq
import math
import time
import zlib
from typing import Any, Sequence

import numpy as np

from swarmpathfinding.core.belief import BeliefMap
from swarmpathfinding.core.capabilities import adapted, designed
from swarmpathfinding.core.geom import Vec2
from swarmpathfinding.core.plan_grid import PlanGrid, plan_grid
from swarmpathfinding.core.protocols import BasePathfinder, EmptyDebugLayer
from swarmpathfinding.core.results import Budget, PathResult
from swarmpathfinding.core.registry import register


def _ms(t0: float) -> float:
    return (time.perf_counter() - t0) * 1000.0


class FreeSpace:
    """Point / segment collision tests on a planning raster. The cells around
    the start and goal are exempt, so an agent hugging a wall can still leave."""

    def __init__(self, grid: PlanGrid, exempt: Sequence[Vec2] = (), exempt_r: float = 0.0):
        self.grid = grid
        self.blocked = grid.blocked
        self.cs = grid.cell_size
        self.w = grid.width * self.cs
        self.h = grid.height * self.cs
        self.exempt = np.array([(p.x, p.y) for p in exempt], dtype=float).reshape(-1, 2)
        self.exempt_r2 = exempt_r * exempt_r
        self.checks = 0

    def _blocked_pts(self, xs: np.ndarray, ys: np.ndarray, exempt: Sequence[int] = ()) -> np.ndarray:
        ix = np.floor(xs / self.cs).astype(int)
        iy = np.floor(ys / self.cs).astype(int)
        out = (ix < 0) | (iy < 0) | (ix >= self.blocked.shape[1]) | (iy >= self.blocked.shape[0])
        hit = out.copy()
        ok = ~out
        hit[ok] = self.blocked[iy[ok], ix[ok]]
        for e in exempt:
            if not hit.any():
                break
            ex, ey = self.exempt[e]
            hit &= ~(((xs - ex) ** 2 + (ys - ey) ** 2 <= self.exempt_r2) & ~out)
        return hit

    def _exempt_ends(self, ax, ay, bx, by) -> list[int]:
        """Exempt points that are an end of this segment (start / goal)."""
        return [
            i
            for i, (ex, ey) in enumerate(self.exempt)
            if (abs(ax - ex) < 1e-9 and abs(ay - ey) < 1e-9) or (abs(bx - ex) < 1e-9 and abs(by - ey) < 1e-9)
        ]

    def point_free(self, x: float, y: float) -> bool:
        return not bool(self._blocked_pts(np.array([x]), np.array([y]))[0])

    def segment_free(self, ax: float, ay: float, bx: float, by: float) -> bool:
        """Exact: tests one point inside every raster cell the segment passes through
        (midpoints between its crossings of the cell lines), so no corner is skipped."""
        self.checks += 1
        cs = self.cs
        ts = [0.0, 1.0]
        dx, dy = bx - ax, by - ay
        if abs(dx) > 1e-12:
            k0, k1 = sorted((ax / cs, bx / cs))
            ks = np.arange(math.floor(k0) + 1, math.ceil(k1))
            ts.extend(((ks * cs - ax) / dx).tolist())
        if abs(dy) > 1e-12:
            m0, m1 = sorted((ay / cs, by / cs))
            ms = np.arange(math.floor(m0) + 1, math.ceil(m1))
            ts.extend(((ms * cs - ay) / dy).tolist())
        t = np.unique(np.clip(np.array(ts), 0.0, 1.0))
        mid = (t[:-1] + t[1:]) / 2.0 if len(t) > 1 else t
        t = np.concatenate([mid, [0.0, 1.0]])
        ends = self._exempt_ends(ax, ay, bx, by) if len(self.exempt) else []
        return not bool(self._blocked_pts(ax + dx * t, ay + dy * t, ends).any())

    def sample(self, rng: np.random.Generator, k: int) -> np.ndarray:
        pts = rng.uniform((0.0, 0.0), (self.w, self.h), size=(k, 2))
        return pts[~self._blocked_pts(pts[:, 0], pts[:, 1])]


_SAMPLING_CAPS = {
    ("world", "continuous"): designed("samples the free space directly; paths are not tied to cells"),
    ("world", "grid"): adapted("samples the grid as if it were open space; straight segments cut across cells"),
    ("knowledge", "fog"): adapted("unknown space counts as free; samples a fresh tree whenever a seen wall blocks the route"),
    ("changes", "slow_edit"): adapted("grows a new tree after the edit"),
    ("changes", "movers"): adapted("moving obstacles are frozen where they were last seen"),
    ("agents", "crowd"): adapted("one tree per agent (agents in the same half-cell share it)"),
}


class SamplingPlanner(BasePathfinder):
    """Shared plumbing: raster, seeded random generator, path building, debug data."""

    capabilities = _SAMPLING_CAPS
    frees_start = False
    max_samples = 2000
    step = 2.0  # longest edge added in one extension (world units)

    def __init__(self, agent_radius: float = 0.35, resolution: float = 0.5):
        self.agent_radius = agent_radius
        self.resolution = resolution

    def debug_layer(self):
        return EmptyDebugLayer()

    def plan(self, belief: Any, starts: Sequence[Vec2], goal: Vec2, budget: Budget | None = None) -> PathResult:
        if not starts or not isinstance(belief, BeliefMap):
            return PathResult(success=False)
        t0 = time.perf_counter()
        start = starts[0]
        grid = plan_grid(belief, self.agent_radius, self.resolution)
        space = FreeSpace(grid, (start, goal), exempt_r=grid.cell_size * 1.5)
        # str hashes change per process, so build the seed from a stable checksum
        key = f"{self.name}:{start.x:.2f}:{start.y:.2f}:{goal.x:.2f}:{goal.y:.2f}"
        seed = zlib.crc32(key.encode())
        rng = np.random.default_rng(seed)
        record = budget is not None and budget.record_trace
        res = self.grow(space, (start.x, start.y), (goal.x, goal.y), rng, record)
        res.runtime_ms = _ms(t0)
        res.extra["collision_checks"] = space.checks
        if res.success:
            res.path = tuple([start] + [Vec2(float(x), float(y)) for x, y in res.extra.pop("pts")[1:-1]] + [goal])
            res.cost = sum(a.dist(b) for a, b in zip(res.path, res.path[1:]))
        else:
            res.extra.pop("pts", None)
            res.path = (start,)
        return res

    def grow(self, space: FreeSpace, s, g, rng, record) -> PathResult:
        raise NotImplementedError

    @staticmethod
    def _trace_back(pos: np.ndarray, parent: np.ndarray, i: int) -> list:
        out = []
        while i >= 0:
            out.append(tuple(pos[i]))
            i = int(parent[i])
        return out[::-1]

    @staticmethod
    def _tree_segments(pos: np.ndarray, parent: np.ndarray, n: int) -> np.ndarray:
        idx = np.nonzero(parent[:n] >= 0)[0]
        return np.hstack([pos[parent[idx]], pos[idx]]) if len(idx) else np.zeros((0, 4))

    @staticmethod
    def draw_overlay(surface, camera, result, blocked=None) -> None:
        draw_sampling(surface, camera, result.extra)


@register("rrt")
class RRTPlanner(SamplingPlanner):
    name = "rrt"
    label = "RRT"
    summary = (
        "Rapidly-exploring random tree: pick a random point, extend the nearest tree node up to 2 units toward it, "
        "keep the step if it is collision-free. Stops at the first path to the goal: quick, jagged, never optimised."
    )
    overlays = ("tree (every extension)", "the random samples")
    goal_bias = 0.1
    max_samples = 6000  # stops at the first path, so this is only the give-up point

    def grow(self, space, s, g, rng, record) -> PathResult:
        N = self.max_samples + 2
        pos = np.zeros((N, 2))
        parent = np.full(N, -1, dtype=int)
        pos[0] = s
        n = 1
        tried = []
        gx, gy = g
        found = -1
        samples = 0
        for _ in range(self.max_samples):
            samples += 1
            q = np.array(g) if rng.random() < self.goal_bias else rng.uniform((0, 0), (space.w, space.h))
            if record:
                tried.append(q)
            d = ((pos[:n] - q) ** 2).sum(1)
            j = int(np.argmin(d))
            dist = math.sqrt(d[j])
            if dist < 1e-9:
                continue
            new = pos[j] + (q - pos[j]) * min(1.0, self.step / dist)
            if not space.segment_free(pos[j, 0], pos[j, 1], new[0], new[1]):
                continue
            pos[n] = new
            parent[n] = j
            n += 1
            if math.hypot(new[0] - gx, new[1] - gy) <= self.step and space.segment_free(new[0], new[1], gx, gy):
                pos[n] = g
                parent[n] = n - 1
                found = n
                n += 1
                break
        res = PathResult(success=found >= 0, expansions=n, samples=samples)
        if found >= 0:
            res.extra["pts"] = self._trace_back(pos, parent, found)
        if record:
            res.extra["tree"] = self._tree_segments(pos, parent, n)
            res.extra["samples"] = np.array(tried).reshape(-1, 2)
        return res


@register("rrt_star")
class RRTStarPlanner(SamplingPlanner):
    name = "rrt_star"
    label = "RRT*"
    summary = (
        "RRT that keeps improving: each new node takes the cheapest parent among its neighbours, then offers itself "
        "as a cheaper parent to them (rewiring). Spends its whole sample budget, so the path shortens toward optimal."
    )
    overlays = ("tree (after rewiring)", "rewire count", "the random samples")
    goal_bias = 0.1
    max_samples = 1500  # always spent: the path keeps improving
    max_until_found = 6000  # keep sampling up to this many while there is no path yet
    gamma = 40.0  # neighbour radius = min(1.5 * step, gamma * sqrt(log n / n))

    def grow(self, space, s, g, rng, record) -> PathResult:
        N = self.max_until_found + 2
        pos = np.zeros((N, 2))
        parent = np.full(N, -1, dtype=int)
        cost = np.zeros(N)
        pos[0] = s
        n = 1
        gx, gy = g
        best_goal, best_cost = -1, math.inf  # tree node that reaches the goal, total cost
        rewires = 0
        tried = []
        first_at = None
        spent = 0
        for it in range(self.max_until_found):
            if it >= self.max_samples and best_goal >= 0:
                break
            spent += 1
            q = np.array(g) if rng.random() < self.goal_bias else rng.uniform((0, 0), (space.w, space.h))
            if record:
                tried.append(q)
            d = ((pos[:n] - q) ** 2).sum(1)
            j = int(np.argmin(d))
            dist = math.sqrt(d[j])
            if dist < 1e-9:
                continue
            new = pos[j] + (q - pos[j]) * min(1.0, self.step / dist)
            if not space.segment_free(pos[j, 0], pos[j, 1], new[0], new[1]):
                continue
            r = min(self.step * 1.5, self.gamma * math.sqrt(math.log(n + 1) / (n + 1)))
            dn = np.sqrt(((pos[:n] - new) ** 2).sum(1))
            near = np.nonzero(dn <= max(r, 1e-9))[0]
            # cheapest collision-free parent
            best_p, best_c = j, cost[j] + math.hypot(*(new - pos[j]))
            for k in near[np.argsort(cost[near] + dn[near])]:
                c = cost[k] + dn[k]
                if c >= best_c:
                    break
                if k != j and space.segment_free(pos[k, 0], pos[k, 1], new[0], new[1]):
                    best_p, best_c = int(k), c
                    break
            pos[n] = new
            parent[n] = best_p
            cost[n] = best_c
            me = n
            n += 1
            # rewire: neighbours that get cheaper through the new node
            for k in near:
                if k == best_p:
                    continue
                c = best_c + dn[k]
                if c + 1e-9 < cost[k] and space.segment_free(new[0], new[1], pos[k, 0], pos[k, 1]):
                    delta = cost[k] - c
                    parent[k] = me
                    cost[k] = c
                    rewires += 1
                    self._push_down(parent, cost, n, int(k), delta)
            dg = math.hypot(new[0] - gx, new[1] - gy)
            if dg <= self.step and best_c + dg < best_cost and space.segment_free(new[0], new[1], gx, gy):
                best_goal, best_cost = me, best_c + dg
                if first_at is None:
                    first_at = it + 1
            if best_goal >= 0:
                # rewiring may have lowered the cost of the goal's parent
                best_cost = min(best_cost, cost[best_goal] + math.hypot(pos[best_goal, 0] - gx, pos[best_goal, 1] - gy))
        res = PathResult(success=best_goal >= 0, expansions=n, samples=spent, rewires=rewires)
        if best_goal >= 0:
            res.extra["pts"] = self._trace_back(pos, parent, best_goal) + [g]
            res.extra["first_solution_after"] = first_at
        if record:
            res.extra["tree"] = self._tree_segments(pos, parent, n)
            res.extra["samples"] = np.array(tried).reshape(-1, 2)
        return res

    @staticmethod
    def _push_down(parent, cost, n, root, delta) -> None:
        """A node got cheaper: so does everything below it."""
        stack = [root]
        kids = parent[:n]
        while stack:
            u = stack.pop()
            for c in np.nonzero(kids == u)[0]:
                cost[c] -= delta
                stack.append(int(c))


@register("bit_star")
class BITStarPlanner(SamplingPlanner):
    name = "bit_star"
    label = "BIT*"
    summary = (
        "Batch Informed Trees: adds random samples in batches and searches the implied graph in order of "
        "estimated total cost, like A*, checking collisions only for edges it actually wants. After the first "
        "path, new samples come only from the ellipse that could still beat it."
    )
    overlays = ("tree", "unconnected samples", "informed ellipse")
    batches = 8
    max_batches = 20  # keep adding batches while there is no path yet
    batch_size = 120
    eta = 1.1  # radius multiplier
    r_max = 4.0  # cap on the connection radius (the formula is huge for small batches)

    def grow(self, space, s, g, rng, record) -> PathResult:
        s = np.array(s, dtype=float)
        g = np.array(g, dtype=float)
        c_min = float(np.linalg.norm(g - s))
        V = np.zeros((1 + self.max_batches * self.batch_size + 1, 2))  # tree vertex positions
        V[0] = s
        nv = 1
        gT = [0.0]
        parent = [-1]
        alive = [True]
        S = g.reshape(1, 2)  # unconnected samples; the goal starts among them
        goal_vid = -1
        c_best = math.inf

        def h(p):
            return math.hypot(g[0] - p[0], g[1] - p[1])

        def g_hat(p):
            return math.hypot(p[0] - s[0], p[1] - s[1])

        batch = 0
        for batch in range(self.max_batches):
            if batch >= self.batches and goal_vid >= 0:
                break
            if c_best < math.inf:  # prune what cannot beat the current solution
                f = np.sqrt(((S - s) ** 2).sum(1)) + np.sqrt(((S - g) ** 2).sum(1))
                S = S[f < c_best]
                for v in range(1, nv):
                    if alive[v] and v != goal_vid and g_hat(V[v]) + h(V[v]) > c_best:
                        alive[v] = False
            S = np.vstack([S, self._informed(rng, space, s, g, c_best, c_min, self.batch_size)])
            taken = np.zeros(len(S), dtype=bool)
            q = len(S) + sum(alive)
            r = min(self.r_max, max(1.0, self.eta * 2.0 * math.sqrt(1.5 * (space.w * space.h / math.pi) * math.log(q) / q)))
            QV = [(gT[v] + h(V[v]), v) for v in range(nv) if alive[v]]
            heapq.heapify(QV)
            QE: list = []
            seq = 0
            expanded: set[int] = set()
            while QV or QE:
                best_e = QE[0][0] if QE else math.inf
                while QV and QV[0][0] <= best_e:
                    _, v = heapq.heappop(QV)
                    if v in expanded or not alive[v]:
                        continue
                    expanded.add(v)
                    pv = V[v]
                    if len(S):
                        d = np.sqrt(((S - pv) ** 2).sum(1))
                        for k in np.nonzero((d <= r) & ~taken)[0]:
                            key = gT[v] + d[k] + h(S[k])
                            if key < c_best:
                                heapq.heappush(QE, (key, seq, v, 0, int(k)))
                                seq += 1
                    # rewiring: tree vertices this one could reach more cheaply
                    dv = np.sqrt(((V[:nv] - pv) ** 2).sum(1))
                    for w in np.nonzero(dv <= r)[0]:
                        w = int(w)
                        if w == v or not alive[w] or parent[w] == v:
                            continue
                        if g_hat(pv) + dv[w] + h(V[w]) < c_best and gT[v] + dv[w] < gT[w]:
                            heapq.heappush(QE, (gT[v] + dv[w] + h(V[w]), seq, v, 1, w))
                            seq += 1
                    best_e = QE[0][0] if QE else math.inf
                if not QE:
                    break
                key, _, v, kind, idx = heapq.heappop(QE)
                if key >= c_best:
                    break  # nothing left in this batch can improve the solution
                if not alive[v]:
                    continue
                px = S[idx] if kind == 0 else V[idx]
                d = math.hypot(px[0] - V[v][0], px[1] - V[v][1])
                if kind == 0 and (taken[idx] or gT[v] + d + h(px) >= c_best):
                    continue
                if kind == 1 and gT[v] + d >= gT[idx]:
                    continue
                if not space.segment_free(V[v][0], V[v][1], px[0], px[1]):
                    continue
                if kind == 0:
                    taken[idx] = True
                    w = nv
                    V[w] = px
                    nv += 1
                    gT.append(gT[v] + d)
                    parent.append(v)
                    alive.append(True)
                    if goal_vid < 0 and px[0] == g[0] and px[1] == g[1]:
                        goal_vid = w
                    heapq.heappush(QV, (gT[w] + h(px), w))
                else:
                    delta = gT[idx] - (gT[v] + d)
                    parent[idx] = v
                    gT[idx] = gT[v] + d
                    self._push_down(parent, gT, idx, delta)
                    expanded.discard(idx)
                    heapq.heappush(QV, (gT[idx] + h(px), idx))
                if goal_vid >= 0:
                    c_best = gT[goal_vid]
            S = S[~taken]
        pos = V[:nv]
        par = np.array(parent)
        res = PathResult(success=goal_vid >= 0, expansions=nv, samples=batch * self.batch_size)
        res.extra["batches"] = batch
        if goal_vid >= 0:
            res.extra["pts"] = self._trace_back(pos, par, goal_vid)
        if record:
            keep = np.array(alive)
            res.extra["tree"] = self._tree_segments(pos, np.where(keep & (par >= 0), par, -1), nv)
            res.extra["samples"] = S.copy()
            if c_best < math.inf:
                res.extra["ellipse"] = ((float(s[0]), float(s[1])), (float(g[0]), float(g[1])), c_best, c_min)
        return res

    @staticmethod
    def _push_down(parent: list[int], gT: list[float], root: int, delta: float) -> None:
        kids: dict[int, list[int]] = {}
        for c, p in enumerate(parent):
            kids.setdefault(p, []).append(c)
        stack = [root]
        while stack:
            u = stack.pop()
            for c in kids.get(u, ()):
                gT[c] -= delta
                stack.append(c)

    @staticmethod
    def _informed(rng, space: FreeSpace, s, g, c_best, c_min, k) -> np.ndarray:
        """k free samples, uniform over the map or (with a solution) over the
        ellipse with foci s, g whose points could lie on a shorter path."""
        if not math.isfinite(c_best) or c_best <= c_min + 1e-9:
            out = space.sample(rng, k)
            tries = 0
            while len(out) < k and tries < 10:
                out = np.vstack([out, space.sample(rng, k - len(out))])
                tries += 1
            return out[:k]
        centre = (s + g) / 2.0
        a = c_best / 2.0
        b = math.sqrt(max(c_best**2 - c_min**2, 0.0)) / 2.0
        ang = math.atan2(g[1] - s[1], g[0] - s[0])
        rot = np.array([[math.cos(ang), -math.sin(ang)], [math.sin(ang), math.cos(ang)]])
        got = []
        for _ in range(20):
            r = np.sqrt(rng.random(k))
            t = rng.random(k) * 2 * math.pi
            unit = np.stack([r * np.cos(t) * a, r * np.sin(t) * b], 1)
            pts = unit @ rot.T + centre
            inside = (pts[:, 0] >= 0) & (pts[:, 1] >= 0) & (pts[:, 0] < space.w) & (pts[:, 1] < space.h)
            pts = pts[inside]
            if len(pts):
                pts = pts[~space._blocked_pts(pts[:, 0], pts[:, 1])]
            got.append(pts)
            if sum(len(p) for p in got) >= k:
                break
        return np.vstack(got)[:k] if got else np.zeros((0, 2))


# ---------------------------------------------------------------- drawing

TREE = (110, 170, 235)
SAMPLE = (200, 200, 120)
ELLIPSE = (235, 170, 80)


def draw_sampling(surface, camera, extra: dict) -> None:
    import pygame

    tree = extra.get("tree")
    if tree is not None and len(tree):
        layer = pygame.Surface(surface.get_size(), pygame.SRCALPHA)
        col = (*TREE, 150)
        for x0, y0, x1, y1 in tree:
            pygame.draw.line(layer, col, camera.local(Vec2(x0, y0)), camera.local(Vec2(x1, y1)), 1)
        surface.blit(layer, (0, 0))
    pts = extra.get("samples")
    if pts is not None and len(pts) and camera.zoom >= 6:
        for x, y in pts[:4000]:
            c = camera.local(Vec2(float(x), float(y)))
            surface.set_at((int(c[0]), int(c[1])), SAMPLE)
    ell = extra.get("ellipse")
    if ell is not None:
        s, g, c_best, c_min = ell
        a = c_best / 2.0
        b = math.sqrt(max(c_best**2 - c_min**2, 0.0)) / 2.0
        cx, cy = (s[0] + g[0]) / 2.0, (s[1] + g[1]) / 2.0
        ang = math.atan2(g[1] - s[1], g[0] - s[0])
        ring = []
        for k in range(64):
            t = 2 * math.pi * k / 64
            ex, ey = a * math.cos(t), b * math.sin(t)
            ring.append(camera.local(Vec2(cx + ex * math.cos(ang) - ey * math.sin(ang), cy + ex * math.sin(ang) + ey * math.cos(ang))))
        pygame.draw.lines(surface, ELLIPSE, True, ring, 1)
