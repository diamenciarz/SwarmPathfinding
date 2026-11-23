"""Flow fields: one search from the goal that every agent then follows.

`flow_field`: a Dijkstra "integration field" from the goal over the whole
known map. Each agent walks downhill on it, so 1 or 1000 agents with the same
goal cost one search. Any map change invalidates the whole field.

`flow_field_tiles`: the map is cut into sectors. Each sector's free cells are
split into connected regions; regions in neighbouring sectors that touch form
a small graph. A search on that graph picks a corridor of regions from the
agent to the goal, and the integration field is only computed inside the
corridor's sectors. Per-sector work is cached by the sector's contents, so a
change only dirties the sectors it touched (Supreme Commander 2 / Emerson
style "flow field tiles").
"""

from __future__ import annotations

import heapq
import math

import numpy as np
import pygame

from swarmpathfinding.core.capabilities import adapted, designed
from swarmpathfinding.core.geom import Cell, Vec2
from swarmpathfinding.core.registry import register
from swarmpathfinding.core.results import Budget
from swarmpathfinding.core.trace import EXPAND, PUSH, SearchTrace
from swarmpathfinding.planners.best_first import SQRT2, GridBestFirst, _MOVES

INF = float("inf")
SECTOR = 8  # cells per sector side


def integration_field(
    blocked: np.ndarray,
    goal: Cell,
    allowed: np.ndarray | None = None,
    trace: SearchTrace | None = None,
) -> tuple[np.ndarray, int]:
    """Dijkstra from the goal: cost-to-goal per cell (inf = unreachable).
    `allowed` limits the search to a mask of cells."""
    h, w = blocked.shape
    flat = blocked.ravel().tolist()
    ok = allowed.ravel().tolist() if allowed is not None else None
    dist = [INF] * (h * w)
    t = goal.y * w + goal.x
    dist[t] = 0.0
    heap = [(0.0, t)]
    if trace is not None:
        trace.add(PUSH, goal.x, goal.y, 0.0, 0.0, 0.0)
    settled = 0
    done = bytearray(h * w)
    while heap:
        d, cur = heapq.heappop(heap)
        if done[cur]:
            continue
        done[cur] = 1
        settled += 1
        cy, cx = divmod(cur, w)
        if trace is not None:
            trace.add(EXPAND, cx, cy, d, d, 0.0)
        for dx, dy, c in _MOVES:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < w and 0 <= ny < h):
                continue
            nb = ny * w + nx
            if flat[nb] or done[nb] or (ok is not None and not ok[nb]):
                continue
            if dx and dy and (flat[cy * w + nx] or flat[ny * w + cx]):
                continue
            nd = d + c
            if nd < dist[nb]:
                dist[nb] = nd
                heapq.heappush(heap, (nd, nb))
                if trace is not None:
                    trace.add(PUSH, nx, ny, nd, nd, 0.0, cx, cy)
    return np.array(dist, dtype=np.float64).reshape(h, w), settled


def descend(field: np.ndarray, blocked: np.ndarray, start: Cell, goal: Cell) -> list[Cell] | None:
    """Follow the field downhill from start to goal (the agent's flow)."""
    h, w = field.shape
    if not math.isfinite(field[start.y, start.x]):
        return None
    out = [start]
    x, y = start.x, start.y
    for _ in range(h * w):
        if x == goal.x and y == goal.y:
            return out
        here, best, bx, by = field[y, x], INF, -1, -1
        for dx, dy, c in _MOVES:
            nx, ny = x + dx, y + dy
            if not (0 <= nx < w and 0 <= ny < h) or blocked[ny, nx]:
                continue
            if dx and dy and (blocked[y, nx] or blocked[ny, x]):
                continue
            if field[ny, nx] >= here - 1e-9:
                continue  # only strictly downhill, so the walk always ends
            v = field[ny, nx] + c
            if v < best - 1e-9:
                best, bx, by = v, nx, ny
        if bx < 0:
            return None
        x, y = bx, by
        out.append(Cell(x, y))
    return None


def nearest_free(blocked: np.ndarray, cell: Cell, radius: int = 3) -> Cell | None:
    """`cell` if free, else the closest free cell within `radius` (agents
    hugging a wall sit inside the inflated margin in continuous worlds)."""
    h, w = blocked.shape
    if not blocked[cell.y, cell.x]:
        return cell
    best, best_d = None, INF
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            x, y = cell.x + dx, cell.y + dy
            if 0 <= x < w and 0 <= y < h and not blocked[y, x] and dx * dx + dy * dy < best_d:
                best, best_d = Cell(x, y), dx * dx + dy * dy
    return best


def compress(cells: list[Cell]) -> list[Cell]:
    """Drop cells in the middle of straight runs (same direction)."""
    if len(cells) <= 2:
        return cells
    out = [cells[0]]
    for a, b, c in zip(cells, cells[1:], cells[2:]):
        if (b.x - a.x, b.y - a.y) != (c.x - b.x, c.y - b.y):
            out.append(b)
    out.append(cells[-1])
    return out


def flow_directions(field: np.ndarray, blocked: np.ndarray) -> np.ndarray:
    """Per cell the (dx, dy) step downhill, (0, 0) where there is none."""
    h, w = field.shape
    best = np.where(np.isfinite(field), field, np.inf)
    out = np.zeros((h, w, 2), dtype=np.int8)
    cur = np.full((h, w), np.inf)
    pad = np.pad(best, 1, constant_values=np.inf)
    bpad = np.pad(blocked, 1, constant_values=True)
    for dx, dy, c in _MOVES:
        raw = pad[1 + dy : 1 + dy + h, 1 + dx : 1 + dx + w]
        nb = raw + c
        if dx and dy:
            side_x = bpad[1 : 1 + h, 1 + dx : 1 + dx + w]
            side_y = bpad[1 + dy : 1 + dy + h, 1 : 1 + w]
            nb = np.where(side_x | side_y, np.inf, nb)
        better = (nb < cur - 1e-9) & (raw < best - 1e-9)
        cur = np.where(better, nb, cur)
        out[better] = (dx, dy)
    return out


# ---------------------------------------------------------------- sectors


class SectorGraph:
    """Regions (connected free areas) per sector, and which regions touch.

    Region labels are cached per sector by the sector's contents, so only
    sectors whose cells changed are relabelled ("dirty")."""

    def __init__(self, size: int = SECTOR) -> None:
        self.size = size
        self._labels: dict[bytes, tuple[np.ndarray, int]] = {}
        self.prev_keys: dict[tuple[int, int], bytes] = {}
        self.dirty: list[tuple[int, int]] = []
        self.relabelled = 0

    def build(self, blocked: np.ndarray) -> None:
        h, w = blocked.shape
        S = self.size
        self.shape = (h, w)
        self.sw, self.sh = (w + S - 1) // S, (h + S - 1) // S
        self.label = np.full((h, w), -1, dtype=np.int32)  # global region id per cell
        self.region_sector: list[tuple[int, int]] = []
        self.region_center: list[tuple[float, float]] = []
        self.dirty = []
        self.relabelled = 0
        keys: dict[tuple[int, int], bytes] = {}
        for sy in range(self.sh):
            for sx in range(self.sw):
                sub = blocked[sy * S : (sy + 1) * S, sx * S : (sx + 1) * S]
                key = sub.tobytes() + bytes(sub.shape)
                keys[(sx, sy)] = key
                if self.prev_keys and self.prev_keys.get((sx, sy)) != key:
                    self.dirty.append((sx, sy))
                hit = self._labels.get(key)
                if hit is None:
                    hit = _label_regions(sub)
                    self._labels[key] = hit
                    self.relabelled += 1
                local, n = hit
                base = len(self.region_sector)
                ys, xs = np.nonzero(local >= 0)
                self.label[sy * S : (sy + 1) * S, sx * S : (sx + 1) * S][local >= 0] = local[local >= 0] + base
                for r in range(n):
                    m = local == r
                    yy, xx = np.nonzero(m)
                    self.region_sector.append((sx, sy))
                    self.region_center.append((sx * S + xx.mean() + 0.5, sy * S + yy.mean() + 0.5))
        if len(self._labels) > 20000:
            self._labels.clear()
        self.prev_keys = keys
        self._edges(blocked)

    def _edges(self, blocked: np.ndarray) -> None:
        """Regions are linked when a legal move crosses their sector border."""
        lab = self.label
        h, w = lab.shape
        edges: dict[int, set[int]] = {i: set() for i in range(len(self.region_sector))}
        S = self.size
        # vertical borders (between sector columns) and horizontal ones
        for x in range(S, w, S):
            a, b = lab[:, x - 1], lab[:, x]
            for y in range(h):
                for dy in (-1, 0, 1):
                    ny = y + dy
                    if not (0 <= ny < h) or a[y] < 0 or b[ny] < 0:
                        continue
                    if dy and (blocked[y, x] or blocked[ny, x - 1]):
                        continue
                    edges[a[y]].add(int(b[ny]))
                    edges[b[ny]].add(int(a[y]))
        for y in range(S, h, S):
            a, b = lab[y - 1, :], lab[y, :]
            for x in range(w):
                for dx in (-1, 0, 1):
                    nx = x + dx
                    if not (0 <= nx < w) or a[x] < 0 or b[nx] < 0:
                        continue
                    if dx and (blocked[y, x] or blocked[y - 1, nx]):
                        continue
                    edges[a[x]].add(int(b[nx]))
                    edges[b[nx]].add(int(a[x]))
        self.edges = edges

    def corridor(self, a: int, b: int) -> list[int] | None:
        """A* over regions, by distance between region centres."""
        cen = self.region_center

        def d(i, j):
            return math.hypot(cen[i][0] - cen[j][0], cen[i][1] - cen[j][1])

        g = {a: 0.0}
        parent = {a: -1}
        heap = [(d(a, b), a)]
        closed = set()
        while heap:
            _, cur = heapq.heappop(heap)
            if cur in closed:
                continue
            if cur == b:
                out = []
                while cur != -1:
                    out.append(cur)
                    cur = parent[cur]
                return out[::-1]
            closed.add(cur)
            for nb in self.edges[cur]:
                ng = g[cur] + d(cur, nb)
                if ng < g.get(nb, INF):
                    g[nb] = ng
                    parent[nb] = cur
                    heapq.heappush(heap, (ng + d(nb, b), nb))
        return None


def _label_regions(sub: np.ndarray) -> tuple[np.ndarray, int]:
    """Connected components of free cells (moves without corner cutting)."""
    h, w = sub.shape
    lab = np.full((h, w), -1, dtype=np.int32)
    n = 0
    for y in range(h):
        for x in range(w):
            if sub[y, x] or lab[y, x] >= 0:
                continue
            stack = [(x, y)]
            lab[y, x] = n
            while stack:
                cx, cy = stack.pop()
                for dx, dy, _ in _MOVES:
                    nx, ny = cx + dx, cy + dy
                    if not (0 <= nx < w and 0 <= ny < h) or sub[ny, nx] or lab[ny, nx] >= 0:
                        continue
                    if dx and dy and (sub[cy, nx] or sub[ny, cx]):
                        continue
                    lab[ny, nx] = n
                    stack.append((nx, ny))
            n += 1
    return lab, n


# ---------------------------------------------------------------- planners

_FLOW_CAPS = {
    **GridBestFirst.capabilities,
    ("agents", "single"): adapted("computes the field over the whole map even for one agent"),
    ("agents", "few"): designed(),
    ("agents", "crowd"): designed("one field serves every agent with the same goal"),
    ("goals", "shared"): designed(),
    ("goals", "opposite"): adapted("one field per goal"),
}


@register("flow_field")
class FlowFieldPlanner(GridBestFirst):
    label = "flow field"
    summary = (
        "One Dijkstra from the goal over the whole map (the integration field); every agent walks downhill on it. "
        "Cost does not grow with the number of agents, but any map change means a new field."
    )
    overlays = ("flow arrows", "integration field (heatmap g)", "wavefront replay")
    frees_start = False
    capabilities = {
        **_FLOW_CAPS,
        ("knowledge", "fog"): adapted("recomputes the whole field whenever the swarm learns something"),
        ("changes", "slow_edit"): adapted("recomputes the whole field after the edit"),
    }

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._fields: dict[tuple, np.ndarray] = {}

    def search(self, blocked, start, goal, budget: Budget | None, max_expansions, trace):
        start = nearest_free(blocked, start)
        if start is None:
            return None, 0, np.zeros_like(blocked)
        key = (goal.x, goal.y, blocked.shape, hash(blocked.tobytes()))
        field = self._fields.get(key)
        expansions = 0
        if field is None or trace is not None:
            field, expansions = integration_field(blocked, goal, trace=trace)
            if len(self._fields) > 8:
                self._fields.clear()
            self._fields[key] = field
        self._debug = {"field": field, "shared_field": expansions == 0}
        cells = descend(field, blocked, start, goal)
        return (compress(cells) if cells else None), expansions, np.isfinite(field)

    @staticmethod
    def draw_overlay(surface, camera, result, blocked=None) -> None:
        field = result.extra.get("field")
        if field is not None:
            draw_flow_arrows(surface, camera, field, result.extra["cell_size"], result.extra.get("blocked"))


@register("flow_field_tiles")
class FlowFieldTilesPlanner(GridBestFirst):
    label = "flow-field tiles"
    summary = (
        "Cuts the map into 8x8 sectors, finds a corridor of sectors on a small region graph, "
        "and builds the flow field only inside it. A change only dirties the sectors it touched."
    )
    overlays = ("sectors, corridor and dirty sectors", "flow arrows", "integration field (heatmap g)", "wavefront replay")
    frees_start = False
    capabilities = {
        **_FLOW_CAPS,
        ("knowledge", "fog"): designed("only sectors where something was discovered are rebuilt"),
        ("changes", "slow_edit"): designed("an edit dirties only its own sector"),
        ("agents", "single"): adapted("the corridor keeps the field small, but paths follow sector regions (near-optimal)"),
    }

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.sectors = SectorGraph()
        self._built_for: bytes | None = None
        self._fields: dict[tuple, np.ndarray] = {}

    def search(self, blocked, start, goal, budget: Budget | None, max_expansions, trace):
        start = nearest_free(blocked, start)
        if start is None:
            return None, 0, np.zeros_like(blocked)
        sg = self.sectors
        bkey = blocked.tobytes() + bytes(str(blocked.shape), "ascii")
        rebuilt = False
        if bkey != self._built_for:
            sg.build(blocked)
            self._built_for = bkey
            rebuilt = True
        ra, rb = int(sg.label[start.y, start.x]), int(sg.label[goal.y, goal.x])
        if ra < 0 or rb < 0:
            return None, 0, np.zeros_like(blocked)
        corridor = sg.corridor(ra, rb)
        if corridor is None:
            self._debug = {"sectors": _sector_debug(sg, [], rebuilt)}
            return None, 0, np.zeros_like(blocked)
        core = {sg.region_sector[r] for r in corridor}
        # one sector of margin around the corridor lets the field cut corners
        # the region graph cannot see (paths stay close to optimal)
        sectors = sorted(
            {(sx + dx, sy + dy) for sx, sy in core for dx in (-1, 0, 1) for dy in (-1, 0, 1)
             if 0 <= sx + dx < sg.sw and 0 <= sy + dy < sg.sh}
        )
        allowed = np.zeros_like(blocked)
        S = sg.size
        for sx, sy in sectors:
            allowed[sy * S : (sy + 1) * S, sx * S : (sx + 1) * S] = True
        fkey = (goal.x, goal.y, tuple(sectors), tuple(sg.prev_keys.get(s) for s in sectors))
        field = self._fields.get(fkey)
        expansions = 0
        if field is None or trace is not None:
            field, expansions = integration_field(blocked, goal, allowed, trace)
            if len(self._fields) > 64:
                self._fields.clear()
            self._fields[fkey] = field
        cells = descend(field, blocked, start, goal)
        if cells is None:
            # the region graph is coarse: fall back to the full map (rare)
            field, more = integration_field(blocked, goal, trace=None)
            expansions += more
            cells = descend(field, blocked, start, goal)
        self._debug = {"field": field, "sectors": _sector_debug(sg, sectors, rebuilt, corridor)}
        return (compress(cells) if cells else None), expansions, np.isfinite(field)

    @staticmethod
    def draw_overlay(surface, camera, result, blocked=None) -> None:
        cs = result.extra["cell_size"]
        info = result.extra.get("sectors")
        if info:
            draw_sectors(surface, camera, info, cs)
        field = result.extra.get("field")
        if field is not None:
            draw_flow_arrows(surface, camera, field, cs, None)


def _sector_debug(sg: SectorGraph, sectors, rebuilt: bool, corridor=None) -> dict:
    return {
        "size": sg.size,
        "grid": (sg.sw, sg.sh),
        "corridor_sectors": list(sectors),
        "dirty": list(sg.dirty) if rebuilt else [],
        "path": [sg.region_center[r] for r in (corridor or [])],
        "regions": len(sg.region_sector),
    }


# ---------------------------------------------------------------- drawing


def draw_flow_arrows(surface, camera, field: np.ndarray, cell_size: float, blocked) -> None:
    px = camera.zoom * cell_size
    if px < 9:
        return
    h, w = field.shape
    vp = camera.viewport
    tl = camera.screen_to_world(vp.left, vp.top)
    br = camera.screen_to_world(vp.right, vp.bottom)
    x0, x1 = max(0, int(tl.x / cell_size)), min(w, int(br.x / cell_size) + 1)
    y0, y1 = max(0, int(br.y / cell_size)), min(h, int(tl.y / cell_size) + 1)
    if (x1 - x0) * (y1 - y0) > 8000:
        return
    sub_blocked = ~np.isfinite(field)
    dirs = flow_directions(field, sub_blocked)
    col = (235, 235, 245)
    for y in range(y0, y1):
        for x in range(x0, x1):
            dx, dy = dirs[y, x]
            if not dx and not dy:
                continue
            c = camera.local(Vec2((x + 0.5) * cell_size, (y + 0.5) * cell_size))
            length = px * 0.35 / (SQRT2 if dx and dy else 1.0)
            tip = (c[0] + dx * length, c[1] - dy * length)
            tail = (c[0] - dx * length, c[1] + dy * length)
            pygame.draw.line(surface, col, tail, tip, 1)
            pygame.draw.circle(surface, col, (int(tip[0]), int(tip[1])), 2)


def draw_sectors(surface, camera, info: dict, cell_size: float) -> None:
    S = info["size"] * cell_size
    sw, sh = info["grid"]
    overlay = pygame.Surface(surface.get_size(), pygame.SRCALPHA)
    for sx, sy in info["corridor_sectors"]:
        a = camera.local(Vec2(sx * S, (sy + 1) * S))
        b = camera.local(Vec2((sx + 1) * S, sy * S))
        pygame.draw.rect(overlay, (80, 150, 255, 38), pygame.Rect(a[0], a[1], b[0] - a[0], b[1] - a[1]))
    for sx, sy in info["dirty"]:
        a = camera.local(Vec2(sx * S, (sy + 1) * S))
        b = camera.local(Vec2((sx + 1) * S, sy * S))
        pygame.draw.rect(overlay, (255, 90, 90, 60), pygame.Rect(a[0], a[1], b[0] - a[0], b[1] - a[1]))
    surface.blit(overlay, (0, 0))
    line = (110, 130, 170)
    for sx in range(sw + 1):
        a = camera.local(Vec2(sx * S, 0))
        b = camera.local(Vec2(sx * S, sh * S))
        pygame.draw.line(surface, line, a, b, 1)
    for sy in range(sh + 1):
        a = camera.local(Vec2(0, sy * S))
        b = camera.local(Vec2(sw * S, sy * S))
        pygame.draw.line(surface, line, a, b, 1)
    pts = [camera.local(Vec2(x * cell_size, y * cell_size)) for x, y in info["path"]]
    if len(pts) >= 2:
        pygame.draw.lines(surface, (120, 200, 255), False, pts, 2)
    for p in pts:
        pygame.draw.circle(surface, (120, 200, 255), p, 4)
