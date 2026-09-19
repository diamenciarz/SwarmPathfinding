from __future__ import annotations

import numpy as np

from swarmpathfinding.core.geom import Cell
from swarmpathfinding.worlds.grid import GridWorld


def empty_grid(width: int = 48, height: int = 36, eight: bool = True) -> GridWorld:
    return GridWorld.empty(width, height, eight=eight)


def random_grid(
    width: int = 48,
    height: int = 36,
    p: float = 0.15,
    rng: np.random.Generator | None = None,
    eight: bool = True,
) -> GridWorld:
    rng = rng or np.random.default_rng()
    blocked = rng.random((height, width)) < p
    world = GridWorld(width=width, height=height, blocked=blocked, eight=eight)
    _carve_border_clearance(world, margin=1)
    match_occupancy(world, p, rng)
    return world


def clustered_grid(
    width: int = 48,
    height: int = 36,
    p: float = 0.30,
    rng: np.random.Generator | None = None,
    steps: int = 4,
    eight: bool = True,
) -> GridWorld:
    """Cellular automata blobs, then occupancy is matched to p."""
    rng = rng or np.random.default_rng()
    seed_p = min(0.70, max(0.25, p + 0.25))
    grid = (rng.random((height, width)) < seed_p).astype(np.uint8)
    for _ in range(steps):
        grid = _ca_smooth(grid)
    world = GridWorld(width=width, height=height, blocked=grid.astype(bool), eight=eight)
    _carve_border_clearance(world, margin=1)
    match_occupancy(world, p, rng)
    world.generation = 0
    return world


def rooms_grid(
    width: int = 48,
    height: int = 36,
    n_rooms: int = 8,
    rng: np.random.Generator | None = None,
    eight: bool = True,
) -> GridWorld:
    rng = rng or np.random.default_rng()
    blocked = np.ones((height, width), dtype=bool)
    rooms: list[tuple[int, int, int, int]] = []
    for _ in range(n_rooms * 3):
        if len(rooms) >= n_rooms:
            break
        rw = int(rng.integers(4, max(5, width // 5)))
        rh = int(rng.integers(4, max(5, height // 5)))
        x = int(rng.integers(1, max(2, width - rw - 1)))
        y = int(rng.integers(1, max(2, height - rh - 1)))
        rooms.append((x, y, rw, rh))
        blocked[y : y + rh, x : x + rw] = False
    centers = [(x + rw // 2, y + rh // 2) for x, y, rw, rh in rooms]
    for (x0, y0), (x1, y1) in zip(centers, centers[1:]):
        _carve_corridor(blocked, x0, y0, x1, y1)
    if centers:
        _carve_corridor(blocked, centers[0][0], centers[0][1], centers[-1][0], centers[-1][1])
    world = GridWorld(width=width, height=height, blocked=blocked, eight=eight)
    _carve_border_clearance(world, margin=0)
    return world


def maze_grid(
    width: int = 49,
    height: int = 37,
    rng: np.random.Generator | None = None,
    eight: bool = True,
) -> GridWorld:
    """Odd-sized recursive-backtracker maze (walls on even cells)."""
    rng = rng or np.random.default_rng()
    width += 1 - width % 2
    height += 1 - height % 2
    blocked = np.ones((height, width), dtype=bool)
    sx, sy = 1, 1
    blocked[sy, sx] = False
    stack = [(sx, sy)]
    dirs = [(2, 0), (-2, 0), (0, 2), (0, -2)]
    while stack:
        x, y = stack[-1]
        options = []
        for dx, dy in dirs:
            nx, ny = x + dx, y + dy
            if 0 < nx < width and 0 < ny < height and blocked[ny, nx]:
                options.append((dx, dy, nx, ny))
        if not options:
            stack.pop()
            continue
        dx, dy, nx, ny = options[int(rng.integers(0, len(options)))]
        blocked[y + dy // 2, x + dx // 2] = False
        blocked[ny, nx] = False
        stack.append((nx, ny))
    return GridWorld(width=width, height=height, blocked=blocked, eight=eight)


def street_grid(
    width: int = 48,
    height: int = 36,
    rng: np.random.Generator | None = None,
    street: int = 3,
    block: int = 6,
    eight: bool = True,
) -> GridWorld:
    """MovingAI-style street: rectangular blocks separated by corridors."""
    rng = rng or np.random.default_rng()
    blocked = np.zeros((height, width), dtype=bool)
    y = street
    while y + block < height - street:
        x = street
        jitter_y = int(rng.integers(0, 2))
        while x + block < width - street:
            bw = block + int(rng.integers(-1, 2))
            bh = block + int(rng.integers(-1, 2))
            bw = max(3, min(bw, width - x - street))
            bh = max(3, min(bh, height - y - street))
            blocked[y + jitter_y : y + jitter_y + bh, x : x + bw] = True
            x += bw + street
        y += block + street
    return GridWorld(width=width, height=height, blocked=blocked, eight=eight)


def open_grid(
    width: int = 48,
    height: int = 36,
    p: float = 0.08,
    rng: np.random.Generator | None = None,
    eight: bool = True,
) -> GridWorld:
    """Mostly open field with a few large blobs (MovingAI 'open')."""
    return clustered_grid(width, height, p=p, rng=rng, steps=5, eight=eight)


def game_grid(
    width: int = 48,
    height: int = 36,
    rng: np.random.Generator | None = None,
    eight: bool = True,
) -> GridWorld:
    """Rooms plus scattered pillars — a cheap MovingAI 'game' analogue."""
    rng = rng or np.random.default_rng()
    world = rooms_grid(width, height, n_rooms=max(6, width // 8), rng=rng, eight=eight)
    free = np.argwhere(~world.blocked)
    rng.shuffle(free)
    n_pillars = max(4, int(0.04 * width * height))
    for i in range(min(n_pillars, len(free))):
        y, x = int(free[i][0]), int(free[i][1])
        if 2 <= x < width - 2 and 2 <= y < height - 2:
            world.blocked[y, x] = True
    world.generation = 0
    return world


def corridor_grid(
    width: int = 48,
    height: int = 36,
    eight: bool = True,
) -> GridWorld:
    """A wide hallway for opposite-direction traffic."""
    blocked = np.ones((height, width), dtype=bool)
    y0 = height // 2 - 3
    y1 = height // 2 + 3
    blocked[y0:y1, 1 : width - 1] = False
    return GridWorld(width=width, height=height, blocked=blocked, eight=eight)


def match_occupancy(world: GridWorld, p: float, rng: np.random.Generator, tol: float = 0.02) -> None:
    """Flip random interior cells until occupancy is within tol of p."""
    h, w = world.height, world.width
    target = int(round(p * w * h))
    target = min(max(target, 0), w * h)

    def count() -> int:
        return int(np.count_nonzero(world.blocked))

    guard = w * h * 4
    while count() > target and guard > 0:
        guard -= 1
        coords = np.argwhere(world.blocked)
        if len(coords) == 0:
            break
        y, x = coords[int(rng.integers(0, len(coords)))]
        world.blocked[int(y), int(x)] = False
    guard = w * h * 4
    while count() < target and guard > 0:
        guard -= 1
        x = int(rng.integers(1, w - 1))
        y = int(rng.integers(1, h - 1))
        world.blocked[y, x] = True
    _carve_border_clearance(world, margin=1)
    world.generation = 0


def pick_interior_wall(world: GridWorld, rng: np.random.Generator) -> Cell | None:
    """A blocked cell with at least two free 4-neighbors — a wall we can delete."""
    candidates: list[Cell] = []
    for y in range(1, world.height - 1):
        for x in range(1, world.width - 1):
            if not world.blocked[y, x]:
                continue
            cell = Cell(x, y)
            free = 0
            for n in cell.neighbors4():
                if world.in_bounds(n) and not world.is_blocked(n):
                    free += 1
            if free >= 2:
                candidates.append(cell)
    if not candidates:
        return None
    return candidates[int(rng.integers(0, len(candidates)))]


def ensure_free(world: GridWorld, cell: Cell, radius: int = 1) -> None:
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            c = Cell(cell.x + dx, cell.y + dy)
            if world.in_bounds(c):
                world.blocked[c.y, c.x] = False


def _carve_border_clearance(world: GridWorld, margin: int) -> None:
    if margin <= 0:
        return
    world.blocked[:margin, :] = False
    world.blocked[-margin:, :] = False
    world.blocked[:, :margin] = False
    world.blocked[:, -margin:] = False


def _ca_smooth(grid: np.ndarray) -> np.ndarray:
    h, w = grid.shape
    nxt = grid.copy()
    for y in range(h):
        for x in range(w):
            total = 0
            for dy in range(-1, 2):
                for dx in range(-1, 2):
                    ny, nx = y + dy, x + dx
                    if 0 <= ny < h and 0 <= nx < w:
                        total += int(grid[ny, nx])
                    else:
                        total += 1
            nxt[y, x] = 1 if total >= 5 else 0
    return nxt


def _carve_corridor(blocked: np.ndarray, x0: int, y0: int, x1: int, y1: int) -> None:
    x, y = x0, y0
    while x != x1:
        blocked[y, x] = False
        x += 1 if x1 > x else -1
    while y != y1:
        blocked[y, x] = False
        y += 1 if y1 > y else -1
    blocked[y, x] = False
