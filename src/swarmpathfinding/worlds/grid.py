from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from swarmpathfinding.core.geom import Cell, Vec2

SQRT2 = math.sqrt(2.0)


@dataclass(slots=True)
class GridWorld:
    width: int
    height: int
    blocked: np.ndarray  # bool [height, width]
    eight: bool = True
    cell_size: float = 1.0
    generation: int = 0

    def __post_init__(self) -> None:
        if self.blocked.shape != (self.height, self.width):
            raise ValueError("blocked shape must be (height, width)")

    @classmethod
    def empty(cls, width: int, height: int, eight: bool = True) -> GridWorld:
        return cls(
            width=width,
            height=height,
            blocked=np.zeros((height, width), dtype=bool),
            eight=eight,
        )

    @property
    def world_size(self) -> Vec2:
        return Vec2(self.width * self.cell_size, self.height * self.cell_size)

    def in_bounds(self, cell: Cell) -> bool:
        return 0 <= cell.x < self.width and 0 <= cell.y < self.height

    def is_blocked(self, cell: Cell) -> bool:
        if not self.in_bounds(cell):
            return True
        return bool(self.blocked[cell.y, cell.x])

    def set_blocked(self, cell: Cell, value: bool) -> bool:
        """Punch (True) or delete (False) a wall. Returns whether occupancy changed."""
        if not self.in_bounds(cell):
            return False
        previous = bool(self.blocked[cell.y, cell.x])
        if previous == value:
            return False
        self.blocked[cell.y, cell.x] = value
        self.generation += 1
        return True

    def punch_wall(self, cell: Cell) -> bool:
        return self.set_blocked(cell, True)

    def delete_wall(self, cell: Cell) -> bool:
        return self.set_blocked(cell, False)

    def occupancy_fraction(self) -> float:
        return float(np.count_nonzero(self.blocked)) / float(self.width * self.height)

    def cell_center(self, cell: Cell) -> Vec2:
        return Vec2(
            (cell.x + 0.5) * self.cell_size,
            (cell.y + 0.5) * self.cell_size,
        )

    def world_to_cell(self, point: Vec2) -> Cell:
        return Cell(
            int(math.floor(point.x / self.cell_size)),
            int(math.floor(point.y / self.cell_size)),
        )

    def neighbors(self, cell: Cell) -> list[Cell]:
        raw = cell.neighbors8() if self.eight else cell.neighbors4()
        out: list[Cell] = []
        for nxt in raw:
            if self.is_blocked(nxt):
                continue
            if self.eight and self._diagonal_squeezed(cell, nxt):
                continue
            out.append(nxt)
        return out

    def _diagonal_squeezed(self, a: Cell, b: Cell) -> bool:
        dx = b.x - a.x
        dy = b.y - a.y
        if abs(dx) != 1 or abs(dy) != 1:
            return False
        return self.is_blocked(Cell(a.x + dx, a.y)) or self.is_blocked(Cell(a.x, a.y + dy))

    def cost(self, a: Cell, b: Cell) -> float:
        if a.x != b.x and a.y != b.y:
            return SQRT2 * self.cell_size
        return self.cell_size

    def line_of_sight(self, a: Cell, b: Cell) -> bool:
        """Supercover Bresenham: diagonal steps cannot squeeze between two walls."""
        if self.is_blocked(a) or self.is_blocked(b):
            return False
        x0, y0 = a.x, a.y
        x1, y1 = b.x, b.y
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        x, y = x0, y0
        if dx >= dy:
            err = dx / 2.0
            while x != x1:
                nx = x + sx
                # horizontal step
                if dy == 0:
                    x = nx
                    if self.is_blocked(Cell(x, y)):
                        return False
                    continue
                err -= dy
                if err < 0:
                    ny = y + sy
                    # diagonal: both orthogonal cells must be free
                    if self.is_blocked(Cell(nx, y)) or self.is_blocked(Cell(x, ny)):
                        return False
                    y = ny
                    err += dx
                x = nx
                if self.is_blocked(Cell(x, y)):
                    return False
        else:
            err = dy / 2.0
            while y != y1:
                ny = y + sy
                if dx == 0:
                    y = ny
                    if self.is_blocked(Cell(x, y)):
                        return False
                    continue
                err -= dx
                if err < 0:
                    nx = x + sx
                    if self.is_blocked(Cell(nx, y)) or self.is_blocked(Cell(x, ny)):
                        return False
                    x = nx
                    err += dy
                y = ny
                if self.is_blocked(Cell(x, y)):
                    return False
        return True

    def sample_free(self, rng: np.random.Generator, margin: int = 1) -> Cell:
        for _ in range(10_000):
            x = int(rng.integers(margin, max(margin + 1, self.width - margin)))
            y = int(rng.integers(margin, max(margin + 1, self.height - margin)))
            cell = Cell(x, y)
            if not self.is_blocked(cell):
                return cell
        raise RuntimeError("no free cell found")


def iter_cells(world: GridWorld):
    for y in range(world.height):
        for x in range(world.width):
            yield Cell(x, y)
