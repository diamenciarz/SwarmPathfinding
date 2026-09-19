from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class Vec2:
    x: float
    y: float

    def __add__(self, other: Vec2) -> Vec2:
        return Vec2(self.x + other.x, self.y + other.y)

    def __sub__(self, other: Vec2) -> Vec2:
        return Vec2(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar: float) -> Vec2:
        return Vec2(self.x * scalar, self.y * scalar)

    __rmul__ = __mul__

    def __truediv__(self, scalar: float) -> Vec2:
        return Vec2(self.x / scalar, self.y / scalar)

    def __neg__(self) -> Vec2:
        return Vec2(-self.x, -self.y)

    def dot(self, other: Vec2) -> float:
        return self.x * other.x + self.y * other.y

    def length_sq(self) -> float:
        return self.x * self.x + self.y * self.y

    def length(self) -> float:
        return math.hypot(self.x, self.y)

    def normalized(self) -> Vec2:
        length = self.length()
        if length == 0.0:
            return Vec2(0.0, 0.0)
        return self / length

    def clamped(self, low: Vec2, high: Vec2) -> Vec2:
        return Vec2(
            min(max(self.x, low.x), high.x),
            min(max(self.y, low.y), high.y),
        )

    def angle(self) -> float:
        return math.atan2(self.y, self.x)

    def as_tuple(self) -> tuple[float, float]:
        return (self.x, self.y)

    def dist(self, other: Vec2) -> float:
        return (self - other).length()

    def dist_sq(self, other: Vec2) -> float:
        return (self - other).length_sq()


@dataclass(frozen=True, slots=True, order=True)
class Cell:
    x: int
    y: int

    def neighbors4(self) -> tuple[Cell, ...]:
        return (
            Cell(self.x + 1, self.y),
            Cell(self.x - 1, self.y),
            Cell(self.x, self.y + 1),
            Cell(self.x, self.y - 1),
        )

    def neighbors8(self) -> tuple[Cell, ...]:
        return (
            Cell(self.x + 1, self.y),
            Cell(self.x - 1, self.y),
            Cell(self.x, self.y + 1),
            Cell(self.x, self.y - 1),
            Cell(self.x + 1, self.y + 1),
            Cell(self.x + 1, self.y - 1),
            Cell(self.x - 1, self.y + 1),
            Cell(self.x - 1, self.y - 1),
        )

    def manhattan(self, other: Cell) -> int:
        return abs(self.x - other.x) + abs(self.y - other.y)

    def chebyshev(self, other: Cell) -> int:
        return max(abs(self.x - other.x), abs(self.y - other.y))

    def octile(self, other: Cell) -> float:
        dx = abs(self.x - other.x)
        dy = abs(self.y - other.y)
        return max(dx, dy) + (math.sqrt(2.0) - 1.0) * min(dx, dy)


def lerp(a: Vec2, b: Vec2, t: float) -> Vec2:
    return a + (b - a) * t


def heading_from_to(origin: Vec2, target: Vec2) -> float:
    return (target - origin).angle()
