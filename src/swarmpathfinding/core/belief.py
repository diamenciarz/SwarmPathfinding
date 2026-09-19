from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Iterator, Literal

import numpy as np

from swarmpathfinding.core.geom import Cell, Vec2

if TYPE_CHECKING:
    from swarmpathfinding.worlds.grid import GridWorld

UNKNOWN = np.int8(-1)
FREE = np.int8(0)
BLOCKED = np.int8(1)


@dataclass(slots=True)
class ObstacleRecord:
    id: str
    vertices: tuple[Vec2, ...]
    position: Vec2
    velocity: Vec2
    last_seen_time: float

    def ghost_position(self, now: float, freeze: bool = False) -> Vec2:
        if freeze:
            return self.position
        dt = max(0.0, now - self.last_seen_time)
        return self.position + self.velocity * dt


@dataclass(slots=True)
class BeliefMap:
    """Shared last-seen copy planners read. Never ground truth."""

    kind: Literal["grid", "continuous"]
    width: int
    height: int
    occupancy: np.ndarray  # int8 [height, width] — grid or fog raster
    seen_time: np.ndarray  # float64 [height, width], -1 never
    obstacles: dict[str, ObstacleRecord] = field(default_factory=dict)
    generation: int = 0
    freespace_unknown: bool = True

    @classmethod
    def unknown_grid(cls, width: int, height: int) -> BeliefMap:
        occupancy = np.full((height, width), UNKNOWN, dtype=np.int8)
        seen_time = np.full((height, width), -1.0, dtype=np.float64)
        return cls(
            kind="grid",
            width=width,
            height=height,
            occupancy=occupancy,
            seen_time=seen_time,
        )

    @classmethod
    def from_truth_grid(cls, world: GridWorld) -> BeliefMap:
        belief = cls.unknown_grid(world.width, world.height)
        belief.occupancy[:, :] = np.where(world.blocked, BLOCKED, FREE)
        belief.seen_time[:, :] = 0.0
        return belief

    def clone_empty(self) -> BeliefMap:
        return BeliefMap.unknown_grid(self.width, self.height)

    def is_unknown(self, cell: Cell) -> bool:
        if not self.in_bounds(cell):
            return False
        return int(self.occupancy[cell.y, cell.x]) == int(UNKNOWN)

    def is_blocked_for_planning(self, cell: Cell) -> bool:
        if not self.in_bounds(cell):
            return True
        value = int(self.occupancy[cell.y, cell.x])
        if value == int(BLOCKED):
            return True
        if value == int(UNKNOWN):
            return not self.freespace_unknown
        return False

    def in_bounds(self, cell: Cell) -> bool:
        return 0 <= cell.x < self.width and 0 <= cell.y < self.height

    def revealed_fraction(self) -> float:
        known = np.count_nonzero(self.occupancy != UNKNOWN)
        return float(known) / float(self.width * self.height)

    def observe_cells(
        self,
        cells: Iterator[Cell] | list[Cell],
        truth: GridWorld,
        now: float,
    ) -> int:
        """Write ground truth for currently visible cells. Returns discrepancy count."""
        discrepancies = 0
        for cell in cells:
            if not self.in_bounds(cell) or not truth.in_bounds(cell):
                continue
            actual = BLOCKED if truth.is_blocked(cell) else FREE
            previous = int(self.occupancy[cell.y, cell.x])
            if previous != int(actual):
                discrepancies += 1
                self.generation += 1
            self.occupancy[cell.y, cell.x] = actual
            self.seen_time[cell.y, cell.x] = now
        return discrepancies

    def observe_obstacle(self, record: ObstacleRecord) -> bool:
        previous = self.obstacles.get(record.id)
        changed = (
            previous is None
            or previous.position.dist_sq(record.position) > 1e-8
            or previous.velocity.dist_sq(record.velocity) > 1e-8
            or previous.vertices != record.vertices
        )
        self.obstacles[record.id] = record
        if changed:
            self.generation += 1
        return changed

    def forget_obstacle(self, obstacle_id: str) -> bool:
        if obstacle_id in self.obstacles:
            del self.obstacles[obstacle_id]
            self.generation += 1
            return True
        return False

    def ghost_obstacles(self, now: float, freeze: bool = False) -> list[ObstacleRecord]:
        ghosts: list[ObstacleRecord] = []
        for record in self.obstacles.values():
            pos = record.ghost_position(now, freeze=freeze)
            ghosts.append(
                ObstacleRecord(
                    id=record.id,
                    vertices=record.vertices,
                    position=pos,
                    velocity=record.velocity,
                    last_seen_time=record.last_seen_time,
                )
            )
        return ghosts
