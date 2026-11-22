from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from swarmpathfinding.core.geom import Vec2


@dataclass(slots=True)
class PathResult:
    path: tuple[Vec2, ...] = ()
    cost: float = float("inf")
    success: bool = False
    expansions: int = 0
    samples: int = 0
    rewires: int = 0
    runtime_ms: float = 0.0
    extra: dict[str, Any] = field(default_factory=dict)

    @property
    def n_waypoints(self) -> int:
        return len(self.path)


@dataclass(slots=True)
class EdgeChange:
    """A graph-cost edit that incremental planners can repair."""

    a: Vec2
    b: Vec2
    old_cost: float
    new_cost: float


@dataclass(slots=True)
class Budget:
    max_ms: float | None = None
    max_expansions: int | None = None
    weight: float = 1.0
    # Ask the planner to record its search events in `PathResult.extra["trace"]`
    # (see core/trace.py). Off by default: it costs time and memory.
    record_trace: bool = False
