from __future__ import annotations

from typing import Any, Sequence

from swarmpathfinding.core.geom import Vec2
from swarmpathfinding.core.protocols import BasePathfinder
from swarmpathfinding.core.registry import register
from swarmpathfinding.core.results import Budget, PathResult


@register("none")
class NonePlanner(BasePathfinder):
    name = "none"
    label = "none (baseline)"
    summary = "Baseline that never plans: agents stay put."

    def plan(
        self,
        belief: Any,
        starts: Sequence[Vec2],
        goal: Vec2,
        budget: Budget | None = None,
    ) -> PathResult:
        return PathResult(path=(), cost=float("inf"), success=False)
