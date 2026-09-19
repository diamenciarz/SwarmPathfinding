from __future__ import annotations

from typing import Any, Sequence

import numpy as np

from swarmpathfinding.core.belief import BLOCKED, UNKNOWN, BeliefMap
from swarmpathfinding.core.geom import Vec2
from swarmpathfinding.core.protocols import BasePathfinder, EmptyDebugLayer
from swarmpathfinding.core.registry import register
from swarmpathfinding.core.results import Budget, PathResult
from swarmpathfinding.worlds.grid import GridWorld


@register("straight_line")
class StraightLinePlanner(BasePathfinder):
    """Phase-2 stub: succeed only if last-seen occupancy allows a straight shot."""

    name = "straight_line"

    def plan(
        self,
        belief: Any,
        starts: Sequence[Vec2],
        goal: Vec2,
        budget: Budget | None = None,
    ) -> PathResult:
        if not starts:
            return PathResult(success=False)
        start = starts[0]
        if isinstance(belief, BeliefMap) and belief.obstacles:
            from shapely.geometry import LineString, Polygon
            from shapely.ops import unary_union

            polys = [
                Polygon([(v.x, v.y) for v in rec.vertices])
                for rec in belief.obstacles.values()
                if len(rec.vertices) >= 3
            ]
            if polys:
                line = LineString([(start.x, start.y), (goal.x, goal.y)])
                if line.intersects(unary_union(polys)):
                    return PathResult(path=(start,), cost=float("inf"), success=False, expansions=1)
        if isinstance(belief, BeliefMap) and belief.kind == "grid":
            blocked = belief.occupancy == BLOCKED
            if not belief.freespace_unknown:
                blocked = blocked | (belief.occupancy == UNKNOWN)
            world = GridWorld(
                width=belief.width,
                height=belief.height,
                blocked=np.asarray(blocked, dtype=bool),
            )
            a = world.world_to_cell(start)
            b = world.world_to_cell(goal)
            if not world.line_of_sight(a, b):
                return PathResult(path=(start,), cost=float("inf"), success=False, expansions=1)
        cost = start.dist(goal)
        return PathResult(path=(start, goal), cost=cost, success=True, expansions=1)

    def debug_layer(self):
        return EmptyDebugLayer()
