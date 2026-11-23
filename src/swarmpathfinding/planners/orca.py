"""ORCA on its own: no global planning, only reciprocal collision avoidance.

Every agent heads straight for its goal and ORCA bends the velocities so the
agents dodge each other. It shows what local avoidance does without a route
planner behind it: perfect for open crowds and crossing groups, stuck behind
the first wall. To use ORCA on top of a real route, keep any planner and set
Avoidance to ORCA instead.
"""

from __future__ import annotations

from typing import Any, Sequence

from swarmpathfinding.core.capabilities import adapted, designed
from swarmpathfinding.core.geom import Vec2
from swarmpathfinding.core.protocols import BasePathfinder, EmptyDebugLayer
from swarmpathfinding.core.registry import register
from swarmpathfinding.core.results import Budget, PathResult

WALLS = "no global route: walks straight at the goal and gets stuck behind walls"


@register("orca")
class OrcaPlanner(BasePathfinder):
    name = "orca"
    label = "ORCA (local only)"
    summary = (
        "Optimal Reciprocal Collision Avoidance with no route planner: each agent aims "
        "straight at its goal and picks the velocity closest to that which cannot hit "
        "a neighbour within 1 s, sharing the dodge 50/50. Turn on Show ORCA to see the "
        "half-planes."
    )
    overlays = (
        "Show ORCA: speed ring, one half-plane per neighbour, allowed velocities shaded, "
        "wanted vs chosen velocity, neighbours ringed",
    )
    forces_avoidance = True  # the sim runs ORCA for this planner whatever Avoidance says
    frees_start = False
    capabilities = {
        ("world", "grid"): adapted(WALLS),
        ("world", "continuous"): adapted(WALLS),
        ("agents", "single"): adapted("nothing to avoid: it is a straight line"),
        ("agents", "crowd"): designed("built for many agents dodging each other"),
        ("goals", "opposite"): designed("groups crossing head-on is the textbook ORCA case"),
        ("changes", "movers"): adapted("dodges agents only, not moving obstacles"),
    }

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
        return PathResult(path=(start, goal), cost=start.dist(goal), success=True, expansions=0)

    def debug_layer(self):
        return EmptyDebugLayer()
