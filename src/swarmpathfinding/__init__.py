"""SwarmPathfinding — grid and continuous pathfinding research lab."""

from swarmpathfinding.core.geom import Cell, Vec2
from swarmpathfinding.core.registry import create_planner, list_planners, register

__version__ = "0.1.0"

__all__ = [
    "Cell",
    "Vec2",
    "create_planner",
    "list_planners",
    "register",
    "__version__",
]
