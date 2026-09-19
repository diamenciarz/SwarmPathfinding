from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterator, Protocol, Sequence, runtime_checkable

from swarmpathfinding.core.geom import Cell, Vec2
from swarmpathfinding.core.results import Budget, EdgeChange, PathResult


@dataclass(frozen=True, slots=True)
class Toggle:
    id: str
    label: str
    default: bool = False


@runtime_checkable
class DebugLayer(Protocol):
    def overlays(self) -> Sequence[Toggle]: ...

    def draw_world(self, surface: Any, camera: Any, state: Any) -> None: ...

    def draw_panel(self, surface: Any, rect: Any, metrics: dict[str, Any]) -> None: ...


@runtime_checkable
class Pathfinder(Protocol):
    name: str

    def plan(
        self,
        belief: Any,
        starts: Sequence[Vec2],
        goal: Vec2,
        budget: Budget | None = None,
    ) -> PathResult: ...

    def update(self, changes: Sequence[EdgeChange]) -> None: ...

    def debug_layer(self) -> DebugLayer: ...


@runtime_checkable
class VisionShape(Protocol):
    name: str

    def contains(self, origin: Vec2, heading: float, point: Vec2) -> bool: ...

    def footprint_polygon(self, origin: Vec2, heading: float):
        """Return a shapely polygon in world meters."""
        ...

    def visible_cells(
        self,
        origin: Vec2,
        heading: float,
        world: Any,
    ) -> Iterator[Cell]: ...


class EmptyDebugLayer:
    def overlays(self) -> Sequence[Toggle]:
        return ()

    def draw_world(self, surface: Any, camera: Any, state: Any) -> None:
        return None

    def draw_panel(self, surface: Any, rect: Any, metrics: dict[str, Any]) -> None:
        return None


class BasePathfinder:
    """Convenience base so a planner only has to implement `plan`."""

    name: str = "base"

    def update(self, changes: Sequence[EdgeChange]) -> None:
        return None

    def debug_layer(self) -> DebugLayer:
        return EmptyDebugLayer()
