from __future__ import annotations

from typing import Any, Callable, TypeVar

from swarmpathfinding.core.protocols import Pathfinder

T = TypeVar("T")

_REGISTRY: dict[str, Callable[..., Pathfinder]] = {}


def register(name: str) -> Callable[[T], T]:
    def decorator(cls: T) -> T:
        _REGISTRY[name] = cls  # type: ignore[assignment]
        setattr(cls, "name", name)
        return cls

    return decorator


def list_planners() -> list[str]:
    if not _REGISTRY:
        ensure_builtin_planners()
    return sorted(_REGISTRY)


def create_planner(name: str, **kwargs: Any) -> Pathfinder:
    if name not in _REGISTRY:
        ensure_builtin_planners()
    if name not in _REGISTRY:
        known = ", ".join(list_planners()) or "(none)"
        raise KeyError(f"unknown planner {name!r}; registered: {known}")
    return _REGISTRY[name](**kwargs)


def ensure_builtin_planners() -> None:
    """Import stub planners so @register side effects run."""
    import swarmpathfinding.planners  # noqa: F401
