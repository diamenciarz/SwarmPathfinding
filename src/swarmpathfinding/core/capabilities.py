"""Which planner suits which scenario, and how it copes with the rest.

Every planner class may set `capabilities = {(trait, value): Rating(...)}`.
Unrated (trait, value) pairs mean "works as is, nothing special". A planner
runs on a scenario unless one of its ratings for that scenario is CANNOT.

Traits are derived from a SimConfig by `scenario_traits`, so the popup menu,
the benchmark runner and the live lab all agree on what a scenario *is*.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Any, Mapping

from swarmpathfinding.core.registry import _REGISTRY, ensure_builtin_planners


class Fit(IntEnum):
    CANNOT = 0  # no honest adaptation; the scenario is skipped for this planner
    ADAPTED = 1  # solves it through a documented adaptation (see the note)
    OK = 2  # works as is, nothing special
    DESIGNED = 3  # the case the algorithm was built for


SYMBOL = {Fit.CANNOT: "x", Fit.ADAPTED: "~", Fit.OK: "ok", Fit.DESIGNED: "*"}


@dataclass(frozen=True, slots=True)
class Rating:
    fit: Fit
    note: str = ""


def designed(note: str = "") -> Rating:
    return Rating(Fit.DESIGNED, note)


def adapted(note: str) -> Rating:
    return Rating(Fit.ADAPTED, note)


def cannot(note: str) -> Rating:
    return Rating(Fit.CANNOT, note)


# trait -> values, in display order
TRAITS: dict[str, tuple[str, ...]] = {
    "world": ("grid", "continuous"),
    "knowledge": ("full", "fog"),
    "changes": ("static", "slow_edit", "movers"),
    "agents": ("single", "few", "crowd"),
    "goals": ("shared", "opposite"),
}
TRAIT_LABELS = {
    "world": "World",
    "knowledge": "Map knowledge",
    "changes": "Changes",
    "agents": "Agents",
    "goals": "Goals",
}
VALUE_LABELS = {
    "grid": "grid",
    "continuous": "polygons",
    "full": "full map",
    "fog": "fog of war",
    "static": "static",
    "slow_edit": "slow edit",
    "movers": "moving obstacles",
    "single": "1",
    "few": "2-10",
    "crowd": "11+",
    "shared": "one goal",
    "opposite": "two groups",
}


def scenario_traits(config: Any) -> dict[str, str]:
    """Trait values of a SimConfig-like object (attributes or a mapping)."""
    get = config.get if isinstance(config, Mapping) else lambda k, d=None: getattr(config, k, d)
    n = int(get("n_agents", 1))
    if get("map_kind") == "movers":
        changes = "movers"
    elif get("changes", "none") == "slow_edit":
        changes = "slow_edit"
    else:
        changes = "static"
    return {
        "world": get("world_kind", "grid"),
        "knowledge": "fog" if get("fow", False) else "full",
        "changes": changes,
        "agents": "single" if n <= 1 else ("few" if n <= 10 else "crowd"),
        "goals": "opposite" if get("starts") == "opposite" else "shared",
    }


def rating(planner: str | type, trait: str, value: str) -> Rating:
    cls = _planner_cls(planner)
    caps: dict = getattr(cls, "capabilities", {}) or {}
    return caps.get((trait, value), Rating(Fit.OK))


@dataclass(frozen=True, slots=True)
class Compatibility:
    fit: Fit  # overall: CANNOT if any trait cannot, DESIGNED if any designed, else worst
    ratings: tuple[tuple[str, str, Rating], ...]

    @property
    def runnable(self) -> bool:
        return self.fit != Fit.CANNOT

    def notes(self) -> list[str]:
        return [r.note for _, _, r in self.ratings if r.note and r.fit in (Fit.ADAPTED, Fit.CANNOT)]


def compatibility(planner: str | type, traits: Mapping[str, str]) -> Compatibility:
    rows = tuple((t, v, rating(planner, t, v)) for t, v in traits.items())
    fits = [r.fit for _, _, r in rows]
    if Fit.CANNOT in fits:
        overall = Fit.CANNOT
    elif Fit.DESIGNED in fits and Fit.ADAPTED not in fits:
        overall = Fit.DESIGNED
    elif Fit.ADAPTED in fits:
        overall = Fit.ADAPTED
    else:
        overall = Fit.OK
    return Compatibility(overall, rows)


def matrix(planners: list[str] | None = None) -> dict[str, dict[tuple[str, str], Rating]]:
    """planner -> (trait, value) -> rating, for every trait value."""
    ensure_builtin_planners()
    names = planners or sorted(_REGISTRY)
    return {n: {(t, v): rating(n, t, v) for t, vals in TRAITS.items() for v in vals} for n in names}


def _planner_cls(planner: str | type):
    if isinstance(planner, type):
        return planner
    ensure_builtin_planners()
    cls = _REGISTRY.get(planner)
    if cls is None:
        raise KeyError(f"unknown planner {planner!r}")
    return cls
