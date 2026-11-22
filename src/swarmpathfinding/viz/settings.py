from __future__ import annotations

import argparse
from dataclasses import dataclass, replace

from swarmpathfinding.generators.packs import ScenarioSpec
from swarmpathfinding.generators.scenarios import GRID_MAPS, POLY_MAPS, START_LAYOUTS
from swarmpathfinding.viz.panel import SidePanel


@dataclass(slots=True)
class LabSettings:
    world_kind: str = "grid"
    map_kind: str = "bernoulli"
    planner: str = "weighted_a_star"
    vision: str = "circle"
    starts: str = "clump"
    density: float = 0.15
    n_agents: int = 8
    vision_radius: float = 6.0
    w: float = 1.0
    seed: int = 1
    fow: bool = True
    los: bool = True
    show_search: bool = False

    @classmethod
    def from_spec(cls, spec: ScenarioSpec, planner: str = "weighted_a_star") -> LabSettings:
        return cls(
            world_kind=spec.world_kind,
            map_kind=spec.map_kind,
            planner=planner,
            vision=spec.vision_kind,
            starts=spec.start_layout,
            density=spec.density if spec.density > 0 else 0.15,
            n_agents=spec.n_agents,
            vision_radius=spec.vision_radius,
            seed=spec.seed if spec.seed else 1,
            fow=spec.fow,
        )


PRESETS: dict[str, LabSettings] = {
    "fog_scout": LabSettings(
        map_kind="bernoulli",
        n_agents=10,
        starts="clump",
        fow=True,
        vision="circle",
        vision_radius=7.0,
        density=0.15,
    ),
    "rts_flanks": LabSettings(
        map_kind="street",
        n_agents=30,
        starts="flanks",
        fow=False,
        density=0.15,
    ),
    "opposite_corridor": LabSettings(
        map_kind="corridor",
        n_agents=20,
        starts="opposite",
        fow=False,
    ),
    "slow_edit_rooms": LabSettings(
        map_kind="rooms",
        n_agents=1,
        fow=False,
    ),
    "utrap": LabSettings(
        world_kind="continuous",
        map_kind="utrap",
        n_agents=4,
        fow=False,
        starts="clump",
    ),
    "movers_ghost": LabSettings(
        world_kind="continuous",
        map_kind="movers",
        n_agents=8,
        fow=True,
        vision="circle",
        vision_radius=8.0,
        starts="scattered",
    ),
    "sparse_polygons": LabSettings(
        world_kind="continuous",
        map_kind="sparse",
        n_agents=1,
        fow=False,
        density=0.08,
    ),
    "maze": LabSettings(map_kind="maze", n_agents=1, fow=False),
}


def parse_lab_args(argv: list[str] | None = None) -> LabSettings:
    parser = argparse.ArgumentParser(description="SwarmPathfinding Pygame live lab.")
    parser.add_argument("--preset", choices=sorted(PRESETS), default="")
    parser.add_argument("--pack", default="", help="scenario pack name; use with --index")
    parser.add_argument("--index", type=int, default=0, help="index into the pack")
    parser.add_argument("--world", dest="world_kind", choices=("grid", "continuous"), default=None)
    parser.add_argument("--map", dest="map_kind", default=None)
    parser.add_argument("--planner", default=None)
    parser.add_argument("--vision", choices=("circle", "cone", "none"), default=None)
    parser.add_argument("--starts", choices=START_LAYOUTS, default=None)
    parser.add_argument("--density", type=float, default=None)
    parser.add_argument("--agents", dest="n_agents", type=int, default=None)
    parser.add_argument("--vision-radius", dest="vision_radius", type=float, default=None)
    parser.add_argument("--w", type=float, default=None)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--fow", dest="fow", action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument("--los", dest="los", action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument(
        "--show-search", dest="show_search", action=argparse.BooleanOptionalAction, default=None
    )
    args = parser.parse_args(argv)

    settings = LabSettings()
    if args.preset:
        settings = replace(PRESETS[args.preset])
    if args.pack:
        from swarmpathfinding.generators.packs import pack_specs

        specs = pack_specs(args.pack, seeds=(args.seed or 0,))
        if not specs:
            raise SystemExit(f"pack {args.pack!r} is empty")
        idx = min(max(args.index, 0), len(specs) - 1)
        settings = LabSettings.from_spec(specs[idx], planner=args.planner or "weighted_a_star")

    updates = {}
    for name in (
        "world_kind",
        "map_kind",
        "planner",
        "vision",
        "starts",
        "density",
        "n_agents",
        "vision_radius",
        "w",
        "seed",
        "fow",
        "los",
        "show_search",
    ):
        value = getattr(args, name)
        if value is not None and value != "":
            updates[name] = value
    if updates:
        settings = replace(settings, **updates)
    return settings


def apply_settings(panel: SidePanel, settings: LabSettings) -> None:
    """Write settings into the side panel (the map list follows world_kind)."""
    _set_cycle(panel, "world_kind", settings.world_kind)
    wanted = list(GRID_MAPS) if settings.world_kind == "grid" else list(POLY_MAPS)
    maps = next(c for c in panel.cycles if c.key == "map_kind")
    maps.options = wanted
    _set_cycle(panel, "map_kind", settings.map_kind)
    _set_cycle(panel, "planner", settings.planner)
    _set_cycle(panel, "vision", settings.vision)
    _set_cycle(panel, "starts", settings.starts)
    _set_slider(panel, "density", settings.density)
    _set_slider(panel, "n_agents", settings.n_agents)
    _set_slider(panel, "vision_radius", settings.vision_radius)
    _set_slider(panel, "w", settings.w)
    _set_slider(panel, "seed", max(1, settings.seed))
    _set_check(panel, "fow", settings.fow)
    _set_check(panel, "los", settings.los)
    _set_check(panel, "show_search", settings.show_search)


def _set_cycle(panel: SidePanel, key: str, value: str) -> None:
    cycle = next(c for c in panel.cycles if c.key == key)
    if value in cycle.options:
        cycle.index = cycle.options.index(value)


def _set_slider(panel: SidePanel, key: str, value: float) -> None:
    slider = next(s for s in panel.sliders if s.key == key)
    slider.value = min(slider.hi, max(slider.lo, value))
    if slider.integer:
        slider.value = float(int(round(slider.value)))


def _set_check(panel: SidePanel, key: str, value: bool) -> None:
    check = next(c for c in panel.checks if c.key == key)
    check.value = bool(value)
