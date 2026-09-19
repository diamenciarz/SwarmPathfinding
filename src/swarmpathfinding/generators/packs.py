"""Named scenario packs from docs/05-benchmark-design.md."""

from __future__ import annotations

from dataclasses import dataclass

from swarmpathfinding.generators.scenarios import Scenario, make_scenario

GRID_DENSITIES = (0.05, 0.15, 0.30, 0.45)
POLY_DENSITIES = (0.02, 0.10, 0.30, 0.50)
PACKS = (
    "single_static",
    "slow_edit",
    "rts_shared_goal",
    "physics_crowd",
    "fow_discover",
    "fow_ghost",
    "opposite",
)


@dataclass(frozen=True, slots=True)
class ScenarioSpec:
    pack: str
    world_kind: str
    map_kind: str
    density: float
    n_agents: int
    seed: int
    start_layout: str = "clump"
    fow: bool = False
    vision_kind: str = "circle"
    vision_radius: float = 6.0
    shared_belief: bool = True
    slow_edit: bool = False
    width: int = 48
    height: int = 36

    @property
    def scenario_id(self) -> str:
        return (
            f"{self.pack}-{self.world_kind}-{self.map_kind}-p{self.density:.2f}"
            f"-n{self.n_agents}-{self.start_layout}-s{self.seed}"
        )


def list_packs() -> tuple[str, ...]:
    return PACKS


def pack_specs(pack: str, seeds: tuple[int, ...] = (0,)) -> list[ScenarioSpec]:
    if pack not in PACKS:
        raise KeyError(f"unknown pack {pack!r}; known: {', '.join(PACKS)}")
    specs: list[ScenarioSpec] = []
    for seed in seeds:
        specs.extend(_specs_for(pack, seed))
    return specs


def build_spec(spec: ScenarioSpec) -> Scenario:
    return make_scenario(
        world_kind=spec.world_kind,
        map_kind=spec.map_kind,
        density=spec.density,
        n_agents=spec.n_agents,
        seed=spec.seed,
        start_layout=spec.start_layout,
        vision_kind=spec.vision_kind,
        vision_radius=spec.vision_radius,
        fow=spec.fow,
        width=spec.width,
        height=spec.height,
        pack=spec.pack,
        slow_edit=spec.slow_edit,
        shared_belief=spec.shared_belief,
        scenario_id=spec.scenario_id,
    )


def _specs_for(pack: str, seed: int) -> list[ScenarioSpec]:
    if pack == "single_static":
        out: list[ScenarioSpec] = []
        for p in GRID_DENSITIES:
            out.append(_g(pack, "bernoulli", p, 1, seed))
            out.append(_g(pack, "clustered", p, 1, seed))
        for kind in ("empty", "rooms", "maze", "street", "open", "game"):
            out.append(_g(pack, kind, 0.0, 1, seed))
        for rho in POLY_DENSITIES:
            out.append(_c(pack, "polygons", rho, 1, seed))
        out.append(_c(pack, "sparse", 0.08, 1, seed))
        out.append(_c(pack, "clutter", 0.35, 1, seed))
        out.append(_c(pack, "utrap", 0.0, 1, seed))
        return out
    if pack == "slow_edit":
        return [
            _g(pack, "bernoulli", 0.15, 1, seed, slow_edit=True),
            _g(pack, "clustered", 0.30, 1, seed, slow_edit=True),
            _g(pack, "rooms", 0.0, 1, seed, slow_edit=True),
            _g(pack, "street", 0.0, 1, seed, slow_edit=True),
            _c(pack, "polygons", 0.10, 1, seed, slow_edit=True),
            _c(pack, "sparse", 0.08, 1, seed, slow_edit=True),
        ]
    if pack == "rts_shared_goal":
        out = []
        for n in (1, 10, 30, 100):
            for layout in ("clump", "scattered", "flanks"):
                out.append(_g(pack, "bernoulli", 0.15, n, seed, start_layout=layout))
                out.append(_g(pack, "street", 0.0, n, seed, start_layout=layout))
        return out
    if pack == "physics_crowd":
        out = []
        for n in (10, 30):
            for layout in ("clump", "scattered"):
                out.append(_c(pack, "polygons", 0.10, n, seed, start_layout=layout))
                out.append(_c(pack, "movers", 0.08, n, seed, start_layout=layout))
        return out
    if pack == "fow_discover":
        return [
            _g(pack, "bernoulli", 0.15, 8, seed, fow=True, shared=True),
            _g(pack, "bernoulli", 0.15, 8, seed, fow=True, shared=False),
            _g(pack, "rooms", 0.0, 8, seed, fow=True, shared=True, vision="cone"),
            _c(pack, "sparse", 0.08, 8, seed, fow=True, shared=True),
        ]
    if pack == "fow_ghost":
        return [
            _c(pack, "movers", 0.08, 8, seed, fow=True, shared=True),
            _c(pack, "movers", 0.08, 8, seed, fow=True, shared=False),
        ]
    if pack == "opposite":
        return [
            _g(pack, "corridor", 0.0, 20, seed, start_layout="opposite"),
            _g(pack, "street", 0.0, 20, seed, start_layout="opposite"),
        ]
    return []


def _g(
    pack: str,
    map_kind: str,
    density: float,
    n: int,
    seed: int,
    start_layout: str = "clump",
    fow: bool = False,
    shared: bool = True,
    slow_edit: bool = False,
    vision: str = "circle",
) -> ScenarioSpec:
    return ScenarioSpec(
        pack=pack,
        world_kind="grid",
        map_kind=map_kind,
        density=density,
        n_agents=n,
        seed=seed,
        start_layout=start_layout,
        fow=fow,
        vision_kind=vision,
        shared_belief=shared,
        slow_edit=slow_edit,
    )


def _c(
    pack: str,
    map_kind: str,
    density: float,
    n: int,
    seed: int,
    start_layout: str = "clump",
    fow: bool = False,
    shared: bool = True,
    slow_edit: bool = False,
) -> ScenarioSpec:
    return ScenarioSpec(
        pack=pack,
        world_kind="continuous",
        map_kind=map_kind,
        density=density,
        n_agents=n,
        seed=seed,
        start_layout=start_layout,
        fow=fow,
        shared_belief=shared,
        slow_edit=slow_edit,
    )
