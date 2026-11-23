"""Benchmark experiments: many random, seed-synchronised scenarios x planners.

An `ExperimentSpec` says which values each scenario setting may take (chip
sets and numeric ranges). Scenario i is drawn from its own RNG and gets seed
`base_seed + i`, so every planner faces exactly the same scenario i, and the
same spec always produces the same scenarios.

`run_trial` steps one `Simulation` headless (the same code the live lab draws)
and returns one flat result row. `BenchmarkRunner` (see runner.py) runs the
trials in worker processes or visibly in the lab.
"""

from __future__ import annotations

import random
import time
from dataclasses import asdict, dataclass, field
from typing import Any

from swarmpathfinding.core.capabilities import compatibility, scenario_traits
from swarmpathfinding.generators.scenarios import GRID_MAPS, POLY_MAPS, START_LAYOUTS
from swarmpathfinding.sim.simulation import SimConfig, Simulation

DT = 1.0 / 30.0

# Metrics a trial reports; (key, label, lower is better)
METRICS: tuple[tuple[str, str, bool], ...] = (
    ("success", "success", False),
    ("arrived_frac", "arrived", False),
    ("makespan", "makespan s", True),
    ("mean_arrival", "mean arrival s", True),
    ("path_len", "path length", True),
    ("plan_ms", "planning ms", True),
    ("expansions", "expansions", True),
    ("plans", "plans", True),
    ("wall_s", "wall time s", True),
)


@dataclass(slots=True)
class ExperimentSpec:
    """What to sample. Chip sets are lists of allowed values; ranges are (lo, hi)."""

    name: str = "experiment"
    n_scenarios: int = 30
    base_seed: int = 1
    planners: list[str] = field(default_factory=lambda: ["weighted_a_star", "straight_line"])
    worlds: list[str] = field(default_factory=lambda: ["grid"])
    grid_maps: list[str] = field(default_factory=lambda: ["bernoulli", "clustered", "rooms", "maze"])
    poly_maps: list[str] = field(default_factory=lambda: ["polygons", "sparse", "clutter"])
    starts: list[str] = field(default_factory=lambda: ["clump", "scattered"])
    fog: list[bool] = field(default_factory=lambda: [True, False])
    changes: list[str] = field(default_factory=lambda: ["none"])
    vision: list[str] = field(default_factory=lambda: ["circle"])
    density: tuple[float, float] = (0.05, 0.30)
    agents: tuple[int, int] = (1, 20)
    vision_radius: tuple[float, float] = (4.0, 10.0)
    edit_at: tuple[float, float] = (1.0, 6.0)
    w: float = 1.0
    time_limit: float = 60.0  # simulated seconds per trial

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> "ExperimentSpec":
        fields = {k: v for k, v in d.items() if k in cls.__dataclass_fields__}
        for key in ("density", "agents", "vision_radius", "edit_at"):
            if key in fields:
                fields[key] = tuple(fields[key])
        return cls(**fields)

    def problems(self) -> list[str]:
        """Why this spec cannot run (empty when it can)."""
        out = []
        if not self.planners:
            out.append("pick at least one planner")
        worlds = [w for w in self.worlds if self._maps_for(w)]
        if not worlds:
            out.append("pick a world with at least one map")
        for name, chips in (("start layout", self.starts), ("fog setting", self.fog), ("changes", self.changes), ("vision", self.vision)):
            if not chips:
                out.append(f"pick at least one {name}")
        return out

    def _maps_for(self, world: str) -> list[str]:
        maps = self.grid_maps if world == "grid" else self.poly_maps
        valid = GRID_MAPS if world == "grid" else POLY_MAPS
        return [m for m in maps if m in valid]

    def scenario(self, i: int) -> SimConfig:
        """Scenario i (planner left at its default). Same spec + i = same scenario."""
        rng = random.Random(f"{self.base_seed}:{i}")
        worlds = [w for w in self.worlds if self._maps_for(w)] or ["grid"]
        world = rng.choice(worlds)
        maps = self._maps_for(world) or list(GRID_MAPS if world == "grid" else POLY_MAPS)
        lo_a, hi_a = sorted(int(v) for v in self.agents)
        return SimConfig(
            world_kind=world,
            map_kind=rng.choice(maps),
            starts=rng.choice(self.starts or list(START_LAYOUTS)),
            density=round(rng.uniform(*sorted(self.density)), 3),
            n_agents=rng.randint(max(1, lo_a), max(1, hi_a)),
            seed=self.base_seed + i,
            vision=rng.choice(self.vision or ["circle"]),
            vision_radius=round(rng.uniform(*sorted(self.vision_radius)), 2),
            fow=bool(rng.choice(self.fog or [True])),
            changes=rng.choice(self.changes or ["none"]),
            edit_at=round(rng.uniform(*sorted(self.edit_at)), 2),
            w=self.w,
        )

    def scenarios(self) -> list[SimConfig]:
        return [self.scenario(i) for i in range(self.n_scenarios)]

    def trials(self) -> list["Trial"]:
        """Every (scenario, planner) pair, in scenario order. Planners that cannot
        run a scenario are kept but marked skipped, so counts stay honest."""
        out = []
        for i, cfg in enumerate(self.scenarios()):
            traits = scenario_traits(cfg)
            for p in self.planners:
                runnable = compatibility(p, traits).runnable
                out.append(Trial(len(out), i, p, cfg.with_(planner=p), skipped=not runnable))
        return out


@dataclass(slots=True)
class Trial:
    trial_id: int
    scenario: int
    planner: str
    config: SimConfig
    skipped: bool = False


def run_trial(config: SimConfig, time_limit: float = 60.0, dt: float = DT) -> dict[str, Any]:
    """Run one trial headless and return its result row (metrics only)."""
    t0 = time.perf_counter()
    sim = Simulation(config)
    sim.run(max_time=time_limit, dt=dt)
    return trial_metrics(sim, time_limit, time.perf_counter() - t0)


def trial_metrics(sim: Simulation, time_limit: float, wall_s: float) -> dict[str, Any]:
    sc = sim.scenario
    assert sc is not None
    st = sim.stats
    n = max(1, len(sc.agents))
    arrivals = [st.arrived_at[a.id] for a in sc.agents if a.id in st.arrived_at]
    success = len(arrivals) == len(sc.agents)
    return {
        "success": int(success),
        "arrived_frac": round(len(arrivals) / n, 4),
        # failures count as the full time limit, so a planner cannot win by giving up
        "makespan": round(max(arrivals) if success and arrivals else time_limit, 3),
        "mean_arrival": round(sum(arrivals) / len(arrivals), 3) if arrivals else time_limit,
        "path_len": round(sum(st.travelled.values()) / n, 3),
        "plan_ms": round(st.plan_ms, 3),
        "expansions": int(st.expansions),
        "plans": int(st.plans),
        "plan_failures": int(st.plan_failures),
        "plan_ms_after_edit": round(st.plan_ms_after_edit, 3),
        "sim_time": round(sim.now, 3),
        "wall_s": round(wall_s, 3),
    }


def _worker(payload: tuple[dict, float]) -> dict[str, Any]:
    """Process-pool entry point (top level so Windows spawn can import it)."""
    cfg, limit = payload
    return run_trial(SimConfig(**cfg), limit)


def row_for(trial: Trial, metrics: dict[str, Any] | None) -> dict[str, Any]:
    """One CSV / results row: ids, scenario settings, traits, then metrics."""
    cfg = trial.config.as_dict()
    row: dict[str, Any] = {"trial": trial.trial_id, "scenario": trial.scenario, "planner": trial.planner}
    row.update({k: v for k, v in cfg.items() if k != "planner"})
    row.update({f"trait_{k}": v for k, v in scenario_traits(trial.config).items()})
    row["skipped"] = int(trial.skipped)
    if metrics:
        row.update(metrics)
    return row
