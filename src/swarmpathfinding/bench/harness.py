from __future__ import annotations

import time
from pathlib import Path

import pandas as pd

from swarmpathfinding.bench.metrics import BenchRow
from swarmpathfinding.core.registry import create_planner, ensure_builtin_planners
from swarmpathfinding.core.results import Budget
from swarmpathfinding.generators.packs import ScenarioSpec, build_spec, pack_specs
from swarmpathfinding.generators.scenarios import apply_pending_edit


def run_spec(spec: ScenarioSpec, algorithm: str, weight: float = 1.0) -> BenchRow:
    ensure_builtin_planners()
    scenario = build_spec(spec)
    planner = create_planner(algorithm)
    budget = Budget(weight=weight)
    starts = [agent.pos for agent in scenario.agents]
    t0 = time.perf_counter()
    result = planner.plan(scenario.belief, starts, scenario.goal, budget)
    plan_ms = (time.perf_counter() - t0) * 1000.0
    if result.runtime_ms:
        plan_ms = result.runtime_ms

    replan_ms = 0.0
    rebuilt = 0
    if spec.slow_edit and scenario.pending_edit is not None:
        gen0 = scenario.world.generation
        apply_pending_edit(scenario)
        rebuilt = max(0, scenario.world.generation - gen0)
        t1 = time.perf_counter()
        again = planner.plan(scenario.belief, starts, scenario.goal, budget)
        replan_ms = (time.perf_counter() - t1) * 1000.0
        if again.runtime_ms:
            replan_ms = again.runtime_ms
        if again.success:
            result = again

    extra = 0.0
    if spec.n_agents > 1 and result.success:
        extra = 0.0  # filled once we have per-agent paths in Phase 4

    return BenchRow(
        scenario_id=spec.scenario_id,
        seed=spec.seed,
        algorithm=algorithm,
        world_kind=spec.world_kind,
        density=scenario.density,
        n_agents=spec.n_agents,
        success=result.success,
        path_length=result.cost if result.success else float("inf"),
        plan_ms=plan_ms,
        expansions=result.expansions,
        replan_ms=replan_ms,
        cells_or_sectors_rebuilt=rebuilt,
        fraction_revealed=scenario.belief.revealed_fraction(),
        samples=result.samples,
        rewires=result.rewires,
        pack=spec.pack,
        map_kind=spec.map_kind,
        start_layout=spec.start_layout,
        slow_edit=spec.slow_edit,
        extra_length_vs_single=extra,
    )


def run_pack(
    pack: str,
    algorithm: str,
    *,
    seeds: tuple[int, ...] = (0,),
    weight: float = 1.0,
    limit: int | None = None,
) -> list[BenchRow]:
    specs = pack_specs(pack, seeds=seeds)
    if limit is not None:
        specs = specs[:limit]
    return [run_spec(spec, algorithm, weight=weight) for spec in specs]


def write_csv(path: str | Path, rows: list[BenchRow]) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    df = pd.DataFrame([row.as_dict() for row in rows])
    df.to_csv(path, index=False)
    return path
