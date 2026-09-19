from __future__ import annotations

from dataclasses import asdict, dataclass


@dataclass(slots=True)
class BenchRow:
    scenario_id: str
    seed: int
    algorithm: str
    world_kind: str
    density: float
    n_agents: int
    success: bool
    path_length: float
    plan_ms: float
    expansions: int
    replan_ms: float = 0.0
    cells_or_sectors_rebuilt: int = 0
    cache_hits: int = 0
    cache_misses: int = 0
    extra_length_vs_single: float = 0.0
    last_arrival: float = 0.0
    physics_contacts: int = 0
    global_replans: int = 0
    fraction_revealed: float = 1.0
    collisions_vs_truth: int = 0
    stale_ghost_incidents: int = 0
    makespan: float = 0.0
    sum_of_costs: float = 0.0
    samples: int = 0
    rewires: int = 0
    suboptimality: float = float("inf")
    pack: str = ""
    map_kind: str = ""
    start_layout: str = ""
    slow_edit: bool = False

    def as_dict(self) -> dict:
        return asdict(self)
