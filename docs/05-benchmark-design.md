# Benchmark design

Every run writes one CSV row with the same columns. Density is a number, not a vibe.

## Worlds

**Grid generators**

- Bernoulli noise: each cell blocked with probability `p ∈ {0.05, 0.15, 0.30, 0.45}`.
- Clustered blobs (cellular automata / drunkard walk) — few large obstacles vs many specks at the same occupancy.
- Rooms + corridors (BSP / dungeon).
- Open / maze / street analogues of the MovingAI families.

**Continuous generators** (Pymunk bodies)

- `n` random convex k-gons, occupancy fraction `ρ` (area of union / map area) in `{0.02, 0.10, 0.30, 0.50}`.
- Concave U-traps (potential-field killers; visibility graph still exact).
- Sparse giants: 3–5 large polygons.
- Dense clutter: 80–200 small polygons.
- Moving polygons with constant velocity (for SIPP vs D\* Lite vs ORCA).

Start and goal are rejection-sampled in free space, with a margin.

## Scenario packs

| Pack | What it answers |
|---|---|
| `single_static` | Grid vs continuous, density sweep, one agent, known map |
| `slow_edit` | After a path exists, **delete one wall**. Cells/sectors rebuilt vs full recompute |
| `rts_shared_goal` | N ∈ {1, 10, 30, 100}, same goal; starts (a) one clump, (b) scattered, (c) two flanks |
| `physics_crowd` | Same as RTS, but Pymunk + ORCA on; last-unit arrival, contacts |
| `fow_discover` | Disk/cone vision, shared last-seen, D\* Lite freespace |
| `fow_ghost` | Mover leaves vision; collisions vs *truth*, stale-belief incidents |
| `opposite` | Two groups, opposite goals, corridor / street (ORCA stress) |

Seeds are part of the scenario id so runs are reproducible.

CLI:

```
python -m swarmpathfinding.bench --pack single_static --algo straight_line
python -m swarmpathfinding.generators --pack slow_edit --out maps/generated
```

## Metrics (CSV columns)

- `scenario_id`, `seed`, `algorithm`, `world_kind`, `density`, `n_agents`
- `success` (bool)
- `path_length` (single agent) / `makespan` / `sum_of_costs`
- `plan_ms`, `expansions` or `samples` or `rewires`
- `replan_ms` after an edit (the D\* Lite / dirty-tile number)
- `cells_or_sectors_rebuilt`, `cache_hits`, `cache_misses`
- swarm: `extra_length_vs_single`, `last_arrival`, `physics_contacts`, `global_replans`
- FOW: `fraction_revealed`, `collisions_vs_truth`, `stale_ghost_incidents`
- `suboptimality` vs a reference (A\* or visibility-graph) when the reference finishes

Time is wall-clock in the process that ran the planner, not frame time, unless the row is a live-lab snapshot (`frame_ms`).
