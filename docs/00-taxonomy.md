# Taxonomy

A decision table for this lab. Short names are defined in [glossary.md](glossary.md).

## Family tree (who is whose parent)

```
Dijkstra
  └─ A*
       ├─ greedy best-first          (only h)
       ├─ weighted A*                (f = g + w·h)
       │    └─ ARA*                  (shrink w, reuse expansions)
       ├─ JPS / JPS+                 (skip symmetric grid corridors)
       ├─ Theta* / Lazy Theta*       (any-angle on a grid)
       ├─ HPA*                       (rooms then doors)
       └─ LPA*                       (repair g when edges change)
            └─ D* Lite (km version)  (LPA* with moving start, goal-rooted)
                 └─ AD*              (ARA* + D* Lite)

Sampling
  ├─ PRM / PRM*
  ├─ RRT → RRT-Connect
  │         └─ RRT* → Informed RRT* → RRTX
  └─ BIT* → AIT* / EIT*

Local / physics
  ├─ potential fields
  └─ RVO → ORCA  (+ Pymunk contacts)

RTS cache
  ├─ flow field            (one Dijkstra map per goal)
  └─ flow-field tiles      (Emerson: sectors + portal cache + dirty rebuild)

Incomplete information
  ├─ TangentBug            (almost no map)
  ├─ LRTA*                 (one step of lookahead)
  └─ D* Lite on last-seen  (this lab’s FOW default)
```

Original D\*, Focussed D\*, and the first D\* Lite that rebuilds every key are **ancestors we do not implement**. See [../algorithms/d_star_lite/README.md](../algorithms/d_star_lite/README.md).

## Which algorithm for which question

| Question | First thing to try | Strong alternative | Usually skip |
|---|---|---|---|
| One agent, static **grid**, small | A\* | Dijkstra (sanity check) | Sampling |
| One agent, large mostly-open grid | JPS | weighted A\* | Visibility graph |
| Path *shape* on a grid (not stair-steps) | Theta\* | A\* + post-smooth | — |
| One agent, **sparse polygons** | Visibility graph + A\* | BIT\* | JPS |
| One agent, **dense polygons** | BIT\* or rasterize + A\*/JPS | RRT-Connect (first path) | Visibility graph (too many edges) |
| Map changes slowly (wall deleted) | D\* Lite | flow-field dirty tiles | Restart A\* every time |
| Obstacles with **known** future motion | SIPP | space-time A\* | D\* Lite alone |
| Unpredictable movers | stale global field + ORCA + physics | RRTX | D\* Lite as a predictor (it is not one) |
| Many units, **same goal**, known map | flow field | D\* Lite field (goal-rooted) | Per-unit A\* |
| Many units, **same goal**, large map, slow edits | **flow-field tiles** (see [03-rts-cache.md](03-rts-cache.md)) | HPA\* + local field | Full-map field every frame |
| Units packed in a clump | one path for the centroid + ORCA | flow field | Per-unit global search |
| Fog of war, known goal | D\* Lite on the **last-seen map** | TangentBug (no-map baseline) | ACO / pheromone as the planner |
| Need a path *this frame*, improve later | weighted A\* / ARA\* / BIT\* | AD\* if the map also changes | Optimal CBS on a physics world |

## Grid vs continuous (this lab’s split)

Continuous worlds use **Pymunk** rigid bodies. Grid worlds stay on cells so we can compare discrete search fairly.

Many “grid algorithms” still run *under* a continuous world: rasterize a coarse occupancy grid for D\* Lite or a flow field, then steer the physical circles with ORCA. That hybrid is how RTS engines work.

## What we are *not* classifying as a path planner here

- **CBS / PIBT / ECBS** — discrete MAPF. Useful contrast on a grid with no physics; not the swarm solution on continuous maps. Mentioned in [01-sota-overview.md](01-sota-overview.md), no algorithm folder in v1.
- **Classical ACO** — optimizer on a known graph; slower than A\* for this domain. See [../algorithms/pheromone_stigmergy/README.md](../algorithms/pheromone_stigmergy/README.md).
- **Berliner B\*** — game-tree proof procedure. Dropped.
