# Algorithms

One folder per algorithm. Right now each folder is a student `README.md` only. `planner.py` / `debug.py` come later.

Template: [_template.md](_template.md). Short names: [../docs/glossary.md](../docs/glossary.md).

## Wave 1 — foundations

| Folder | Role |
|---|---|
| [dijkstra](dijkstra/) | Uninformed baseline; flow fields are Dijkstra from the goal |
| [a_star](a_star/) | Common ancestor |
| [greedy_best_first](greedy_best_first/) | Contrast: `h` only |
| [weighted_a_star](weighted_a_star/) | Good-enough knob `w` |
| [jps](jps/) | Fast uniform grids |
| [lpa_star](lpa_star/) | Prerequisite for D* Lite (documented; live lab uses D* Lite) |
| [d_star_lite](d_star_lite/) | Incremental replanner we will implement (`km` version only) |
| [ara_star](ara_star/) | Anytime repairing |
| [ad_star](ad_star/) | Anytime + dynamic |

## Wave 2 — continuous & any-angle

| Folder | Role |
|---|---|
| [theta_star](theta_star/) | Any-angle on a grid |
| [visibility_graph](visibility_graph/) | Exact sparse polygons |
| [prm](prm/) | Multi-query roadmap |
| [rrt](rrt/) | RRT + RRT-Connect |
| [rrt_star](rrt_star/) | Asymptotically optimal sampling |
| [bit_star](bit_star/) | Default anytime continuous |
| [potential_fields](potential_fields/) | Local; shows local minima |

## Wave 3 — dynamic continuous / time

| Folder | Role |
|---|---|
| [rrtx](rrtx/) | Continuous analogue of D* Lite |
| [sipp](sipp/) | Known moving obstacles |
| [field_d_star](field_d_star/) | Incremental any-angle (after D* Lite) |

## Wave 4 — RTS swarm, cache, physics

| Folder | Role |
|---|---|
| [flow_field](flow_field/) | Shared-goal Dijkstra map |
| [flow_field_tiles](flow_field_tiles/) | Emerson sectors + portal cache + dirty rebuild |
| [orca](orca/) | Local avoidance on Pymunk |
| [group_pathfinding](group_pathfinding/) | One path for a packed clump |

## Wave 5 — fog of war

| Folder | Role |
|---|---|
| [bug_tangent](bug_tangent/) | No-map baseline |
| [lrta_star](lrta_star/) | One-step lookahead |
| [pheromone_stigmergy](pheromone_stigmergy/) | **Not implemented as a planner** — why |

Fog-of-war D* Lite lives in [d_star_lite](d_star_lite/), not a second folder.
