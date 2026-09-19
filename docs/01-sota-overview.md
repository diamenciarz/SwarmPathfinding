# SOTA overview

Short names: [glossary.md](glossary.md). Decision table: [00-taxonomy.md](00-taxonomy.md). Citations: [references.md](references.md).

This is a map of the literature as it applies to *this* lab, not a complete history of motion planning.

## Two problems that look similar but are not

| | **MAPF** (warehouse / discrete) | **Physical RTS swarm** (this project’s default) |
|---|---|---|
| State | Agents occupy graph vertices / grid cells | Continuous positions, radii, velocity, mass, contacts |
| Collision | Same cell or swapping an edge at the same timestep | Rigid-body overlap / impulse (Pymunk) |
| Typical solvers | CBS, ECBS, EECBS, PIBT, LNS | Shared global field or cached path + **ORCA** + physics |
| Scale in papers | Classical: 200×200 grids, 1000+ agents (Wang, Xu et al. 2025) | ORCA: thousands of agents in milliseconds (van den Berg et al. 2011) |

On **continuous maps we always use physics**. Discrete MAPF solvers stay as an optional *grid-only contrast*, not as the swarm solution. No CBS/PIBT folder in v1.

## Grid worlds

- **A\*** (Hart, Nilsson, Raphael 1968) is still the baseline. Optimal if `h` is admissible and consistent. On uniform 4/8-connected grids the usual `h` is Manhattan / octile.
- **JPS** (Harabor & Grastien, AAAI 2011) is the practical SOTA for *uniform-cost* grids: it skips symmetric corridors and is often more than 10× faster than A\* with **zero preprocessing**. **JPS+** adds offline jump-point tables.
- **HPA\*** (Botea, Müller, Schaeffer 2004) is “plan rooms then doors.” Useful on large maps; not optimal unless you refine.
- **Theta\*** / **Lazy Theta\*** (Nash et al. 2007/2010) keep a grid but allow **any-angle** shortcuts when line of sight is free. Paths look continuous; they are *not* guaranteed Euclidean-optimal. **Anya** (Harabor et al. 2016) *is* optimal any-angle, at more implementation cost.
- **JPS** shines on large, mostly-open grids. **A\*** is simpler and often enough on small or dense maps. **Theta\*** is for path *shape*, not speed.

## Continuous worlds (obstacles = n-polygons + physics)

Density decides which family wins.

| Density | Prefer | Why |
|---|---|---|
| **Sparse** (few polygons, tiny occupancy) | **Visibility graph** + A\* (Lozano-Pérez & Wesley 1979) | Shortest path among polygons *is* a tangent/vertex path. Graph size is O(V²) in vertices — cheap when V is small. Exact for a translating point. |
| **Dense** (many vertices, narrow corridors) | **Sampling** (PRM, RRT, RRT\*, **BIT\***) or a coarse grid / navmesh | Visibility graphs explode; line-of-sight tests dominate. |
| **Very dense / maze-like** | Rasterize + A\*/JPS, or a **navmesh** | Sampling wastes shots in narrow passages. |

Sampling SOTA for *static* continuous 2D:

- **RRT / RRT-Connect** — fast first feasible path, not optimal.
- **RRT\*** / **PRM\*** (Karaman & Frazzoli 2011) — asymptotically optimal (converge to the optimum as samples → ∞).
- **Informed RRT\*** — after the first path, sample only inside an ellipsoid that can still improve it.
- **BIT\*** / **AIT\*** / **EIT\*** (Gammell et al.) — typically the best *anytime* continuous planners in recent comparisons: faster first solution *and* faster convergence than RRT\*.

**Potential fields** are local, cheap, and fail in U-shaped obstacles (local minima). Steering layer only.

**This lab’s hybrid:** agents and obstacles stay continuous and physical. Flow fields and D\* Lite may still run on a coarse occupancy grid *under* that world. StarCraft 2, Planetary Annihilation, and Riot’s Swarm all do some version of this.

## Dynamic maps

Three different problems:

1. **Unknown static terrain / fog of war.** You discover walls as you walk. **D\* Lite** was built for this (Koenig & Likhachev, AAAI 2002 / IEEE TRO 2005). Same *paths* as Stentz’ Focussed D\* (1995), but it is LPA\* with a moving start, not a simplified D\*. **This is the default FOW planner.** Details: [../algorithms/d_star_lite/README.md](../algorithms/d_star_lite/README.md).
2. **Known moving obstacles** (trajectories known). **SIPP** (Phillips & Likhachev, ICRA 2011) searches *safe time intervals* per cell. Same optimality as space-time A\*, much smaller state space.
3. **Unpredictable movers.** Incremental graph repair is not a crystal ball.
   - Grid: D\* Lite / **AD\*** for the *map*, plus a **local** avoider.
   - Continuous + physics: **RRTX** (Otte & Frazzoli 2015/2016) to repair a goal-rooted tree, *or* a slightly stale global field + **ORCA** + Pymunk.

**LPA\*** searches start→goal with a *fixed* start. **D\* Lite** reverses the search (goal is the root) because the *agent* moves. A goal-rooted tree stays valid as the agent walks; you only repair where edge costs (or last-seen obstacles) changed.

D\* Lite does **not** predict motion. A pedestrian can walk into the path after you planned (Zhao 2025, street-sim study).

## Good-enough vs optimal

The engineering default is bounded-suboptimal or anytime:

- **Weighted A\***: `f = g + w·h` with `w > 1`. Bounded-suboptimal if `h` is admissible.
- **ARA\*** (Likhachev, Gordon, Thrun 2003): start with large `w`, then decrease, *reusing* expansions.
- **AD\***: ARA\* + D\* Lite. Anytime *and* incremental. The “I have 5 ms this frame” algorithm.
- **RRT-Connect / BIT\***: first feasible path quickly, then improve.

Pay for optimality only on small static instances or when you need a proof.

## RTS swarms: reuse is a cache problem

StarCraft-style: many units, **one click-to-move target**, starts scattered. The literature’s answer is not “run A\* per unit.” It is **a cache of shared searches**, invalidated regionally. Full recipe: [03-rts-cache.md](03-rts-cache.md).

Headline results from practice:

- One **flow field** (Dijkstra from the goal) serves every unit going to that goal, from *anywhere*. Cost is O(map), independent of agent count.
- Emerson (*Game AI Pro* ch. 23): split the map into **sectors**, cache a small field per **portal**, dirty only the sector that changed. Two armies sharing a valley share tiles even with different final goals.
- Riot *Swarm*: one Dijkstra map; minions **stamp** their collision onto the field so later minions path around clumps.

**ORCA** (van den Berg, Guy, Lin, Manocha 2011) is the local layer: each agent solves a tiny linear program in velocity space, taking half of each pairwise collision. Decentralized, millisecond-scale for thousands of agents. It will not go around a building. It sits *under* a global planner, *on top of* Pymunk.

## Fog of war

Not the same as “dynamic obstacles.” The ground-truth world can be static; the **last-seen map** is stale.

This lab’s model (see [04-fog-of-war.md](04-fog-of-war.md)):

- Last-seen map = obstacle **positions** + **velocities** (zero if static).
- Each agent has a **VisionShape**. If it sees a discrepancy, it updates the shared last-seen map for the whole swarm.
- Fog = complement of the **union** of all vision shapes (a polygon; rebuild it on a delay / skipped frames).

Default planner: D\* Lite on that last-seen map, with the **freespace assumption** (unknown space is treated as walkable until seen).

**Pheromone / ACO:** not used as a planner. Attractive trails duplicate D\* Lite’s `g` field; classical ACO is an optimizer on a *known* graph. Reasoning: [../algorithms/pheromone_stigmergy/README.md](../algorithms/pheromone_stigmergy/README.md).

## Learning-based MAPF

Wang, Xu et al. 2025 survey 200+ papers. Classical methods still scale to more agents; learned policies handle fewer agents but adapt to partial observability. Out of scope for v1.
