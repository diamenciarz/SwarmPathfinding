# Glossary

Read this once. The rest of the docs use the short names.

## Algorithms and fields

| Short name | Full name | One-line meaning |
|---|---|---|
| **SOTA** | state of the art | Best currently known methods in the literature |
| **RTS** | real-time strategy | Games like StarCraft: many units, shared map, fog of war |
| **MAPF** | multi-agent path finding | Discrete problem: many agents on a graph, no two occupy the same cell or edge at the same time |
| **A\*** | A-star | Heuristic graph search; optimal if the heuristic never overestimates |
| **D\*** | Dynamic A-star (Stentz 1994/95) | Incremental replanner for changing maps; searches from the goal. Replaced in practice by D\* Lite |
| **D\* Lite** | Dynamic A-star Lite | Same *paths* as D\*, much simpler; built on LPA\*. This lab implements only the `km` version |
| **LPA\*** | Lifelong Planning A-star | Incremental A\*: repairs `g` values when edge costs change; start is fixed |
| **ARA\*** | Anytime Repairing A-star | Weighted A\* that starts sloppy and tightens the weight, reusing work |
| **AD\*** | Anytime Dynamic A-star | ARA\* + D\* Lite: anytime *and* incremental |
| **JPS** | Jump Point Search | A\* that skips symmetric grid corridors; often 10× faster on uniform grids |
| **JPS+** | Jump Point Search plus | JPS with precomputed jump distances |
| **HPA\*** | Hierarchical Pathfinding A-star | Plan on a coarse “rooms and doors” graph, refine locally |
| **Theta\*** / **Lazy Theta\*** | Theta-star | A\* that takes any-angle shortcuts when line of sight is free |
| **Anya** | (named algorithm) | Optimal any-angle pathfinding on a grid |
| **RRT** | Rapidly-exploring Random Tree | Grow a tree of random samples until you hit the goal |
| **RRT-Connect** | RRT with two trees | Grow from start *and* goal; first feasible path, not optimal |
| **RRT\*** | RRT-star | RRT that rewires toward shorter paths; asymptotically optimal |
| **Informed RRT\*** | Informed RRT-star | After the first path, sample only inside an ellipsoid that can improve it |
| **PRM** / **PRM\*** | Probabilistic Roadmap | Sample free space, connect nearby samples, then search the graph |
| **BIT\*** | Batch Informed Trees | Batches of samples + A\*-like search; strong anytime continuous planner |
| **AIT\*** / **EIT\*** | Adaptive / Effort Informed Trees | BIT\* descendants with better heuristics |
| **RRTX** | RRT-X | RRT\* that *repairs* the tree when obstacles move (continuous analogue of D\* Lite) |
| **SIPP** | Safe Interval Path Planning | Plan in space *and* time using “this cell is free from t1 to t2” intervals |
| **ORCA** | Optimal Reciprocal Collision Avoidance | Each agent takes half of each pairwise collision in velocity space |
| **RVO** | Reciprocal Velocity Obstacles | Predecessor of ORCA; same idea, more oscillation |
| **CBS** | Conflict-Based Search | Optimal discrete MAPF: plan each agent, then branch on collisions |
| **ECBS** / **EECBS** | Enhanced / Explicit Estimation CBS | Bounded-suboptimal CBS (faster, slightly longer paths) |
| **PIBT** | Priority Inheritance with Backtracking | Very fast suboptimal discrete MAPF; scales to thousands of agents |
| **LNS** | Large Neighborhood Search | Destroy-and-repair local search for MAPF |
| **ACO** | Ant Colony Optimization | Metaheuristic: ants deposit pheromone on a *known* graph. Not used as a planner here |
| **LRTA\*** | Learning Real-Time A-star | Take one step, update the heuristic, repeat; works with little lookahead |
| **LOS** | line of sight | Straight-line visibility between two points |
| **FOW** | fog of war | Everything outside the union of the agents’ vision shapes |
| **PDE** | partial differential equation | Continuum Crowds solves one on a grid to get a flow |
| **LP** | linear program | Tiny optimization ORCA solves per agent per tick |
| **UI** | user interface | On-screen controls and debug drawings |

## Three words that decide “good enough vs optimal”

**Admissible heuristic.** `h(n)` never *overestimates* the true remaining cost. Example: Euclidean (straight-line) distance when every step costs at least that far. A\* with an admissible heuristic finds an **optimal** path.

**Inadmissible heuristic.** `h(n)` is allowed to overestimate. Search is faster (it charges toward the goal) but may miss the true shortest path. Greedy best-first is the extreme: it uses only `h`.

**Bounded-suboptimal.** You pick a number `w ≥ 1` (the *suboptimality bound*). The algorithm may return a path whose cost is at most `w` times the optimal cost. Weighted A\* with an *admissible* `h` and `f = g + w·h` is bounded-suboptimal with that `w`.

- `w = 1` → optimal A\*
- `w = 1.1` → “at most 10% longer than optimal, please be fast”

**Anytime** means: return *some* path quickly, then spend leftover time making it better. ARA\* and BIT\* are anytime.

**Incremental** means: when the map changes a little, *repair* the previous search instead of starting from scratch. LPA\*, D\* Lite, AD\*, RRTX are incremental.

**Consistent (or monotone) heuristic.** A stronger property than admissible: `h(n) ≤ cost(n, n') + h(n')` for every edge. Consistency implies admissible. A\* with a consistent heuristic never needs to re-open a closed node.

## Letters you will see on a search node

- **g(n)** — cost of the best path found so far from the search *root* to `n`. In A\* the root is the start. In D\* Lite the root is the goal, so `g` is “cost to the goal.”
- **h(n)** — heuristic guess of remaining cost.
- **f(n) = g(n) + h(n)** — A\*’s priority. Weighted A\* uses `g + w·h`.
- **rhs(n)** — D\* Lite / LPA\* “one-step lookahead”: `min over neighbors n' of (cost(n→n') + g(n'))`. A node is *consistent* when `g = rhs`.
- **km** — D\* Lite’s key modifier. Avoids rebuilding the whole priority queue when the robot moves.

## World and swarm words

- **Last-seen map** — shared copy of obstacle *positions* and, if they move, *velocities*. Updated when any agent’s vision sees a discrepancy.
- **VisionShape** — the region one agent can currently see (disk or cone, later clipped by line of sight).
- **Fog of war** — the complement of the **union** of all agents’ vision shapes. A (possibly ugly) polygon; it does not need to be rebuilt every physics frame.
- **Flow field** — a direction stored in every cell, pointing downhill on a distance-to-goal map. One field, many units.
- **Dirty region / dirty sector** — the part of a cached field that must be rebuilt after a wall appears or vanishes.
