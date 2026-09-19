# D* Lite

## One-sentence idea

Keep a distance-to-goal map and, when a wall appears, disappears, or is *revealed in fog*, repair only the ripple of cells whose numbers actually changed — instead of running A* again.

## When to use / when not to

- Use when the **goal is stable**, the agent (or army) **moves**, and the map (or last-seen map) **changes a little at a time**: unknown terrain, fog of war, a building destroyed, a door opened.
- Use as a **shared field**: one D* Lite map per goal serves every unit walking to that goal, from any start.
- Do not use when the map is static and known — A* or JPS is simpler and enough.
- Do not use as a **predictor** of moving obstacles. It reacts *after* an edge cost (or last-seen pose) changes. For chaos, put ORCA on top of a slightly stale field.
- Do not use the original D*, Focussed D*, or the first D* Lite that rebuilds every key in the heap. Those find the same paths and are strictly worse to implement. **This lab implements only the `km` version** (Koenig & Likhachev, IEEE TRO 2005, “second version”).

## World it assumes

A graph. In this lab: an 8-connected grid (uniform cost 1, blocked = infinity), or a coarse grid rasterized under a polygonal physics world. The search is **goal-rooted**.

It can run on **ground truth** (cheating, fully known) or on the **last-seen map** (fog of war). Default FOW: unknown cells are **walkable** (freespace assumption).

## How it works

### The picture

A* from the goal is a **heatmap**: each cell stores “how far is the goal.” You walk by always stepping to a neighbor with a smaller number.

If a wall appears, most numbers are still right. Only a **ripple** around the wall is wrong. D* Lite’s job is to find that ripple and fix those cells.

### Three numbers per cell

- **g(u)** — what we currently believe it costs to reach the **goal** from `u`.
- **rhs(u)** — one-step lookahead:

  `rhs(u) = 0` if `u` is the goal, else `min over neighbors n of (cost(u→n) + g(n))`.

- A cell is **consistent** when `g(u) = rhs(u)`. If every cell that matters is consistent, `g` *is* the true distance-to-goal.

Path: from the robot, step to a neighbor `n` that minimizes `cost(robot→n) + g(n)`. Repeat. Ties break arbitrarily.

When an edge cost changes (wall appears or vanishes, or a scout updates the last-seen map):

1. Recompute `rhs` of the cells that touch the changed edges.
2. Cells with `g ≠ rhs` are **inconsistent**. Push them on a priority queue.
3. Pop the most important inconsistent cell, make it consistent (or push `g` to infinity if it was too optimistic), update neighbors’ `rhs`.
4. Stop when the robot’s cell is consistent **and** nothing cheaper than the robot is still waiting in the queue.

Two flavors of inconsistent:

- **Overconsistent** (`g > rhs`): we found a *better* way (a wall vanished, or we had not computed this cell yet). Set `g ← rhs`.
- **Underconsistent** (`g < rhs`): we were too optimistic (a wall appeared). Set `g ← ∞` so the cell is recomputed from its neighbors.

Each cell is expanded at most twice per repair (once under, once over). That is the termination argument.

### Why search backward from the goal

The **goal does not move**. The **robot does**. A tree rooted at the goal stays valid as you walk — you just start from a different leaf. A tree rooted at the robot would have its root move every step.

That is also why **one field serves a whole army** with the same target: starts do not appear in the stored numbers.

**LPA\*** is the same repair logic with a *fixed* start, searching start→goal. D* Lite is LPA* with start and goal swapped, plus `km` so the moving robot does not force a heap rebuild. Read [../lpa_star/README.md](../lpa_star/README.md) first if the `rhs` idea is new.

### The `km` trick (the version we implement)

When *one* robot moves, the heuristic `h(robot, u)` changes for every `u`. A naive implementation would **recompute every key in the heap**. That is the first D* Lite in the 2002/2005 papers — same paths, extra work. We do not implement it.

The second version keeps a running offset **km** (key modifier):

- Keys in the heap are *lower bounds* on the “true” keys.
- When the robot moves from `s` to `s_new` and then notices a cost change, add `h(s, s_new)` to `km`.
- New keys are computed as:

  `k(u) = [ min(g(u), rhs(u)) + h(robot, u) + km ,  min(g(u), rhs(u)) ]`

- Compare keys lexicographically (first component, then second).

Old keys stay in the heap. If you pop a stale key, you throw it away (or re-insert with the fresh key). You never sort the whole queue.

**Shared-goal RTS field:** if the field is not tied to one robot, units just *read* `g`. You do not need per-unit `km`. `km` matters when a *private* D* Lite follows one agent (different start each tick). Implement `km` anyway: it is cheap and it is the standard algorithm.

### Worked example (small grid)

Eight-connected, cost 1, octile heuristic. Start `S` at `(0,0)`, goal `G` at `(3,2)`. Unknown cells are assumed free.

```
y=2   .  .  .  G
y=1   .  .  #  .     # is a wall the robot does not know yet
y=0   S  .  .  .
      x=0 1  2  3
```

**First search.** This *is* A* from `G`. After `ComputeShortestPath`, `g` on free cells is the octile distance to `G`. The greedy path from `S` looks like a diagonal toward `G`, through `(2,1)` — the cell that is secretly a wall.

```
g after first search (∞ on the unknown wall, but we treated it as 0 extra):

y=2   3  2  1  0
y=1   3  2  1  1
y=0   4  3  2  2
```

**The robot steps to `(1,0)`.** Path still looks fine. No cost change, so no repair.

**The robot steps to `(2,0)` and *sees* `(2,1)` blocked.** Edge costs into `(2,1)` become ∞. `rhs(2,1)` becomes ∞. Neighbors that used `(2,1)` as a best successor become inconsistent — a small set, not the whole map.

Repair raises `g` around the wall just enough that the downhill path from the robot now goes `(2,0) → (3,1) → (3,2)`. Cells behind the robot, and cells whose best route never used the wall, keep their old `g`.

That is the whole trick. On a 100×100 map, discovering one wall might touch tens of cells, not ten thousand.

(The paper’s figure is the same story with chess-style cell names B1→E3. Same ripple.)

### Freespace and fog of war

D* Lite’s original job *is* fog of war: plan as if unseen space is empty; when a VisionShape hits a wall, bump those edge costs and repair. In this lab the wall write goes to the **shared last-seen map**, so the first scout to see the wall repairs the field the whole army is reading. See [../../docs/04-fog-of-war.md](../../docs/04-fog-of-war.md).

A wall *disappearing* (or a last-seen blocker that is no longer there when someone looks) is a cost *decrease*. That is the slow-edit RTS case. Same code path.

### What we will not implement

| Name | Why it exists | Why we skip it |
|---|---|---|
| D\* (Stentz 1994) | First incremental heuristic replanner | Long, nested conditions; hard to extend. Same paths as D\* Lite |
| Focussed D\* | D\* plus a focusing heuristic | Same |
| D\* Lite, first version | Rebuilds all keys when the robot moves | Same paths; the `km` version avoids the rebuild |
| LPA\* as the live pather | Teaching sibling (fixed start) | Documented; optional later. Live lab uses D\* Lite |

## Invariants / guarantees

- Complete on a finite graph with finite positive costs.
- Optimal (same path as A\* / Focussed D\*) if `h` is admissible and, for the `km` version, **forward-backward consistent** (octile / Euclidean on a uniform grid is fine).
- Incremental: first call ≡ A\*; later calls repair.
- Not anytime by itself. For “return a sloppy path now, tighten later” use AD\* ([../ad_star/README.md](../ad_star/README.md)).
- Does not forecast moving obstacles.

## Complexity

Per repair, work is proportional to the **inconsistent ripple**, not the map, in the common case (a local edit, `h` focusing the queue toward the robot). Worst case a big edit near the goal can touch a large fraction of the map — then you have paid about as much as A\*.

Priority queue: binary heap is enough. Keys are pairs of floats.

## Relation to others

- Parent: [LPA\*](../lpa_star/README.md).
- Sibling: A\* is the first iteration.
- Child: [AD\*](../ad_star/README.md) = this + anytime weights.
- Continuous analogue: [RRTX](../rrtx/README.md).
- Shared-goal cousin: [flow field](../flow_field/README.md) is Dijkstra from the goal *without* repair; [flow-field tiles](../flow_field_tiles/README.md) repair by dropping a *sector* instead of a ripple of cells.

## Swarm / cache note

One D\* Lite field per goal. Cache key = goal cell (and the last-seen map generation). When a scout updates last-seen, run `UpdateVertex` on the touched cells and `ComputeShortestPath` once — not once per unit. Units only follow `g`.

If units have **different goals**, you have different fields (or fall back to portal tiles).

## Fog-of-war note

Runs on the last-seen map. Unknown = walkable. Visible discrepancies update last-seen, then this planner. Ghost movers (last-seen pose + velocity) are cost maps that *move* between observations; treat a ghost as a moving blocker and dirty its old and new cells.

## Debug overlay contract

**Map**

- Heatmap of `g` (and a toggle for `rhs`).
- Inconsistent cells highlighted.
- Priority-queue “frontier” of the current repair.
- Path as greedy descent from each selected agent.
- Flash cells that changed `g` on the last repair (the ripple).

**Panel**

- Queue size, expansions this repair, expansions lifetime.
- `km`.
- Cells made inconsistent by the last edit.
- Replan milliseconds vs a “restart A*” counterfactual if we compute it.

## Sources

- Koenig, S., & Likhachev, M. (2002). D\* Lite. *AAAI*.
- Koenig, S., & Likhachev, M. (2005). Fast replanning for navigation in unknown terrain. *IEEE Trans. Robotics* 21(3). **Implement the second version (Figure 5 in that paper).**
- Koenig, S., & Likhachev, M. (2002/2004). LPA\*.
- Stentz, A. (1994/95). D\* / Focussed D\* — historical only.
