# Jump Point Search (JPS)

## One-sentence idea

A* on a uniform grid that **skips down empty corridors** in one hop, because all the stair-step paths through a hall cost the same.

## When to use / when not to

- Use on **large, mostly open, uniform-cost grids**. This is the SOTA single-agent grid pather without preprocessing (Harabor & Grastien, AAAI 2011). Often >10× fewer expansions than A*.
- Do not use on non-uniform costs (swamp tiles, portals with weird weights) — symmetry breaks. Use A*.
- Do not use on polygons; there is no grid symmetry. (You may rasterize first.)
- **JPS+** precomputes jump distances. Faster queries, extra memory, rebuild when the map changes. Prefer plain JPS if walls flip often.

## World it assumes

8-connected (sometimes 4) occupancy grid, uniform traversal cost.

## How it works

On an empty grid, many shortest paths are **symmetric**: right-then-up costs the same as up-then-right. A* expands every cell on that rectangle. JPS says: only stop at a **jump point**, a cell where the symmetry breaks — a “forced neighbor” created by a wall, or the goal.

From a node, you **scan** (not expand) in a direction until you hit a wall, a forced neighbor, or the goal. That landing cell is the successor A* sees. Intermediate cells are never pushed on the heap.

Forced neighbor, picture:

```
. . x          moving right into this row
# n .          wall below `n` forces you to also consider the diagonal up-right
```

When the cell left of `n` is a wall and you arrived horizontally, `n` has a forced diagonal neighbor. `n` is a jump point.

The A* loop is unchanged: `f = g + h` (octile). Only the neighbor generator is special.

## Invariants / guarantees

Complete; **optimal** on uniform 8-connected grids (same cost as A*). No extra memory in the online form.

## Complexity

Expansions ~ number of jump points on the path’s “interesting” frontier, not corridor length. Scan cost is linear in skipped cells but cheap (tight loop, no heap).

## Relation to others

- Parent: A*.
- Cousins: Rectangular Symmetry Reduction; HPA* (different kind of skip).
- Not a replacement for Theta*: JPS paths still follow grid edges (you can post-smooth).

## Swarm / cache note

Per query. For a shared goal, a flow field still wins at large N. JPS is the right *portal* pather or single-unit pather on a big open grid.

## Fog-of-war note

Works on last-seen occupancy. Map edits require restarting (or JPS+ rebuild). D* Lite is better if edits are frequent and local.

## Debug overlay contract

Draw jump points as dots, scan rays as lines, ordinary A* open/closed for comparison (toggle). Panel: expansions vs A* expansions, path cost (should match A*).

## Sources

- Harabor, D., & Grastien, A. (2011). Online graph pruning for pathfinding on grid maps. *AAAI*.
- Harabor’s blog / zerowidth visual explanation of JPS.
