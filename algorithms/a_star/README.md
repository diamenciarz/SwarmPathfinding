# A*

## One-sentence idea

Walk outward from the start, always expanding the cell that looks cheapest **so far plus a guess of the rest**, until you hit the goal.

## When to use / when not to

- Use as the **baseline** on a known static graph (grid or visibility graph).
- Use when you need an **optimal** path and the map is small or the query is rare.
- Do not use naively for hundreds of units every frame — share a field or cache ([../../docs/03-rts-cache.md](../../docs/03-rts-cache.md)).
- Do not restart A* on every tiny map edit — that is what [D\* Lite](../d_star_lite/README.md) is for.

## World it assumes

Any graph with non-negative edge costs. This lab: 4- or 8-connected occupancy grids, or the visibility graph of polygons.

## How it works

Each node `n` has:

- `g(n)` — cost of the best path found from **start** to `n`
- `h(n)` — heuristic guess of cost from `n` to **goal**
- `f(n) = g(n) + h(n)` — pop the smallest `f` from a priority queue (the *open* set)

When you expand `n`, look at each neighbor `n'`. If going through `n` improves `g(n')`, record the parent pointer and push `n'` on open. *Closed* nodes are already expanded. With a **consistent** `h`, you never need to re-open closed nodes.

Stop when you expand the goal. Reconstruct the path by following parents backward.

**Worked 4-connected snippet** (cost 1, Manhattan `h`):

```
S . .
. # G
```

Manhattan from S to G is 3. A* expands S, then the cell right of S (f = 1+2) and the cell below S (blocked neighbor aside), and so on, until G. It will not wander into the far corner because those cells have large `h`.

### Heuristics on our grids

| Neighborhood | Admissible `h` |
|---|---|
| 4-connected, cost 1 | Manhattan: `|Δx| + |Δy|` |
| 8-connected, cost 1 / √2 | Octile: `max(Δx,Δy) + (√2-1)·min(Δx,Δy)` |
| Any-angle / continuous | Euclidean: `√(Δx²+Δy²)` |

If `h` never overestimates, A* is optimal. If you inflate it, you get [weighted A*](../weighted_a_star/README.md).

## Invariants / guarantees

- Complete on finite graphs.
- Optimal if `h` is admissible (and, for the common “don’t re-open closed” implementation, consistent).
- Not incremental, not anytime.

## Complexity

Worst case you expand most of the map (bad `h`, or a labyrinth). With a good `h` on an open grid, the expanded region is a fat ellipse around the start–goal line. Time ~ expansions × log(open) with a heap.

## Relation to others

- Parent: Dijkstra is A* with `h = 0`.
- Children: weighted A*, JPS, Theta*, LPA* / D* Lite, ARA*.

## Swarm / cache note

Nothing is shared. N units ⇒ N searches unless they share a [flow field](../flow_field/README.md) or you plan once for a [cluster centroid](../group_pathfinding/README.md).

## Fog-of-war note

You *can* run A* on the last-seen map and recompute from scratch when last-seen changes. That is correct and wasteful. D* Lite is the same first search, then cheaper repairs.

## Debug overlay contract

Open set, closed set, `g` or `f` heatmap, parent arrows (search tree), final path. Panel: expansions, open size, path cost, time.

## Sources

- Hart, Nilsson, Raphael (1968).
- Amit Patel’s *Introduction to A\** (redblobgames) — best student explainer.
