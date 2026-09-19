# ARA* (Anytime Repairing A*)

## One-sentence idea

Run weighted A* with a large `w` to get a path *now*, then shrink `w` and **reuse** the old expansions until you hit `w = 1` or time runs out.

## When to use / when not to

- Use when you have a **time budget** (one frame, 5 ms) and a static or slowly changing graph.
- Do not use as the only tool if the map is thrashing — that is [AD*](../ad_star/README.md) (this + D* Lite).
- Do not use if you always wait for the optimum anyway; then A* is simpler.

## World it assumes

Same as A*.

## How it works

1. Pick `w_0` large (e.g. 2.5). Run weighted A*. You have a path with bound `w_0`.
2. Drop `w` (e.g. 2.5 → 2.0 → 1.5 → 1.0).
3. Nodes already expanded with a worse `w` may be **inconsistent** under the new weight. ARA* keeps an `INCONS` list of those, seeds the next search from them, and does not start from scratch.
4. At any moment you can interrupt and return the current best path plus the current `w` as the suboptimality bound.

“Repairing” = reuse, not map repair. Map repair is D* Lite / AD*.

## Invariants / guarantees

Anytime: after the first solution, you always have a feasible path. Each iteration is bounded-suboptimal with the current `w`. With enough time, `w → 1` and the path is optimal.

## Complexity

First solution ≈ weighted A*. Later iterations are cheaper than independent restarts because of reuse. Still not cheap if the first `w` was too small.

## Relation to others

- Parent: weighted A*.
- Combined with D* Lite → AD*.

## Swarm / cache note

Per query. Fine for one centroid path under a frame budget.

## Fog-of-war note

Restarts (or AD*) when last-seen changes.

## Debug overlay contract

Current path, current `w`, overlay of `INCONS`, a strip chart of path cost vs time. Panel: bound `w`, cost, expansions per iteration.

## Sources

- Likhachev, Gordon, Thrun (2003). ARA*: Anytime A* with provable bounds on sub-optimality. *NIPS*.
