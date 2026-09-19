# Theta* (and Lazy Theta*)

## One-sentence idea

A* on a grid that is allowed to connect a cell to a **parent-of-parent** if the straight line is free, so paths cut corners instead of walking in stair-steps.

## When to use / when not to

- Use when you stay on a **grid** but want paths that *look* continuous (units, ships).
- Do not use when you need a true Euclidean shortest path among polygons — that is a [visibility graph](../visibility_graph/README.md) (or Anya). Theta* is **not** guaranteed Euclidean-optimal.
- Do not use when speed on a uniform grid is the only goal — [JPS](../jps/README.md) is faster and grid-optimal (but stair-stepped).

## World it assumes

Occupancy grid plus a **line-of-sight** test (Bresenham / supercover) that refuses to squeeze through diagonal pinches.

## How it works

Ordinary A*, 8-connected. When you relax neighbor `n'` from `n`:

- If `LOS(parent(n), n')`, consider the path `start ↝ parent(n) → n'` with cost `g(parent(n)) + Euclidean(parent(n), n')`.
- Else the usual `g(n) + Euclidean(n, n')`.

So the search tree can contain long diagonal edges that skip grid steps. Path vertices are still grid points (usually corners).

**Lazy Theta\*** delays the LOS check until a node is expanded, assuming LOS and fixing it if the assumption was wrong. Faster, slightly worse paths.

## Invariants / guarantees

Complete. Paths are any-angle and usually a few percent shorter than 8-connected A*. Not Euclidean-optimal (Anya is).

## Complexity

Same expansions as A*, plus a LOS test per relaxation (Theta*) or per expansion (Lazy). LOS is the cost: 2–8× slower than A* in wall-clock on large maps is common; the path looks much better.

## Relation to others

- Parent: A*.
- Optimal any-angle: Anya (not in v1 unless we want it later).
- Incremental any-angle: [Field D*](../field_d_star/README.md).

## Swarm / cache note

Per query. A flow field of *grid* arrows plus a short LOS smooth near the unit is often cheaper for crowds.

## Fog-of-war note

LOS must run on last-seen occupancy, not truth (otherwise you cheat through fog).

## Debug overlay contract

Search tree with **straight** parent segments (this is the point), LOS rays that succeeded/failed, grid path vs Theta* path. Panel: path length vs A*, LOS tests, expansions.

## Sources

- Nash, Daniel, Koenig, Felner (2007). Theta*: Any-angle path planning on grids. *AAAI*.
- Nash & Koenig — Lazy Theta*.
