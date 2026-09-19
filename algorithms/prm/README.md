# PRM (Probabilistic Roadmap)

## One-sentence idea

Sprinkle random free points, connect neighbors that can see each other, then search that roadmap — a reusable highway for many queries.

## When to use / when not to

- Use for **many start–goal pairs** on the same static (or slowly changing) continuous map (*multi-query*).
- Do not use for a single panicked query — [RRT-Connect](../rrt/README.md) finds *one* path faster.
- Dense narrow corridors need many samples or they stay disconnected.

## World it assumes

Continuous free space (polygons + inflated radius). Collision = shapely / Pymunk shape query.

## How it works

1. Sample N points uniformly in the map; keep those in free space.
2. For each sample, try to connect to its k nearest (or all within radius r) with a straight-line local planner.
3. Add start and goal, connect them to nearby samples.
4. A* on the resulting graph.

**PRM\*** (Karaman & Frazzoli 2011) grows k / r with `log N` so that, as N → ∞, the path cost → optimal (asymptotically optimal).

## Invariants / guarantees

Probabilistically complete (probability of missing a path → 0 as N → ∞, if a path with clearance exists). PRM* is asymptotically optimal. A finite N is neither.

## Complexity

Build: N samples + neighbor LOS tests. Query: A* on a graph of size N. The roadmap is the cache.

## Relation to others

Visibility graph is PRM with vertices = obstacle corners instead of random points. RRT is a *tree* grown for one query.

## Swarm / cache note

**The roadmap is the cache.** All units query it. When a polygon is deleted, drop edges that intersected it and re-sample that neighborhood — do not rebuild N from scratch if you can help it.

## Fog-of-war note

Sample only in last-seen-free space, or allow samples in unknown (freespace) and let collision checks use last-seen blockers.

## Debug overlay contract

Samples, edges, the query path, failed connections. Panel: N, edges, connected-component count, query time vs build time.

## Sources

- Kavraki, Svestka, Latombe, Overmars (1996).
- Karaman & Frazzoli (2011). PRM*.
