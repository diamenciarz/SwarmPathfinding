# Visibility graph

## One-sentence idea

Among polygons, a shortest path for a point robot only turns at **obstacle vertices**. Connect mutually visible vertices, then run A*.

## When to use / when not to

- Use in **sparse** continuous worlds: few polygons, tiny occupancy, exact geometry matters.
- Do not use when vertex count is huge (dense clutter): O(V²) candidate edges, each needing a LOS test. Then sample ([BIT*](../bit_star/README.md), [RRT*](../rrt_star/README.md)) or rasterize.
- Inflate polygons by the agent radius first (Minkowski sum). We plan for a **point**; Pymunk circles have radius.

## World it assumes

2D polygons (shapely), static for the query. Physics still simulates the bodies; the graph is a planning overlay.

## How it works

1. Vertices = start, goal, and every obstacle corner (after inflation).
2. Edge `u–v` exists if the open segment `uv` does not hit any obstacle (**line of sight**).
3. Edge weight = Euclidean length.
4. A* (or Dijkstra) on that graph.

Why this is enough: a shortest path among polygonal obstacles is a polyline that hugs vertices (taut string). Proof is classical (Lozano-Pérez & Wesley 1979).

**Worked sketch:** one rectangle between S and G. The graph has S, G, and 4 corners. Two corners see both S and G; A* returns S → near corner → far corner → G, the taut path.

## Invariants / guarantees

Complete. **Optimal** for a translating point (or inflated-radius disk) among static polygons. Not for rotating shapes, not for moving obstacles unless you rebuild.

## Complexity

Naive: O(V²) LOS tests. Fine for tens of vertices. Painful for thousands. Polygon simplification (fewer corners, same homotopy) is fair game. Spatial indexes (AABB tree) for LOS.

Rebuild cost on a wall-delete: only edges that involved that polygon’s vertices.

## Relation to others

- Discrete cousin: Theta* approximates this on a grid.
- Sampling cousins: PRM is a *random* visibility graph.
- Dynamic continuous: RRTX repairs a sampled graph instead.

## Swarm / cache note

The **graph** is shared. A* queries are per start, but all units can share the graph. Cache key = last-seen polygon set. When one wall disappears, delete its vertices and incident edges, re-test LOS only where that polygon used to block.

## Fog-of-war note

Build the graph from **last-seen** polygons, not truth. Unknown space has no vertices (freespace: you try a straight line S–G if LOS in the belief).

## Debug overlay contract

All vertices, all visibility edges (faint), the A* path (bold), inflated outlines. Panel: V, E, LOS tests, path length.

## Sources

- Lozano-Pérez, T., & Wesley, M. A. (1979). *CACM*.
