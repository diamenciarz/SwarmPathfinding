# Flow-field tiles (Emerson)

## One-sentence idea

Don’t build a flow field for the whole map: split into **sectors**, A* on the **portal graph**, cache a tiny field per portal hop, and when a wall vanishes **dirty only that sector**.

## When to use / when not to

- Use on **large RTS maps** with many units, shared *or* different goals that share corridors. This is the production recipe we will demo live (you asked to see it).
- Do not use on a 32×32 toy map — a single [flow field](../flow_field/README.md) is clearer.
- Do not rebuild the world every frame; the cache is the algorithm.

## World it assumes

Grid partitioned into sectors (e.g. 8×8 or 16×16 cells). Portals = contiguous walkable runs on sector borders.

## How it works

Full walkthrough: [../../docs/03-rts-cache.md](../../docs/03-rts-cache.md). Short version:

1. Sectors → portals → portal graph (edge cost = cheap A* *inside one sector*).
2. Move order: A* on portals from here to the goal’s sector → a highway of portals.
3. For each hop, a **tile**: a mini flow field on that sector pointing at the next portal (or the goal). **Cache key = (sector id, portal window id)**.
4. Units follow the tile under their feet. If physics shoves them off-highway, request that sector’s tile on demand.
5. **Wall deleted in sector B:** mark B dirty; rebuild B’s portals and neighbor portal-graph edges; drop cached tiles that used those portals; time-slice the rebuilds.

Two armies through the same valley share tiles even with different final goals.

## Invariants / guarantees

Not globally grid-optimal (highway + local field is hierarchical, like HPA*). Complete if the portal graph is. The *point* is bounded rebuild cost after a local edit, not optimality.

## Complexity

Portal A* is tiny (dozens of nodes). Tile build is O(sector cells). Cache hits are O(1) lookups. A wall flip costs O(one sector), not O(map).

## Relation to others

- Parent: flow field + HPA*.
- Cell-level cousin: D* Lite ripple vs sector-level dirty flag.

## Swarm / cache note

This **is** the cache. Side panel must show hits/misses, resident tiles, dirty sectors, ms spent rebuilding this frame. That overlay *is* the experiment.

## Fog-of-war note

Portal graph from last-seen occupancy. Unknown cells are walkable, so a portal may exist in fog that a wall will later split — then that sector goes dirty when a scout looks.

## Debug overlay contract

Sector grid, portals as thick door segments, highway highlighted, per-sector arrows, **dirty tint**, tiles currently being built. Panel: cache size, hits, misses, dirty queue, rebuild ms, sectors touched by last wall-delete vs map size.

## Sources

- Emerson, E. (2013). Crowd pathfinding and steering using flow field tiles. *Game AI Pro*, ch. 23.
