# Flow field (Dijkstra map)

## One-sentence idea

Run Dijkstra **once from the goal**; every cell stores a downhill arrow. Any number of units, from any start, just follow the arrow.

## When to use / when not to

- Use when **many units share a goal** (RTS click-to-move, Riot Swarm). This is the first swarm optimization to implement.
- Do not use when every unit has a unique goal — you need one field per goal, which explodes. Then [flow-field tiles](../flow_field_tiles/README.md) (share *portals*) or per-unit JPS.
- Full-map fields get expensive on huge maps; tiles fix that.

## World it assumes

Grid (or a coarse grid under polygons). Units may be continuous circles that *read* the cell they sit in.

## How it works

Three arrays, one value per cell (Emerson’s layout):

1. **Cost field** — 1 for floor, 255 for wall, maybe extra for swamp. Units can **stamp** their footprint as extra cost so the next integration paths around a clump.
2. **Integration field** — Dijkstra / BFS from the goal: `integration[c] = min over neighbors n of integration[n] + cost[c]`. Goal is 0. Walls stay at max.
3. **Flow field** — at each cell, arrow toward the neighbor with smallest integration (8-connected).

Each unit: look up the cell, add the arrow to desired velocity, then ORCA / Pymunk.

**Why starts don’t matter:** the field is goal-rooted. Ten units in a clump and ten across the map read the same arrows.

When the goal is static, you can skip cells whose best cost already came from that goal and only lower them if a *moving* goal offers something cheaper (Riot).

## Invariants / guarantees

Optimal *grid* paths for a single agent on the cost field. With stamps, paths are congestion-aware, not optimal. Not incremental unless you replace integration with D* Lite.

## Complexity

O(cells) to build, O(1) per unit per tick. Independent of N after the build. This is the whole point.

## Relation to others

- Dijkstra from the goal + arrows.
- Incremental version: D* Lite’s `g` map *is* an integration field you can repair.
- Hierarchical version: flow-field tiles.

## Swarm / cache note

Cache key = goal cell (+ stamp generation if you stamp). Invalidate on cost edits. For one wall flip, either recompute the field or repair with D* Lite — measure both in `slow_edit`.

## Fog-of-war note

Cost field from last-seen occupancy. Unknown = walkable (cost 1). A newly seen wall dirties the field.

## Debug overlay contract

Cost (walls), integration heatmap, flow arrows, stamp footprints, units’ desired vs actual velocity. Panel: build ms, N, cache hits, cells updated this tick.

## Sources

- Emerson, E. *Game AI Pro* ch. 23 (also tiles).
- Treuille, Cooper, Popović (2006). Continuum crowds (PDE version of the same idea).
- Riot Games, “The tech behind Swarm.”
