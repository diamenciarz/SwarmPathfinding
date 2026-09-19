# RTS cache: reuse, dirty sectors, one wall disappears

This is the StarCraft case: many units, **one target**, starts that may be a clump or scattered across the map. Map edits are **slow** (a building finishes, a wall is destroyed). The question is not “which A\* variant is fastest per unit.” It is **how much of the search is a cache, and how small a rebuild you can get away with**.

We *do* want to see this live in Pygame: flow arrows, dirty sectors tinted, cache hit/miss on the side panel.

## What can be reused

| Shared thing | Who can reuse it | Invalidated when |
|---|---|---|
| **Flow field / Dijkstra map for a goal** | Every unit going to that same goal, from *anywhere* | Goal cell moves, or any cost in the field’s region changes |
| **Flow-field *tile* for a portal** | Any path that crosses the same sector portal, even different final goals | That sector’s cost field is dirtied |
| **Portal / highway graph** | All units | A sector’s connectivity changes |
| **One A\* / D\* Lite path for a tight cluster** | Units within a few radii of each other | Cluster splits, or the path’s corridor is dirtied |
| **ORCA / physics** | Never shared (local), but cheap | — |

A goal-rooted structure (flow field, D\* Lite `g` map) is the important trick: **starts do not matter**. Ten units in a clump and ten units on the other side of the map read the *same* numbers.

## Emerson’s production recipe (flow-field tiles)

Elijah Emerson, “Crowd Pathfinding and Steering Using Flow Field Tiles,” *Game AI Pro*, ch. 23. This is the version RTS engines actually ship, and the one this lab will demo.

1. Split the map into **sectors** (for example 10×10 cells, or larger).
2. Find **portals** — contiguous walkable openings on sector borders. Portals are the “doors.”
3. Build a **portal graph**: nodes are portals, an edge exists if you can walk inside one sector from portal A to portal B. Edge cost = a cheap A\* (or Dijkstra) *inside that one sector*.
4. For a move order: A\* on the **portal graph** from the unit’s sector to the goal’s sector → a list of portals to cross. This is the highway.
5. For each hop, build a **small flow field** on that sector that points at the *next* portal (or at the goal, on the last sector). **Cache key = (sector, portal-window id)**.
6. Units look up the cached tile for the cell they stand on and follow the arrow. If they get shoved into a sector that was not on the highway, request that tile on demand.
7. **When a wall disappears** (or appears):
   - Mark that sector’s cost field **dirty**.
   - Rebuild portals on the dirty sector and its neighbors (a door may have opened).
   - Drop cached tiles whose key used those portals.
   - Leave every other sector’s cache alone.
   - Rebuild is **time-sliced** across frames (a millisecond budget), so one explosion does not hitch the game.

Two armies walking through the same valley **share tiles** even if they are going to different bases, because many hops are “cross this portal into that portal.”

### Tiny example

Map of 3×2 sectors. A wall sits in sector B, blocking the only portal between A and C.

```
[ A ]--portal--[ B █ ]--blocked--[ C  goal ]
   units
```

Portal-graph A\* says: A → B is possible, B → C is not. Units in A cannot yet have a field into C.

The wall in B is deleted.

- Only B (and portals on the A|B and B|C borders) go dirty.
- Portal graph grows one edge B→C.
- Cached field “A toward portal A|B” may still be valid.
- New fields: “B toward portal B|C”, “C toward goal.”
- Units already in A do not recompute a map-sized Dijkstra.

That delta — *sectors rebuilt vs full recompute* — is the KPI for this scenario. See [05-benchmark-design.md](05-benchmark-design.md).

## Non-hierarchical version (Riot Swarm, Dijkstra maps)

If the map is small enough, skip sectors:

- One integration field from the goal (Dijkstra / fast marching).
- Every unit, from any start, follows decreasing cost.
- Units can **stamp** their occupied cells as extra cost so the next wave paths around a clump (soft congestion).
- When the goal is static and a *moving* goal also writes into the field, a cell updates only if the moving goal offers a **lower** cost (Riot’s extra trick).

When a single wall flips, you *could* restart the Dijkstra. D\* Lite is the same field with a **ripple repair** instead of a restart. On a small map the difference is modest; on a large map with rare edits, D\* Lite (or dirty tiles) wins.

## D\* Lite is the cell-level version of the same idea

A wall disappearing is an **edge-cost decrease**. Only inconsistent cells around that wall are repaired. A wall appearing is a cost increase. Slow, rare edits are D\* Lite’s best case. Fast chaotic motion is not — then keep the field slightly stale and dodge with ORCA.

One shared D\* Lite field per goal serves the whole army. There is no per-unit `km` if nobody owns the field; units just read `g` at their cell. Per-unit `km` only matters if you run a private D\* Lite per agent (different goals, or private last-seen maps).

## What the Pygame overlay must show

- Flow arrows (or a hue for integration cost).
- Sector boundaries and portals.
- Dirty sectors in a warning tint until they finish rebuilding.
- Cache: hits, misses, tiles resident, milliseconds spent this frame on rebuild.
- After a punch-wall / delete-wall: flash the ripple (D\* Lite) or the dirty sector (tiles).

Algorithm folders: [../algorithms/flow_field/README.md](../algorithms/flow_field/README.md), [../algorithms/flow_field_tiles/README.md](../algorithms/flow_field_tiles/README.md).
