# Dijkstra

## One-sentence idea

A* with no guess: expand closest-to-start first until you have distances to everyone (or until you hit the goal).

## When to use / when not to

- Use as a **sanity check** against A* (same path on uniform costs if A*’s `h` is admissible).
- Use when you need **distances from one source to many targets** — that is a [flow field](../flow_field/README.md): Dijkstra *from the goal*, then every cell stores a downhill arrow.
- Do not use as a per-unit pather on a big map; A* expands fewer nodes.

## World it assumes

Graph with non-negative edge costs. Grid or portal graph.

## How it works

Priority queue ordered by `g(n)` only (`h = 0`). Pop the smallest `g`, relax neighbors. On a uniform grid this is a diamond (4-connected) or a rounded diamond (8-connected) growing from the source.

**Flow-field view:** run Dijkstra from the *goal*. Then at each cell, the arrow points to the neighbor with smallest `g`. That field is the integration field in Emerson / Riot Swarm.

## Invariants / guarantees

Complete; optimal. Not informed, not incremental, not anytime.

## Complexity

Expands every node closer to the source than the goal (for a single query). For a full field: every reachable cell, once.

## Relation to others

A* = Dijkstra + `h`. Flow field = Dijkstra from the goal + arrows.

## Swarm / cache note

A full Dijkstra field is *the* shared structure for a common goal. Cache by goal. Dirty the field when costs change, or repair with D* Lite.

## Fog-of-war note

Can run on last-seen costs. Prefer D* Lite if those costs twitch.

## Debug overlay contract

`g` heatmap, closed set (the filled region), path, for a field: arrows. Panel: expansions, max `g`.

## Sources

- Dijkstra, E. W. (1959). A note on two problems in connexion with graphs.
- “Dijkstra maps” in rogue-like / game-AI folklore (same algorithm, goal-rooted).
