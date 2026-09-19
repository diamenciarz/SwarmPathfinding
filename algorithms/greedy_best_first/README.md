# Greedy best-first search

## One-sentence idea

Always expand the cell that *looks* closest to the goal (`h` only). Fast, and happy to walk into a trap.

## When to use / when not to

- Use as a **contrast** in the live lab: same map as A*, watch it hug the heuristic and fail on U-shaped walls.
- Do not use when you care about path length or about finishing at all in a maze.

## World it assumes

Same graphs as A*.

## How it works

Priority queue ordered by `h(n)` only. No `g`. Parent pointers still build a path. On an open field it looks brilliant (almost a straight line). On a concave obstacle it slides into the pocket and may wander the long way around — or, if you close nodes carelessly, get stuck.

This is the extreme **inadmissible** / unweighted case of “ignore true cost.”

## Invariants / guarantees

Not optimal. Completeness depends on tie-breaking and re-opening; treat it as **not a serious pather**.

## Complexity

Often fewer expansions than A* on open maps; can be worse in mazes.

## Relation to others

A* interpolates between Dijkstra (`h = 0`) and this (`g = 0`). Weighted A* with huge `w` behaves similarly but still tracks `g`.

## Swarm / cache note

Nothing shared. Not worth caching.

## Fog-of-war note

Will charge into unknown space even more eagerly than freespace A*. Fine as a demo of a bad idea.

## Debug overlay contract

Open/closed, `h` heatmap, path. Panel: expansions, path cost vs A* cost (suboptimality).

## Sources

- Standard AI textbook (Russell & Norvig): greedy best-first vs A*.
