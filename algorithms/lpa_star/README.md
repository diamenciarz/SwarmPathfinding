# LPA* (Lifelong Planning A*)

## One-sentence idea

A* that **remembers** `g` values and, when some edge costs change, repairs only the vertices whose best path actually broke.

## When to use / when not to

- Use when the **start and goal stay put** and the graph changes repeatedly (doors, a tower-defense maze, a static army waiting while the map is edited).
- Do not use when the **agent is the thing that moves** — that is [D\* Lite](../d_star_lite/README.md), which is this algorithm with the search reversed.
- This lab **documents** LPA* so D* Lite is readable. The live pather we will implement is D* Lite (`km` version), not a separate LPA* loop. A “fixed start” toggle can call the same code with the robot pinned.

## World it assumes

A finite directed graph with non-negative edge costs. Heuristic `h(u)` estimates cost from `u` to the **goal**, admissible and consistent.

## How it works

A* stores one number, `g(u)` = cost from **start** to `u`. LPA* stores two:

- **g(u)** — current estimate of start-distance.
- **rhs(u)** — one-step lookahead from the start side:

  `rhs(start) = 0`, else `rhs(u) = min over predecessors p of (g(p) + cost(p→u))`.

Consistent means `g = rhs`. If the whole graph is consistent, `g` is the true start-distance and a shortest path is: from the goal, repeatedly step to a predecessor `p` minimizing `g(p) + cost(p→u)` until you hit the start.

The priority queue holds **exactly the inconsistent vertices**. Key of `u`:

`k(u) = [ min(g,rhs) + h(u) ,  min(g,rhs) ]`

`ComputeShortestPath` pops the smallest key until the goal is consistent and no remaining key is smaller than the goal’s key. Same over/under-consistent cases as D* Lite (set `g ← rhs`, or `g ← ∞` and try again).

When edges change, update `rhs` of the affected vertices, fix membership in the queue, run `ComputeShortestPath` again.

**First call, empty map knowledge, all `g = ∞`:** only the start is inconsistent with `rhs = 0`. The first search *is* A*.

## Invariants / guarantees

- Complete; optimal with a consistent heuristic.
- Incremental. A vertex is expanded at most twice per `ComputeShortestPath`.
- Start is **fixed**. If the start moves you must either restart or switch to D* Lite.

## Complexity

Same as D* Lite: work ~ size of the inconsistent set, focused by `h`.

## Relation to others

- Parent: A* (first iteration) and DynamicSWSF-FP (the `rhs` idea).
- Child: D* Lite = reverse the edges, swap start/goal, add `km`.

## Swarm / cache note

A shared LPA* tree is rooted at a **fixed start**, so it does *not* serve units with different starts. For a shared-goal army use D* Lite or a flow field.

## Fog-of-war note

Can run on last-seen costs, but as soon as the scout *moves*, you want D* Lite.

## Debug overlay contract

Heatmap of `g` (start-distance), inconsistent cells, queue, path reconstructed backward from the goal. Panel: expansions this repair, queue size.

## Sources

- Koenig, S., & Likhachev, M. (2002/2004). Incremental / Lifelong Planning A*.
- Koenig, S., & Likhachev, M. (2005). IEEE TRO 21(3) — §III is LPA*, §IV is D* Lite.
