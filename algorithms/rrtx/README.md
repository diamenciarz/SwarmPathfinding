# RRTX

## One-sentence idea

RRT* that **keeps** the graph when obstacles move or appear: rewire a cascade from the damaged nodes instead of growing a new tree.

## When to use / when not to

- Use as the **continuous analogue of D\* Lite**: single-query, asymptotically optimal, dynamic obstacles (Otte & Frazzoli, WAFR 2014 / IJRR 2016).
- Do not use on a grid — D* Lite is simpler and exact.
- Do not use if movers are chaotic and cheap local avoidance is enough: stale BIT*/field + ORCA is often the practical RTS choice.

## World it assumes

Continuous state space, kinodynamic optional (we start with 2D point/circle). Goal-rooted graph (so the robot can move without invalidating the root).

## How it works

RRT* plus:

- Each node stores a small neighbor set (so an iteration stays O(log n)).
- The shortest-path **tree is rooted at the goal**.
- When an obstacle appears, find nodes whose edges go through it (`V_obs`), then a **rewiring cascade** down the descendants: recompute parents, like D* Lite’s ripple in a sampled graph.
- When an obstacle disappears, new edges may appear; rewire to take them.
- `ε`-consistency: don’t chase microscopic improvements, so the cascade can stop.

The robot always follows the current tree; sampling continues, so the path still improves in the static stretches.

## Invariants / guarantees

Probabilistically complete. Asymptotically optimal in the static case. Reacts to obstacle changes without a full restart. Not a predictor: like D* Lite, it repairs *after* the geometry changes.

## Complexity

Amortized O(log n) per new sample (with neighbor pruning). Obstacle update O(|descendants of V_obs| log n) — local if the obstacle is small and far from the goal-rooted trunk.

## Relation to others

- Parent: RRT*.
- Grid cousin: D* Lite.
- First-path cousin: RRT-Connect (no repair, no optimality).

## Swarm / cache note

Goal-rooted: **many units with the same goal can follow the same tree.** Different starts are just different leaves. This is the sampled-space version of a shared D* Lite field. Cache key = goal + last-seen geometry generation.

## Fog-of-war note

Collision and “obstacle appeared” events come from last-seen updates, not from truth. A wall revealed in a VisionShape is an RRTX obstacle-add.

## Debug overlay contract

Graph + shortest-path subtree, `V_obs` flash, cascade nodes, current robot-to-goal path. Panel: n, rewires this update, cascade size, best cost.

## Sources

- Otte, M., & Frazzoli, E. (2014). RRTX. *WAFR*.
- Otte & Frazzoli (2016). *IJRR* 35(7).
