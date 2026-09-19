# RRT*

## One-sentence idea

RRT that, after adding a node, **rewires** nearby nodes if the new node offers a cheaper parent — so the tree drifts toward the shortest path as you keep sampling.

## When to use / when not to

- Use as the classic **asymptotically optimal** sampler. Good teaching demo: watch path length fall over time.
- Prefer [BIT*](../bit_star/README.md) in the live lab if you want the current anytime default; keep RRT* as the baseline it beat.
- **Informed RRT\***: once you have a path of cost `c`, sample only inside the ellipse of foci start/goal and major axis `c`. Same optimality, faster convergence.

## World it assumes

Same as RRT. Connection radius `r(n)` shrinks like `(log n / n)^{1/d}` (Karaman & Frazzoli).

## How it works

When `q_new` is added:

1. Consider neighbors within `r(n)`.
2. Choose as parent the neighbor that minimizes `g(neighbor) + dist(neighbor, q_new)` among collision-free links.
3. For each neighbor, if `g(q_new) + dist` is better than `g(neighbor)`, **rewire** that neighbor to `q_new`.

Keep iterating after the first path; the path cost is nonincreasing.

## Invariants / guarantees

Probabilistically complete. **Asymptotically optimal** (path cost → optimum as samples → ∞, almost surely). Anytime in the weak sense: you have a path and it improves.

## Complexity

Heavier than RRT: neighbor queries + extra collision checks per rewire. Informed sampling cuts wasted samples after the first path.

## Relation to others

- Parent: RRT.
- Informed RRT* ⊂ this with elliptical sampling.
- BIT* = batch samples + A*-like search on an implicit random geometric graph (often faster).
- RRTX = this + repair when obstacles move.

## Swarm / cache note

Tree is start-rooted. Weak sharing. A PRM* roadmap shares better.

## Fog-of-war note

Same as RRT; rewires do not help if a new wall slices the tree — use RRTX or restart.

## Debug overlay contract

Tree with `g` as color, current best path bold, informed ellipse once a path exists, rewire flashes. Panel: nodes, best cost vs time, collision checks.

## Sources

- Karaman & Frazzoli (2011). *IJRR*.
- Gammell, Srinivasa, Barfoot. Informed RRT*.
