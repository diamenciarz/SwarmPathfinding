# Field D*

## One-sentence idea

D* Lite plus **interpolation**: you may leave a cell at any point on an edge, so paths are any-angle *and* incremental.

## When to use / when not to

- Use when you want D* Lite’s repairs but hate stair-step paths (field robots, Ferguson & Stentz 2006).
- Do not use as the first incremental planner we code — get [D\* Lite](../d_star_lite/README.md) right first. Field D* is a later upgrade.
- Anya is the exact any-angle alternative but not incremental.

## World it assumes

Grid with a cost per cell. Path is a polyline through edge points, not just cell centers.

## How it works

D* Lite stores a `g` at cell corners (or centers). Field D* estimates `g` at a point on an edge by **linear interpolation** of the two corners. When expanding, the best step may aim at a point partway along a neighbor edge, not at a grid neighbor.

Repairs stay D* Lite-like: inconsistent corners, queue, `km`. Extracted paths are smoother.

## Invariants / guarantees

Incremental. Paths are shorter than 8-connected D* Lite, not always Euclidean-optimal (interpolation is an approximation).

## Complexity

A bit more work per expansion (interpolation, more successors). Ripple size similar to D* Lite.

## Relation to others

- Parent: D* Lite.
- Any-angle static cousin: Theta*, Anya.

## Swarm / cache note

Same as D* Lite: one field per goal.

## Fog-of-war note

Same as D* Lite on last-seen costs.

## Debug overlay contract

Corner `g` heatmap, interpolated path (not cell-center polyline), inconsistent corners. Panel: same as D* Lite plus path length vs 8-connected D* Lite.

## Sources

- Ferguson, D., & Stentz, A. (2006). The Field D* algorithm. *J. Field Robotics*.
