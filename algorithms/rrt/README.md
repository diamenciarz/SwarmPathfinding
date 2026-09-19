# RRT and RRT-Connect

## One-sentence idea

Grow a tree of random samples toward unexplored space until you bump into the goal (RRT-Connect grows a second tree from the goal and tries to shake hands).

## When to use / when not to

- Use to get **any** feasible path quickly in continuous space, especially if the first path may be ugly.
- Do not use when you need a short path — that is [RRT*](../rrt_star/README.md) or [BIT*](../bit_star/README.md).
- Do not use on a clean grid; A*/JPS is deterministic and faster there.

## World it assumes

Continuous 2D (polygons, inflated radius). Step size `ε`. Collision along the steered segment.

## How it works

**RRT**

1. Sample a random point `q_rand` (with a small bias toward the goal).
2. Find the nearest tree node `q_near`.
3. Steer from `q_near` toward `q_rand` by at most `ε` → `q_new`.
4. If the segment is free, add `q_new`.
5. Repeat until `q_new` is close to the goal.

Trees explore Voronoi-like: they grow into large empty regions first. That is why they find *a* path through open space fast and struggle in narrow passages.

**RRT-Connect:** one tree from start, one from goal. Each iteration, grow one tree, then try to connect the other toward the new node. First feasible path, still not optimal.

## Invariants / guarantees

Probabilistically complete. **Not** optimal. Paths are jagged (post-smooth with shortcuts).

## Complexity

Per iteration: nearest-neighbor (kd-tree) + one collision check. First-path time is the metric that matters.

## Relation to others

- Child: RRT*, Informed RRT*, RRTX.
- Competitor for first path: BIT* often wins on quality *and* time in recent comparisons, but RRT-Connect is simpler.

## Swarm / cache note

A tree is start-specific. Poorly shared. Build one path for a cluster centroid, or switch to a roadmap / field.

## Fog-of-war note

Collision checks use last-seen obstacles. Unknown = free (freespace). The tree can go into fog; a new last-seen wall invalidates colliding edges — then you want [RRTX](../rrtx/README.md) or a restart.

## Debug overlay contract

The tree (every edge), latest steer, goal bias samples in a different color, the extracted path. Connect: two trees, handshake segment. Panel: nodes, collision checks, first-path time, path length.

## Sources

- LaValle (1998). RRT.
- Kuffner & LaValle (2000). RRT-Connect.
