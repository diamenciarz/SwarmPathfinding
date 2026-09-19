# Group / centroid pathfinding

## One-sentence idea

If tens of units are **close together**, plan **once** for a virtual body the size of the clump (or for a leader), and let ORCA keep members from stacking.

## When to use / when not to

- Use when a selection is a compact blob (same start region, same goal) — the “no need to recalculate each agent” case you described.
- Do not use when the group is split across the map; then a [flow field](../flow_field/README.md) is the right share.
- If the clump **splits** (a wall bisects it), break into two groups and plan twice.

## World it assumes

Physical agents with positions. A clustering radius R. Global planner underneath (A*, D* Lite, visibility graph, …).

## How it works

Two textbook options (*Game AI Pro 2*, hierarchical group navigation):

1. **Leader:** pick one unit (or the closest to the centroid). Plan for it. Followers seek a formation offset from the leader (slot in a rectangle / disk).
2. **Virtual body:** the group *is* a fat agent at the centroid with radius ≈ bounding disk (or a capsule along the motion). Plan for that fat agent so the path’s clearance fits the blob. Members track slots relative to the centroid.

We will implement (2) as default: clearance is honest. Leader-follower as a toggle.

**When to replan the group:** goal change, last-seen corridor dirtied, centroid drifted farther than R from the path, or cluster detection reports a split.

Members never run their own global search while clustered.

## Invariants / guarantees

No optimality for members. Completeness follows the underlying planner for the fat body; a fat body may not fit a hole a single unit would. That is correct (the blob really doesn’t fit).

## Complexity

One global query per cluster per replan, plus cheap slot tracking. Independent of cluster size for the expensive part.

## Relation to others

- Complements flow fields (fields win when starts are scattered; centroid wins when they are packed).
- Local: ORCA among members.

## Swarm / cache note

Cache the group path until a dirty event. Cluster IDs on the overlay.

## Fog-of-war note

The fat body plans on last-seen geometry. A scout on the rim of the blob still updates last-seen for everyone.

## Debug overlay contract

Cluster hulls, centroid, fat-body radius, the single path, formation slots, members color-coded by cluster. Panel: clusters, queries this second, splits/merges.

## Sources

- *Game AI Pro 2*, ch. 20 — hierarchical architecture for group navigation (leader vs virtual entity).
