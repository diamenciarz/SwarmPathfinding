# TangentBug (Bug family)

## One-sentence idea

Walk toward the goal; if you see an obstacle, slide along the **locally shortest tangent** until the goal is closer again. Almost no map.

## When to use / when not to

- Use as the **no-map baseline** for fog of war: how far do you get with a VisionShape and zero last-seen memory?
- Do not use as the RTS army pather. Paths hug walls and look silly; a shared last-seen + D* Lite is the real answer.
- Bug1/Bug2 (tactile only) are ancestors; we document TangentBug because we have range/vision.

## World it assumes

Continuous 2D, range sensor or VisionShape. Need own pose and goal pose. Obstacles arbitrary shape, finite.

## How it works

Lumelsky & Stepanov (Bug1/Bug2): two modes.

1. **Motion-to-goal:** go straight toward G until you hit (or see) an obstacle.
2. **Boundary-follow:** follow the obstacle until a *leave condition*.

Bug1: circumnavigate fully, remember the closest point to G, go there, then leave. Worst-case path length is about 1.5 × perimeters touched. Ugly but guaranteed.

Bug2: follow until you hit the start–goal line closer to G than the hit point. Usually shorter, can loop worse on weird mazes.

**TangentBug / DistBug:** with a range sensor, the “hit” is at a distance. At each step you look at the **discontinuities** of the range polar plot (the silhouettes) and aim at the tangent point that locally decreases distance to G. You leave the wall when a straight shot to G is clear or a heuristic says G is closer.

Completeness: if G is reachable, Bug-style algorithms reach it or correctly report trapped (for the tactile versions with the classic leave conditions).

## Invariants / guarantees

Complete (classic Bug, under their sensor model). Not optimal. Tiny memory (a few points: hit, leave, `d_min`).

## Complexity

O(1) extra memory. Per tick: vision / range query.

## Relation to others

- Opposite of D* Lite: no heatmap, no last-seen.
- Potential fields also local; Bug has a proof, fields do not.

## Swarm / cache note

Nothing shared unless you cheat and write walls into last-seen (then you have stopped being Bug). A swarm of Bugs will pile on the same wall.

## Fog-of-war note

This *is* a FOW algorithm with empty last-seen. VisionShape = the sensor. Compare time-to-goal vs D* Lite on a growing last-seen map on the same scenario.

## Debug overlay contract

Mode (to-goal vs follow), hit point, leave point, `d_min`, current tangent, range polar or vision wedge. Panel: mode switches, path length vs D* Lite.

## Sources

- Lumelsky & Stepanov (1987). *Algorithmica*. Bug1/Bug2.
- Kamon, Rimon, Rivlin. TangentBug / DistBug.
