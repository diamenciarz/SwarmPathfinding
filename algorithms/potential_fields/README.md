# Potential fields

## One-sentence idea

The goal pulls, obstacles push; you follow the sum of those forces. Cheap, local, and it **gets stuck in U-shaped pockets**.

## When to use / when not to

- Use as a **steering layer** (nudge off a wall, separate from a friend) *under* a global path or flow field.
- Do not use as a global planner. Concave obstacles create **local minima** that are not the goal.

## World it assumes

Continuous 2D. Agents are points or small circles. Obstacles are polygons or circles.

## How it works

Attractive potential toward the goal (often quadratic near, linear far). Repulsive potential from each obstacle surface (strong when close, zero beyond a distance `d0`). Velocity (or acceleration) follows `-∇U`.

**U-trap:**

```
S ---->  ( wall )
         (  G behind the C-shape )
```

The repulsion from the two arms and the attraction to G cancel in the pocket. The agent stops. This is the demo: same map, A* / visibility graph succeeds, the field hangs.

Fixes people try (and we can show as failed extras): random walk, wall following, harmonic potentials. They are Bug-like patches, not a reason to drop a global planner.

## Invariants / guarantees

None that matter: not complete, not optimal. Can oscillate between two obstacles.

## Complexity

O(obstacles nearby) per agent per tick. Spatial hash recommended.

## Relation to others

ORCA is the modern local layer (velocity-space, reciprocal, fewer oscillations). Potential fields are the teaching ancestor.

## Swarm / cache note

Fully local; nothing to cache. N agents scale linearly. Combine with a shared flow field: field gives the desired velocity, this (or ORCA) modifies it.

## Fog-of-war note

Forces from **last-seen** obstacles only. Unknown space exerts no repulsion (freespace). A wall in fog does not push you until you see it — you may ram it.

## Debug overlay contract

Force arrows per agent (attract vs repulse vs sum), potential heatmap if we rasterize it, mark local-minima stalls. Panel: |force|, distance to goal, stall flag.

## Sources

- Khatib, O. (1986). Real-time obstacle avoidance for manipulators and mobile robots. (classic potential field.)
