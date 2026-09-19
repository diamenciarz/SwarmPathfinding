# ORCA (Optimal Reciprocal Collision Avoidance)

## One-sentence idea

Each agent, locally, picks a velocity close to what it wanted that is guaranteed not to hit neighbors **if they each take half the avoidance** — a tiny linear program in velocity space, not a new global path.

## When to use / when not to

- Use as the **local layer** on Pymunk: units following a flow field / path still push and dodge.
- Do not use as a global planner. It will not go around a building. It will jitter in a doorway if the global path threads a pin-hole.

## World it assumes

Continuous 2D, circular agents (radius `r`), current positions and velocities. Static polygons become velocity obstacles too (the agent takes *all* of that avoidance). Physics engine still resolves residual overlap.

## How it works

Velocity obstacle of B as seen by A: the set of velocities that would hit B within time `τ` if B kept its velocity. Naive VO causes **oscillation** (A dodges, B dodges the other way, they swap).

**RVO:** each takes half. **ORCA:** that half is a **half-plane** in velocity space (linear constraint). Intersection of half-planes = allowed velocities. Pick the point in that polygon closest to the *preferred* velocity (from the global planner). That closest-point query is a 2D linear program, O(neighbors).

If the LP is infeasible (too packed), fall back to a slower speed / stopping. Pymunk then soaks true contacts.

## Invariants / guarantees

If every agent uses ORCA and the LP is feasible, no collision within `τ` under the circular, constant-velocity model. No global optimality. Can deadlock in a narrow corridor (need the global planner to pick another door).

## Complexity

O(N × k) per tick if each agent looks at k neighbors (spatial hash). Thousands of agents in milliseconds in the original demos.

## Relation to others

- Parent: VO (Fiorini), RVO (van den Berg 2008).
- Global partners: flow field, D* Lite, group path.
- Contrast: potential fields (more local minima, more oscillation).

## Swarm / cache note

Not cached, not shared, and that is fine: it is cheap. **Do not** replan A* because two units bumped; let ORCA handle the bump.

## Fog-of-war note

ORCA neighbors = currently **visible** agents + last-seen ghosts you choose to dodge. A ghost with a stale velocity can make you sidestep empty air. That is a feature to show.

## Debug overlay contract

Per selected agent: preferred velocity, ORCA velocity, neighbor half-planes in a small velocity-space inset, collision cones. Panel: LP fails, max |v_pref − v_orca|, contacts Pymunk still reported.

## Sources

- van den Berg, Lin, Manocha (2008). RVO. *ICRA*.
- van den Berg, Guy, Lin, Manocha (2011). Reciprocal n-body collision avoidance. *ISRR*. (ORCA / RVO2)
- Sunshine-Hill, *Game AI Pro 3* ch. 19 — how they really work.
