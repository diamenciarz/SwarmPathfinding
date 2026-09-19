# Fog of war

Incomplete information is **not** the same as a dynamic world. The ground-truth map may be static. What the swarm *knows* is a stale copy.

## The model we will use

```
ground truth          (grid, or polygons + Pymunk)
        ↑
each agent’s VisionShape   (disk or cone; later clipped by LOS)
        ↓  only on discrepancy
shared last-seen map       (obstacle pose + velocity + timestamp)
        ↓
planners run here, never on ground truth
```

### Last-seen map

A copy of **obstacle positions** and, if they move, **velocities**. Static walls have velocity zero.

When any agent sees a discrepancy inside its VisionShape — an obstacle appeared, vanished, moved, or its velocity changed — it **writes the new record for the whole swarm**. There is no private map in the default mode. (A private-map toggle can exist later as an ablation: “what is sharing worth?”)

Inside fog, a moving obstacle is **not** updated. The last-seen pose and velocity stay. A planner *may* coast the ghost (`position + velocity * (now - last_seen_time)`) or freeze it. Default: **coast with velocity**, clamped to the map. That is why we store velocity at all. The ghost is drawn as a translucent body in Pygame.

### VisionShape

An interface, not a single shape:

- `CircleVision(radius)`
- `ConeVision(range, half_angle)`
- later: those shapes **clipped by line of sight** against obstacles (true RTS vision)

Each tick (or less often), every agent reports: “inside this shape, here is what I see.”

### Fog polygon

**Fog of war = map rectangle minus the union of all current VisionShapes.**

That union is a possibly ugly polygon (overlapping disks and cones, later with LOS notches). We do **not** recompute it every physics step:

- Rebuild when any agent has moved more than ε, or every N frames (for example 5–10), whichever comes first.
- For drawing, a coarse grid mask is enough (one bit per cell: visible / fog). For continuous worlds, `shapely.union_all` on the vision polygons, then `map_bounds.difference(union)`, cached.

Planners do not need the fog polygon. They need the **last-seen obstacle list**. Fog is for drawing and for “this cell has never been observed” (unknown vs last-seen-free).

### Unknown vs last-seen-free

A cell (or a region) can be:

| State | Meaning | Default planning cost |
|---|---|---|
| **Unknown** | No agent has ever seen it | **Freespace**: treat as walkable (D\* Lite’s original assumption) |
| **Last-seen free** | Someone saw empty space here | Walkable |
| **Last-seen occupied** | Someone saw an obstacle here (pose + velocity) | Blocked / moving blocker |
| **Currently visible** | Inside the vision union *this frame* | Ground truth, and last-seen is refreshed |

Freespace is optimistic: the army marches into the fog in a straight-ish line toward the goal. When a scout’s VisionShape hits a wall that was assumed empty, the shared last-seen map gains a blocker and D\* Lite repairs a ripple. That is the textbook unknown-terrain demo, now with a swarm.

Pessimistic unknown (treat unseen as blocked) is the other extreme: the army will not commit through fog. Useful as a toggle, not the default.

## What the literature does (and what we steal)

| Assumption | Family | Behaviour |
|---|---|---|
| Unknown = walkable | **D\* Lite**, original D\* | Head toward the goal; repair when a wall is seen. Koenig & Likhachev, IEEE TRO 2005, “planning with the freespace assumption.” |
| Unknown = blocked | frontier exploration first | Reveal a corridor before committing. |
| No map at all | **Bug1 / Bug2 / TangentBug** | Walk toward the goal; on contact, follow the boundary. Guaranteed to finish if a path exists; ugly. Excellent *baseline*. |
| One step of lookahead | **LRTA\*** | Move, bump `h`, repeat. When you cannot afford a full repair. |

## Pheromone: decision

**Not used as a planner.** Attractive trails duplicate the D\* Lite `g` field. Evaporation as “forget a stale obstacle” is already handled by last-seen pose+velocity plus vision. Classical ACO is an optimizer on a *known* graph and loses to A\*. Full reasoning: [../algorithms/pheromone_stigmergy/README.md](../algorithms/pheromone_stigmergy/README.md).

## Scenarios we will actually run

1. One agent, disk vision, D\* Lite, freespace — discover a wall, watch the ripple.
2. Swarm, **shared** last-seen: the first scout that sees a wall updates everyone. RTS “one observer updates the army.”
3. Same swarm, **private** last-seen (ablation).
4. TangentBug as a no-map baseline on the same map.
5. A moving obstacle that **leaves vision**: last-seen ghost coasts; collision vs *truth* is the metric.

## Debug overlay

- Ground truth faint, last-seen bold.
- Fog as a dark overlay (the complement of the vision union).
- Vision shapes outlined per agent.
- Ghosts of last-seen movers (translucent), with a velocity tick.
- Timestamp / age of each last-seen record on hover or in the panel.
- D\* Lite `g` / `rhs` on the *belief* grid, not on truth.
