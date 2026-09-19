# SIPP (Safe Interval Path Planning)

## One-sentence idea

If you **know** when a cell will be occupied, don’t search every timestep — search the few intervals when it is actually free.

## When to use / when not to

- Use when moving obstacles have **known trajectories** (patrols, trains, recorded ghosts).
- Do not use for fog-of-war surprises or unpredictable crowds — you do not have the intervals. Use D* Lite + ORCA.
- This is the low-level pather inside a lot of continuous-time MAPF.

## World it assumes

Grid (or a graph) plus, for each cell, a list of **safe intervals**: maximal time ranges with no collision. Obstacles are predicted.

## How it works

Naive space-time A* has a state `(cell, t)` for every tick. Most ticks in an empty cell look the same.

A **safe interval** is a maximal `[t_start, t_end)` the cell is empty. State becomes `(cell, interval_id)` — a handful per cell even if time is long.

A* on that graph: from `(c, I)`, you wait inside I and then step to a neighbor’s overlapping safe interval, paying time. Optimality matches space-time A* with a much smaller state space.

Phillips & Likhachev, ICRA 2011: hundreds of dynamic obstacles, still real-time on large grids.

## Invariants / guarantees

Complete and optimal w.r.t. the predicted occupancy (same as time-expanded A*). Garbage-in if the prediction is wrong.

## Complexity

States ~ cells × (number of gaps in occupancy). Many cells have *one* interval `[0, ∞)`.

## Relation to others

- Contrast: D* Lite has no time axis.
- With MAPF: SIPP as the single-agent subsolver for continuous-time CBS (not v1).

## Swarm / cache note

Intervals come from **other agents’ published paths** plus moving polygons. Shared: the interval map. Each unit still needs its own SIPP query unless they share a start.

## Fog-of-war note

You only have intervals where last-seen velocities let you **extrapolate**. That is a guess. Label it as such on the debug panel. Invisible movers have no interval (freespace) until seen.

## Debug overlay contract

Safe intervals as a Gantt on hover, space-time path, predicted obstacle footprints. Panel: states expanded vs naive space-time count, makespan.

## Sources

- Phillips, M., & Likhachev, M. (2011). SIPP. *ICRA*.
