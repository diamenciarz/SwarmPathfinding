# Pheromone / stigmergy (and why it is not a planner here)

## One-sentence idea

Ants coordinate by writing into the world. Two different computer ideas stole that picture; **only one was ever a path planner, and it is the wrong tool for this lab.**

## Decision

**We do not implement a pheromone path planner.**  
Known-goal RTS fog of war is already [D\* Lite on a shared last-seen map](../d_star_lite/README.md). Attractive trails duplicate `g`. Evaporation as “forget a mover” is already last-seen pose + velocity. Classical ACO is an optimizer on a *known* graph and loses to A*.

This note exists so a student can see the fork and why we took it.

## Two ideas people mix up

### 1. ACO (Dorigo) — not FOW, not RTS

Many fake ants iterate on a **known** graph. They deposit pheromone on edges of short tours; pheromone evaporates; the colony converges to a short path.

That is a **centralized metaheuristic**. For a 2D map it is a slow, stochastic A*. Fine for TSP-like combinatorics. Not fine for “go there, now, under fog.”

### 2. Stigmergy / virtual pheromones — useful for *search*, not for a known goal

Real robots (Kilobots, drone coverage papers, SPACE 2026) write a **field**:

- **Attractive:** “this way reached the goal / the victim.”
- **Repulsive:** “I already looked here” (spreads explorers).
- **Evaporation:** stale marks fade.

That helps when the **goal location is unknown** (coverage, foraging, search-and-rescue). It does **not** beat a goal-rooted `g` map when the click-to-move target is known.

If we ever add a “find the hidden object” scenario, a repulsive exploration field on top of last-seen would earn a prototype. It is not v1.

### 3. The one share that *does* help, already in the lab

A shared [LRTA*](../lrta_star/README.md) `h` table, a shared D* Lite `g` field, and a shared last-seen obstacle list **are** stigmergy: the first scout to see a wall writes for the army. We just don’t call it pheromone.

## When to use / when not to

- Use this README as a warning label.
- Do not put ACO in the algorithm picker.
- Do not draw a pheromone heatmap in v1 (it would imply we plan on it).

## World it assumes

None in v1.

## How it works

Skipped as a planner. Sketch only: discrete field `τ[cell]`, update `τ ← (1-ρ)τ + deposit`, follow `argmax τ` or `argmax τ^α · heuristic^β`. That is ACO’s state transition. Replace `heuristic` with D* Lite’s `g` and you have reinvented “follow the field.”

## Invariants / guarantees

ACO: none that A* doesn’t already have better. Stigmergic coverage: empirical, density-dependent (at high robot density, pheromone can *hurt* coverage vs random walks — Royal Society Open Science 2019, N>300 Kilobots).

## Swarm / cache note

A pheromone field would be yet another shared grid. We already have integration / `g` / last-seen.

## Fog-of-war note

Evaporation is a blunt way to forget fog-stale obstacles. Last-seen **velocity** is a sharper way: the ghost coasts until someone looks.

## Debug overlay contract

None in v1.

## Sources

- Dorigo, M., et al. Ant Colony Optimization.
- Kilobot stigmergy limits: *Royal Society Open Science* (2019).
- Dual-pheromone exploration (SPACE, 2026) — coverage, not known-goal pathing.
- Koenig & Likhachev (2005) — freespace D* Lite is the FOW planner we actually want.
