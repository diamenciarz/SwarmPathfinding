# SwarmPathfinding

A Python lab for **comparing pathfinding algorithms** on grids and continuous polygonal worlds, with swarms, physics, slow map edits, and fog of war.

Algorithm folders still hold student markdown only (no A\* / D\* Lite yet). Phase 2 is the **package skeleton + Pygame live lab**.

## What we are measuring

1. **Grid vs continuous.** Which algorithms belong on tiles vs n-gons with a 2D physics engine.
2. **Density.** Sparse obstacles (tiny fraction of the play area) vs dense clutter.
3. **RTS swarms + cache.** StarCraft-style: many units, one target, starts scattered. How much search can be shared, and what is rebuilt when a single wall disappears.
4. **Static vs dynamic, optimal vs good-enough.**
5. **Fog of war.** A shared last-seen copy of obstacle poses (and velocities). Each agent has a vision shape. Fog is the complement of the union of those shapes.

## Layout

```
docs/                 cross-cutting notes (start at glossary.md)
algorithms/<id>/      student README.md now; planner.py later
src/swarmpathfinding/ package (worlds, vision, belief, viz, stub planners)
tests/
```

## Setup

Python 3.10+ (this machine has 3.10). From the repo root:

```
py -3 -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -e ".[dev]"
python -m ipykernel install --user --name swarmpathfinding --display-name SwarmPathfinding
pytest
python -m swarmpathfinding
```

### How to test

| What | Command / notebook |
|---|---|
| Unit tests | `pytest` |
| Pygame sandbox | `python -m swarmpathfinding` |
| A named preset | `python -m swarmpathfinding --preset fog_scout` |
| A pack row | `python -m swarmpathfinding --pack fow_ghost --index 0` |
| CSV pack | `python -m swarmpathfinding.bench --pack slow_edit --algo straight_line` |
| Notebooks | `notebooks/00_how_to_test.ipynb`, `01_live_presets.ipynb`, `02_run_pack.ipynb` |

The live window: left/center is the world, right is the side panel. **LMB** adds a wall, **RMB** deletes one, **G+LMB** sets the goal, **E** applies a pending slow-edit, **Space** pauses, **.** steps, **R** regenerates, **Esc** quits.

Notebooks are **launchers**, not a second UI. `launch_lab(...)` blocks the kernel until you close Pygame. Use the **SwarmPathfinding** kernel (the project `.venv`).

Stub planners: `none`, `straight_line` (succeeds only if the last-seen map allows a straight shot). Real algorithms come in Phase 4.

Scenario packs (density, spawn layout, slow-edit, FOW, movers):

```
python -m swarmpathfinding.bench --list
python -m swarmpathfinding.bench --pack single_static --algo straight_line --seeds 0
python -m swarmpathfinding.generators --pack slow_edit --out maps/generated
```

Live lab maps now include `street` / `open` / `game` / `corridor` / `movers`. Starts: `clump`, `scattered`, `flanks`, `opposite`. Press **E** to apply a pending wall-delete (slow-edit).

Start here:

1. [docs/glossary.md](docs/glossary.md) — every short name and the words *admissible* / *bounded-suboptimal*
2. [docs/00-taxonomy.md](docs/00-taxonomy.md) — which algorithm for which question
3. [algorithms/d_star_lite/README.md](algorithms/d_star_lite/README.md) — the incremental replanner, explained on a small grid
4. [docs/03-rts-cache.md](docs/03-rts-cache.md) — shared fields, dirty sectors
5. [docs/04-fog-of-war.md](docs/04-fog-of-war.md) — last-seen map + vision union

## Not in v1

- Berliner B\* (game-tree search, not a map pather)
- Original D\* / Focussed D\* / the first D\* Lite that rebuilds the whole heap — we document why D\* Lite (the `km` version) replaced them, and implement only that
- Classical ant-colony optimization as a planner (see [algorithms/pheromone_stigmergy/README.md](algorithms/pheromone_stigmergy/README.md))
- Jupyter as the main UI (Pygame is the lab)

## Next

Implement real planners (A\* first, then D\* Lite), flow-field cache overlays, ORCA on Pymunk. Each algorithm folder grows `planner.py` + `debug.py` implementing the UI contract in `docs/06-debug-ui-contract.md`.
