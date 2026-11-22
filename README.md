# SwarmPathfinding

A Python lab for **comparing pathfinding algorithms** on grids and continuous polygonal worlds, with swarms, physics, slow map edits, and fog of war.

Algorithm folders still hold student markdown only. The one real planner so far is **weighted A\*** (`weighted_a_star`); D\* Lite and the rest are still to come. Phase 2 is the **package skeleton + Pygame live lab**.

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

The live window: the world is on the left, the side panel on the right. Changing any panel control applies at once (sliders apply on release). Planner and `w` changes replan on the same map; everything else rebuilds the scenario with the same seed.

| Input | Action |
|---|---|
| **LMB** / **RMB** | add / delete a wall (grid cell, or a box polygon in continuous worlds) |
| **G** + **LMB** | set the goal for every agent |
| **E** | apply the pending slow-edit (the wall outlined in gold) |
| **Wheel** / **MMB** drag / **F** | zoom / pan / fit the view |
| **Space** / **.** | pause / single step |
| **R** | restart the scenario |
| **M** | open the lab menu (also the button at the top of the panel) |
| **Esc** | close the menu, or quit when it is closed |

**Lab menu.** A popup over the paused view (the simulation resumes when it closes). Tab **1 Scenario**: set the scenario on the left; the right side lists every planner with a badge for how it fits this scenario (star = designed for it, check = works as is, ~ = adapted, cross = cannot) and the notes on each adaptation. Click a planner, then **Apply** (Enter) to load it all into the live view, or **Randomise** for a random scenario. Tab **2 Planners**: every planner against every scenario trait; hover a cell for the note, the highlighted column is the current scenario, and **Use in current scenario** switches planner on the same map. Planners declare these ratings in their `capabilities` dict (see `core/capabilities.py`).

With fog of war on, the view shows what the swarm remembers, not the truth: unknown is dark, remembered-but-not-visible is dimmed, and memories the world has since contradicted are tinted red ("stale"). In continuous worlds moving obstacles that are out of sight are drawn as yellow ghosts extrapolated along their last-seen velocity. **Show search** paints the cells the focus agent's planner expanded (agent 0, ringed in orange).

Notebooks are **launchers**, not a second UI. `launch_lab(...)` blocks the kernel until you close Pygame. Use the **SwarmPathfinding** kernel (the project `.venv`).

Planners: `weighted_a_star` (8-connected A\* on the last-seen map, f = g + w·h; `w` = 1 is optimal, larger `w` expands fewer cells for a path at most `w` times longer; continuous worlds are searched on a rasterised map with obstacles inflated by the agent radius), plus the stubs `none` and `straight_line` (succeeds only if the last-seen map allows a straight shot). Each agent plans its own route; unknown cells count as free.

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

## Headless simulation

`swarmpathfinding.sim.Simulation` is the stepping the live lab draws, with no pygame. Same `SimConfig` + seed = same run, so benchmarks match what you watch:

```python
from swarmpathfinding.sim import SimConfig, Simulation
sim = Simulation(SimConfig(map_kind="rooms", n_agents=10, fow=True, changes="slow_edit", edit_at=3.0))
sim.run(max_time=60)      # True when every agent finished its route
sim.stats                 # plans, plan_ms, expansions, plans/ms after the edit, travelled, arrival times
```

`changes="slow_edit"` deletes the pre-picked wall automatically at `edit_at` seconds (Changes: *auto edit* in the panel); with `"none"` it waits for **E**.

Planners rate themselves per scenario trait in `capabilities` (`core/capabilities.py`): designed for, works as is, works through a stated adaptation, or cannot. `compatibility(planner, scenario_traits(config))` says whether a planner may run a scenario and how it copes.

## Next

Implement more planners (D\* Lite next), flow-field cache overlays, ORCA on Pymunk. Each algorithm folder grows `planner.py` + `debug.py` implementing the UI contract in `docs/06-debug-ui-contract.md`.
