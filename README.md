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

**Benchmarks** (menu tab **3 Benchmark**). Choose how many scenarios, the base seed, the time limit per trial (simulated seconds, default 60) and a mode, then which values each setting may take: chips for worlds, maps, starts, fog, changes, vision and planners, and two-knob ranges for density, agent count, vision radius and edit time. Scenario *i* always gets seed `base + i` and the same random settings, so every planner faces exactly the same scenarios. **Headless** runs the trials in worker processes (all CPU cores but one) while you keep using the lab; **visual x4 / x16 / max** plays each trial in the live view (Space pauses, Esc cancels). All modes give identical numbers. Progress shows as a gradient bar (`29/150`) with one bar per planner, an ETA, Pause and Cancel, and a strip along the bottom of the other tabs. Each finished trial is appended to `benches/out/lab/<time>_<name>.csv`. **Load a random scenario of it** puts a random scenario from the current settings into the live view.

**Results** (tab **4**): a summary table (best value per column highlighted), a "where it wins" heatmap (for scenarios with each trait value, the share in which each planner had the best makespan; failures count as the time limit), a box plot of any metric, and the scenario list (click one to load it with its winner). **Load random trial**, **Export summary** (a `_summary.csv` next to the trial CSV) and **Open last CSV** (results from an earlier session).

From Python:

```python
from swarmpathfinding.bench.experiment import ExperimentSpec
from swarmpathfinding.bench.runner import BenchmarkRunner

if __name__ == "__main__":  # needed for worker processes on Windows
    spec = ExperimentSpec(n_scenarios=100, worlds=["grid", "continuous"], agents=(1, 30))
    rows = BenchmarkRunner(spec, mode="headless").run_all()
```

With fog of war on, the view shows what the swarm remembers, not the truth: unknown is dark, remembered-but-not-visible is dimmed, and memories the world has since contradicted are tinted red ("stale"). In continuous worlds moving obstacles that are out of sight are drawn as yellow ghosts extrapolated along their last-seen velocity. **Show search** paints the cells the focus agent's planner expanded (agent 0, ringed in orange).

Notebooks are **launchers**, not a second UI. `launch_lab(...)` blocks the kernel until you close Pygame. Use the **SwarmPathfinding** kernel (the project `.venv`).

Planners: the **A\* family** on an 8-connected grid (one search, different priorities): `dijkstra` (f = g), `a_star` (f = g + h, octile h), `greedy_best_first` (f = h) and `weighted_a_star` (f = g + w·h; larger `w` expands fewer cells for a path at most `w` times longer). Continuous worlds are searched on a rasterised map with obstacles inflated by the agent radius. **JPS** (`jps`: A\* that only queues jump points; same paths, far fewer queue operations; its scans show as faint blue cells), **Theta\*** (`theta_star`: any-angle, a cell may take its grandparent as parent when it can see it; line-of-sight tests show as green / red lines during replay) and **Lazy Theta\*** (`lazy_theta_star`: checks sight only on expansion). Plus the stubs `none` and `straight_line`. Each agent plans its own route; unknown cells count as free.

**Watching a search.** Tick **Show search** to see agent 0's search. The **Search** section adds a g / h / f heatmap, the open set (frontier, green), the closed set (expanded, purple), parent arrows (zoom in) and a cell inspector (hover a cell for g, h, f, its parent and when it was expanded). The bar along the bottom replays the search: start, step back, play / pause, step, end, speed and a scrubber. Keys: **P** play / pause, **Left / Right** one expansion (Shift: 10), **Home / End**. Replaying pauses the simulation.

**Compare panes.** In the menu's Scenario tab tick **compare** on 2 to 4 planners and Apply: each gets its own pane on the same scenario and seed, with one camera, edits and goal clicks applied to every pane, and a metrics strip underneath. The replay bar steps every pane in lock-step by expansion count, so you can watch Dijkstra flood while A\* aims. **C** goes back to one pane.

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
