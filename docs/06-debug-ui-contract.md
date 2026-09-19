# Debug UI contract

The live lab is **Pygame**. Each algorithm folder will later grow a `debug.py` (or methods on the planner) that implements this contract. Jupyter is not the UI; a notebook may only *launch* this window.

## Window layout

```
┌─────────────────────────────────┬──────────────────┐
│  world (camera)                 │  side panel      │
│  grid or polygons + agents      │  pickers, knobs  │
│  algorithm overlay              │  live metrics    │
│  fog / vision / ghosts          │  overlay toggles │
└─────────────────────────────────┴──────────────────┘
```

Global controls (not owned by one algorithm): algorithm picker, map picker, density, N agents, vision shape, FOW on/off, `w` slider, pause / step, punch-wall, delete-wall, drag obstacle.

## Protocol (to implement later)

```python
class Toggle:
    id: str
    label: str
    default: bool

class DebugLayer(Protocol):
    def overlays(self) -> list[Toggle]: ...
    def draw_world(self, surface, camera, state) -> None: ...
    def draw_panel(self, surface, rect, metrics) -> None: ...
```

`draw_world` uses the camera (world-meters → pixels). It must tolerate being called when the planner has not finished: draw whatever exists (partial tree, partial field).

`draw_panel` is a *column of numbers and sparkline-sized bars*, not a matplotlib window. If an algorithm wants a tiny plot (expansions over time), it blits a pygame surface it owns.

## What every planner should be able to show

Minimum, even for Dijkstra:

- Start(s) and goal
- Result path
- Open / closed (or “expanded this repair”)
- One cost heatmap (`g`, integration field, or sampling density)

Then extras listed in each algorithm README’s **Debug overlay contract** section.

Fog, vision unions, last-seen ghosts, and dirty sectors are **world/swarm overlays**, always available, not per-algorithm.

## Frame budget

Vision-union / fog polygon may skip frames ([04-fog-of-war.md](04-fog-of-war.md)). Debug heatmaps may update every k planner expansions, not every draw. Never block the physics step on a full-map blit: cache the overlay surface until the planner dirties it.
