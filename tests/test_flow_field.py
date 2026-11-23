"""Flow fields: one field per goal, optimal descent; tiles stay local."""

import numpy as np
import pytest

from swarmpathfinding.core.geom import Cell
from swarmpathfinding.core.registry import create_planner
from swarmpathfinding.core.results import Budget
from swarmpathfinding.generators.scenarios import make_scenario
from swarmpathfinding.planners.flow_field import SectorGraph, descend, flow_directions, integration_field
from swarmpathfinding.sim.simulation import SimConfig, Simulation

CASES = [(mk, seed) for mk in ("bernoulli", "clustered", "rooms", "maze") for seed in range(6)]


@pytest.mark.parametrize("mk,seed", CASES)
def test_flow_field_is_optimal_tiles_close(mk, seed):
    sc = make_scenario(world_kind="grid", map_kind=mk, density=0.25, n_agents=1, seed=seed, fow=False)
    st = [sc.agents[0].pos]
    a = create_planner("a_star").plan(sc.belief, st, sc.goal)
    f = create_planner("flow_field").plan(sc.belief, st, sc.goal)
    t = create_planner("flow_field_tiles").plan(sc.belief, st, sc.goal)
    assert a.success == f.success == t.success
    if a.success:
        assert abs(a.cost - f.cost) < 1e-6
        assert t.cost <= a.cost * 1.1 + 1e-6


def test_field_is_shared_between_agents():
    sc = make_scenario(world_kind="grid", map_kind="bernoulli", density=0.2, n_agents=10, seed=2, fow=False, start_layout="scattered")
    p = create_planner("flow_field")
    results = [p.plan(sc.belief, [a.pos], sc.goal) for a in sc.agents]
    assert results[0].expansions > 0
    assert all(r.expansions == 0 for r in results[1:])
    assert all(r.success for r in results)


def test_descent_and_directions_agree():
    blocked = np.zeros((6, 6), dtype=bool)
    blocked[2, 1:5] = True
    field, n = integration_field(blocked, Cell(0, 5))
    path = descend(field, blocked, Cell(5, 0), Cell(0, 5))
    assert path[0] == Cell(5, 0) and path[-1] == Cell(0, 5)
    dirs = flow_directions(field, blocked)
    for a, b in zip(path, path[1:]):
        assert field[b.y, b.x] < field[a.y, a.x]
    assert tuple(dirs[0, 5]) != (0, 0)
    assert tuple(dirs[5, 0]) == (0, 0)  # the goal has nowhere to go


def test_sector_graph_dirty_sectors_and_cache():
    blocked = np.zeros((24, 24), dtype=bool)
    sg = SectorGraph(8)
    sg.build(blocked)
    assert sg.relabelled == 1  # nine identical empty sectors share one labelling
    assert len(sg.region_sector) == 9
    blocked2 = blocked.copy()
    blocked2[3, 3] = True
    sg.build(blocked2)
    assert sg.dirty == [(0, 0)]
    # a wall splitting a sector makes two regions there
    blocked3 = blocked.copy()
    blocked3[0:8, 4] = True
    sg.build(blocked3)
    assert sum(1 for s in sg.region_sector if s == (0, 0)) == 2


def test_tiles_unreachable_goal():
    sc = make_scenario(world_kind="grid", map_kind="empty", n_agents=1, seed=1, fow=False)
    world = sc.world
    g = world.world_to_cell(sc.goal)
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            if dx or dy:
                world.punch_wall(Cell(g.x + dx, g.y + dy))
    from swarmpathfinding.core.belief import BeliefMap

    r = create_planner("flow_field_tiles").plan(BeliefMap.from_truth_grid(world), [sc.agents[0].pos], sc.goal)
    assert not r.success


@pytest.mark.parametrize("planner", ["flow_field", "flow_field_tiles"])
@pytest.mark.parametrize("world,fow", [("grid", True), ("continuous", False)])
def test_crowds_arrive(planner, world, fow):
    cfg = SimConfig(world_kind=world, map_kind="bernoulli" if world == "grid" else "sparse", n_agents=25, fow=fow, planner=planner, seed=3, starts="scattered")
    assert Simulation(cfg).run(max_time=60)


def test_d_star_lite_crowd_in_fog_arrives():
    """Regression: greedy path extraction could strand an agent on a repaired tree."""
    cfg = SimConfig(map_kind="bernoulli", n_agents=60, fow=True, planner="d_star_lite", seed=3, starts="scattered")
    assert Simulation(cfg).run(max_time=60)


def test_overlays_draw():
    import os

    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    import pygame

    from swarmpathfinding.viz.live import LiveApp
    from swarmpathfinding.viz.settings import LabSettings

    app = LiveApp(LabSettings(n_agents=5, map_kind="rooms", fow=False, planner="flow_field_tiles"))
    app.panel.get("show_search").value = True
    app._on_panel_change("show_search", True)
    app.camera.zoom = 30
    app.draw()
    res = app.sim.results[app.sim.focus_id()]
    assert "sectors" in res.extra and "field" in res.extra
    app.panel.get("planner").index = app.panel.get("planner").options.index("flow_field")
    app._on_panel_change("planner", True)
    app.draw()
    assert isinstance(app.screen, pygame.Surface)
