"""Visibility graph, RRT, RRT* and BIT*: valid, reproducible, sensible lengths."""

import math
import os

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

import numpy as np  # noqa: E402
import pytest  # noqa: E402
from shapely.geometry import LineString, Polygon  # noqa: E402

from swarmpathfinding.core.belief import BeliefMap, ObstacleRecord  # noqa: E402
from swarmpathfinding.core.capabilities import Fit, compatibility, scenario_traits  # noqa: E402
from swarmpathfinding.core.geom import Vec2  # noqa: E402
from swarmpathfinding.core.plan_grid import plan_grid  # noqa: E402
from swarmpathfinding.core.registry import create_planner  # noqa: E402
from swarmpathfinding.core.results import Budget  # noqa: E402
from swarmpathfinding.sim.simulation import SimConfig, Simulation  # noqa: E402

SAMPLERS = ("rrt", "rrt_star", "bit_star")
ALL = ("visibility_graph", *SAMPLERS)


def _wall_belief():
    """A 30x20 room with one wall in the middle (gap at the top)."""
    b = BeliefMap.unknown_grid(30, 20)
    b.kind = "continuous"
    b.occupancy[:, :] = 0
    verts = (Vec2(14, 0), Vec2(16, 0), Vec2(16, 15), Vec2(14, 15))
    b.obstacles["wall"] = ObstacleRecord("wall", verts, Vec2(15, 7.5), Vec2(0, 0), 0.0)
    return b


S, G = Vec2(5, 5), Vec2(25, 5)
# shortest route: up to the gap at y = 15 + r, over the wall, back down
OPT = math.hypot(9, 10.35) * 2 + 2


@pytest.mark.parametrize("name", ALL)
def test_paths_clear_the_wall(name):
    r = create_planner(name).plan(_wall_belief(), [S], G)
    assert r.success
    # the raster is coarse (0.5 cells), so only the wall itself is a hard limit
    wall = Polygon([(14, 0), (16, 0), (16, 15), (14, 15)])
    assert not LineString([(p.x, p.y) for p in r.path]).intersects(wall)
    assert r.path[0] == S and r.path[-1] == G


def test_visibility_graph_is_shortest():
    r = create_planner("visibility_graph").plan(_wall_belief(), [S], G)
    assert r.cost == pytest.approx(OPT, rel=0.03)
    assert len(r.path) <= 5  # start, two corners, goal


def test_rrt_star_and_bit_star_beat_rrt():
    b = _wall_belief()
    rrt = create_planner("rrt").plan(b, [S], G).cost
    assert create_planner("rrt_star").plan(b, [S], G).cost <= rrt + 1e-9
    assert create_planner("bit_star").plan(b, [S], G).cost < OPT * 1.1


@pytest.mark.parametrize("name", SAMPLERS)
def test_samplers_are_reproducible(name):
    a = create_planner(name).plan(_wall_belief(), [S], G)
    b = create_planner(name).plan(_wall_belief(), [S], G)
    assert a.path == b.path


@pytest.mark.parametrize("name", ALL)
def test_debug_data_only_when_recording(name):
    p = create_planner(name)
    quiet = p.plan(_wall_belief(), [S], G)
    loud = p.plan(_wall_belief(), [S], G, Budget(record_trace=True))
    keys = {"tree", "vg_edges"}
    assert not keys & set(quiet.extra)
    assert keys & set(loud.extra)


def test_bit_star_reports_informed_ellipse():
    r = create_planner("bit_star").plan(_wall_belief(), [S], G, Budget(record_trace=True))
    s, g, c_best, c_min = r.extra["ellipse"]
    assert c_best == pytest.approx(r.cost, rel=1e-6) and c_min == pytest.approx(20.0)


@pytest.mark.parametrize("name", ALL)
def test_blocked_goal_fails_cleanly(name):
    b = _wall_belief()
    box = (Vec2(22, 2), Vec2(28, 2), Vec2(28, 8), Vec2(22, 8))
    ring = [ObstacleRecord(f"b{i}", v, Vec2(25, 5), Vec2(0, 0), 0.0) for i, v in enumerate([
        (Vec2(21, 1), Vec2(29, 1), Vec2(29, 2), Vec2(21, 2)),
        (Vec2(21, 8), Vec2(29, 8), Vec2(29, 9), Vec2(21, 9)),
        (Vec2(21, 1), Vec2(22, 1), Vec2(22, 9), Vec2(21, 9)),
        (Vec2(28, 1), Vec2(29, 1), Vec2(29, 9), Vec2(28, 9)),
    ])]
    del box
    for rec in ring:
        b.obstacles[rec.id] = rec
    p = create_planner(name)
    if name != "visibility_graph":
        p.max_samples = 400
        if hasattr(p, "max_until_found"):
            p.max_until_found = 400
        if hasattr(p, "max_batches"):
            p.max_batches = p.batches = 3
    r = p.plan(b, [S], G)
    assert not r.success and r.path == (S,)


def test_paths_respect_the_planning_raster_on_maps():
    for seed in (1, 2, 3):
        sim = Simulation(SimConfig(world_kind="continuous", map_kind="polygons", fow=False, n_agents=1, planner="none", seed=seed))
        a = sim.scenario.agents[0]
        grid = plan_grid(sim.scenario.belief, 0.35, 0.5)
        for name in SAMPLERS:
            r = create_planner(name).plan(sim.scenario.belief, [a.pos], sim.goal_for(a))
            assert r.success
            segs = list(zip(r.path, r.path[1:]))
            for i, (u, v) in enumerate(segs):
                assert not grid.segment_blocked(u, v, 0.8 if i == 0 else 0.0, 0.8 if i == len(segs) - 1 else 0.0)


def test_visibility_graph_on_a_grid_world():
    sim = Simulation(SimConfig(world_kind="grid", map_kind="rooms", fow=False, n_agents=1, planner="visibility_graph", seed=2))
    a = sim.scenario.agents[0]
    res = sim.results[a.id]
    assert res.success
    astar = create_planner("a_star").plan(sim.scenario.belief, [a.pos], sim.goal_for(a))
    assert res.cost <= astar.cost + 1e-6  # any-angle: never longer than the 8-connected path


def test_sim_runs_to_the_goal():
    for name in ("visibility_graph", "rrt"):
        sim = Simulation(SimConfig(world_kind="continuous", map_kind="sparse", fow=True, n_agents=3, planner=name, seed=5))
        assert sim.run(60)


def test_capabilities():
    poly = scenario_traits(SimConfig(world_kind="continuous"))
    grid = scenario_traits(SimConfig(world_kind="grid"))
    for name in ALL:
        assert compatibility(name, poly).runnable and compatibility(name, grid).runnable
        assert compatibility(name, {"world": "continuous"}).fit == Fit.DESIGNED
        assert compatibility(name, {"world": "grid"}).fit == Fit.ADAPTED


def test_live_view_shows_their_debug_layers():
    import pygame

    from swarmpathfinding.viz.live import LiveApp
    from swarmpathfinding.viz.settings import LabSettings

    for name in ALL:
        app = LiveApp(LabSettings(world_kind="continuous", map_kind="sparse", n_agents=2, planner=name, show_search=True))
        assert app.panel.get("show_search").visible
        assert not app.panel.get("heatmap").visible  # no step-by-step trace for these
        extra = app.sim.results[app.sim.focus_id()].extra
        assert {"tree", "vg_edges"} & set(extra)
        app.draw()
    pygame.quit()
    assert np.isfinite(OPT)
