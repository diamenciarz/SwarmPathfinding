"""ORCA: the solver itself, the sim integration, benchmarks and the live view."""

import math
import os

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

import numpy as np  # noqa: E402
import pygame  # noqa: E402

from swarmpathfinding.bench.experiment import ExperimentSpec, row_for, split_variant, variant_label  # noqa: E402
from swarmpathfinding.core.capabilities import Fit, compatibility, scenario_traits  # noqa: E402
from swarmpathfinding.sim.orca import orca_step, solve, wall_lines  # noqa: E402
from swarmpathfinding.sim.simulation import SimConfig, Simulation  # noqa: E402

R = 0.35
DT = 1 / 30


def _swap(n, radius=8.0, t_max=20.0):
    ang = np.arange(n) * 2 * math.pi / n
    pos = np.stack([radius * np.cos(ang), radius * np.sin(ang)], 1)
    goal = -pos.copy()
    vel = np.zeros_like(pos)
    t, min_sep = 0.0, 9e9
    while t < t_max:
        d = goal - pos
        dist = np.linalg.norm(d, axis=1, keepdims=True)
        pref = np.where(dist > 1e-9, d / np.maximum(dist, 1e-9) * np.minimum(6.0, dist / DT), 0.0)
        active = dist[:, 0] > 0.05
        if not active.any():
            break
        vel, _ = orca_step(pos, vel, pref, active, R, 6.0, DT)
        pos = pos + vel * DT
        t += DT
        sep = np.linalg.norm(pos[:, None] - pos[None], axis=-1) + np.eye(n) * 99
        sep[~active] = 99
        sep[:, ~active] = 99
        min_sep = min(min_sep, float(sep.min()))
    return t, min_sep


def test_head_on_pair_passes_without_touching():
    t, sep = _swap(2)
    assert sep >= 2 * R - 1e-3
    assert t < 2 * 8.0 / 6.0 * 1.3  # a small detour, not a stall


def test_circle_swap_is_collision_free():
    t, sep = _swap(12)
    assert t < 20.0
    assert sep >= 2 * R * 0.95


def test_no_neighbours_keeps_preferred_velocity():
    assert solve([], (3.0, -1.0), 6.0) == (3.0, -1.0)
    assert math.hypot(*solve([], (30.0, 0.0), 6.0)) == pytest_approx(6.0)


def pytest_approx(v):
    import pytest

    return pytest.approx(v)


def test_wall_line_stops_approach_but_allows_sliding():
    pos, wall = (0.0, 0.0), [(1.0, 0.0)]  # wall 1 unit to the right
    lines = wall_lines(pos, R, wall, tau=0.5)
    v = solve(lines, (6.0, 0.0), 6.0, n_obst=1)
    assert v[0] <= (1.0 - R) / 0.5 + 1e-6
    slide = solve(lines, (0.0, 6.0), 6.0, n_obst=1)
    assert slide == pytest_approx((0.0, 6.0))


def _sim(avoidance, planner="a_star", **kw):
    cfg = dict(world_kind="grid", map_kind="empty", starts="opposite", n_agents=16, fow=False, seed=3)
    cfg.update(kw)
    return Simulation(SimConfig(planner=planner, avoidance=avoidance, **cfg))


def test_orca_cuts_overlap_and_still_arrives():
    off, on = _sim("none"), _sim("orca")
    assert off.run(40) and on.run(40)
    assert on.stats.overlap_s < off.stats.overlap_s * 0.1


def test_orca_runs_are_deterministic():
    a, b = _sim("orca"), _sim("orca")
    a.run(10)
    b.run(10)
    assert [x.pos for x in a.scenario.agents] == [x.pos for x in b.scenario.agents]
    assert a.stats.overlap_s == b.stats.overlap_s


def test_orca_on_polygons_and_debug_record():
    sim = _sim("orca", world_kind="continuous", map_kind="sparse", n_agents=8, starts="clump")
    for _ in range(10):
        sim.step(DT, sync_plans=True)
    dbg = sim.orca_debug
    assert dbg is not None and dbg.agent == 0
    assert math.hypot(*dbg.new) <= dbg.max_speed + 1e-6


def test_avoidance_none_has_no_debug():
    sim = _sim("none")
    sim.step(DT, sync_plans=True)
    assert sim.orca_debug is None


def test_orca_planner_forces_avoidance_and_rates_scenarios():
    sim = _sim("none", planner="orca")
    assert sim.avoidance == "orca"
    sim.step(DT, sync_plans=True)
    assert sim.orca_debug is not None
    crowd = scenario_traits(SimConfig(n_agents=40, starts="opposite"))
    assert compatibility("orca", crowd).fit == Fit.ADAPTED  # grid walls are its weak spot
    assert compatibility("orca", crowd).runnable


def test_benchmark_variants_and_rows():
    spec = ExperimentSpec(n_scenarios=1, planners=["a_star", "orca"], avoidance=["none", "orca"])
    trials = spec.trials()
    assert [t.planner for t in trials] == ["a_star", "a_star+orca", "orca"]
    assert trials[1].config.avoidance == "orca" and trials[1].config.planner == "a_star"
    row = row_for(trials[1], {"overlap_s": 0.0})
    assert row["algo"] == "a_star" and row["avoidance"] == "orca"
    assert split_variant(variant_label("a_star", "orca")) == ("a_star", "orca")
    assert split_variant("dijkstra") == ("dijkstra", "none")


def test_live_view_avoidance_switches_without_rebuild():
    from swarmpathfinding.viz.live import LiveApp
    from swarmpathfinding.viz.menu import planner_label
    from swarmpathfinding.viz.settings import LabSettings

    app = LiveApp(LabSettings(n_agents=6, starts="opposite", map_kind="empty"))
    app.draw()
    assert not app.panel.get("show_orca").visible
    scenario = app.sim.scenario
    app.panel.get("avoidance").index = 1
    app._on_panel_change("avoidance", True)
    assert app.sim.scenario is scenario and app.sim.config.avoidance == "orca"
    assert app.panel.get("show_orca").visible
    app.paused = False
    for _ in range(5):
        app.tick(DT)
    app.draw()  # ORCA overlay drawn
    assert planner_label("a_star+orca").endswith("+ ORCA")
    pygame.quit()
