"""JPS matches A*; Theta* is any-angle and never longer than A*."""

import numpy as np
import pytest

from swarmpathfinding.core.geom import Cell
from swarmpathfinding.core.registry import create_planner
from swarmpathfinding.core.results import Budget
from swarmpathfinding.core.trace import LOS, SCAN
from swarmpathfinding.generators.scenarios import make_scenario
from swarmpathfinding.planners.any_angle import line_of_sight
from swarmpathfinding.sim.simulation import SimConfig, Simulation

CASES = [(mk, seed) for mk in ("bernoulli", "clustered", "rooms", "maze", "street") for seed in range(8)]


def _plan(name, sc, trace=False):
    return create_planner(name).plan(sc.belief, [sc.agents[0].pos], sc.goal, Budget(record_trace=trace))


@pytest.mark.parametrize("mk,seed", CASES)
def test_jps_equals_a_star_and_theta_not_longer(mk, seed):
    sc = make_scenario(world_kind="grid", map_kind=mk, density=0.25, n_agents=1, seed=seed, fow=False)
    a, j, t = _plan("a_star", sc), _plan("jps", sc), _plan("theta_star", sc)
    assert a.success == j.success == t.success
    if a.success:
        assert abs(a.cost - j.cost) < 1e-6
        assert j.expansions <= a.expansions
        assert t.cost <= a.cost + 1e-6


def test_theta_cuts_corners_on_open_map():
    sc = make_scenario(world_kind="grid", map_kind="empty", n_agents=1, seed=1, fow=False)
    a, t = _plan("a_star", sc), _plan("theta_star", sc)
    assert t.cost < a.cost - 0.1 or len(t.path) <= 3


def test_line_of_sight():
    blocked = np.zeros((5, 5), dtype=bool)
    flat = blocked.ravel().tolist()
    assert line_of_sight(flat, 5, 5, 0, 0, 4, 3)
    blocked[2, 2] = True
    flat = blocked.ravel().tolist()
    assert not line_of_sight(flat, 5, 5, 0, 0, 4, 4)  # straight through (2, 2)
    assert line_of_sight(flat, 5, 5, 0, 0, 4, 0)
    blocked[:] = False
    blocked[1, 0] = True  # a corner squeeze between (0,1) and (1,0)
    flat = blocked.ravel().tolist()
    assert not line_of_sight(flat, 5, 5, 0, 0, 1, 1)


def test_traces_record_scans_and_sight_tests():
    sc = make_scenario(world_kind="grid", map_kind="bernoulli", density=0.2, n_agents=1, seed=7, fow=False)
    j = _plan("jps", sc, trace=True).extra["trace"]
    t = _plan("theta_star", sc, trace=True).extra["trace"]
    lz = _plan("lazy_theta_star", sc, trace=True).extra["trace"]
    assert SCAN in j.kinds
    n_t = sum(1 for k in t.kinds if k == LOS)
    n_l = sum(1 for k in lz.kinds if k == LOS)
    assert n_t > 0 and 0 < n_l < n_t


@pytest.mark.parametrize("planner", ["jps", "theta_star", "lazy_theta_star"])
@pytest.mark.parametrize("world", ["grid", "continuous"])
def test_agents_arrive(planner, world):
    cfg = SimConfig(world_kind=world, map_kind="bernoulli" if world == "grid" else "sparse", n_agents=3, fow=False, planner=planner, seed=2)
    sim = Simulation(cfg)
    assert sim.run(max_time=60)


def test_jps_start_on_goal():
    blocked = np.zeros((4, 4), dtype=bool)
    from swarmpathfinding.planners.any_angle import jps_search, theta_search

    cells, n, _ = jps_search(blocked, Cell(1, 1), Cell(1, 1), None)
    assert cells == [Cell(1, 1)]
    cells, n, _ = theta_search(blocked, Cell(1, 1), Cell(1, 1), None)
    assert cells == [Cell(1, 1)]
