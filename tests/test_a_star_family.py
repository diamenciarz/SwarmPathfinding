"""Dijkstra, A*, greedy best-first and weighted A* share one search."""

import numpy as np
import pytest

from swarmpathfinding.core.capabilities import Fit, compatibility, scenario_traits
from swarmpathfinding.core.registry import create_planner
from swarmpathfinding.core.results import Budget
from swarmpathfinding.core.trace import EXPAND
from swarmpathfinding.generators.scenarios import make_scenario
from swarmpathfinding.sim.simulation import SimConfig

FAMILY = ("dijkstra", "a_star", "greedy_best_first", "weighted_a_star")


@pytest.fixture(scope="module")
def scenario():
    return make_scenario(world_kind="grid", map_kind="clustered", density=0.2, n_agents=1, seed=3, fow=False)


def _plan(name, sc, w=1.0, trace=False):
    return create_planner(name).plan(sc.belief, [sc.agents[0].pos], sc.goal, Budget(weight=w, record_trace=trace))


def test_dijkstra_and_a_star_are_optimal_and_agree(scenario):
    d, a = _plan("dijkstra", scenario), _plan("a_star", scenario)
    assert d.success and a.success
    assert abs(d.cost - a.cost) < 1e-6
    assert a.expansions < d.expansions  # the heuristic focuses the search


def test_greedy_is_fast_but_not_better(scenario):
    a, g = _plan("a_star", scenario), _plan("greedy_best_first", scenario)
    assert g.success and g.expansions <= a.expansions
    assert g.cost >= a.cost - 1e-6


def test_weighted_ignores_weight_for_others(scenario):
    for name in ("dijkstra", "a_star", "greedy_best_first"):
        assert _plan(name, scenario, w=1.0).cost == _plan(name, scenario, w=3.0).cost
    assert _plan("weighted_a_star", scenario, w=3.0).expansions <= _plan("weighted_a_star", scenario, w=1.0).expansions


@pytest.mark.parametrize("name", FAMILY)
def test_trace_has_parents_and_h(scenario, name):
    r = _plan(name, scenario, trace=True)
    tr = r.extra["trace"]
    assert sum(1 for k in tr.kinds if k == EXPAND) == r.expansions
    assert len(tr.h) == len(tr.kinds) == len(tr.px)
    # every expanded cell but the start has a parent
    exp = [i for i, k in enumerate(tr.kinds) if k == EXPAND]
    assert tr.px[exp[0]] == -1 and all(tr.px[i] >= 0 for i in exp[1:])


@pytest.mark.parametrize("name", FAMILY)
def test_family_runs_everywhere(name):
    for world in ("grid", "continuous"):
        traits = scenario_traits(SimConfig(world_kind=world, map_kind="bernoulli" if world == "grid" else "polygons"))
        assert compatibility(name, traits).runnable
    assert compatibility(name, {"world": "grid"}).fit == Fit.DESIGNED
