"""LPA* and D* Lite: same costs as A* after any sequence of edits and moves."""

import random

import pytest

from swarmpathfinding.core.belief import BeliefMap
from swarmpathfinding.core.capabilities import Fit, rating
from swarmpathfinding.core.geom import Cell
from swarmpathfinding.core.registry import create_planner
from swarmpathfinding.core.results import Budget
from swarmpathfinding.generators.scenarios import make_scenario
from swarmpathfinding.sim.simulation import SimConfig, Simulation
from swarmpathfinding.viz.search_view import KEPT, SearchPlayer


@pytest.mark.parametrize("seed", range(6))
def test_repairs_match_fresh_a_star(seed):
    rng = random.Random(seed)
    sc = make_scenario(world_kind="grid", map_kind="bernoulli", density=0.2, n_agents=1, seed=seed, fow=False)
    world, pos = sc.world, sc.agents[0].pos
    planners = {n: create_planner(n) for n in ("d_star_lite", "lpa_star")}
    reused = 0
    for _ in range(10):
        belief = BeliefMap.from_truth_grid(world)
        ref = create_planner("a_star").plan(belief, [pos], sc.goal)
        for name, p in planners.items():
            r = p.plan(belief, [pos], sc.goal, Budget(agent_id=7))
            assert r.success == ref.success, name
            if ref.success:
                assert abs(r.cost - ref.cost) < 1e-6, name
            reused += bool(r.extra["reused"])
        if ref.success and len(ref.path) > 3 and rng.random() < 0.5:
            pos = ref.path[2]
        for _ in range(rng.randint(1, 5)):
            c = Cell(rng.randrange(world.width), rng.randrange(world.height))
            if c in (world.world_to_cell(pos), world.world_to_cell(sc.goal)):
                continue
            world.punch_wall(c) if rng.random() < 0.6 else world.delete_wall(c)
    assert reused > 0


def test_d_star_keeps_tree_while_agent_moves_lpa_restarts():
    sc = make_scenario(world_kind="grid", map_kind="empty", n_agents=1, seed=1, fow=False)
    belief = BeliefMap.from_truth_grid(sc.world)
    d, lp = create_planner("d_star_lite"), create_planner("lpa_star")
    first = d.plan(belief, [sc.agents[0].pos], sc.goal, Budget(agent_id=0))
    lp.plan(belief, [sc.agents[0].pos], sc.goal, Budget(agent_id=0))
    moved = first.path[2]
    again = d.plan(belief, [moved], sc.goal, Budget(agent_id=0))
    assert again.extra["reused"] and again.expansions < first.expansions
    assert not lp.plan(belief, [moved], sc.goal, Budget(agent_id=0)).extra["reused"]


@pytest.mark.parametrize("planner", ["d_star_lite", "lpa_star"])
def test_fog_run_arrives_and_reuses(planner):
    sim = Simulation(SimConfig(map_kind="clustered", n_agents=3, fow=True, planner=planner, seed=4, changes="slow_edit"))
    sim.record_focus_trace = True
    reused = False
    for _ in range(int(60 * 30)):
        sim.step(1 / 30, sync_plans=True)
        r = sim.results.get(sim.focus_id())
        if r is not None and r.extra.get("reused"):
            reused = True
            tr = r.extra.get("trace")
            if tr is not None and tr.base_g is not None:
                p = SearchPlayer()
                p.load(tr, r.extra["closed"].shape, at_end=False)
                assert (p.state == KEPT).any()
        if sim.done:
            break
    assert sim.done
    if planner == "d_star_lite":
        assert reused


def test_capabilities():
    assert rating("d_star_lite", "changes", "slow_edit").fit == Fit.DESIGNED
    assert rating("d_star_lite", "knowledge", "fog").fit == Fit.DESIGNED
    assert rating("lpa_star", "knowledge", "fog").fit == Fit.ADAPTED
