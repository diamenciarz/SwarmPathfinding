import subprocess
import sys

from swarmpathfinding.core.capabilities import Fit, compatibility, matrix, scenario_traits
from swarmpathfinding.core.geom import Vec2
from swarmpathfinding.core.registry import create_planner
from swarmpathfinding.core.results import Budget
from swarmpathfinding.core.trace import EXPAND, PUSH
from swarmpathfinding.generators.scenarios import make_scenario
from swarmpathfinding.sim import SimConfig, Simulation


def test_sim_does_not_import_pygame():
    code = "import sys, swarmpathfinding.sim; print('pygame' in sys.modules)"
    out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, check=True)
    assert out.stdout.strip() == "False"


def test_headless_run_reaches_goal():
    sim = Simulation(SimConfig(map_kind="bernoulli", n_agents=5, fow=True, seed=2))
    assert sim.run(max_time=60.0)
    assert len(sim.stats.arrived_at) == 5
    assert sim.stats.plans >= 1 and sim.stats.expansions > 0
    assert all(d > 0 for d in sim.stats.travelled.values())


def test_continuous_headless_run():
    sim = Simulation(SimConfig(world_kind="continuous", map_kind="polygons", n_agents=4, fow=False, seed=3))
    assert sim.run(max_time=60.0)


def test_same_config_same_run():
    cfg = SimConfig(map_kind="clustered", n_agents=6, fow=True, seed=5, changes="slow_edit")
    a, b = Simulation(cfg.with_()), Simulation(cfg.with_())
    a.run(30.0)
    b.run(30.0)
    assert a.stats.arrived_at == b.stats.arrived_at
    assert a.stats.expansions == b.stats.expansions
    assert [ag.pos for ag in a.scenario.agents] == [ag.pos for ag in b.scenario.agents]


def test_slow_edit_applies_automatically():
    sim = Simulation(SimConfig(map_kind="rooms", n_agents=2, fow=False, changes="slow_edit", edit_at=1.0))
    assert sim.scenario.pending_edit is not None
    while sim.now < 0.9:
        sim.step(1 / 30)
    assert sim.scenario.pending_edit is not None
    while sim.now < 1.1:
        sim.step(1 / 30)
    assert sim.scenario.pending_edit is None
    assert sim.stats.edit_time is not None and abs(sim.stats.edit_time - 1.0) < 0.05


def test_manual_edit_stays_pending_without_changes_trait():
    sim = Simulation(SimConfig(map_kind="rooms", n_agents=1, fow=False, changes="none", edit_at=0.1))
    for _ in range(30):
        sim.step(1 / 30)
    assert sim.scenario.pending_edit is not None
    assert sim.apply_pending_edit()


def test_traits_and_compatibility():
    traits = scenario_traits(SimConfig(world_kind="continuous", map_kind="movers", fow=True, n_agents=30))
    assert traits == {
        "world": "continuous",
        "knowledge": "fog",
        "changes": "movers",
        "agents": "crowd",
        "goals": "shared",
    }
    comp = compatibility("weighted_a_star", traits)
    assert comp.runnable and comp.fit == Fit.ADAPTED
    assert any("raster" in n for n in comp.notes())
    single_grid = scenario_traits(SimConfig(world_kind="grid", fow=False, n_agents=1))
    assert compatibility("weighted_a_star", single_grid).fit == Fit.DESIGNED
    m = matrix()
    assert {"none", "straight_line", "weighted_a_star"} <= set(m)
    assert all(len(row) == 12 for row in m.values())


def test_trace_matches_expansions():
    sc = make_scenario(world_kind="grid", map_kind="bernoulli", n_agents=1, seed=4, fow=False)
    planner = create_planner("weighted_a_star")
    r = planner.plan(sc.belief, [sc.agents[0].pos], sc.goal, Budget(record_trace=True))
    trace = r.extra["trace"]
    assert trace.kinds[0] == PUSH
    assert sum(1 for k in trace.kinds if k == EXPAND) == r.expansions
    _, closed = trace.state_at(len(trace), sc.belief.occupancy.shape)
    assert (closed == r.extra["closed"]).all()
    plain = planner.plan(sc.belief, [sc.agents[0].pos], sc.goal, Budget())
    assert "trace" not in plain.extra


def test_set_goal_resets_arrivals():
    sim = Simulation(SimConfig(map_kind="empty", n_agents=1, fow=False))
    sim.run(30.0)
    assert sim.stats.arrived_at
    sim.set_goal(Vec2(5.5, 5.5))
    assert not sim.stats.arrived_at
