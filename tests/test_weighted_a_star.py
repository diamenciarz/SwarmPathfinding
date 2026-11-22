from swarmpathfinding.core.belief import BeliefMap
from swarmpathfinding.core.geom import Cell, Vec2
from swarmpathfinding.core.registry import create_planner
from swarmpathfinding.core.results import Budget
from swarmpathfinding.generators.scenarios import make_scenario
from swarmpathfinding.worlds.grid import GridWorld


def test_optimal_on_empty_grid():
    world = GridWorld.empty(10, 10)
    belief = BeliefMap.from_truth_grid(world)
    planner = create_planner("weighted_a_star")
    r = planner.plan(belief, [Vec2(0.5, 0.5)], Vec2(9.5, 0.5))
    assert r.success
    assert abs(r.cost - 9.0) < 1e-6


def test_routes_around_wall_and_fails_when_sealed():
    world = GridWorld.empty(10, 10)
    for y in range(9):
        world.punch_wall(Cell(5, y))
    planner = create_planner("weighted_a_star")
    r = planner.plan(BeliefMap.from_truth_grid(world), [Vec2(1.5, 1.5)], Vec2(8.5, 1.5))
    assert r.success and r.cost > 7.0
    world.punch_wall(Cell(5, 9))
    r = planner.plan(BeliefMap.from_truth_grid(world), [Vec2(1.5, 1.5)], Vec2(8.5, 1.5))
    assert not r.success


def test_unknown_is_optimistic():
    belief = BeliefMap.unknown_grid(10, 10)
    r = create_planner("weighted_a_star").plan(belief, [Vec2(0.5, 0.5)], Vec2(9.5, 9.5))
    assert r.success


def test_weight_bounds_cost_and_cuts_expansions():
    sc = make_scenario(world_kind="grid", map_kind="bernoulli", density=0.25, n_agents=1, seed=4, fow=False)
    planner = create_planner("weighted_a_star")
    start = sc.agents[0].pos
    opt = planner.plan(sc.belief, [start], sc.goal, Budget(weight=1.0))
    fast = planner.plan(sc.belief, [start], sc.goal, Budget(weight=2.5))
    assert opt.success and fast.success
    assert fast.expansions <= opt.expansions
    assert opt.cost <= fast.cost <= 2.5 * opt.cost + 1e-6


def test_continuous_avoids_known_obstacles():
    sc = make_scenario(world_kind="continuous", map_kind="utrap", n_agents=1, seed=2, fow=False)
    planner = create_planner("weighted_a_star")
    start, goal = sc.agents[0].pos, sc.goal
    assert not sc.world.line_of_sight(start, goal)  # a straight shot would hit the trap
    r = planner.plan(sc.belief, [start], goal)
    assert r.success
    world = sc.world
    for a, b in zip(r.path[1:-2], r.path[2:-1]):
        assert world.line_of_sight(a, b)
