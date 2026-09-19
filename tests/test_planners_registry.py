from swarmpathfinding.core.geom import Vec2
from swarmpathfinding.core.registry import create_planner, list_planners
from swarmpathfinding.generators.scenarios import make_scenario
from swarmpathfinding.swarm.group import centroid, cluster_positions


def test_registry_lists_stubs():
    names = list_planners()
    assert "none" in names
    assert "straight_line" in names


def test_none_planner_fails():
    planner = create_planner("none")
    result = planner.plan(None, [Vec2(0, 0)], Vec2(1, 1))
    assert not result.success


def test_straight_line_empty_map():
    sc = make_scenario(world_kind="grid", map_kind="empty", n_agents=1, seed=1, fow=False)
    planner = create_planner("straight_line")
    result = planner.plan(sc.belief, [sc.agents[0].pos], sc.goal)
    assert result.success
    assert len(result.path) == 2


def test_straight_line_utrap_known_map_blocked():
    sc = make_scenario(
        world_kind="continuous",
        map_kind="utrap",
        n_agents=1,
        seed=1,
        fow=False,
        clump=True,
    )
    planner = create_planner("straight_line")
    # start left of the C, goal inside the pocket — the wall should block a straight shot
    from swarmpathfinding.core.geom import Vec2

    start = Vec2(8.0, 18.0)
    goal = Vec2(28.0, 18.0)
    result = planner.plan(sc.belief, [start], goal)
    assert not result.success


def test_clusters():
    pts = [Vec2(0, 0), Vec2(0.2, 0.1), Vec2(10, 10)]
    groups = cluster_positions(pts, radius=1.0)
    assert len(groups) == 2
    c = centroid([pts[0], pts[1]])
    assert abs(c.x - 0.1) < 1e-6
