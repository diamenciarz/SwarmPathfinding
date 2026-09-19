import numpy as np

from swarmpathfinding.generators.grid_maps import clustered_grid, maze_grid, random_grid, rooms_grid, street_grid
from swarmpathfinding.generators.polygon_maps import moving_polygons, random_polygons, u_trap
from swarmpathfinding.generators.scenarios import make_scenario


def test_bernoulli_occupancy_targeted():
    rng = np.random.default_rng(0)
    world = random_grid(40, 40, p=0.2, rng=rng)
    occ = world.occupancy_fraction()
    assert abs(occ - 0.2) < 0.05


def test_rooms_have_free_space():
    world = rooms_grid(40, 30, n_rooms=6, rng=np.random.default_rng(1))
    assert world.occupancy_fraction() < 0.9
    assert world.occupancy_fraction() > 0.2


def test_maze_has_passages():
    world = maze_grid(21, 21, rng=np.random.default_rng(2))
    assert not world.blocked[1, 1]
    assert world.occupancy_fraction() > 0.4


def test_clustered_occupancy_targeted():
    world = clustered_grid(32, 32, p=0.25, rng=np.random.default_rng(3))
    assert abs(world.occupancy_fraction() - 0.25) < 0.06


def test_street_has_corridors():
    world = street_grid(48, 36, rng=np.random.default_rng(4))
    assert world.occupancy_fraction() > 0.2
    # a street row near the origin should have free cells
    assert not np.all(world.blocked[2])


def test_polygons_valid():
    world = random_polygons(40, 30, occupancy=0.1, n=8, rng=np.random.default_rng(4))
    assert all(obs.polygon.is_valid for obs in world.obstacles)
    trap = u_trap()
    assert trap.obstacles[0].polygon.is_valid


def test_polygon_regenerate_does_not_crash_union():
    """Live-lab crash: unary_union TopologyException on invalid random k-gons."""
    for seed in range(12):
        world = random_polygons(48, 36, occupancy=0.10, n=20, rng=np.random.default_rng(seed))
        frac = world.occupancy_fraction()
        assert 0.0 <= frac < 1.0
        union = world.obstacle_union()
        assert union.is_valid or union.is_empty


def test_moving_polygons_have_velocity():
    world = moving_polygons(40, 30, n_static=3, n_moving=2, rng=np.random.default_rng(5))
    movers = [obs for obs in world.obstacles if obs.velocity.length_sq() > 0]
    assert len(movers) == 2
    world.step_movers(0.2)
    assert world.generation >= 1


def test_make_scenario_grid():
    sc = make_scenario(world_kind="grid", map_kind="bernoulli", n_agents=5, seed=7, fow=True)
    assert len(sc.agents) == 5
    assert sc.belief.revealed_fraction() == 0.0
    sc2 = make_scenario(world_kind="grid", map_kind="empty", n_agents=3, seed=1, fow=False)
    assert sc2.belief.revealed_fraction() == 1.0
