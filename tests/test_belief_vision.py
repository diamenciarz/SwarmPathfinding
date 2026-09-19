from swarmpathfinding.core.belief import BLOCKED, BeliefMap
from swarmpathfinding.core.geom import Cell, Vec2
from swarmpathfinding.physics.bodies import Agent
from swarmpathfinding.vision.shapes import CircleVision, ConeVision
from swarmpathfinding.worlds.grid import GridWorld


def test_observe_discrepancy():
    world = GridWorld.empty(10, 10)
    world.punch_wall(Cell(4, 4))
    belief = BeliefMap.unknown_grid(10, 10)
    n = belief.observe_cells([Cell(4, 4), Cell(0, 0)], world, now=1.0)
    assert n == 2
    assert int(belief.occupancy[4, 4]) == int(BLOCKED)
    n2 = belief.observe_cells([Cell(4, 4)], world, now=2.0)
    assert n2 == 0


def test_freespace_unknown_not_blocked():
    belief = BeliefMap.unknown_grid(5, 5)
    assert not belief.is_blocked_for_planning(Cell(2, 2))
    belief.freespace_unknown = False
    assert belief.is_blocked_for_planning(Cell(2, 2))


def test_circle_vision_cells():
    world = GridWorld.empty(20, 20)
    vision = CircleVision(radius=3.0)
    cells = list(vision.visible_cells(Vec2(10.5, 10.5), 0.0, world, require_los=False))
    assert Cell(10, 10) in cells
    assert all(world.in_bounds(c) for c in cells)
    assert len(cells) > 10


def test_cone_excludes_behind():
    world = GridWorld.empty(20, 20)
    vision = ConeVision(radius=5.0, half_angle=0.4)
    origin = Vec2(10.5, 10.5)
    heading = 0.0  # +x
    cells = list(vision.visible_cells(origin, heading, world, require_los=False))
    xs = [c.x for c in cells]
    assert max(xs) > 10
    # should not see far to the left
    assert min(xs) >= 9


def test_vision_blocked_by_wall_when_los_required():
    world = GridWorld.empty(12, 12)
    for y in range(12):
        world.punch_wall(Cell(6, y))
    world.delete_wall(Cell(6, 0))  # keep one hole unused
    vision = CircleVision(radius=8.0)
    origin = Vec2(3.5, 6.5)
    cells = list(vision.visible_cells(origin, 0.0, world, require_los=True))
    assert all(c.x <= 6 for c in cells)
