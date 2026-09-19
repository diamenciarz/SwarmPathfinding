from swarmpathfinding.core.geom import Cell
from swarmpathfinding.worlds.grid import GridWorld


def test_neighbors_skip_blocked_and_squeeze():
    world = GridWorld.empty(5, 5)
    world.punch_wall(Cell(2, 1))
    world.punch_wall(Cell(1, 2))
    # diagonal from (1,1) to (2,2) is squeezed
    n = world.neighbors(Cell(1, 1))
    assert Cell(2, 2) not in n
    assert Cell(1, 0) in n


def test_line_of_sight_open():
    world = GridWorld.empty(8, 8)
    assert world.line_of_sight(Cell(0, 0), Cell(7, 0))
    assert world.line_of_sight(Cell(0, 0), Cell(7, 7))


def test_line_of_sight_blocked():
    world = GridWorld.empty(8, 8)
    world.punch_wall(Cell(3, 0))
    assert not world.line_of_sight(Cell(0, 0), Cell(7, 0))


def test_punch_and_delete():
    world = GridWorld.empty(4, 4)
    assert world.punch_wall(Cell(1, 1))
    assert world.is_blocked(Cell(1, 1))
    assert not world.punch_wall(Cell(1, 1))
    assert world.delete_wall(Cell(1, 1))
    assert not world.is_blocked(Cell(1, 1))
