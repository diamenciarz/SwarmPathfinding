from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from swarmpathfinding.core.belief import BeliefMap, ObstacleRecord
from swarmpathfinding.core.geom import Cell, Vec2
from swarmpathfinding.generators.grid_maps import (
    clustered_grid,
    corridor_grid,
    empty_grid,
    ensure_free,
    game_grid,
    maze_grid,
    open_grid,
    pick_interior_wall,
    random_grid,
    rooms_grid,
    street_grid,
)
from swarmpathfinding.generators.polygon_maps import (
    dense_clutter,
    moving_polygons,
    pick_deletable_obstacle,
    random_polygons,
    sparse_giants,
    u_trap,
)
from swarmpathfinding.generators.spawn import spawn_agents
from swarmpathfinding.physics.bodies import Agent
from swarmpathfinding.worlds.continuous import ContinuousWorld
from swarmpathfinding.worlds.grid import GridWorld

GRID_MAPS = ("empty", "bernoulli", "clustered", "rooms", "maze", "street", "open", "game", "corridor")
POLY_MAPS = ("polygons", "sparse", "clutter", "utrap", "movers")
START_LAYOUTS = ("clump", "scattered", "flanks", "opposite")


@dataclass(slots=True)
class PendingEdit:
    kind: str  # "cell" | "obstacle"
    cell: Cell | None = None
    obstacle_id: str | None = None


@dataclass(slots=True)
class Scenario:
    world: GridWorld | ContinuousWorld
    agents: list[Agent]
    goal: Vec2
    seed: int
    map_kind: str
    density: float
    belief: BeliefMap
    rng: np.random.Generator = field(repr=False)
    pack: str = "custom"
    scenario_id: str = ""
    start_layout: str = "clump"
    goals: tuple[Vec2, ...] = ()
    pending_edit: PendingEdit | None = None
    shared_belief: bool = True
    fow: bool = True
    world_kind: str = "grid"

    @property
    def starts(self) -> list[Vec2]:
        return [agent.pos for agent in self.agents]


def make_scenario(
    *,
    world_kind: str = "grid",
    map_kind: str = "bernoulli",
    density: float = 0.15,
    n_agents: int = 8,
    seed: int = 1,
    clump: bool | None = None,
    start_layout: str = "clump",
    vision_kind: str = "circle",
    vision_radius: float = 6.0,
    fow: bool = True,
    width: int = 48,
    height: int = 36,
    eight: bool = True,
    pack: str = "custom",
    slow_edit: bool = False,
    shared_belief: bool = True,
    scenario_id: str = "",
) -> Scenario:
    rng = np.random.default_rng(seed)
    if clump is not None:
        start_layout = "clump" if clump else "scattered"
    if world_kind == "grid":
        world = _make_grid(map_kind, width, height, density, rng, eight)
        start_cell = world.sample_free(rng, margin=2)
        goal_cell = world.sample_free(rng, margin=2)
        ensure_free(world, start_cell, radius=1)
        ensure_free(world, goal_cell, radius=1)
        if start_cell == goal_cell:
            goal_cell = world.sample_free(rng, margin=2)
            ensure_free(world, goal_cell, radius=1)
        goal = world.cell_center(goal_cell)
        extra = None
        if start_layout == "opposite":
            extra = world.cell_center(start_cell)
        agents = spawn_agents(
            n_agents,
            world,
            goal,
            rng,
            start_layout=start_layout,
            vision_kind=vision_kind,
            vision_radius=vision_radius,
            extra_goal=extra,
        )
        belief = BeliefMap.unknown_grid(world.width, world.height)
        if not fow:
            belief = BeliefMap.from_truth_grid(world)
        pending = None
        if slow_edit:
            wall = pick_interior_wall(world, rng)
            if wall is not None:
                pending = PendingEdit(kind="cell", cell=wall)
        goals = tuple(dict.fromkeys([*(a.goal or goal for a in agents), goal]))
        return Scenario(
            world=world,
            agents=agents,
            goal=goal,
            seed=seed,
            map_kind=map_kind,
            density=world.occupancy_fraction(),
            belief=belief,
            rng=rng,
            pack=pack,
            scenario_id=scenario_id or f"{pack}-{world_kind}-{map_kind}-s{seed}-n{n_agents}",
            start_layout=start_layout,
            goals=goals,
            pending_edit=pending,
            shared_belief=shared_belief,
            fow=fow,
            world_kind="grid",
        )

    world = _make_poly(map_kind, float(width), float(height), density, rng)
    goal = world.sample_free(rng, radius=0.4, margin=2.0)
    extra = None
    if start_layout == "opposite":
        extra = world.sample_free(rng, radius=0.4, margin=2.0)
    agents = spawn_agents(
        n_agents,
        world,
        goal,
        rng,
        start_layout=start_layout,
        vision_kind=vision_kind,
        vision_radius=vision_radius,
        extra_goal=extra,
    )
    raster_w, raster_h = int(width), int(height)
    belief = BeliefMap.unknown_grid(raster_w, raster_h)
    if not fow:
        belief.occupancy[:, :] = 0
        belief.seen_time[:, :] = 0.0
        for obs in world.obstacles:
            belief.observe_obstacle(
                ObstacleRecord(
                    id=obs.id,
                    vertices=obs.vertices(),
                    position=obs.position,
                    velocity=obs.velocity,
                    last_seen_time=0.0,
                )
            )
    pending = None
    if slow_edit:
        oid = pick_deletable_obstacle(world, rng)
        if oid is not None:
            pending = PendingEdit(kind="obstacle", obstacle_id=oid)
    goals = tuple(dict.fromkeys([*(a.goal or goal for a in agents), goal]))
    return Scenario(
        world=world,
        agents=agents,
        goal=goal,
        seed=seed,
        map_kind=map_kind,
        density=world.occupancy_fraction(),
        belief=belief,
        rng=rng,
        pack=pack,
        scenario_id=scenario_id or f"{pack}-{world_kind}-{map_kind}-s{seed}-n{n_agents}",
        start_layout=start_layout,
        goals=goals,
        pending_edit=pending,
        shared_belief=shared_belief,
        fow=fow,
        world_kind="continuous",
    )


def apply_pending_edit(scenario: Scenario) -> bool:
    """Delete the pre-chosen wall / polygon. Used by the slow_edit pack."""
    edit = scenario.pending_edit
    if edit is None:
        return False
    world = scenario.world
    if edit.kind == "cell" and edit.cell is not None and isinstance(world, GridWorld):
        changed = world.delete_wall(edit.cell)
        if changed and not scenario.fow:
            scenario.belief = BeliefMap.from_truth_grid(world)
        scenario.pending_edit = None
        return changed
    if edit.kind == "obstacle" and edit.obstacle_id and isinstance(world, ContinuousWorld):
        changed = world.remove_obstacle(edit.obstacle_id)
        if changed:
            scenario.belief.forget_obstacle(edit.obstacle_id)
        scenario.pending_edit = None
        return changed
    return False


def _make_grid(kind: str, width: int, height: int, density: float, rng, eight: bool) -> GridWorld:
    if kind == "empty":
        return empty_grid(width, height, eight=eight)
    if kind == "bernoulli":
        return random_grid(width, height, p=density, rng=rng, eight=eight)
    if kind == "clustered":
        return clustered_grid(width, height, p=density, rng=rng, eight=eight)
    if kind == "rooms":
        return rooms_grid(width, height, rng=rng, eight=eight)
    if kind == "maze":
        return maze_grid(width, height, rng=rng, eight=eight)
    if kind == "street":
        return street_grid(width, height, rng=rng, eight=eight)
    if kind == "open":
        return open_grid(width, height, p=min(density, 0.12), rng=rng, eight=eight)
    if kind == "game":
        return game_grid(width, height, rng=rng, eight=eight)
    if kind == "corridor":
        return corridor_grid(width, height, eight=eight)
    return random_grid(width, height, p=density, rng=rng, eight=eight)


def _make_poly(kind: str, width: float, height: float, density: float, rng) -> ContinuousWorld:
    if kind == "sparse":
        return sparse_giants(width, height, occupancy=max(0.04, density), rng=rng)
    if kind == "clutter":
        return dense_clutter(width, height, occupancy=max(0.2, density), rng=rng)
    if kind == "utrap":
        return u_trap(width, height)
    if kind == "movers":
        return moving_polygons(width, height, rng=rng)
    return random_polygons(width, height, occupancy=density, rng=rng)
