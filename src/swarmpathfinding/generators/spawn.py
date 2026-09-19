from __future__ import annotations

import numpy as np

from swarmpathfinding.core.geom import Vec2
from swarmpathfinding.physics.bodies import Agent
from swarmpathfinding.vision.shapes import make_vision
from swarmpathfinding.worlds.continuous import ContinuousWorld
from swarmpathfinding.worlds.grid import GridWorld

LAYOUTS = ("clump", "scattered", "flanks", "opposite")


def spawn_agents(
    n: int,
    world: GridWorld | ContinuousWorld,
    goal: Vec2,
    rng: np.random.Generator,
    start_layout: str = "clump",
    vision_kind: str = "circle",
    vision_radius: float = 6.0,
    radius: float = 0.35,
    extra_goal: Vec2 | None = None,
) -> list[Agent]:
    vision = make_vision(vision_kind, radius=vision_radius)
    if start_layout == "scattered":
        origins = [_sample_pos(world, rng, radius) for _ in range(n)]
        goals = [goal] * n
    elif start_layout == "flanks":
        left = _sample_in_band(world, rng, radius, x_frac=(0.08, 0.22))
        right = _sample_in_band(world, rng, radius, x_frac=(0.78, 0.92))
        origins = []
        for i in range(n):
            anchor = left if i < (n + 1) // 2 else right
            origins.append(_jitter_free(world, rng, radius, anchor))
        goals = [goal] * n
    elif start_layout == "opposite":
        left = _sample_in_band(world, rng, radius, x_frac=(0.08, 0.18))
        right = extra_goal or _sample_in_band(world, rng, radius, x_frac=(0.82, 0.92))
        origins = []
        goals = []
        for i in range(n):
            if i < (n + 1) // 2:
                origins.append(_jitter_free(world, rng, radius, left))
                goals.append(right)
            else:
                origins.append(_jitter_free(world, rng, radius, right))
                goals.append(left)
    else:  # clump
        anchor = _sample_pos(world, rng, radius)
        origins = [_jitter_free(world, rng, radius, anchor) for _ in range(n)]
        goals = [goal] * n

    agents: list[Agent] = []
    for i, (pos, g) in enumerate(zip(origins, goals)):
        agent = Agent(id=i, pos=pos, radius=radius, vision=vision, goal=g)
        agent.look_at(g)
        agents.append(agent)
    return agents


def _sample_pos(world: GridWorld | ContinuousWorld, rng: np.random.Generator, radius: float) -> Vec2:
    if isinstance(world, GridWorld):
        return world.cell_center(world.sample_free(rng, margin=2))
    return world.sample_free(rng, radius=radius, margin=2.0)


def _sample_in_band(
    world: GridWorld | ContinuousWorld,
    rng: np.random.Generator,
    radius: float,
    x_frac: tuple[float, float],
) -> Vec2:
    lo, hi = x_frac
    size = world.world_size
    for _ in range(4000):
        x = float(rng.uniform(lo * size.x, hi * size.x))
        y = float(rng.uniform(0.15 * size.y, 0.85 * size.y))
        pos = Vec2(x, y)
        if isinstance(world, GridWorld):
            cell = world.world_to_cell(pos)
            if not world.is_blocked(cell) and world.in_bounds(cell):
                return world.cell_center(cell)
        elif not world.blocked(pos, radius=radius):
            return pos
    return _sample_pos(world, rng, radius)


def _jitter_free(
    world: GridWorld | ContinuousWorld,
    rng: np.random.Generator,
    radius: float,
    anchor: Vec2,
    spread: float = 0.9,
) -> Vec2:
    for _ in range(40):
        pos = anchor + Vec2(float(rng.normal(0, spread)), float(rng.normal(0, spread)))
        if isinstance(world, GridWorld):
            cell = world.world_to_cell(pos)
            if world.in_bounds(cell) and not world.is_blocked(cell):
                return world.cell_center(cell)
        elif not world.blocked(pos, radius=radius):
            return pos
    return _sample_pos(world, rng, radius)
