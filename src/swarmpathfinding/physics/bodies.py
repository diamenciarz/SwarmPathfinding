from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import pymunk

from swarmpathfinding.core.geom import Vec2
from swarmpathfinding.vision.shapes import CircleVision
from swarmpathfinding.worlds.continuous import ContinuousWorld, PolygonObstacle
from swarmpathfinding.worlds.grid import GridWorld


@dataclass(slots=True)
class Agent:
    id: int
    pos: Vec2
    vel: Vec2 = field(default_factory=lambda: Vec2(0.0, 0.0))
    heading: float = 0.0
    radius: float = 0.35
    vision: object = field(default_factory=lambda: CircleVision(6.0))
    body: pymunk.Body | None = None
    shape: pymunk.Circle | None = None
    goal: Vec2 | None = None

    def sync_from_body(self) -> None:
        if self.body is None:
            return
        self.pos = Vec2(self.body.position.x, self.body.position.y)
        self.vel = Vec2(self.body.velocity.x, self.body.velocity.y)
        if self.vel.length_sq() > 1e-6:
            self.heading = self.vel.angle()

    def look_at(self, target: Vec2) -> None:
        if self.pos.dist_sq(target) > 1e-8:
            self.heading = (target - self.pos).angle()


def spawn_agents(
    n: int,
    world: GridWorld | ContinuousWorld,
    goal: Vec2,
    rng: np.random.Generator,
    clump: bool = True,
    vision_kind: str = "circle",
    vision_radius: float = 6.0,
    radius: float = 0.35,
    start_layout: str | None = None,
    extra_goal: Vec2 | None = None,
) -> list[Agent]:
    from swarmpathfinding.generators.spawn import spawn_agents as _spawn

    layout = start_layout or ("clump" if clump else "scattered")
    return _spawn(
        n,
        world,
        goal,
        rng,
        start_layout=layout,
        vision_kind=vision_kind,
        vision_radius=vision_radius,
        radius=radius,
        extra_goal=extra_goal,
    )


def attach_pymunk(agents: list[Agent], space: pymunk.Space) -> None:
    for agent in agents:
        body = pymunk.Body(mass=1.0, moment=pymunk.moment_for_circle(1.0, 0, agent.radius))
        body.position = (agent.pos.x, agent.pos.y)
        shape = pymunk.Circle(body, agent.radius)
        shape.friction = 0.4
        shape.elasticity = 0.05
        space.add(body, shape)
        agent.body = body
        agent.shape = shape


def make_space_for_world(
    world: ContinuousWorld,
    shapes_out: dict[str, tuple[pymunk.Shape, pymunk.Body | None]] | None = None,
) -> tuple[pymunk.Space, dict[str, pymunk.Body]]:
    """Static polys for walls; kinematic bodies for movers so agents cannot tunnel.

    Pass `shapes_out` to receive obstacle id -> (shape, kinematic body or None),
    which `remove_obstacle_body` needs when an obstacle is deleted later.
    """
    space = pymunk.Space()
    space.gravity = (0, 0)
    static = space.static_body
    thick = 1.0
    w, h = world.width, world.height
    borders = [
        pymunk.Segment(static, (0, 0), (w, 0), thick),
        pymunk.Segment(static, (0, 0), (0, h), thick),
        pymunk.Segment(static, (w, 0), (w, h), thick),
        pymunk.Segment(static, (0, h), (w, h), thick),
    ]
    for seg in borders:
        seg.friction = 0.8
        space.add(seg)
    movers: dict[str, pymunk.Body] = {}
    for obs in world.obstacles:
        entry = add_obstacle_body(space, obs)
        if entry is None:
            continue
        if shapes_out is not None:
            shapes_out[obs.id] = entry
        if entry[1] is not None:
            movers[obs.id] = entry[1]
    return space, movers


def add_obstacle_body(
    space: pymunk.Space, obs: PolygonObstacle
) -> tuple[pymunk.Shape, pymunk.Body | None] | None:
    verts = [(v.x, v.y) for v in obs.vertices()]
    if len(verts) < 3:
        return None
    if obs.velocity.length_sq() > 0.0:
        cx, cy = obs.position.x, obs.position.y
        local = [(x - cx, y - cy) for x, y in verts]
        body = pymunk.Body(body_type=pymunk.Body.KINEMATIC)
        body.position = (cx, cy)
        body.velocity = (obs.velocity.x, obs.velocity.y)
        poly = pymunk.Poly(body, local)
        poly.friction = 0.8
        space.add(body, poly)
        return poly, body
    poly = pymunk.Poly(space.static_body, verts)
    poly.friction = 0.8
    space.add(poly)
    return poly, None


def remove_obstacle_body(space: pymunk.Space, entry: tuple[pymunk.Shape, pymunk.Body | None]) -> None:
    shape, body = entry
    if body is not None:
        space.remove(shape, body)
    else:
        space.remove(shape)


def sync_mover_bodies(world: ContinuousWorld, movers: dict[str, pymunk.Body]) -> None:
    for obs in world.obstacles:
        body = movers.get(obs.id)
        if body is None:
            continue
        body.position = (obs.position.x, obs.position.y)
        body.velocity = (obs.velocity.x, obs.velocity.y)
