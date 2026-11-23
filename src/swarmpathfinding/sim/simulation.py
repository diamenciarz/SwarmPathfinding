"""The swarm simulation, without any drawing.

The live lab draws a `Simulation`; the benchmark runner steps one headless.
Both therefore produce the same numbers for the same `SimConfig` and seed.

One step: agents move along their routes (physics in continuous worlds), the
swarm looks around and updates the shared last-seen map, routes that now cross
something newly seen are queued, and queued agents replan within a time budget.
"""

from __future__ import annotations

import time
from dataclasses import asdict, dataclass, field, replace
from typing import Any

import numpy as np
from shapely.geometry import Point, Polygon, box

from swarmpathfinding.core.belief import BeliefMap, ObstacleRecord
from swarmpathfinding.core.geom import Cell, Vec2
from swarmpathfinding.core.plan_grid import plan_grid
from swarmpathfinding.core.registry import create_planner, ensure_builtin_planners
from swarmpathfinding.core.results import Budget, PathResult
from swarmpathfinding.generators.scenarios import Scenario, apply_pending_edit, make_scenario
from swarmpathfinding.physics.bodies import (
    add_obstacle_body,
    attach_pymunk,
    make_space_for_world,
    remove_obstacle_body,
    sync_mover_bodies,
)
from swarmpathfinding.sim.visibility import GridVisionCache, raster_visible, vision_union
from swarmpathfinding.worlds.continuous import ContinuousWorld, PolygonObstacle
from swarmpathfinding.worlds.grid import GridWorld

AGENT_SPEED = 6.0  # world units per second
WAYPOINT_EPS = 0.2
ARRIVE_EPS = 0.5
PLAN_BUDGET_MS = 8.0  # replanning time per step; the rest waits for the next step
RETRY_S = 0.5  # how often an agent without a path tries again
CHANGES = ("none", "slow_edit")


@dataclass(slots=True)
class SimConfig:
    """Everything that defines one run. Same config + same seed = same run."""

    world_kind: str = "grid"
    map_kind: str = "bernoulli"
    starts: str = "clump"
    density: float = 0.15
    n_agents: int = 8
    seed: int = 1
    vision: str = "circle"
    vision_radius: float = 6.0
    fow: bool = True
    los: bool = True
    planner: str = "weighted_a_star"
    w: float = 1.0
    # "slow_edit": the pre-picked wall / polygon is deleted automatically at `edit_at`
    # seconds. With "none" it stays available for a manual edit (E in the live lab).
    changes: str = "none"
    edit_at: float = 3.0
    # Local collision avoidance between agents, on top of the planner's route:
    # "none" (agents pass through each other on grids, push on polygons) or "orca".
    avoidance: str = "none"

    def with_(self, **kwargs: Any) -> SimConfig:
        return replace(self, **kwargs)

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class SimStats:
    """Running totals for one run (what a benchmark row reports)."""

    plans: int = 0
    plan_ms: float = 0.0
    expansions: int = 0
    plan_failures: int = 0
    edit_time: float | None = None
    plans_after_edit: int = 0
    plan_ms_after_edit: float = 0.0
    travelled: dict[int, float] = field(default_factory=dict)
    arrived_at: dict[int, float] = field(default_factory=dict)
    # agent-pair seconds spent overlapping (closer than two radii), arrived agents excluded
    overlap_s: float = 0.0


class Simulation:
    def __init__(self, config: SimConfig | None = None) -> None:
        ensure_builtin_planners()
        self.config = config or SimConfig()
        self.scenario: Scenario | None = None
        self.planner: Any = None
        self.now = 0.0
        self.space = None
        self.movers: dict = {}
        self.obstacle_bodies: dict = {}
        self.vision_cache = GridVisionCache()
        self.visible: np.ndarray | None = None
        self.union = None
        self.seen_ids: set[str] = set()
        self.truth_gen = -1
        # per-agent planning state
        self.results: dict[int, PathResult] = {}
        self.waypoints: dict[int, list[Vec2]] = {}
        self.plan_gen: dict[int, int] = {}
        self.retry_at: dict[int, float] = {}
        self.dirty: list[int] = []
        self.plan_ms = 0.0
        self.record_focus_trace = False
        self.orca_debug = None  # OrcaDebug for the focus agent, when avoidance runs
        self.stats = SimStats()
        self.rebuild()

    # ------------------------------------------------------------------ setup

    def rebuild(self, config: SimConfig | None = None) -> None:
        if config is not None:
            self.config = config
        c = self.config
        self.scenario = make_scenario(
            world_kind=c.world_kind,
            map_kind=c.map_kind,
            density=float(c.density),
            n_agents=int(c.n_agents),
            seed=int(c.seed),
            start_layout=c.starts,
            vision_kind=c.vision,
            vision_radius=float(c.vision_radius),
            fow=bool(c.fow),
            slow_edit=True,
        )
        self.space = None
        self.movers = {}
        self.obstacle_bodies = {}
        if isinstance(self.scenario.world, ContinuousWorld):
            self.space, self.movers = make_space_for_world(self.scenario.world, self.obstacle_bodies)
            attach_pymunk(self.scenario.agents, self.space)
        self.vision_cache.clear()
        self.visible = None
        self.union = None
        self.seen_ids = set()
        self.truth_gen = -1
        self.now = 0.0
        self.results.clear()
        self.waypoints.clear()
        self.plan_gen.clear()
        self.retry_at.clear()
        self.dirty = []
        self.stats = SimStats(travelled={a.id: 0.0 for a in self.scenario.agents})
        self.planner = create_planner(c.planner)
        self.reveal()
        self.replan_all(sync=True)

    def set_planner(self, name: str, sync: bool = True) -> None:
        self.config.planner = name
        self.planner = create_planner(name)
        self.replan_all(sync=sync)

    def set_weight(self, w: float, sync: bool = True) -> None:
        self.config.w = float(w)
        self.replan_all(sync=sync)

    def set_view_option(self, **kwargs: Any) -> None:
        """Options that change what the swarm sees but not the scenario (e.g. los)."""
        for key, value in kwargs.items():
            setattr(self.config, key, value)
        self.vision_cache.clear()

    # ------------------------------------------------------------------ queries

    def goal_for(self, agent) -> Vec2:
        assert self.scenario is not None
        return agent.goal or self.scenario.goal

    def focus_id(self) -> int:
        assert self.scenario is not None
        return self.scenario.agents[0].id if self.scenario.agents else -1

    def arrived(self, agent) -> bool:
        res = self.results.get(agent.id)
        return res is not None and res.success and not self.waypoints.get(agent.id)

    @property
    def done(self) -> bool:
        """Every agent has finished its route."""
        assert self.scenario is not None
        return all(self.arrived(a) for a in self.scenario.agents)

    def stale_count(self) -> int:
        sc = self.scenario
        assert sc is not None
        world = sc.world
        if isinstance(world, GridWorld):
            return int(np.count_nonzero(sc.belief.stale_cells(world.blocked)))
        live = {obs.id for obs in world.obstacles}
        return sum(1 for oid in sc.belief.obstacles if oid not in live)

    def fog_memory(self) -> tuple[list[ObstacleRecord], set[str]]:
        """Split the remembered obstacles for drawing under fog.

        Returns (records drawn as remembered walls, ids drawn as moving ghosts).
        Every obstacle in view is included in the first list too, so the part of
        a half-visible obstacle that sticks out of view stays drawn (dimmed by the
        fog) instead of being cut off. Movers out of view are ghosts instead.
        """
        sc = self.scenario
        assert sc is not None
        remembered: list[ObstacleRecord] = []
        ghost_ids: set[str] = set()
        for rec in sc.belief.obstacles.values():
            if rec.id not in self.seen_ids and rec.velocity.length_sq() > 0.0:
                ghost_ids.add(rec.id)
            else:
                remembered.append(rec)
        return remembered, ghost_ids

    # ------------------------------------------------------------------ fog / belief

    def reveal(self) -> None:
        """Update the shared last-seen map from what the swarm can see now."""
        sc = self.scenario
        assert sc is not None
        c = self.config
        world = sc.world
        agents = sc.agents
        self.union = vision_union(agents) if c.vision != "none" else None
        if isinstance(world, GridWorld):
            if not c.fow:
                if world.generation != self.truth_gen:
                    sc.belief = BeliefMap.from_truth_grid(world)
                    self.truth_gen = world.generation
                self.visible = None
                return
            self.visible = self.vision_cache.visible_mask(world, agents, require_los=bool(c.los))
            sc.belief.observe_mask(self.visible, world.blocked, self.now)
            return
        assert isinstance(world, ContinuousWorld)
        belief = sc.belief
        if not c.fow:
            self.visible = None
            live = set()
            for obs in world.obstacles:
                live.add(obs.id)
                belief.observe_obstacle(_record(obs, self.now))
            for oid in [oid for oid in belief.obstacles if oid not in live]:
                belief.forget_obstacle(oid)
            self.seen_ids = live
            return
        union = self.union
        self.visible = raster_visible(union, belief.width, belief.height)
        if self.visible.any():
            belief.occupancy[self.visible & (belief.occupancy < 0)] = 0
            belief.seen_time[self.visible] = self.now
        seen: set[str] = set()
        if union is not None and not union.is_empty:
            for obs in world.obstacles:
                if union.intersects(obs.polygon):
                    seen.add(obs.id)
                    belief.observe_obstacle(_record(obs, self.now))
            # A remembered obstacle whose ghost sits in plain view but is not
            # actually there was deleted or moved away: forget it.
            for ghost in belief.ghost_obstacles(self.now):
                if ghost.id in seen or len(ghost.vertices) < 3:
                    continue
                if union.contains(Polygon([(v.x, v.y) for v in ghost.vertices]).centroid):
                    belief.forget_obstacle(ghost.id)
        self.seen_ids = seen

    # ------------------------------------------------------------------ planning

    def replan_all(self, sync: bool = False) -> None:
        assert self.scenario is not None
        self.dirty = [a.id for a in self.scenario.agents]
        self.retry_at.clear()
        self._plan_dirty(budget_ms=None if sync else PLAN_BUDGET_MS)

    def _plan_dirty(self, budget_ms: float | None) -> None:
        sc = self.scenario
        assert sc is not None
        by_id = {a.id: a for a in sc.agents}
        focus = self.focus_id()
        t0 = time.perf_counter()
        cache: dict[tuple, PathResult] = {}
        incremental = bool(getattr(self.planner, "incremental", False))
        done = 0
        while self.dirty:
            if budget_ms is not None and done and (time.perf_counter() - t0) * 1000.0 > budget_ms:
                break
            aid = self.dirty.pop(0)
            agent = by_id.get(aid)
            if agent is None:
                continue
            goal = self.goal_for(agent)
            record = self.record_focus_trace and aid == focus
            key = (round(agent.pos.x * 2), round(agent.pos.y * 2), goal.x, goal.y)
            # incremental planners keep per-agent state, so they never share a result
            result = None if record or incremental else cache.get(key)
            if result is None:
                t1 = time.perf_counter()
                budget = Budget(weight=self.config.w, record_trace=record, agent_id=aid)
                result = self.planner.plan(sc.belief, [agent.pos], goal, budget)
                if not result.runtime_ms:
                    result.runtime_ms = (time.perf_counter() - t1) * 1000.0
                cache[key] = result
                self._count_plan(result)
            self.results[aid] = result
            self.plan_gen[aid] = sc.belief.generation
            if result.success and len(result.path) >= 2:
                self.waypoints[aid] = list(result.path[1:])
            else:
                self.waypoints[aid] = []
                if not result.success:
                    self.retry_at[aid] = self.now + RETRY_S
            done += 1
        if done:
            res = self.results.get(focus)
            self.plan_ms = res.runtime_ms if res else 0.0

    def _count_plan(self, result: PathResult) -> None:
        s = self.stats
        s.plans += 1
        s.plan_ms += result.runtime_ms
        s.expansions += result.expansions
        if not result.success:
            s.plan_failures += 1
        if s.edit_time is not None:
            s.plans_after_edit += 1
            s.plan_ms_after_edit += result.runtime_ms

    def _check_paths(self) -> None:
        """Queue agents whose route now crosses something the swarm has learned about."""
        sc = self.scenario
        assert sc is not None
        gen = sc.belief.generation
        grid = None
        for agent in sc.agents:
            aid = agent.id
            if aid in self.dirty:
                continue
            wps = self.waypoints.get(aid)
            if not wps:
                if self.retry_at.get(aid, 0.0) <= self.now and self.plan_gen.get(aid) != gen:
                    if agent.pos.dist(self.goal_for(agent)) > ARRIVE_EPS:
                        self.dirty.append(aid)
                continue
            if self.plan_gen.get(aid) == gen:
                continue
            if grid is None:
                grid = plan_grid(sc.belief, agent.radius)
            pts = [agent.pos, *wps]
            margin = grid.cell_size * 1.5 if sc.belief.kind == "continuous" else 0.0
            last = len(pts) - 2
            if any(
                grid.segment_blocked(a, b, margin if i == 0 else 0.0, margin if i == last else 0.0)
                for i, (a, b) in enumerate(zip(pts, pts[1:]))
            ):
                self.dirty.append(aid)
            else:
                self.plan_gen[aid] = gen

    # ------------------------------------------------------------------ stepping

    def step(self, dt: float, sync_plans: bool = False) -> None:
        """Advance by `dt` seconds. `sync_plans` replans every queued agent now
        (headless runs) instead of spreading the work over frames."""
        sc = self.scenario
        assert sc is not None
        self.now += dt
        world = sc.world
        before = {a.id: a.pos for a in sc.agents}
        self._move_agents(dt)
        if isinstance(world, ContinuousWorld):
            world.step_movers(dt)
            if self.movers:
                sync_mover_bodies(world, self.movers)
            if self.space is not None:
                self.space.step(dt)
                for agent in sc.agents:
                    agent.sync_from_body()
                    wps = self.waypoints.get(agent.id)
                    if wps:
                        agent.look_at(wps[0])
        for agent in sc.agents:
            self.stats.travelled[agent.id] = self.stats.travelled.get(agent.id, 0.0) + agent.pos.dist(before[agent.id])
        self._count_overlaps(dt)
        if (
            self.config.changes == "slow_edit"
            and sc.pending_edit is not None
            and self.now >= self.config.edit_at
        ):
            self.apply_pending_edit(sync=sync_plans)
        self.reveal()
        self._check_paths()
        self._plan_dirty(None if sync_plans else PLAN_BUDGET_MS)
        for agent in sc.agents:
            if agent.id not in self.stats.arrived_at and self.arrived(agent):
                self.stats.arrived_at[agent.id] = self.now

    def run(self, max_time: float = 60.0, dt: float = 1.0 / 30.0) -> bool:
        """Step headless until every agent arrives or `max_time` passes. Returns success."""
        while self.now < max_time:
            if self.done and (self.config.changes != "slow_edit" or self.scenario.pending_edit is None):
                return True
            self.step(dt, sync_plans=True)
        return self.done

    @property
    def avoidance(self) -> str:
        if getattr(self.planner, "forces_avoidance", False):
            return "orca"
        return self.config.avoidance

    def _move_agents(self, dt: float) -> None:
        sc = self.scenario
        assert sc is not None
        world = sc.world
        agents = sc.agents
        pref = np.zeros((len(agents), 2))
        has_target = [False] * len(agents)
        avoid = self.avoidance == "orca" and len(agents) > 1
        parked = [a for a in agents if self.arrived(a)] if avoid else []
        for k, agent in enumerate(agents):
            wps = self.waypoints.get(agent.id) or []
            while len(wps) > 1 and agent.pos.dist(wps[0]) < WAYPOINT_EPS:
                wps.pop(0)
            if wps and agent.pos.dist(wps[0]) < WAYPOINT_EPS * 0.5:
                wps.pop(0)
            if avoid and wps:
                # Pushed off the path by a neighbour: skip waypoints already passed
                # instead of walking back to them (that jams a crowd).
                while len(wps) > 1 and self._passed(agent.pos, wps[0], wps[1]):
                    wps.pop(0)
                # A crowd cannot all stand on one goal point: an agent on its last leg
                # that touches an agent already parked at the same goal has arrived
                # (the usual RTS "bump an arrived unit" rule).
                # Arrival also counts once the body covers the goal point, so the
                # first agent there is not boxed out by neighbours 0.1 short of it.
                if len(wps) == 1 and (agent.pos.dist(wps[0]) < agent.radius or self._touches_parked(agent, parked)):
                    wps.pop(0)
            if not wps:
                continue
            target = wps[0]
            delta = target - agent.pos
            dist = delta.length()
            agent.look_at(target)
            v = delta.normalized() * min(AGENT_SPEED, dist / max(dt, 1e-6))
            pref[k] = (v.x, v.y)
            has_target[k] = True
        vel = pref
        self.orca_debug = None
        if avoid:
            from swarmpathfinding.sim.orca import orca_step

            pos = np.array([(a.pos.x, a.pos.y) for a in agents])
            cur = np.array([(a.vel.x, a.vel.y) for a in agents])
            active = np.array([not self.arrived(a) for a in agents])
            vel, self.orca_debug = orca_step(pos, cur, pref, active, agents[0].radius, AGENT_SPEED, dt, debug_agent=0)
        for k, agent in enumerate(agents):
            vx, vy = float(vel[k, 0]), float(vel[k, 1])
            if agent.body is not None:
                agent.body.velocity = (vx, vy)
                continue
            assert isinstance(world, GridWorld)
            if not has_target[k] and vx == 0.0 and vy == 0.0:
                agent.vel = Vec2(0.0, 0.0)
                continue
            step = Vec2(vx * dt, vy * dt)
            before = agent.pos
            moved = self._grid_move(world, agent.pos, step)
            if has_target[k] and step.length_sq() > 1e-12 and moved == before and agent.id not in self.dirty:
                # Walked into something: replan from here.
                self.dirty.append(agent.id)
            agent.pos = moved
            agent.vel = (moved - before) * (1.0 / max(dt, 1e-6))

    @staticmethod
    def _passed(pos: Vec2, w0: Vec2, w1: Vec2) -> bool:
        """Is `pos` already beyond waypoint w0 on the leg to w1 (and still near
        it, so a waypoint across a wall is never skipped)?"""
        leg = w1 - w0
        off = pos - w0
        return off.length_sq() < 1.0 and off.x * leg.x + off.y * leg.y > 0.0

    def _touches_parked(self, agent, parked: list) -> bool:
        goal = self.goal_for(agent)
        reach = (2 * agent.radius + 0.1) ** 2
        return any(
            other.pos.dist_sq(agent.pos) <= reach and self.goal_for(other).dist_sq(goal) < 1e-6
            for other in parked
        )

    def _count_overlaps(self, dt: float) -> None:
        sc = self.scenario
        assert sc is not None
        live = [a for a in sc.agents if not self.arrived(a)]
        if len(live) < 2:
            return
        pos = np.array([(a.pos.x, a.pos.y) for a in live])
        d2 = ((pos[:, None, :] - pos[None, :, :]) ** 2).sum(-1)
        lim = (2 * live[0].radius * 0.95) ** 2
        pairs = int(np.count_nonzero(np.triu(d2 < lim, 1)))
        self.stats.overlap_s += pairs * dt

    def _grid_move(self, world: GridWorld, pos: Vec2, step: Vec2) -> Vec2:
        """Move on the true grid; a wall stops the agent (and is felt, i.e. remembered)."""
        sc = self.scenario
        assert sc is not None
        for candidate in (pos + step, pos + Vec2(step.x, 0.0), pos + Vec2(0.0, step.y)):
            cell = world.world_to_cell(candidate)
            if not world.is_blocked(cell):
                return candidate
            sc.belief.observe_cells([cell], world, self.now)
        return pos

    # ------------------------------------------------------------------ edits

    def after_edit(self, sync: bool = True) -> None:
        if self.stats.edit_time is None:
            self.stats.edit_time = self.now
        self.sync_physics()
        self.vision_cache.clear()
        self.reveal()
        self.replan_all(sync=sync)

    def apply_pending_edit(self, sync: bool = True) -> bool:
        assert self.scenario is not None
        if apply_pending_edit(self.scenario):
            self.after_edit(sync=sync)
            return True
        return False

    def set_cell(self, cell: Cell, blocked: bool, sync: bool = True) -> bool:
        world = self.scenario.world if self.scenario else None
        if not isinstance(world, GridWorld):
            return False
        changed = world.punch_wall(cell) if blocked else world.delete_wall(cell)
        if changed:
            self.after_edit(sync=sync)
        return changed

    def add_box(self, center: Vec2, half: float = 1.2, sync: bool = True) -> str | None:
        world = self.scenario.world if self.scenario else None
        if not isinstance(world, ContinuousWorld):
            return None
        poly = box(center.x - half, center.y - half, center.x + half, center.y + half)
        oid = f"user{world.generation}"
        world.add_obstacle(PolygonObstacle(id=oid, polygon=poly))
        self.after_edit(sync=sync)
        return oid

    def remove_obstacle_at(self, point: Vec2, sync: bool = True) -> bool:
        world = self.scenario.world if self.scenario else None
        if not isinstance(world, ContinuousWorld):
            return False
        p = Point(point.x, point.y)
        for obs in reversed(world.obstacles):
            if obs.polygon.contains(p):
                world.remove_obstacle(obs.id)
                self.after_edit(sync=sync)
                return True
        return False

    def sync_physics(self) -> None:
        """Keep pymunk bodies in step with the obstacle list after adds / deletes."""
        sc = self.scenario
        if self.space is None or sc is None or not isinstance(sc.world, ContinuousWorld):
            return
        live = {obs.id: obs for obs in sc.world.obstacles}
        for oid in [oid for oid in self.obstacle_bodies if oid not in live]:
            remove_obstacle_body(self.space, self.obstacle_bodies.pop(oid))
            self.movers.pop(oid, None)
        for oid, obs in live.items():
            if oid in self.obstacle_bodies:
                continue
            entry = add_obstacle_body(self.space, obs)
            if entry is not None:
                self.obstacle_bodies[oid] = entry
                if entry[1] is not None:
                    self.movers[oid] = entry[1]

    def set_goal(self, point: Vec2, sync: bool = True) -> None:
        sc = self.scenario
        assert sc is not None
        sc.goal = point
        sc.goals = (point,)
        for agent in sc.agents:
            agent.goal = point
            agent.look_at(point)
        self.stats.arrived_at.clear()
        self.replan_all(sync=sync)


def _record(obs: PolygonObstacle, now: float) -> ObstacleRecord:
    return ObstacleRecord(
        id=obs.id,
        vertices=obs.vertices(),
        position=obs.position,
        velocity=obs.velocity,
        last_seen_time=now,
    )
