from __future__ import annotations

import time
from typing import Any

import numpy as np
import pygame
from shapely.geometry import Point, Polygon, box

from swarmpathfinding.core.belief import BeliefMap, ObstacleRecord
from swarmpathfinding.core.geom import Vec2
from swarmpathfinding.core.plan_grid import plan_grid
from swarmpathfinding.core.registry import create_planner, ensure_builtin_planners, list_planners
from swarmpathfinding.core.results import Budget, PathResult
from swarmpathfinding.generators.scenarios import GRID_MAPS as GMAPS
from swarmpathfinding.generators.scenarios import POLY_MAPS as PMAPS
from swarmpathfinding.generators.scenarios import START_LAYOUTS, Scenario, apply_pending_edit, make_scenario
from swarmpathfinding.physics.bodies import (
    add_obstacle_body,
    attach_pymunk,
    make_space_for_world,
    remove_obstacle_body,
    sync_mover_bodies,
)
from swarmpathfinding.viz.camera import Camera
from swarmpathfinding.viz.colors import BANNER_BG, PANEL_BG, PANEL_FG
from swarmpathfinding.viz.draw import (
    draw_agents,
    draw_belief_grid,
    draw_floor,
    draw_fog_raster,
    draw_ghosts,
    draw_goal,
    draw_grid,
    draw_paths,
    draw_pending_cell,
    draw_pending_polygon,
    draw_polygons,
    draw_remembered,
    draw_search,
    draw_visible_truth,
    draw_vision_union,
)
from swarmpathfinding.viz.fog import GridVisionCache, raster_visible, vision_union
from swarmpathfinding.viz.panel import Check, Cycle, SidePanel, Slider
from swarmpathfinding.viz.settings import LabSettings, apply_settings, parse_lab_args
from swarmpathfinding.worlds.continuous import ContinuousWorld, PolygonObstacle
from swarmpathfinding.worlds.grid import GridWorld

PANEL_W = 320
AGENT_SPEED = 6.0  # world units per second
WAYPOINT_EPS = 0.2
ARRIVE_EPS = 0.5
PLAN_BUDGET_MS = 8.0  # replanning time per frame; the rest waits for the next frame
RETRY_S = 0.5  # how often an agent without a path tries again
HELP = [
    ("LMB", "add wall"),
    ("RMB", "delete wall"),
    ("G+LMB", "set goal"),
    ("E", "apply edit"),
    ("Wheel", "zoom"),
    ("MMB", "drag pan"),
    ("F", "fit view"),
    ("Space", "pause"),
    (".", "step"),
    ("R", "restart"),
    ("Esc", "quit"),
]
# Panel keys that only need a replan, not a new scenario.
REPLAN_KEYS = {"planner", "w"}
VIEW_KEYS = {"show_search", "los"}


class LiveApp:
    def __init__(self, settings: LabSettings | None = None) -> None:
        pygame.init()
        pygame.display.set_caption("SwarmPathfinding")
        self.screen = pygame.display.set_mode((1280, 800), pygame.RESIZABLE)
        self.clock = pygame.time.Clock()
        self.font = _font(18)
        self.small = _font(14)
        ensure_builtin_planners()
        self.camera = Camera()
        self.panel = self._make_panel()
        self.scenario: Scenario | None = None
        self.planner: Any = None
        self.paused = True
        self.place_goal = False
        self.panning = False
        self.now = 0.0
        self.frame = 0
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
        self._layout()
        if settings is not None:
            apply_settings(self.panel, settings)
        self._sync_panel_state()
        self.rebuild()

    # ------------------------------------------------------------------ panel

    def _make_panel(self) -> SidePanel:
        panel = SidePanel(pygame.Rect(0, 0, PANEL_W, 800), self.font, self.small)
        planners = list_planners() or ["none"]
        default_planner = "weighted_a_star" if "weighted_a_star" in planners else planners[0]
        panel.sections = ["Scenario", "Swarm", "Planner"]
        panel.widgets = [
            Cycle("world_kind", "World", ["grid", "continuous"], 0, section="Scenario"),
            Cycle("map_kind", "Map", list(GMAPS), 1, section="Scenario"),
            Cycle("starts", "Starts", list(START_LAYOUTS), 0, section="Scenario"),
            Slider("density", "Density", 0.02, 0.50, 0.15, section="Scenario"),
            Slider("n_agents", "Agents", 1, 100, 8, integer=True, section="Scenario"),
            Slider("seed", "Seed", 1, 50, 1, integer=True, section="Scenario"),
            Cycle("vision", "Vision", ["circle", "cone", "none"], 0, section="Swarm"),
            Slider("vision_radius", "Vision radius", 2.0, 16.0, 6.0, section="Swarm"),
            Check("fow", "Fog of war", True, section="Swarm"),
            Check("los", "Vision needs LOS", True, section="Swarm", hint="grid only"),
            Cycle("planner", "Planner", planners, planners.index(default_planner), section="Planner"),
            Slider("w", "Weight w", 1.0, 3.0, 1.0, section="Planner"),
            Check("show_search", "Show search", False, section="Planner"),
        ]
        panel.button_label = "Restart scenario  (R)"
        panel.set_help(HELP)
        panel.set_on_change(self._on_panel_change)
        return panel

    def _on_panel_change(self, key: str, final: bool) -> None:
        if key == "world_kind":
            world_kind = self.panel.get("world_kind").value
            maps = self.panel.get("map_kind")
            wanted = list(GMAPS) if world_kind == "grid" else list(PMAPS)
            if maps.options != wanted:
                maps.options = wanted
                maps.index = 0
        self._sync_panel_state()
        if not final:
            return
        if key in VIEW_KEYS:
            self.vision_cache.clear()
            return
        if key in REPLAN_KEYS:
            if key == "planner":
                self.planner = create_planner(self.panel.get("planner").value)
            self._replan_all(sync=self.paused)
            return
        self.rebuild()

    def _sync_panel_state(self) -> None:
        """Show / enable controls that actually do something for the current choice."""
        vals = self.panel.values()
        planner_cls_uses_w = bool(getattr(_planner_class(vals["planner"]), "uses_weight", False))
        self.panel.get("w").visible = planner_cls_uses_w
        grid = vals["world_kind"] == "grid"
        self.panel.get("los").enabled = grid
        has_search = vals["planner"] not in ("none", "straight_line")
        self.panel.get("show_search").visible = has_search
        self.panel.get("vision_radius").enabled = vals["vision"] != "none"
        self.panel.get("density").visible = vals["map_kind"] in DENSITY_MAPS

    def _layout(self) -> None:
        w, h = self.screen.get_size()
        self.panel.rect = pygame.Rect(w - PANEL_W, 0, PANEL_W, h)
        self.camera.set_viewport(pygame.Rect(0, 0, max(1, w - PANEL_W), h))

    # ------------------------------------------------------------------ scenario

    def rebuild(self) -> None:
        vals = self.panel.values()
        self.scenario = make_scenario(
            world_kind=vals["world_kind"],
            map_kind=vals["map_kind"],
            density=float(vals["density"]),
            n_agents=int(vals["n_agents"]),
            seed=int(vals["seed"]),
            start_layout=vals["starts"],
            vision_kind=vals["vision"],
            vision_radius=float(vals["vision_radius"]),
            fow=bool(vals["fow"]),
            slow_edit=True,
        )
        size = self.scenario.world.world_size
        self.camera.fit(size.x, size.y)
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
        self.planner = create_planner(vals["planner"])
        self._reveal()
        self._replan_all(sync=True)

    # ------------------------------------------------------------------ fog / belief

    def _reveal(self) -> None:
        """Update the shared last-seen map from what the swarm can see now."""
        sc = self.scenario
        assert sc is not None
        vals = self.panel.values()
        world = sc.world
        agents = sc.agents
        self.union = vision_union(agents) if vals["vision"] != "none" else None
        if isinstance(world, GridWorld):
            if not vals["fow"]:
                if world.generation != self.truth_gen:
                    sc.belief = BeliefMap.from_truth_grid(world)
                    self.truth_gen = world.generation
                self.visible = None
                return
            self.visible = self.vision_cache.visible_mask(world, agents, require_los=bool(vals["los"]))
            sc.belief.observe_mask(self.visible, world.blocked, self.now)
            return
        assert isinstance(world, ContinuousWorld)
        belief = sc.belief
        if not vals["fow"]:
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

    def _goal_for(self, agent) -> Vec2:
        assert self.scenario is not None
        return agent.goal or self.scenario.goal

    def _replan_all(self, sync: bool = False) -> None:
        assert self.scenario is not None
        self.dirty = [a.id for a in self.scenario.agents]
        self.retry_at.clear()
        self._plan_dirty(budget_ms=None if sync else PLAN_BUDGET_MS)

    def _plan_dirty(self, budget_ms: float | None) -> None:
        sc = self.scenario
        assert sc is not None
        by_id = {a.id: a for a in sc.agents}
        weight = float(self.panel.get("w").value)
        t0 = time.perf_counter()
        cache: dict[tuple, PathResult] = {}
        done = 0
        while self.dirty:
            if budget_ms is not None and done and (time.perf_counter() - t0) * 1000.0 > budget_ms:
                break
            aid = self.dirty.pop(0)
            agent = by_id.get(aid)
            if agent is None:
                continue
            goal = self._goal_for(agent)
            key = (round(agent.pos.x * 2), round(agent.pos.y * 2), goal.x, goal.y)
            result = cache.get(key)
            if result is None:
                t1 = time.perf_counter()
                result = self.planner.plan(sc.belief, [agent.pos], goal, Budget(weight=weight))
                if not result.runtime_ms:
                    result.runtime_ms = (time.perf_counter() - t1) * 1000.0
                cache[key] = result
            self.results[aid] = result
            self.plan_gen[aid] = sc.belief.generation
            if result.success and len(result.path) >= 2:
                self.waypoints[aid] = list(result.path[1:])
            else:
                self.waypoints[aid] = []
                self.retry_at[aid] = self.now + RETRY_S
            done += 1
        if done:
            focus = self.results.get(self._focus_id())
            self.plan_ms = focus.runtime_ms if focus else 0.0

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
                    if agent.pos.dist(self._goal_for(agent)) > ARRIVE_EPS:
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

    def _focus_id(self) -> int:
        assert self.scenario is not None
        return self.scenario.agents[0].id if self.scenario.agents else -1

    # ------------------------------------------------------------------ simulation

    def tick(self, dt: float) -> None:
        if self.paused or self.scenario is None:
            return
        self.step(dt)

    def step(self, dt: float) -> None:
        sc = self.scenario
        assert sc is not None
        self.now += dt
        world = sc.world
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
        self._reveal()
        self._check_paths()
        self._plan_dirty(PLAN_BUDGET_MS)

    def _move_agents(self, dt: float) -> None:
        sc = self.scenario
        assert sc is not None
        world = sc.world
        for agent in sc.agents:
            wps = self.waypoints.get(agent.id) or []
            while len(wps) > 1 and agent.pos.dist(wps[0]) < WAYPOINT_EPS:
                wps.pop(0)
            if wps and agent.pos.dist(wps[0]) < WAYPOINT_EPS * 0.5:
                wps.pop(0)
            if not wps:
                if agent.body is not None:
                    agent.body.velocity = (0, 0)
                continue
            target = wps[0]
            delta = target - agent.pos
            dist = delta.length()
            agent.look_at(target)
            if agent.body is not None:
                speed = min(AGENT_SPEED, dist / max(dt, 1e-6))
                v = delta.normalized() * speed
                agent.body.velocity = (v.x, v.y)
                continue
            assert isinstance(world, GridWorld)
            step = delta.normalized() * min(AGENT_SPEED * dt, dist)
            moved = self._grid_move(world, agent.pos, step)
            if moved == agent.pos and agent.id not in self.dirty:
                # Walked into something: replan from here.
                self.dirty.append(agent.id)
            agent.pos = moved

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

    def _after_edit(self) -> None:
        self._sync_physics()
        self.vision_cache.clear()
        self._reveal()
        self._replan_all(sync=self.paused)

    def _sync_physics(self) -> None:
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

    def _set_goal(self, point: Vec2) -> None:
        sc = self.scenario
        assert sc is not None
        sc.goal = point
        sc.goals = (point,)
        for agent in sc.agents:
            agent.goal = point
            agent.look_at(point)
        self._replan_all(sync=self.paused)

    def _click_world(self, button: int, pos: tuple[int, int]) -> None:
        sc = self.scenario
        if sc is None:
            return
        world_pt = self.camera.screen_to_world(*pos)
        size = sc.world.world_size
        if not (0 <= world_pt.x < size.x and 0 <= world_pt.y < size.y):
            return
        if self.place_goal:
            if button == 1:
                self._set_goal(world_pt)
            return
        world = sc.world
        if isinstance(world, GridWorld):
            cell = world.world_to_cell(world_pt)
            changed = world.punch_wall(cell) if button == 1 else world.delete_wall(cell)
            if changed:
                self._after_edit()
            return
        assert isinstance(world, ContinuousWorld)
        if button == 3:
            p = Point(world_pt.x, world_pt.y)
            for obs in reversed(world.obstacles):
                if obs.polygon.contains(p):
                    world.remove_obstacle(obs.id)
                    self._after_edit()
                    return
            return
        s = 1.2
        poly = box(world_pt.x - s, world_pt.y - s, world_pt.x + s, world_pt.y + s)
        world.add_obstacle(PolygonObstacle(id=f"user{world.generation}", polygon=poly))
        self._after_edit()

    # ------------------------------------------------------------------ events

    def handle(self, event: pygame.event.Event) -> bool:
        if event.type == pygame.QUIT:
            return False
        if event.type == pygame.VIDEORESIZE:
            self.screen = pygame.display.set_mode((event.w, event.h), pygame.RESIZABLE)
            self._layout()
        action = self.panel.handle(event)
        if action == "button":
            self.rebuild()
            return True
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                return False
            if event.key == pygame.K_SPACE:
                self.paused = not self.paused
            if event.key == pygame.K_PERIOD and self.scenario is not None:
                self.paused = True
                self.step(1.0 / 30.0)
            if event.key == pygame.K_r:
                self.rebuild()
            if event.key == pygame.K_f and self.scenario is not None:
                size = self.scenario.world.world_size
                self.camera.fit(size.x, size.y)
            if event.key == pygame.K_e and self.scenario is not None:
                if apply_pending_edit(self.scenario):
                    self._after_edit()
            if event.key == pygame.K_g:
                self.place_goal = True
        if event.type == pygame.KEYUP and event.key == pygame.K_g:
            self.place_goal = False
        if event.type == pygame.MOUSEWHEEL and self.camera.contains_screen(*pygame.mouse.get_pos()):
            factor = 1.1 if event.y > 0 else 1 / 1.1
            mx, my = pygame.mouse.get_pos()
            self.camera.zoom_at(mx, my, factor)
        if event.type == pygame.MOUSEBUTTONDOWN and not self.panel.dragging:
            if event.button == 2 and self.camera.contains_screen(*event.pos):
                self.panning = True
            elif event.button in (1, 3) and self.camera.contains_screen(*event.pos):
                self._click_world(event.button, event.pos)
        if event.type == pygame.MOUSEBUTTONUP and event.button == 2:
            self.panning = False
        if event.type == pygame.MOUSEMOTION and self.panning:
            self.camera.pan(event.rel[0], event.rel[1])
        return True

    # ------------------------------------------------------------------ drawing

    def draw(self) -> None:
        self.screen.fill(PANEL_BG)
        world_surface = self.screen.subsurface(self.camera.viewport)
        world_surface.fill((12, 14, 18))
        sc = self.scenario
        if sc is None:
            self.panel.draw(self.screen)
            pygame.display.flip()
            return
        world = sc.world
        vals = self.panel.values()
        fow = bool(vals["fow"])
        focus = self.results.get(self._focus_id())
        if isinstance(world, GridWorld):
            if fow:
                draw_belief_grid(world_surface, self.camera, world, sc.belief, self.visible)
            else:
                draw_grid(world_surface, self.camera, world)
        elif fow:
            # Under fog: fog, then remembered obstacle shapes (dimmed), then what is in view now.
            remembered, ghost_ids = self._fog_memory()
            draw_floor(world_surface, self.camera, world)
            draw_fog_raster(world_surface, self.camera, sc.belief, self.visible)
            draw_remembered(world_surface, self.camera, remembered)
            draw_visible_truth(world_surface, self.camera, world, self.union)
            draw_ghosts(world_surface, self.camera, [g for g in sc.belief.ghost_obstacles(self.now) if g.id in ghost_ids])
        else:
            draw_polygons(world_surface, self.camera, world)
        if vals["show_search"] and focus is not None and "closed" in focus.extra:
            draw_search(world_surface, self.camera, focus.extra["closed"], focus.extra["cell_size"])
        self._draw_pending(world_surface)
        if self.union is not None:
            draw_vision_union(world_surface, self.camera, self.union)
        fid = self._focus_id()
        paths = [[a.pos, *self.waypoints.get(a.id, [])] for a in sc.agents if a.id != fid]
        focus_agent = sc.agents[0] if sc.agents else None
        focus_path = [focus_agent.pos, *self.waypoints.get(fid, [])] if focus_agent else None
        draw_paths(world_surface, self.camera, paths, focus_path)
        goals = {(g.x, g.y): g for g in (*(self._goal_for(a) for a in sc.agents), sc.goal)}
        for g in goals.values():
            draw_goal(world_surface, self.camera, g)
        draw_agents(world_surface, self.camera, sc.agents, focus=fid)
        if self.place_goal:
            self._banner(world_surface, "Goal mode: click to place the goal for every agent")
        elif self.paused:
            self._banner(world_surface, "Paused: press Space to run, . to step")
        self._metrics()
        self.panel.draw(self.screen)
        pygame.display.flip()

    def _fog_memory(self) -> tuple[list[ObstacleRecord], set[str]]:
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

    def _draw_pending(self, surface: pygame.Surface) -> None:
        sc = self.scenario
        assert sc is not None
        edit = sc.pending_edit
        if edit is None:
            return
        world = sc.world
        if edit.kind == "cell" and edit.cell is not None and isinstance(world, GridWorld):
            draw_pending_cell(surface, self.camera, edit.cell.x, edit.cell.y, world.cell_size)
        elif edit.kind == "obstacle" and isinstance(world, ContinuousWorld):
            for obs in world.obstacles:
                if obs.id == edit.obstacle_id:
                    draw_pending_polygon(surface, self.camera, obs.polygon)

    def _banner(self, surface: pygame.Surface, text: str) -> None:
        img = self.small.render(text, True, PANEL_FG)
        rect = img.get_rect(topleft=(12, 10)).inflate(16, 8)
        pygame.draw.rect(surface, BANNER_BG, rect, border_radius=4)
        surface.blit(img, (rect.x + 8, rect.y + 4))

    def _metrics(self) -> None:
        sc = self.scenario
        assert sc is not None
        world = sc.world
        n = len(sc.agents)
        ok = sum(1 for a in sc.agents if self.results.get(a.id) and self.results[a.id].success)
        arrived = sum(
            1
            for a in sc.agents
            if (self.results.get(a.id) is not None and self.results[a.id].success and not self.waypoints.get(a.id))
        )
        focus = self.results.get(self._focus_id())
        cost = f"{focus.cost:.1f}" if focus and focus.success else "-"
        if not self.panel.get("fow").value:
            revealed = "all"
        else:
            revealed = f"{sc.belief.revealed_fraction():.0%}"
        if isinstance(world, GridWorld):
            stale = int(np.count_nonzero(sc.belief.stale_cells(world.blocked)))
        else:
            live = {obs.id for obs in world.obstacles}
            stale = sum(1 for oid in sc.belief.obstacles if oid not in live)
        if sc.pending_edit is not None:
            edit = "press E"
        else:
            edit = "done"
        lines = [
            ("state", "paused" if self.paused else "running"),
            ("time", f"{self.now:.1f} s"),
            ("fps", f"{self.clock.get_fps():.0f}"),
            ("walls", f"{world.occupancy_fraction():.0%}"),
            ("seen", revealed),
            ("stale", str(stale)),
            ("paths", f"{ok}/{n}"),
            ("arrived", f"{arrived}/{n}"),
            ("cost", cost),
            ("expanded", str(focus.expansions) if focus else "-"),
            ("plan", f"{self.plan_ms:.1f} ms"),
            ("edit", edit),
        ]
        self.panel.set_metrics(lines)

    # ------------------------------------------------------------------ loop

    def run(self) -> int:
        running = True
        while running:
            dt = min(self.clock.tick(60) / 1000.0, 1.0 / 20.0)
            self.frame += 1
            for event in pygame.event.get():
                running = self.handle(event)
                if not running:
                    break
            if running:
                self.tick(dt)
                self.draw()
        pygame.quit()
        return 0


DENSITY_MAPS = {"bernoulli", "clustered", "open", "polygons", "sparse", "clutter"}


def _planner_class(name: str):
    from swarmpathfinding.core.registry import _REGISTRY

    return _REGISTRY.get(name)


def _record(obs: PolygonObstacle, now: float) -> ObstacleRecord:
    return ObstacleRecord(
        id=obs.id,
        vertices=obs.vertices(),
        position=obs.position,
        velocity=obs.velocity,
        last_seen_time=now,
    )


def _font(size: int) -> pygame.font.Font:
    for name in ("consolas", "dejavusansmono", "menlo", "couriernew"):
        path = pygame.font.match_font(name)
        if path:
            return pygame.font.Font(path, size)
    return pygame.font.Font(None, size + 4)


def launch_lab(settings: LabSettings | None = None, **kwargs) -> int:
    """Open the Pygame window. Blocks until the window is closed.

    Notebooks should call this from a cell. The kernel is busy until you quit (Esc).
    """
    if settings is None:
        settings = LabSettings(**kwargs) if kwargs else LabSettings()
    elif kwargs:
        from dataclasses import replace

        settings = replace(settings, **kwargs)
    return LiveApp(settings).run()


def launch_spec(spec, planner: str = "weighted_a_star") -> int:
    from swarmpathfinding.viz.settings import LabSettings as _LS

    return launch_lab(_LS.from_spec(spec, planner=planner))


def main(argv: list[str] | None = None) -> int:
    settings = parse_lab_args(argv)
    return LiveApp(settings).run()


if __name__ == "__main__":
    raise SystemExit(main())
