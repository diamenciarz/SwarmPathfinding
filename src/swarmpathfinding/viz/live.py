from __future__ import annotations

import sys
from typing import Any

import pygame
from shapely.geometry import box

from swarmpathfinding.core.belief import ObstacleRecord
from swarmpathfinding.core.geom import Vec2
from swarmpathfinding.core.registry import create_planner, ensure_builtin_planners, list_planners
from swarmpathfinding.core.results import PathResult
from swarmpathfinding.generators.scenarios import GRID_MAPS as GMAPS
from swarmpathfinding.generators.scenarios import POLY_MAPS as PMAPS
from swarmpathfinding.generators.scenarios import Scenario, make_scenario
from swarmpathfinding.generators.scenarios import apply_pending_edit
from swarmpathfinding.physics.bodies import attach_pymunk, make_space_for_world, sync_mover_bodies
from swarmpathfinding.viz.camera import Camera
from swarmpathfinding.viz.colors import PANEL_BG
from swarmpathfinding.viz.draw import (
    draw_agents,
    draw_ghosts,
    draw_goal,
    draw_grid,
    draw_last_seen_grid,
    draw_path,
    draw_polygons,
)
from swarmpathfinding.viz.fog import FogCache, draw_fog_mask, draw_vision, visible_mask_grid
from swarmpathfinding.viz.panel import Check, Cycle, SidePanel, Slider
from swarmpathfinding.viz.settings import LabSettings, apply_settings, parse_lab_args
from swarmpathfinding.worlds.continuous import ContinuousWorld, PolygonObstacle
from swarmpathfinding.worlds.grid import GridWorld

PANEL_W = 320
HELP = [
    "LMB  add wall",
    "RMB  delete wall",
    "G+LMB  set goal",
    "Wheel  zoom",
    "MMB drag  pan",
    "Space  pause",
    ".  step",
    "R  regenerate",
    "E  apply pending edit",
]


class LiveApp:
    def __init__(self, settings: LabSettings | None = None) -> None:
        pygame.init()
        pygame.display.set_caption("SwarmPathfinding")
        self.screen = pygame.display.set_mode((1280, 800), pygame.RESIZABLE)
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("consolas", 18) or pygame.font.Font(None, 18)
        self.small = pygame.font.SysFont("consolas", 14) or pygame.font.Font(None, 14)
        ensure_builtin_planners()
        self.camera = Camera()
        self.panel = self._make_panel()
        self.fog_cache = FogCache()
        self.scenario: Scenario | None = None
        self.planner: Any = None
        self.result = PathResult()
        self.paused = True
        self.place_goal = False
        self.panning = False
        self.now = 0.0
        self.space = None
        self.movers = {}
        self.discrepancies = 0
        self.frame = 0
        self._layout()
        if settings is not None:
            apply_settings(self.panel, settings)
            self._on_panel_change()
            apply_settings(self.panel, settings)
        self.rebuild()

    def _make_panel(self) -> SidePanel:
        panel = SidePanel(pygame.Rect(0, 0, PANEL_W, 800), self.font, self.small)
        planners = list_planners() or ["none"]
        panel.cycles = [
            Cycle("world_kind", "World", ["grid", "continuous"], 0),
            Cycle("map_kind", "Map", list(GMAPS), 1),
            Cycle("planner", "Planner", planners, planners.index("straight_line") if "straight_line" in planners else 0),
            Cycle("vision", "Vision", ["circle", "cone", "none"], 0),
            Cycle("starts", "Starts", ["clump", "scattered", "flanks", "opposite"], 0),
        ]
        panel.sliders = [
            Slider("density", "Density", 0.02, 0.50, 0.15),
            Slider("n_agents", "Agents", 1, 100, 8, integer=True),
            Slider("vision_radius", "Vision radius", 2.0, 16.0, 6.0),
            Slider("w", "Weight w", 1.0, 3.0, 1.0),
            Slider("seed", "Seed", 1, 50, 1, integer=True),
        ]
        panel.checks = [
            Check("fow", "Fog of war", True),
            Check("los", "Vision needs LOS", True),
        ]
        panel.set_help(HELP)
        panel.set_on_change(self._on_panel_change)
        return panel

    def _on_panel_change(self) -> None:
        # map list depends on world kind
        world_kind = next(c.value for c in self.panel.cycles if c.key == "world_kind")
        maps = next(c for c in self.panel.cycles if c.key == "map_kind")
        wanted = list(GMAPS) if world_kind == "grid" else list(PMAPS)
        if maps.options != wanted:
            maps.options = wanted
            maps.index = 0

    def _layout(self) -> None:
        w, h = self.screen.get_size()
        self.panel.rect = pygame.Rect(w - PANEL_W, 0, PANEL_W, h)
        self.camera.viewport = pygame.Rect(0, 0, w - PANEL_W, h)

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
        if isinstance(self.scenario.world, ContinuousWorld):
            self.space, self.movers = make_space_for_world(self.scenario.world)
            attach_pymunk(self.scenario.agents, self.space)
        self.fog_cache = FogCache()
        self.now = 0.0
        self.planner = create_planner(vals["planner"])
        self.replan()
        self._reveal()

    def replan(self) -> None:
        assert self.scenario is not None
        starts = [agent.pos for agent in self.scenario.agents]
        self.result = self.planner.plan(self.scenario.belief, starts, self.scenario.goal)

    def _reveal(self) -> None:
        assert self.scenario is not None
        vals = self.panel.values()
        if not vals["fow"]:
            if isinstance(self.scenario.world, GridWorld):
                self.scenario.belief = type(self.scenario.belief).from_truth_grid(self.scenario.world)
            return
        world = self.scenario.world
        if not isinstance(world, GridWorld):
            self._reveal_continuous()
            return
        require_los = bool(vals["los"])
        all_cells = []
        for agent in self.scenario.agents:
            all_cells.extend(
                list(
                    agent.vision.visible_cells(
                        agent.pos, agent.heading, world, require_los=require_los
                    )
                )
            )
        self.discrepancies = self.scenario.belief.observe_cells(all_cells, world, self.now)

    def _reveal_continuous(self) -> None:
        assert self.scenario is not None
        world = self.scenario.world
        assert isinstance(world, ContinuousWorld)
        from shapely.geometry import Point

        seen_ids: set[str] = set()
        for agent in self.scenario.agents:
            foot = agent.vision.footprint_polygon(agent.pos, agent.heading)
            for obs in world.obstacles:
                if foot.intersects(obs.polygon):
                    seen_ids.add(obs.id)
                    self.scenario.belief.observe_obstacle(
                        ObstacleRecord(
                            id=obs.id,
                            vertices=obs.vertices(),
                            position=obs.position,
                            velocity=obs.velocity,
                            last_seen_time=self.now,
                        )
                    )
            # mark raster cells
            origin = agent.pos
            reach = getattr(agent.vision, "radius", 6.0)
            for obs in world.obstacles:
                if obs.id in seen_ids:
                    continue
        # raster fog: cells whose center is in any footprint
        belief = self.scenario.belief
        cell_w = world.width / belief.width
        cell_h = world.height / belief.height
        footprints = [a.vision.footprint_polygon(a.pos, a.heading) for a in self.scenario.agents]
        for y in range(belief.height):
            for x in range(belief.width):
                p = Point((x + 0.5) * cell_w, (y + 0.5) * cell_h)
                if any(fp.contains(p) or fp.covers(p) for fp in footprints):
                    if int(belief.occupancy[y, x]) < 0:
                        belief.occupancy[y, x] = 0
                    belief.seen_time[y, x] = self.now

    def tick(self, dt: float) -> None:
        if self.paused or self.scenario is None:
            return
        self.now += dt
        self._follow_path(dt)
        world = self.scenario.world
        if isinstance(world, ContinuousWorld):
            world.step_movers(dt)
            if self.movers:
                sync_mover_bodies(world, self.movers)
            if self.space is not None:
                self.space.step(dt)
                for agent in self.scenario.agents:
                    agent.sync_from_body()
        self._reveal()
        self.replan()

    def _follow_path(self, dt: float) -> None:
        assert self.scenario is not None
        if len(self.result.path) < 2:
            return
        target = self.result.path[1]
        speed = 6.0
        for agent in self.scenario.agents:
            waypoint = agent.goal or target
            delta = waypoint - agent.pos
            dist = delta.length()
            if dist < 0.15:
                continue
            step = delta.normalized() * min(speed * dt, dist)
            agent.pos = agent.pos + step
            agent.look_at(waypoint)
            if agent.body is not None:
                agent.body.position = (agent.pos.x, agent.pos.y)
                agent.body.velocity = (0, 0)

    def handle(self, event: pygame.event.Event) -> bool:
        if event.type == pygame.QUIT:
            return False
        if event.type == pygame.VIDEORESIZE:
            self.screen = pygame.display.set_mode((event.w, event.h), pygame.RESIZABLE)
            self._layout()
            if self.scenario:
                size = self.scenario.world.world_size
                self.camera.fit(size.x, size.y)
        action = self.panel.handle(event)
        if action == "regen":
            self.rebuild()
            return True
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                return False
            if event.key == pygame.K_SPACE:
                self.paused = not self.paused
            if event.key == pygame.K_PERIOD:
                self.tick(1.0 / 30.0)
            if event.key == pygame.K_r:
                self.rebuild()
            if event.key == pygame.K_e:
                if self.scenario is not None and apply_pending_edit(self.scenario):
                    self._reveal()
                    self.replan()
            if event.key == pygame.K_g:
                self.place_goal = True
        if event.type == pygame.KEYUP and event.key == pygame.K_g:
            self.place_goal = False
        if event.type == pygame.MOUSEWHEEL and self.camera.contains_screen(*pygame.mouse.get_pos()):
            factor = 1.1 if event.y > 0 else 1 / 1.1
            mx, my = pygame.mouse.get_pos()
            self.camera.zoom_at(mx, my, factor)
        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 2 and self.camera.contains_screen(*event.pos):
                self.panning = True
            elif event.button in (1, 3) and self.camera.contains_screen(*event.pos):
                self._click_world(event.button, event.pos)
        if event.type == pygame.MOUSEBUTTONUP and event.button == 2:
            self.panning = False
        if event.type == pygame.MOUSEMOTION and self.panning:
            self.camera.pan(event.rel[0], event.rel[1])
        return True

    def _click_world(self, button: int, pos: tuple[int, int]) -> None:
        if self.scenario is None:
            return
        world_pt = self.camera.screen_to_world(*pos)
        if self.place_goal:
            self.scenario.goal = world_pt
            for agent in self.scenario.agents:
                agent.look_at(world_pt)
            self.replan()
            return
        world = self.scenario.world
        if isinstance(world, GridWorld):
            cell = world.world_to_cell(world_pt)
            changed = world.punch_wall(cell) if button == 1 else world.delete_wall(cell)
            if changed:
                self._reveal()
                self.replan()
            return
        if isinstance(world, ContinuousWorld) and button == 3:
            for obs in list(world.obstacles):
                if obs.polygon.contains(pygame_point_to_shapely(world_pt)):
                    world.remove_obstacle(obs.id)
                    self.scenario.belief.forget_obstacle(obs.id)
                    self.replan()
                    return
        if isinstance(world, ContinuousWorld) and button == 1:
            s = 1.2
            poly = box(world_pt.x - s, world_pt.y - s, world_pt.x + s, world_pt.y + s)
            oid = f"user{world.generation}"
            world.add_obstacle(PolygonObstacle(id=oid, polygon=poly))
            self.replan()

    def draw(self) -> None:
        self.screen.fill(PANEL_BG)
        world_surface = self.screen.subsurface(self.camera.viewport)
        world_surface.fill((12, 14, 18))
        if self.scenario is None:
            self.panel.draw(self.screen)
            pygame.display.flip()
            return
        world = self.scenario.world
        vals = self.panel.values()
        if isinstance(world, GridWorld):
            draw_grid(world_surface, self.camera, world)
            if vals["fow"]:
                draw_last_seen_grid(world_surface, self.camera, self.scenario.belief, world.cell_size)
        else:
            draw_polygons(world_surface, self.camera, world)
            ghosts = self.scenario.belief.ghost_obstacles(self.now)
            draw_ghosts(world_surface, self.camera, ghosts)
        draw_path(world_surface, self.camera, self.result)
        for g in self.scenario.goals or (self.scenario.goal,):
            draw_goal(world_surface, self.camera, g)
        draw_agents(world_surface, self.camera, self.scenario.agents)
        if vals["vision"] != "none":
            draw_vision(world_surface, self.camera, self.scenario.agents)
        if vals["fow"] and isinstance(world, GridWorld):
            if self.fog_cache.needs_rebuild(self.scenario.agents, self.frame):
                self.fog_cache.mask = visible_mask_grid(
                    world, self.scenario.agents, require_los=bool(vals["los"])
                )
                self.fog_cache.frame = self.frame
                self.fog_cache.last_positions = tuple(
                    (a.pos.x, a.pos.y) for a in self.scenario.agents
                )
            if self.fog_cache.mask is not None:
                draw_fog_mask(world_surface, self.camera, world, self.fog_cache.mask)
        self._metrics()
        self.panel.draw(self.screen)
        pygame.display.flip()

    def _metrics(self) -> None:
        assert self.scenario is not None
        occ = self.scenario.world.occupancy_fraction()
        lines = [
            f"fps  {self.clock.get_fps():5.1f}",
            f"paused  {self.paused}",
            f"t  {self.now:6.2f}s",
            f"occupancy  {occ:.3f}",
            f"revealed  {self.scenario.belief.revealed_fraction():.3f}",
            f"discrepancies  {self.discrepancies}",
            f"path  {'ok' if self.result.success else 'none'}  cost={self.result.cost if self.result.success else float('inf'):.2f}",
            f"planner  {self.planner.name}",
            f"layout  {self.scenario.start_layout}",
            f"edit  {'ready' if self.scenario.pending_edit else 'none'}",
        ]
        self.panel.set_metrics(lines)

    def run(self) -> int:
        running = True
        while running:
            dt = self.clock.tick(60) / 1000.0
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


def pygame_point_to_shapely(point: Vec2):
    from shapely.geometry import Point

    return Point(point.x, point.y)


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


def launch_spec(spec, planner: str = "straight_line") -> int:
    from swarmpathfinding.viz.settings import LabSettings as _LS

    return launch_lab(_LS.from_spec(spec, planner=planner))


def main(argv: list[str] | None = None) -> int:
    settings = parse_lab_args(argv)
    return LiveApp(settings).run()


if __name__ == "__main__":
    raise SystemExit(main())
