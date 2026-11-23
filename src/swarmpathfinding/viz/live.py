from __future__ import annotations

import pygame

from swarmpathfinding.core.geom import Vec2
from swarmpathfinding.core.registry import ensure_builtin_planners, list_planners
from swarmpathfinding.core.results import PathResult
from swarmpathfinding.generators.scenarios import GRID_MAPS as GMAPS
from swarmpathfinding.generators.scenarios import POLY_MAPS as PMAPS
from swarmpathfinding.generators.scenarios import START_LAYOUTS, Scenario
from swarmpathfinding.sim.simulation import CHANGES, SimConfig, Simulation
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
from swarmpathfinding.viz.menu import MenuOverlay, set_form_values
from swarmpathfinding.viz.panel import Check, Cycle, SidePanel, Slider
from swarmpathfinding.viz.settings import LabSettings, apply_settings, parse_lab_args
from swarmpathfinding.worlds.continuous import ContinuousWorld
from swarmpathfinding.worlds.grid import GridWorld

PANEL_W = 320
HELP = [
    ("M", "lab menu"),
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
# Panel keys that make up a SimConfig.
SIM_KEYS = tuple(SimConfig.__dataclass_fields__)


class LiveApp:
    """Pygame front end: draws a `Simulation` and turns input into edits."""

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
        self.paused = True
        self.place_goal = False
        self.panning = False
        self.frame = 0
        self._layout()
        # SimConfig fields that have no panel control yet (set from settings / CLI).
        self.extra_config: dict = {}
        if settings is not None:
            apply_settings(self.panel, settings)
            self.extra_config["edit_at"] = settings.edit_at
        self._sync_panel_state()
        self.sim = Simulation(self._config())
        self.sim.record_focus_trace = bool(self.panel.get("show_search").value)
        self._fit()
        self.menu = MenuOverlay(self.font, self.small, self.apply_config, lambda: self._config().as_dict())
        self.menu.on_close = self._menu_closed
        self.menu.on_bench_start = self._bench_started
        self._paused_before_menu = True
        self.bench_steps = 0  # sim steps per frame for a visual benchmark (0 = none)
        self._bench_acc = 0.0
        self.toast = ""
        self._toast_until = 0.0

    # ------------------------------------------------------------------ menu

    def open_menu(self, tab: int | None = None) -> None:
        if self.menu.is_open:
            return
        self._paused_before_menu = self.paused
        self.paused = True
        self.panning = False
        self.place_goal = False
        self.menu.open(tab)

    def _menu_closed(self) -> None:
        self.paused = self._paused_before_menu
        if self.menu.toast:
            self.show_toast(self.menu.toast)
            self.menu.toast = ""

    def show_toast(self, text: str, seconds: float = 4.0) -> None:
        self.toast = text
        self._toast_until = pygame.time.get_ticks() / 1000.0 + seconds

    # ------------------------------------------------------------------ benchmark

    @property
    def runner(self):
        return self.menu.runner

    def visual_bench_active(self) -> bool:
        r = self.menu.runner
        return r is not None and r.mode == "visual" and r.running

    def _bench_started(self, runner, mode: str) -> None:
        from swarmpathfinding.bench.runner import SPEEDS

        if runner.mode != "visual":
            self.bench_steps = 0
            return
        self.bench_steps = SPEEDS.get(mode, 4)
        self._bench_acc = 0.0
        self._paused_before_menu = False
        self.menu.close()

    def cancel_visual_bench(self) -> None:
        if self.visual_bench_active():
            self.menu.runner.cancel()
            self.show_toast("Benchmark cancelled")

    def _bench_tick(self, dt: float) -> None:
        from swarmpathfinding.bench.experiment import DT

        runner = self.menu.runner
        if runner is None or not runner.running:
            return
        if runner.mode != "visual":
            runner.poll()
            return
        if self.paused:
            return
        if self.bench_steps >= 1000:
            steps = 1_000_000  # bounded by the per-frame time budget
        else:
            self._bench_acc += self.bench_steps * dt / DT
            steps = int(self._bench_acc)
            self._bench_acc -= steps
        sim = runner.visual_step(steps)
        if sim is not None and sim is not self.sim:
            self.sim = sim
            set_form_values(self.panel, {k: v for k, v in sim.config.as_dict().items() if k != "planner"})
            self.panel.get("planner").index = self.panel.get("planner").options.index(sim.config.planner)
            self._sync_panel_state()
            self._fit()

    def apply_config(self, values: dict) -> None:
        """Load menu values into the live view. Only a planner change keeps the map."""
        planner_only = set(values) == {"planner"}
        panel_keys = {w.key for w in self.panel.widgets}
        set_form_values(self.panel, {k: v for k, v in values.items() if k in panel_keys})
        for k, v in values.items():
            if k not in panel_keys and k in SIM_KEYS:
                self.extra_config[k] = v
        self._sync_panel_state()
        if planner_only:
            self.sim.set_planner(values["planner"], sync=True)
        else:
            self.rebuild()

    # ------------------------------------------------------------------ sim access

    @property
    def scenario(self) -> Scenario | None:
        return self.sim.scenario

    @property
    def results(self) -> dict[int, PathResult]:
        return self.sim.results

    @property
    def waypoints(self) -> dict[int, list[Vec2]]:
        return self.sim.waypoints

    def _config(self) -> SimConfig:
        vals = {**self.extra_config, **self.panel.values()}
        return SimConfig(**{k: vals[k] for k in SIM_KEYS if k in vals})

    def _fit(self) -> None:
        size = self.sim.scenario.world.world_size
        self.camera.fit(size.x, size.y)

    def rebuild(self) -> None:
        self.cancel_visual_bench()
        self.sim.rebuild(self._config())
        self._fit()

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
            Cycle(
                "changes",
                "Changes",
                list(CHANGES),
                0,
                section="Scenario",
                display={"none": "none (E = manual)", "slow_edit": "auto edit"},
            ),
            Cycle("vision", "Vision", ["circle", "cone", "none"], 0, section="Swarm"),
            Slider("vision_radius", "Vision radius", 2.0, 16.0, 6.0, section="Swarm"),
            Check("fow", "Fog of war", True, section="Swarm"),
            Check("los", "Vision needs LOS", True, section="Swarm", hint="grid only"),
            Cycle("planner", "Planner", planners, planners.index(default_planner), section="Planner"),
            Slider("w", "Weight w", 1.0, 3.0, 1.0, section="Planner"),
            Check("show_search", "Show search", False, section="Planner"),
        ]
        panel.button_label = "Restart scenario  (R)"
        panel.top_button_label = "Lab menu  (M)"
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
        self.cancel_visual_bench()
        value = self.panel.get(key).value
        if key == "show_search":
            self.sim.record_focus_trace = bool(value)
            self.sim.replan_all(sync=True)
            return
        if key == "los":
            self.sim.set_view_option(los=bool(value))
            return
        if key == "planner":
            self.sim.set_planner(value, sync=self.paused)
            return
        if key == "w":
            self.sim.set_weight(value, sync=self.paused)
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

    # ------------------------------------------------------------------ simulation

    def tick(self, dt: float) -> None:
        if self.menu.runner is not None:
            self._bench_tick(dt)
            if self.visual_bench_active():
                return
        if self.paused:
            return
        self.step(dt)

    def step(self, dt: float) -> None:
        self.sim.step(dt)

    # ------------------------------------------------------------------ edits

    def _click_world(self, button: int, pos: tuple[int, int]) -> None:
        sc = self.scenario
        if sc is None or self.visual_bench_active():
            return
        world_pt = self.camera.screen_to_world(*pos)
        size = sc.world.world_size
        if not (0 <= world_pt.x < size.x and 0 <= world_pt.y < size.y):
            return
        sync = self.paused
        if self.place_goal:
            if button == 1:
                self.sim.set_goal(world_pt, sync=sync)
            return
        world = sc.world
        if isinstance(world, GridWorld):
            self.sim.set_cell(world.world_to_cell(world_pt), blocked=(button == 1), sync=sync)
        elif button == 3:
            self.sim.remove_obstacle_at(world_pt, sync=sync)
        else:
            self.sim.add_box(world_pt, sync=sync)

    # ------------------------------------------------------------------ events

    def handle(self, event: pygame.event.Event) -> bool:
        if event.type == pygame.QUIT:
            return False
        if event.type == pygame.VIDEORESIZE:
            self.screen = pygame.display.set_mode((event.w, event.h), pygame.RESIZABLE)
            self._layout()
        if self.menu.is_open:
            self.menu.handle(event)
            return True
        action = self.panel.handle(event)
        if action == "button":
            self.rebuild()
            return True
        if action == "top":
            self.open_menu()
            return True
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                if self.visual_bench_active():
                    self.cancel_visual_bench()
                    return True
                return False
            if event.key == pygame.K_SPACE:
                self.paused = not self.paused
            if event.key == pygame.K_PERIOD and self.scenario is not None:
                self.paused = True
                self.step(1.0 / 30.0)
            if event.key == pygame.K_r:
                self.rebuild()
            if event.key == pygame.K_f and self.scenario is not None:
                self._fit()
            if event.key == pygame.K_e and self.scenario is not None:
                self.sim.apply_pending_edit(sync=self.paused)
            if event.key == pygame.K_g:
                self.place_goal = True
            if event.key == pygame.K_m:
                self.open_menu()
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
        fow = bool(self.sim.config.fow)
        focus = self.results.get(self.sim.focus_id())
        if isinstance(world, GridWorld):
            if fow:
                draw_belief_grid(world_surface, self.camera, world, sc.belief, self.sim.visible)
            else:
                draw_grid(world_surface, self.camera, world)
        elif fow:
            # Under fog: fog, then remembered obstacle shapes (dimmed), then what is in view now.
            remembered, ghost_ids = self.sim.fog_memory()
            draw_floor(world_surface, self.camera, world)
            draw_fog_raster(world_surface, self.camera, sc.belief, self.sim.visible)
            draw_remembered(world_surface, self.camera, remembered)
            draw_visible_truth(world_surface, self.camera, world, self.sim.union)
            draw_ghosts(world_surface, self.camera, [g for g in sc.belief.ghost_obstacles(self.sim.now) if g.id in ghost_ids])
        else:
            draw_polygons(world_surface, self.camera, world)
        if vals["show_search"] and focus is not None and "closed" in focus.extra:
            draw_search(world_surface, self.camera, focus.extra["closed"], focus.extra["cell_size"])
        self._draw_pending(world_surface)
        if self.sim.union is not None:
            draw_vision_union(world_surface, self.camera, self.sim.union)
        fid = self.sim.focus_id()
        paths = [[a.pos, *self.waypoints.get(a.id, [])] for a in sc.agents if a.id != fid]
        focus_agent = sc.agents[0] if sc.agents else None
        focus_path = [focus_agent.pos, *self.waypoints.get(fid, [])] if focus_agent else None
        draw_paths(world_surface, self.camera, paths, focus_path)
        goals = {(g.x, g.y): g for g in (*(self.sim.goal_for(a) for a in sc.agents), sc.goal)}
        for g in goals.values():
            draw_goal(world_surface, self.camera, g)
        draw_agents(world_surface, self.camera, sc.agents, focus=fid)
        now_s = pygame.time.get_ticks() / 1000.0
        if self.visual_bench_active():
            r = self.menu.runner
            cur = r.current
            what = f"scenario {cur.scenario}, {cur.planner}" if cur else "starting"
            state = "paused, Space resumes" if self.paused else "Space pauses"
            self._banner(world_surface, f"Benchmark {r.progress.done}/{r.progress.total} ({what}): {state}, Esc cancels")
        elif self.toast and now_s < self._toast_until:
            self._banner(world_surface, self.toast)
        elif self.place_goal:
            self._banner(world_surface, "Goal mode: click to place the goal for every agent")
        elif self.paused:
            self._banner(world_surface, "Paused: press Space to run, . to step")
        self._metrics()
        self.panel.draw(self.screen)
        self.menu.draw(self.screen)
        pygame.display.flip()

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
        sim = self.sim
        sc = sim.scenario
        assert sc is not None
        n = len(sc.agents)
        ok = sum(1 for a in sc.agents if sim.results.get(a.id) and sim.results[a.id].success)
        arrived = sum(1 for a in sc.agents if sim.arrived(a))
        focus = sim.results.get(sim.focus_id())
        cost = f"{focus.cost:.1f}" if focus and focus.success else "-"
        revealed = f"{sc.belief.revealed_fraction():.0%}" if sim.config.fow else "all"
        if sc.pending_edit is None:
            edit = "done"
        elif sim.config.changes == "slow_edit":
            edit = f"at {sim.config.edit_at:.0f} s"
        else:
            edit = "press E"
        lines = [
            ("state", "paused" if self.paused else "running"),
            ("time", f"{sim.now:.1f} s"),
            ("fps", f"{self.clock.get_fps():.0f}"),
            ("walls", f"{sc.world.occupancy_fraction():.0%}"),
            ("seen", revealed),
            ("stale", str(sim.stale_count())),
            ("paths", f"{ok}/{n}"),
            ("arrived", f"{arrived}/{n}"),
            ("cost", cost),
            ("expanded", str(focus.expansions) if focus else "-"),
            ("plan", f"{sim.plan_ms:.1f} ms"),
            ("edit", edit),
        ]
        r = self.menu.runner
        if r is not None:
            state = "" if r.running else (" done" if r.finished and not r.cancelled else " stop")
            lines.append(("bench", f"{r.progress.done}/{r.progress.total}{state}"))
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
        if self.menu.runner is not None and self.menu.runner.running:
            self.menu.runner.cancel()
        pygame.quit()
        return 0


DENSITY_MAPS = {"bernoulli", "clustered", "open", "polygons", "sparse", "clutter"}


def _planner_class(name: str):
    from swarmpathfinding.core.registry import _REGISTRY

    return _REGISTRY.get(name)


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
