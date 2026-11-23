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
from swarmpathfinding.viz.colors import PANE_EDGE
from swarmpathfinding.viz.menu import MenuOverlay, planner_label, set_form_values
from swarmpathfinding.viz.search_view import HEATMAPS, SearchBar, SearchPlayer, draw_inspector, draw_search_state
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
    ("P", "replay search"),
    ("<- ->", "search step"),
    ("C", "end compare"),
]
# Panel keys that only change how the search is drawn.
VIEW_KEYS = {"heatmap", "show_open", "show_closed", "arrows", "inspector"}
STRIP_H = 26  # compare-mode metrics strip
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
        self.compare_sims: list[Simulation] = []
        self.players: list[SearchPlayer] = [SearchPlayer()]
        self.search_bar = SearchBar(self.small)
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
        self.menu.scenario.compare = [sim.config.planner for sim in self.panes] if self.compare_sims else []

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
        if self.compare_sims:
            self.set_compare([])
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
        """Load menu values into the live view. Only a planner change keeps the map.
        A "compare" list of 2-4 planners opens synchronised compare panes."""
        values = dict(values)
        compare = values.pop("compare", None)
        planner_only = set(values) == {"planner"}
        panel_keys = {w.key for w in self.panel.widgets}
        set_form_values(self.panel, {k: v for k, v in values.items() if k in panel_keys})
        for k, v in values.items():
            if k not in panel_keys and k in SIM_KEYS:
                self.extra_config[k] = v
        self._sync_panel_state()
        if compare and len(compare) >= 2:
            self.set_compare(compare)
        elif planner_only:
            self.sim.set_planner(values["planner"], sync=True)
        else:
            if self.compare_sims:
                self.compare_sims = []
                self._layout()
            self.rebuild()
            self._fit()

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
        cfg = self._config()
        self.sim.rebuild(cfg)
        for sim in self.compare_sims:
            sim.rebuild(cfg.with_(planner=sim.config.planner))
        self._reset_players()
        self._fit()

    # ------------------------------------------------------------------ panel

    def _make_panel(self) -> SidePanel:
        panel = SidePanel(pygame.Rect(0, 0, PANEL_W, 800), self.font, self.small)
        planners = list_planners() or ["none"]
        default_planner = "weighted_a_star" if "weighted_a_star" in planners else planners[0]
        panel.sections = ["Scenario", "Swarm", "Planner", "Search"]
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
            Cycle("heatmap", "Heatmap", list(HEATMAPS), 0, section="Search", display={"off": "off", "g": "g (cost so far)", "h": "h (to goal)", "f": "f (priority)"}),
            Check("show_open", "Open set (frontier)", True, section="Search"),
            Check("show_closed", "Closed set (expanded)", True, section="Search"),
            Check("arrows", "Parent arrows", False, section="Search"),
            Check("inspector", "Cell inspector (hover)", True, section="Search"),
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
        if key in VIEW_KEYS:
            return
        self.cancel_visual_bench()
        value = self.panel.get(key).value
        if key == "show_search":
            for sim in self.panes:
                sim.record_focus_trace = bool(value)
                sim.replan_all(sync=True)
            self._reset_players()
            return
        if key == "los":
            for sim in self.panes:
                sim.set_view_option(los=bool(value))
            return
        if key == "planner":
            self.sim.set_planner(value, sync=self.paused)
            return
        if key == "w":
            for sim in self.panes:
                sim.set_weight(value, sync=self.paused)
            return
        self.rebuild()

    def _sync_panel_state(self) -> None:
        """Show / enable controls that actually do something for the current choice."""
        vals = self.panel.values()
        planner_cls_uses_w = bool(getattr(_planner_class(vals["planner"]), "uses_weight", False))
        self.panel.get("w").visible = planner_cls_uses_w
        grid = vals["world_kind"] == "grid"
        self.panel.get("los").enabled = grid
        has_search = any(bool(getattr(_planner_class(p), "records_trace", False)) for p in self._pane_planners(vals["planner"]))
        self.panel.get("show_search").visible = has_search
        for key in VIEW_KEYS:
            self.panel.get(key).visible = has_search and bool(vals["show_search"])
        self.panel.get("vision_radius").enabled = vals["vision"] != "none"
        self.panel.get("density").visible = vals["map_kind"] in DENSITY_MAPS

    def _pane_planners(self, first: str) -> list[str]:
        others = [s.config.planner for s in getattr(self, "compare_sims", [])]
        return [first, *others]

    @property
    def panes(self) -> list[Simulation]:
        """Every simulation on screen: the main one, then compare panes."""
        return [self.sim, *self.compare_sims]

    def world_rect(self) -> pygame.Rect:
        w, h = self.screen.get_size()
        return pygame.Rect(0, 0, max(1, w - PANEL_W), h)

    def pane_rects(self) -> list[pygame.Rect]:
        area = self.world_rect()
        n = 1 + len(getattr(self, "compare_sims", []))
        if n == 1:
            return [area]
        area = pygame.Rect(area.x, area.y, area.width, area.height - STRIP_H * (2 if n > 2 else 1))
        if n == 2:
            half = area.width // 2
            return [pygame.Rect(area.x, area.y, half - 1, area.height), pygame.Rect(area.x + half + 1, area.y, area.width - half - 1, area.height)]
        hw, hh = area.width // 2, area.height // 2
        rects = [
            pygame.Rect(area.x, area.y, hw - 1, hh - 1),
            pygame.Rect(area.x + hw + 1, area.y, area.width - hw - 1, hh - 1),
            pygame.Rect(area.x, area.y + hh + 1, hw - 1, area.height - hh - 1),
            pygame.Rect(area.x + hw + 1, area.y + hh + 1, area.width - hw - 1, area.height - hh - 1),
        ]
        return rects[:n]

    def pane_cameras(self) -> list[Camera]:
        """One camera per pane, all showing the same part of the world."""
        cams = []
        for i, rect in enumerate(self.pane_rects()):
            if i == 0:
                cams.append(self.camera)
            else:
                cams.append(Camera(self.camera.x, self.camera.y, self.camera.zoom, rect))
        return cams

    def pane_at(self, pos: tuple[int, int]) -> int | None:
        for i, rect in enumerate(self.pane_rects()):
            if rect.collidepoint(pos):
                return i
        return None

    def _to_pane0(self, pos: tuple[int, int]) -> tuple[int, int]:
        i = self.pane_at(pos)
        if not i:
            return pos
        rects = self.pane_rects()
        return pos[0] - rects[i].x + rects[0].x, pos[1] - rects[i].y + rects[0].y

    def _layout(self) -> None:
        w, h = self.screen.get_size()
        self.panel.rect = pygame.Rect(w - PANEL_W, 0, PANEL_W, h)
        self.camera.set_viewport(self.pane_rects()[0])
        self.search_bar.layout(self.world_rect() if not self.compare_sims else self.pane_rects()[0].union(self.pane_rects()[-1]))

    # ------------------------------------------------------------------ compare panes

    def set_compare(self, planners: list[str]) -> None:
        """Show 2 to 4 synchronised panes, one per planner, on the same scenario.
        An empty or single-item list goes back to one pane."""
        planners = list(dict.fromkeys(planners))[:4]
        self.cancel_visual_bench()
        if len(planners) < 2:
            self.compare_sims = []
        else:
            self.panel.get("planner").index = self.panel.get("planner").options.index(planners[0])
            cfg = self._config()
            self.sim.rebuild(cfg)
            self.compare_sims = [Simulation(cfg.with_(planner=p)) for p in planners[1:]]
            for sim in self.compare_sims:
                sim.record_focus_trace = self.sim.record_focus_trace
                if sim.record_focus_trace:
                    sim.replan_all(sync=True)
        self.players = [SearchPlayer() for _ in self.panes]
        self._sync_panel_state()
        self._layout()
        self._fit()

    def _reset_players(self) -> None:
        self.players = [SearchPlayer() for _ in self.panes]

    def _update_players(self) -> None:
        if len(self.players) != len(self.panes):
            self._reset_players()
        for sim, player in zip(self.panes, self.players):
            res = sim.results.get(sim.focus_id()) if sim.scenario is not None else None
            trace = res.extra.get("trace") if res is not None else None
            shape = res.extra["closed"].shape if res is not None and "closed" in res.extra else None
            player.offer(trace, shape)

    def search_active(self) -> bool:
        return bool(self.panel.get("show_search").value) and any(p.trace is not None for p in self.players)

    def _search_cmd(self, cmd: str, arg: int = 0) -> None:
        """Apply a replay command to every pane's player, in lock-step by expansion."""
        if not self.search_active():
            return
        self.paused = True
        lead = self.players[0]
        if cmd == "toggle":
            playing = not lead.playing
            for p in self.players:
                if p.trace is None:
                    continue
                if playing and p.at_end:
                    p.seek(0)
                p.playing = playing
                p.frozen = True
        elif cmd == "step":
            for p in self.players:
                p.step(arg)
        elif cmd == "start":
            for p in self.players:
                p.restart()
        elif cmd == "end":
            for p in self.players:
                p.to_end()

    # ------------------------------------------------------------------ simulation

    def tick(self, dt: float) -> None:
        for p in self.players:
            p.speed_index = self.players[0].speed_index
            p.tick(dt)
        if self.menu.runner is not None:
            self._bench_tick(dt)
            if self.visual_bench_active():
                return
        if self.paused:
            return
        self.step(dt)

    def step(self, dt: float) -> None:
        for sim in self.panes:
            sim.step(dt)

    # ------------------------------------------------------------------ edits

    def _click_world(self, button: int, pos: tuple[int, int]) -> None:
        sc = self.scenario
        if sc is None or self.visual_bench_active():
            return
        world_pt = self.camera.screen_to_world(*self._to_pane0(pos))
        size = sc.world.world_size
        if not (0 <= world_pt.x < size.x and 0 <= world_pt.y < size.y):
            return
        sync = self.paused
        for sim in self.panes:  # compare panes get the same edit
            if self.place_goal:
                if button == 1:
                    sim.set_goal(world_pt, sync=sync)
                continue
            world = sim.scenario.world
            if isinstance(world, GridWorld):
                sim.set_cell(world.world_to_cell(world_pt), blocked=(button == 1), sync=sync)
            elif button == 3:
                sim.remove_obstacle_at(world_pt, sync=sync)
            else:
                sim.add_box(world_pt, sync=sync)

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
        if self.search_active() and self.search_bar.handle(event, self.players[0]):
            # the bar drives the lead player; mirror its position on the other panes
            self.paused = True
            self._mirror_players()
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
                for sim in self.panes:
                    sim.apply_pending_edit(sync=self.paused)
            if event.key == pygame.K_p:
                self._search_cmd("toggle")
            if event.key in (pygame.K_LEFT, pygame.K_RIGHT):
                n = 10 if pygame.key.get_mods() & pygame.KMOD_SHIFT else 1
                self._search_cmd("step", n if event.key == pygame.K_RIGHT else -n)
            if event.key == pygame.K_HOME:
                self._search_cmd("start")
            if event.key == pygame.K_END:
                self._search_cmd("end")
            if event.key == pygame.K_c and self.compare_sims:
                self.set_compare([])
            if event.key == pygame.K_g:
                self.place_goal = True
            if event.key == pygame.K_m:
                self.open_menu()
        if event.type == pygame.KEYUP and event.key == pygame.K_g:
            self.place_goal = False
        if event.type == pygame.MOUSEWHEEL and self.pane_at(pygame.mouse.get_pos()) is not None:
            factor = 1.1 if event.y > 0 else 1 / 1.1
            mx, my = self._to_pane0(pygame.mouse.get_pos())
            self.camera.zoom_at(mx, my, factor)
        if event.type == pygame.MOUSEBUTTONDOWN and not self.panel.dragging:
            if event.button == 2 and self.pane_at(event.pos) is not None:
                self.panning = True
            elif event.button in (1, 3) and self.pane_at(event.pos) is not None:
                self._click_world(event.button, event.pos)
        if event.type == pygame.MOUSEBUTTONUP and event.button == 2:
            self.panning = False
        if event.type == pygame.MOUSEMOTION and self.panning:
            self.camera.pan(event.rel[0], event.rel[1])
        return True

    # ------------------------------------------------------------------ drawing

    def draw(self) -> None:
        self.screen.fill(PANEL_BG)
        sc = self.scenario
        if sc is None:
            self.panel.draw(self.screen)
            pygame.display.flip()
            return
        self._update_players()
        vals = self.panel.values()
        cams = self.pane_cameras()
        for i, (sim, cam) in enumerate(zip(self.panes, cams)):
            surf = self.screen.subsurface(cam.viewport)
            surf.fill((12, 14, 18))
            self._draw_sim(surf, cam, sim, self.players[i] if i < len(self.players) else None, vals)
            if self.compare_sims:
                pygame.draw.rect(surf, PANE_EDGE, surf.get_rect(), 1)
                self._pane_label(surf, sim)
        world_surface = self.screen.subsurface(cams[0].viewport)
        if self.compare_sims:
            self._draw_strip()
        if self.search_active():
            title = planner_label(self.sim.config.planner)
            self.search_bar.draw(self.screen, self.players[0], f"agent 0, {title}")
            if vals["inspector"]:
                self._draw_inspector()
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

    def _draw_sim(self, surface: pygame.Surface, camera: Camera, sim: Simulation, player: SearchPlayer | None, vals: dict) -> None:
        sc = sim.scenario
        assert sc is not None
        world = sc.world
        if isinstance(world, GridWorld):
            if sim.config.fow:
                draw_belief_grid(surface, camera, world, sc.belief, sim.visible)
            else:
                draw_grid(surface, camera, world)
        elif sim.config.fow:
            # Under fog: fog, then remembered obstacle shapes (dimmed), then what is in view now.
            remembered, ghost_ids = sim.fog_memory()
            draw_floor(surface, camera, world)
            draw_fog_raster(surface, camera, sc.belief, sim.visible)
            draw_remembered(surface, camera, remembered)
            draw_visible_truth(surface, camera, world, sim.union)
            draw_ghosts(surface, camera, [g for g in sc.belief.ghost_obstacles(sim.now) if g.id in ghost_ids])
        else:
            draw_polygons(surface, camera, world)
        focus = sim.results.get(sim.focus_id())
        if vals["show_search"] and player is not None and player.trace is not None:
            draw_search_state(
                surface, camera, player, player.trace.cell_size, vals["heatmap"],
                vals["show_open"], vals["show_closed"], vals["arrows"],
            )
        elif vals["show_search"] and focus is not None and "closed" in focus.extra:
            draw_search(surface, camera, focus.extra["closed"], focus.extra["cell_size"])
        self._draw_pending(surface, camera, sim)
        if sim.union is not None:
            draw_vision_union(surface, camera, sim.union)
        fid = sim.focus_id()
        paths = [[a.pos, *sim.waypoints.get(a.id, [])] for a in sc.agents if a.id != fid]
        focus_agent = sc.agents[0] if sc.agents else None
        focus_path = [focus_agent.pos, *sim.waypoints.get(fid, [])] if focus_agent else None
        draw_paths(surface, camera, paths, focus_path)
        goals = {(g.x, g.y): g for g in (*(sim.goal_for(a) for a in sc.agents), sc.goal)}
        for g in goals.values():
            draw_goal(surface, camera, g)
        draw_agents(surface, camera, sc.agents, focus=fid)

    def _pane_label(self, surface: pygame.Surface, sim: Simulation) -> None:
        img = self.font.render(planner_label(sim.config.planner), True, PANEL_FG)
        rect = img.get_rect(topright=(surface.get_width() - 10, 8)).inflate(14, 6)
        pygame.draw.rect(surface, BANNER_BG, rect, border_radius=4)
        surface.blit(img, (rect.x + 7, rect.y + 3))

    def _draw_strip(self) -> None:
        """Compare mode: one metrics cell per pane under the panes."""
        area = self.world_rect()
        rows = 2 if len(self.panes) > 2 else 1
        top = area.bottom - STRIP_H * rows
        pygame.draw.rect(self.screen, PANEL_BG, (area.x, top, area.width, STRIP_H * rows))
        per_row = 2
        col_w = area.width // per_row
        for i, sim in enumerate(self.panes):
            sc = sim.scenario
            n = len(sc.agents)
            arrived = sum(1 for a in sc.agents if sim.arrived(a))
            focus = sim.results.get(sim.focus_id())
            cost = f"{focus.cost:.1f}" if focus and focus.success else "-"
            exp = focus.expansions if focus else 0
            text = f"{planner_label(sim.config.planner)}: home {arrived}/{n}, cost {cost}, {exp} expanded, {sim.stats.plan_ms:.0f} ms"
            x = area.x + (i % per_row) * col_w + 10
            y = top + (i // per_row) * STRIP_H + 5
            self.screen.blit(self.small.render(text, True, PANEL_FG), (x, y))

    def _draw_inspector(self) -> None:
        mouse = pygame.mouse.get_pos()
        i = self.pane_at(mouse)
        if i is None or i >= len(self.players) or self.search_bar.rect.collidepoint(mouse):
            return
        player = self.players[i]
        if player.trace is None:
            return
        pt = self.camera.screen_to_world(*self._to_pane0(mouse))
        cs = player.trace.cell_size
        rows = player.cell_info(int(pt.x // cs), int(pt.y // cs))
        if rows:
            draw_inspector(self.screen, self.small, mouse, rows, self.world_rect())

    def _mirror_players(self) -> None:
        lead = self.players[0]
        for p in self.players[1:]:
            p.speed_index = lead.speed_index
            p.playing = lead.playing
            p.frozen = lead.frozen
            if p.trace is None:
                continue
            if lead.at_end:
                p.seek(p.total)
            else:
                p.seek(p._event_after_expansions(lead.expanded))

    def _draw_pending(self, surface: pygame.Surface, camera: Camera | None = None, sim: Simulation | None = None) -> None:
        camera = camera or self.camera
        sim = sim or self.sim
        sc = sim.scenario
        assert sc is not None
        edit = sc.pending_edit
        if edit is None:
            return
        world = sc.world
        if edit.kind == "cell" and edit.cell is not None and isinstance(world, GridWorld):
            draw_pending_cell(surface, camera, edit.cell.x, edit.cell.y, world.cell_size)
        elif edit.kind == "obstacle" and isinstance(world, ContinuousWorld):
            for obs in world.obstacles:
                if obs.id == edit.obstacle_id:
                    draw_pending_polygon(surface, camera, obs.polygon)

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
