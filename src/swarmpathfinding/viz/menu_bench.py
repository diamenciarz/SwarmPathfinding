"""Benchmark and Results tabs of the lab menu, plus the progress bars."""

from __future__ import annotations

import math
import random
import time
from pathlib import Path
from typing import TYPE_CHECKING

import pygame

from swarmpathfinding.bench import analysis as A
from swarmpathfinding.bench.experiment import METRICS, ExperimentSpec
from swarmpathfinding.bench.runner import OUT_DIR, BenchmarkRunner, default_workers
from swarmpathfinding.core.capabilities import TRAIT_LABELS, TRAITS
from swarmpathfinding.core.registry import list_planners
from swarmpathfinding.generators.scenarios import GRID_MAPS, POLY_MAPS, START_LAYOUTS
from swarmpathfinding.viz.colors import (
    ACCENT,
    BUTTON,
    CARD_EDGE,
    FIT_CANNOT,
    FIT_OK,
    PANEL_BG,
    PANEL_FG,
    PANEL_MUTED,
    ROW_HOT,
)
from swarmpathfinding.viz.panel import Chips, Cycle, Range, SidePanel, Slider

if TYPE_CHECKING:
    from swarmpathfinding.viz.menu import MenuOverlay

MODES = ("headless", "x4", "x16", "max")
MODE_LABEL = {"headless": "headless", "x4": "visual x4", "x16": "visual x16", "max": "visual max"}
BAR_FROM = (60, 120, 220)
BAR_TO = (90, 215, 160)
# planner colours for plots, in order
SERIES = [(110, 200, 255), (255, 160, 90), (150, 220, 120), (210, 130, 230), (240, 210, 90), (120, 230, 220), (240, 110, 130)]
SHORT = {
    "grid": "grid",
    "continuous": "polygons",
    "full": "full",
    "fog": "fog",
    "static": "static",
    "slow_edit": "edit",
    "movers": "movers",
    "single": "1",
    "few": "2-10",
    "crowd": "11+",
    "shared": "1 goal",
    "opposite": "2 groups",
}


# ---------------------------------------------------------------- drawing helpers


def _lerp(a, b, t):
    return tuple(int(a[i] + (b[i] - a[i]) * t) for i in range(3))


def gradient_bar(surface, rect: pygame.Rect, frac: float, font, label: str = "", dim: bool = False) -> None:
    """Rounded bar filled left to right with a blue-to-green gradient."""
    pygame.draw.rect(surface, BUTTON, rect, border_radius=rect.height // 2)
    w = int(rect.width * max(0.0, min(1.0, frac)))
    if w > 0:
        fill = pygame.Surface((w, rect.height), pygame.SRCALPHA)
        for x in range(w):
            c = _lerp(BAR_FROM, BAR_TO, x / max(1, rect.width - 1))
            if dim:
                c = _lerp(c, (60, 64, 72), 0.6)
            pygame.draw.line(fill, c, (x, 0), (x, rect.height))
        mask = pygame.Surface((w, rect.height), pygame.SRCALPHA)
        pygame.draw.rect(mask, (255, 255, 255, 255), mask.get_rect(), border_radius=rect.height // 2)
        fill.blit(mask, (0, 0), special_flags=pygame.BLEND_RGBA_MIN)
        surface.blit(fill, rect.topleft)
    if label:
        img = font.render(label, True, PANEL_FG)
        surface.blit(img, img.get_rect(center=rect.center))


def _text(surface, text, pos, font, color) -> int:
    surface.blit(font.render(text, True, color), pos)
    return pos[1] + font.get_linesize()


def _button(surface, rect, label, font, mouse, primary=False, enabled=True) -> None:
    from swarmpathfinding.viz.menu import _button as b

    b(surface, rect, label, font, mouse, primary, enabled)


def _label(name: str) -> str:
    from swarmpathfinding.viz.menu import planner_label

    return planner_label(name)


def fmt_s(seconds: float | None) -> str:
    if seconds is None:
        return "-"
    seconds = int(seconds)
    return f"{seconds // 60}:{seconds % 60:02d}" if seconds >= 60 else f"{seconds} s"


def fmt(v: float) -> str:
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return "-"
    if abs(v) >= 1000:
        return f"{v:,.0f}"
    if abs(v) >= 100:
        return f"{v:.0f}"
    return f"{v:.2f}" if abs(v) < 10 else f"{v:.1f}"


def draw_progress(surface, rect: pygame.Rect, runner: BenchmarkRunner, font, small, sub_bars: bool = True) -> int:
    """Overall bar ("29/150"), state line, then one thin bar per planner."""
    p = runner.progress
    y = rect.y
    state = "cancelled" if runner.cancelled else ("done" if runner.finished else ("paused" if runner.paused else "running"))
    mode = MODE_LABEL.get(runner.mode, runner.mode)
    elapsed = (p.finished or time.perf_counter()) - p.started
    _text(surface, f"{runner.spec.name}: {state}, {mode}", (rect.x, y), small, ACCENT)
    y += small.get_linesize() + 2
    gradient_bar(surface, pygame.Rect(rect.x, y, rect.width, 22), p.fraction, small, f"{p.done}/{p.total}", dim=runner.paused)
    y += 26
    info = f"elapsed {fmt_s(elapsed)}   ETA {fmt_s(p.eta_s())}"
    if p.skipped:
        info += f"   skipped {p.skipped} (planner cannot run it)"
    y = _text(surface, info, (rect.x, y), small, PANEL_MUTED)
    if runner.error:
        y = _text(surface, runner.error, (rect.x, y), small, FIT_CANNOT)
    if sub_bars:
        for i, (name, (done, total)) in enumerate(p.per_planner.items()):
            _text(surface, _label(name), (rect.x, y + 1), small, SERIES[i % len(SERIES)])
            bar = pygame.Rect(rect.x + 150, y + 3, rect.width - 150, 12)
            gradient_bar(surface, bar, done / total if total else 0, small, dim=runner.paused)
            img = small.render(f"{done}/{total}", True, PANEL_MUTED)
            surface.blit(img, (bar.right - img.get_width() - 6, y))
            y += 18
    return y


# ---------------------------------------------------------------- Benchmark tab


class BenchmarkTab:
    def __init__(self, menu: "MenuOverlay") -> None:
        self.menu = menu
        planners = list_planners()
        defaults = [p for p in ("weighted_a_star", "straight_line") if p in planners] or planners[:1]
        f = SidePanel(pygame.Rect(0, 0, 330, 400), menu.font, menu.small)
        f.title = None
        f.button_label = None
        f.edge_line = False
        f.sections = ["Run", "Scenarios", "Planners"]
        f.widgets = [
            Slider("n_scenarios", "Scenarios", 5, 500, 30, integer=True, section="Run"),
            Slider("base_seed", "Base seed", 1, 1000, 1, integer=True, section="Run"),
            Slider("time_limit", "Time limit per trial (s)", 10, 300, 60, integer=True, section="Run"),
            Cycle("mode", "Mode", list(MODES), 0, section="Run", display=MODE_LABEL),
            Chips("worlds", "World", ["grid", "continuous"], {"grid"}, section="Scenarios", display={"continuous": "polygons"}),
            Chips("grid_maps", "Grid maps", list(GRID_MAPS), {"bernoulli", "clustered", "rooms", "maze"}, section="Scenarios"),
            Chips("poly_maps", "Polygon maps", list(POLY_MAPS), {"polygons", "sparse", "clutter"}, section="Scenarios"),
            Chips("starts", "Starts", list(START_LAYOUTS), {"clump", "scattered"}, section="Scenarios"),
            Chips("fog", "Fog of war", [True, False], {True, False}, section="Scenarios", display={True: "fog", False: "full map"}),
            Chips("changes", "Changes", ["none", "slow_edit"], {"none"}, section="Scenarios", display={"slow_edit": "auto edit"}),
            Chips("vision", "Vision", ["circle", "cone"], {"circle"}, section="Scenarios"),
            Range("density", "Density", 0.02, 0.5, (0.05, 0.30), section="Scenarios"),
            Range("agents", "Agents", 1, 100, (1, 20), integer=True, section="Scenarios"),
            Range("vision_radius", "Vision radius", 2.0, 16.0, (4.0, 10.0), section="Scenarios"),
            Range("edit_at", "Edit at (s)", 0.5, 30.0, (1.0, 6.0), section="Scenarios"),
            Chips("planners", "Planners", planners, set(defaults), section="Planners", display={p: _label(p) for p in planners}),
            Slider("w", "Weight w (weighted A*)", 1.0, 3.0, 1.0, section="Planners"),
        ]
        f.set_on_change(self._on_change)
        self.form = f
        self._preview_key = None
        self._preview: tuple[int, int, list] = (0, 0, [])
        self._run_rect = pygame.Rect(0, 0, 0, 0)
        self._random_rect = pygame.Rect(0, 0, 0, 0)
        self._pause_rect = pygame.Rect(0, 0, 0, 0)
        self._cancel_rect = pygame.Rect(0, 0, 0, 0)
        self._results_rect = pygame.Rect(0, 0, 0, 0)
        self._sync()

    # ------------------------------------------------------------ spec

    def spec(self) -> ExperimentSpec:
        v = self.form.values()
        return ExperimentSpec(
            name=f"bench-s{int(v['base_seed'])}",
            n_scenarios=int(v["n_scenarios"]),
            base_seed=int(v["base_seed"]),
            planners=v["planners"],
            worlds=v["worlds"],
            grid_maps=v["grid_maps"],
            poly_maps=v["poly_maps"],
            starts=v["starts"],
            fog=v["fog"],
            changes=v["changes"],
            vision=v["vision"],
            density=tuple(v["density"]),
            agents=tuple(int(a) for a in v["agents"]),
            vision_radius=tuple(v["vision_radius"]),
            edit_at=tuple(v["edit_at"]),
            w=float(v["w"]),
            time_limit=float(v["time_limit"]),
        )

    def _on_change(self, key: str, final: bool) -> None:
        self._sync()

    def _sync(self) -> None:
        v = self.form.values()
        self.form.get("grid_maps").visible = "grid" in v["worlds"]
        self.form.get("poly_maps").visible = "continuous" in v["worlds"]
        self.form.get("edit_at").visible = "slow_edit" in v["changes"]
        self.form.get("w").visible = "weighted_a_star" in v["planners"]

    def preview(self) -> tuple[int, int, list]:
        """(trials, skipped, first scenarios) for the current spec, cached."""
        spec = self.spec()
        key = repr(spec.as_dict())
        if key != self._preview_key:
            trials = spec.trials() if not spec.problems() else []
            skipped = sum(t.skipped for t in trials)
            sample = []
            for i in range(min(6, spec.n_scenarios)):
                ts = [t for t in trials if t.scenario == i]
                if ts:
                    sample.append((i, ts[0].config, [t.planner for t in ts if not t.skipped]))
            self._preview = (len(trials), skipped, sample)
            self._preview_key = key
        return self._preview

    # ------------------------------------------------------------ actions

    def run(self) -> None:
        spec = self.spec()
        if spec.problems():
            return
        runner = self.menu.runner
        if runner is not None and runner.running:
            return
        mode = self.form.get("mode").value
        self.menu.start_benchmark(BenchmarkRunner(spec, mode="headless" if mode == "headless" else "visual"), mode)

    def load_random(self) -> None:
        spec = self.spec()
        if spec.problems():
            return
        i = random.randrange(spec.n_scenarios)
        cfg = spec.scenario(i)
        planner = self.menu.current_config().get("planner")
        if planner not in spec.planners:
            planner = spec.planners[0]
        self.menu.on_apply({**cfg.as_dict(), "planner": planner})
        self.menu.toast = f"Loaded scenario {i} of {spec.name} (seed {cfg.seed})"
        self.menu.close()

    def handle(self, event: pygame.event.Event) -> None:
        self.form.handle(event)
        if event.type != pygame.MOUSEBUTTONDOWN or event.button != 1:
            return
        runner = self.menu.runner
        pos = event.pos
        if self._run_rect.collidepoint(pos):
            self.run()
        elif self._random_rect.collidepoint(pos):
            self.load_random()
        elif runner is not None and self._pause_rect.collidepoint(pos) and runner.running:
            runner.pause(not runner.paused)
        elif runner is not None and self._cancel_rect.collidepoint(pos) and runner.running:
            runner.cancel()
        elif runner is not None and self._results_rect.collidepoint(pos):
            self.menu.tab = 3

    # ------------------------------------------------------------ drawing

    def draw(self, surface, area: pygame.Rect, mouse) -> None:
        m = self.menu
        form_w = 330
        self.form.rect = pygame.Rect(area.x, area.y, form_w, area.height)
        self.form.draw(surface)
        sep_x = area.x + form_w + 8
        pygame.draw.line(surface, CARD_EDGE, (sep_x, area.y), (sep_x, area.bottom), 1)
        r = pygame.Rect(sep_x + 14, area.y, area.right - sep_x - 14, area.height)
        spec = self.spec()
        problems = spec.problems()
        y = _text(surface, "PLAN", (r.x, r.y + 6), m.small, ACCENT) + 2
        if problems:
            for p in problems:
                y = _text(surface, f"Cannot run yet: {p}", (r.x, y), m.small, FIT_CANNOT)
        else:
            n, skipped, sample = self.preview()
            line = f"{spec.n_scenarios} scenarios x {len(spec.planners)} planners = {n} trials"
            y = _text(surface, line, (r.x, y), m.font, PANEL_FG)
            sub = f"same seeds for every planner (seed {spec.base_seed} to {spec.base_seed + spec.n_scenarios - 1})"
            if skipped:
                sub += f", {skipped} skipped where a planner cannot run"
            y = _text(surface, sub, (r.x, y), m.small, PANEL_MUTED)
            workers = f"{default_workers()} worker processes" if self.form.get("mode").value == "headless" else "drawn in the lab, one trial at a time"
            y = _text(surface, f"time limit {spec.time_limit:.0f} s per trial, {workers}", (r.x, y), m.small, PANEL_MUTED) + 6
            y = _text(surface, "FIRST SCENARIOS", (r.x, y), m.small, ACCENT) + 2
            for i, cfg, runs in sample:
                fog = "fog" if cfg.fow else "full"
                edit = f", edit at {cfg.edit_at:.1f} s" if cfg.changes == "slow_edit" else ""
                desc = f"#{i}  {cfg.map_kind} ({'grid' if cfg.world_kind == 'grid' else 'polygons'}), {cfg.n_agents} agents, {cfg.starts}, {fog}{edit}"
                y = _text(surface, desc, (r.x, y), m.small, PANEL_FG)
        y += 10
        runner = m.runner
        busy = runner is not None and runner.running
        self._run_rect = pygame.Rect(r.x, y, 190, 32)
        _button(surface, self._run_rect, "Run benchmark", m.small, mouse, primary=True, enabled=not problems and not busy)
        self._random_rect = pygame.Rect(self._run_rect.right + 10, y, 250, 32)
        _button(surface, self._random_rect, "Load a random scenario of it", m.small, mouse, enabled=not problems)
        y += 46
        pygame.draw.line(surface, CARD_EDGE, (r.x, y - 6), (r.right, y - 6), 1)
        if runner is None:
            _text(surface, "No benchmark has run yet in this session.", (r.x, y + 4), m.small, PANEL_MUTED)
            self._pause_rect = self._cancel_rect = self._results_rect = pygame.Rect(0, 0, 0, 0)
            return
        y = draw_progress(surface, pygame.Rect(r.x, y, r.width, 200), runner, m.font, m.small) + 8
        self._pause_rect = pygame.Rect(r.x, y, 110, 28)
        self._cancel_rect = pygame.Rect(r.x + 120, y, 110, 28)
        self._results_rect = pygame.Rect(r.x + 240, y, 150, 28)
        _button(surface, self._pause_rect, "Resume" if runner.paused else "Pause", m.small, mouse, enabled=busy)
        _button(surface, self._cancel_rect, "Cancel", m.small, mouse, enabled=busy)
        _button(surface, self._results_rect, "Results  (4)", m.small, mouse, primary=runner.finished)
        if runner.csv_path is not None:
            _text(surface, f"CSV: {runner.csv_path}", (r.x, y + 36), m.small, PANEL_MUTED)


# ---------------------------------------------------------------- Results tab


class ResultsTab:
    def __init__(self, menu: "MenuOverlay") -> None:
        self.menu = menu
        self.metric = Cycle("metric", "Plot", [k for k, _, _ in METRICS if k not in ("success",)], 1)
        self.metric.index = self.metric.options.index("makespan")
        self.metric.display = {k: lbl for k, lbl, _ in METRICS}
        self.loaded_rows: list[dict] | None = None
        self.loaded_from: str = ""
        self.list_scroll = 0
        self._list_rect = pygame.Rect(0, 0, 0, 0)
        self._list_h = 0
        self._trial_rows: list[tuple[pygame.Rect, int]] = []
        self._random_rect = pygame.Rect(0, 0, 0, 0)
        self._export_rect = pygame.Rect(0, 0, 0, 0)
        self._open_rect = pygame.Rect(0, 0, 0, 0)
        self.message = ""

    def rows(self) -> list[dict]:
        if self.menu.runner is not None and self.menu.runner.rows:
            return self.menu.runner.rows
        return self.loaded_rows or []

    def source(self) -> str:
        r = self.menu.runner
        if r is not None and r.rows:
            return f"{r.spec.name}, {r.progress.done}/{r.progress.total} trials" + ("" if r.finished else " so far")
        return self.loaded_from or "no results yet"

    def open_last_csv(self) -> None:
        files = sorted(Path(OUT_DIR).glob("*.csv")) if Path(OUT_DIR).exists() else []
        files = [f for f in files if not f.name.endswith("_summary.csv")]
        if not files:
            self.message = f"No CSV in {OUT_DIR}"
            return
        self.loaded_rows = A.read_rows(files[-1])
        self.loaded_from = files[-1].name
        if self.menu.runner is not None and not self.menu.runner.running:
            self.menu.runner = None
        self.message = f"Loaded {files[-1]}"

    def load_trial(self, scenario: int, planner: str | None = None) -> None:
        rows = [r for r in self.rows() if int(r["scenario"]) == scenario]
        if not rows:
            return
        win = A.winners(self.rows()).get(scenario)
        planner = planner or (win[0] if win else rows[0]["planner"])
        row = next((r for r in rows if r["planner"] == planner), rows[0])
        from swarmpathfinding.sim.simulation import SimConfig

        cfg = {}
        for k, f in SimConfig.__dataclass_fields__.items():
            if k in row and row[k] != "":
                cfg[k] = _coerce(row[k], f.default)
        cfg["planner"] = planner
        self.menu.on_apply(cfg)
        self.menu.toast = f"Loaded trial scenario {scenario} with {_label(planner)}"
        self.menu.close()

    def load_random(self) -> None:
        scs = sorted({int(r["scenario"]) for r in A.ran(self.rows())})
        if scs:
            self.load_trial(random.choice(scs))

    def handle(self, event: pygame.event.Event) -> None:
        if event.type == pygame.MOUSEWHEEL and self._list_rect.collidepoint(pygame.mouse.get_pos()):
            max_scroll = max(0, self._list_h - self._list_rect.height)
            self.list_scroll = min(max_scroll, max(0, self.list_scroll - event.y * 40))
        if event.type != pygame.MOUSEBUTTONDOWN or event.button != 1:
            return
        pos = event.pos
        if self.metric.rect.collidepoint(pos):
            self.metric.cycle(1 if pos[0] >= self.metric.rect.centerx else -1)
        elif self._random_rect.collidepoint(pos):
            self.load_random()
        elif self._export_rect.collidepoint(pos) and self.rows():
            r = self.menu.runner
            base = r.csv_path if r is not None and r.csv_path else Path(OUT_DIR) / "results.csv"
            out = A.write_summary_csv(self.rows(), base.with_name(base.stem + "_summary.csv"))
            self.message = f"Wrote {out}"
        elif self._open_rect.collidepoint(pos):
            self.open_last_csv()
        elif self._list_rect.collidepoint(pos):
            for rect, sc in self._trial_rows:
                if rect.collidepoint(pos):
                    self.load_trial(sc)

    # ------------------------------------------------------------ drawing

    def draw(self, surface, area: pygame.Rect, mouse) -> None:
        m = self.menu
        rows = self.rows()
        # top bar
        _text(surface, f"Results: {self.source()}", (area.x, area.y + 8), m.small, ACCENT)
        bx = area.right
        self._open_rect = pygame.Rect(bx - 150, area.y + 2, 150, 28)
        self._export_rect = pygame.Rect(self._open_rect.x - 160, area.y + 2, 150, 28)
        self._random_rect = pygame.Rect(self._export_rect.x - 180, area.y + 2, 170, 28)
        _button(surface, self._random_rect, "Load random trial", m.small, mouse, primary=True, enabled=bool(rows))
        _button(surface, self._export_rect, "Export summary", m.small, mouse, enabled=bool(rows))
        _button(surface, self._open_rect, "Open last CSV", m.small, mouse)
        top = area.y + 38
        if self.message:
            _text(surface, self.message, (area.x, top), m.small, PANEL_MUTED)
        top += 20
        if not rows:
            _text(surface, "Run a benchmark (tab 3) or open the last CSV to see results here.", (area.x, top + 10), m.font, PANEL_MUTED)
            return
        planners = A.planners_of(rows)
        colors = {p: SERIES[i % len(SERIES)] for i, p in enumerate(planners)}
        n = len(planners)
        line = m.small.get_linesize()
        summary_h = line * 2 + 8 + n * (line + 4)
        heat_h = 76 + n * 26
        self._draw_summary(surface, pygame.Rect(area.x, top, area.width, summary_h), rows, colors)
        y = top + summary_h + 6
        self._draw_heatmap(surface, pygame.Rect(area.x, y, area.width, heat_h), rows, planners, colors)
        y += heat_h + 6
        pygame.draw.line(surface, CARD_EDGE, (area.x, y - 4), (area.right, y - 4), 1)
        left_w = int(area.width * 0.5)
        bottom = pygame.Rect(area.x, y + 4, area.width, max(60, area.bottom - y - 4))
        self._draw_box(surface, pygame.Rect(bottom.x, bottom.y, left_w - 16, bottom.height), rows, planners, colors, mouse)
        self._draw_trials(surface, pygame.Rect(bottom.x + left_w, bottom.y, bottom.width - left_w, bottom.height), rows, colors, mouse)

    def _draw_summary(self, surface, rect, rows, colors) -> None:
        m = self.menu
        summ = A.summary(rows)
        cols = [("trials", "trials", None), ("wins", "wins", False), ("success", "success", False), ("makespan", "makespan s", True), ("path_len", "path", True), ("plan_ms", "plan ms", True), ("expansions", "expanded", True)]
        y = _text(surface, "SUMMARY  (means over trials that ran; failures count as the time limit)", (rect.x, rect.y), m.small, ACCENT) + 4
        name_w = 180
        col_w = (rect.width - name_w) // len(cols)
        for j, (_, lbl, _) in enumerate(cols):
            img = m.small.render(lbl, True, PANEL_MUTED)
            surface.blit(img, (rect.x + name_w + j * col_w + col_w - img.get_width() - 4, y))
        y += m.small.get_linesize() + 2
        best = {}
        for key, _, low in cols:
            if low is None:
                continue
            vals = [s[key] for s in summ.values() if not math.isnan(s[key])]
            if vals:
                best[key] = min(vals) if low else max(vals)
        for p, s in summ.items():
            if y > rect.bottom - 16:
                break
            pygame.draw.circle(surface, colors[p], (rect.x + 6, y + 8), 5)
            _text(surface, _label(p), (rect.x + 16, y), m.small, PANEL_FG)
            for j, (key, _, low) in enumerate(cols):
                v = s[key]
                txt = f"{v:.0%}" if key == "success" else (f"{v:g}" if key in ("trials", "wins") else fmt(v))
                color = ACCENT if key in best and not math.isnan(v) and abs(v - best[key]) < 1e-9 and len(summ) > 1 else PANEL_FG
                img = m.small.render(txt, True, color)
                surface.blit(img, (rect.x + name_w + j * col_w + col_w - img.get_width() - 4, y))
            y += m.small.get_linesize() + 4

    def _draw_heatmap(self, surface, rect, rows, planners, colors) -> None:
        m = self.menu
        mat = A.win_matrix(rows)
        y = _text(surface, "WHERE IT WINS  (share of those scenarios with the best makespan)", (rect.x, rect.y), m.small, ACCENT) + 4
        name_w = 180
        columns = [(t, v) for t, vals in TRAITS.items() for v in vals]
        col_w = (rect.width - name_w) // len(columns)
        gx = rect.x + name_w
        for trait, vals in TRAITS.items():
            gw = col_w * len(vals)
            img = m.small.render(TRAIT_LABELS[trait], True, PANEL_MUTED)
            surface.blit(img, img.get_rect(midtop=(gx + gw // 2, y)))
            pygame.draw.line(surface, CARD_EDGE, (gx + 4, y + 16), (gx + gw - 4, y + 16), 1)
            gx += gw
        y += 20
        for j, (t, v) in enumerate(columns):
            n = mat[(t, v)][planners[0]][1] if planners else 0
            img = m.small.render(SHORT[v], True, PANEL_FG if n else PANEL_MUTED)
            cx = rect.x + name_w + j * col_w + col_w // 2
            surface.blit(img, img.get_rect(midtop=(cx, y)))
            nimg = m.small.render(f"n={n}", True, PANEL_MUTED)
            surface.blit(nimg, nimg.get_rect(midtop=(cx, y + 15)))
        y += 34
        cell_h = 24
        for p in planners:
            if y + cell_h > rect.bottom:
                break
            pygame.draw.circle(surface, colors[p], (rect.x + 6, y + cell_h // 2), 5)
            _text(surface, _label(p), (rect.x + 16, y + (cell_h - 14) // 2), m.small, PANEL_FG)
            for j, col in enumerate(columns):
                share, n = mat[col][p]
                cell = pygame.Rect(rect.x + name_w + j * col_w + 1, y, col_w - 2, cell_h)
                if n == 0 or math.isnan(share):
                    pygame.draw.rect(surface, PANEL_BG, cell, border_radius=3)
                    continue
                pygame.draw.rect(surface, _lerp((30, 34, 44), (60, 170, 120), share), cell, border_radius=3)
                img = m.small.render(f"{share:.0%}", True, PANEL_FG)
                if img.get_width() < cell.width:
                    surface.blit(img, img.get_rect(center=cell.center))
            y += cell_h + 2

    def _draw_box(self, surface, rect, rows, planners, colors, mouse) -> None:
        m = self.menu
        _text(surface, "DISTRIBUTION OF", (rect.x, rect.y + 4), m.small, ACCENT)
        self.metric.rect = pygame.Rect(rect.x + 140, rect.y, 170, 24)
        _button(surface, self.metric.rect, self.metric.shown, m.small, mouse)
        lower = dict((k, low) for k, _, low in METRICS)[self.metric.value]
        _text(surface, "lower = better" if lower else "higher = better", (self.metric.rect.right + 12, rect.y + 4), m.small, PANEL_MUTED)
        key = self.metric.value
        data = {p: A.box_stats(A.metric_values(rows, p, key)) for p in planners}
        vals = [v for s in data.values() if s for v in (s[0], s[4])]
        plot = pygame.Rect(rect.x + 130, rect.y + 36, rect.width - 140, rect.height - 60)
        if not vals:
            _text(surface, "no data", (plot.x, plot.y), m.small, PANEL_MUTED)
            return
        lo, hi = min(vals), max(vals)
        if hi - lo < 1e-9:
            hi = lo + 1
        sx = lambda v: plot.x + int((v - lo) / (hi - lo) * plot.width)  # noqa: E731
        row_h = min(30, plot.height // max(1, len(planners)))
        for i, p in enumerate(planners):
            s = data[p]
            cy = plot.y + i * row_h + row_h // 2
            _text(surface, _label(p)[:14], (rect.x, cy - 8), m.small, colors[p])
            if s is None:
                continue
            mn, q1, md, q3, mx = s
            pygame.draw.line(surface, PANEL_MUTED, (sx(mn), cy), (sx(mx), cy), 1)
            for v in (mn, mx):
                pygame.draw.line(surface, PANEL_MUTED, (sx(v), cy - 5), (sx(v), cy + 5), 1)
            box = pygame.Rect(sx(q1), cy - row_h // 3, max(2, sx(q3) - sx(q1)), 2 * (row_h // 3))
            pygame.draw.rect(surface, _lerp(colors[p], (24, 27, 34), 0.55), box, border_radius=3)
            pygame.draw.rect(surface, colors[p], box, 1, border_radius=3)
            pygame.draw.line(surface, PANEL_FG, (sx(md), box.y), (sx(md), box.bottom), 2)
        # axis
        ay = plot.y + len(planners) * row_h + 4
        pygame.draw.line(surface, CARD_EDGE, (plot.x, ay), (plot.right, ay), 1)
        for t in (0, 0.5, 1):
            v = lo + t * (hi - lo)
            img = m.small.render(fmt(v), True, PANEL_MUTED)
            surface.blit(img, img.get_rect(midtop=(sx(v), ay + 2)))

    def _draw_trials(self, surface, rect, rows, colors, mouse) -> None:
        m = self.menu
        _text(surface, "SCENARIOS  (click one to load it with its winner)", (rect.x, rect.y + 4), m.small, ACCENT)
        self._list_rect = pygame.Rect(rect.x, rect.y + 26, rect.width, rect.height - 26)
        win = A.winners(rows)
        by_sc: dict[int, list[dict]] = {}
        for r in rows:
            by_sc.setdefault(int(r["scenario"]), []).append(r)
        prev = surface.get_clip()
        surface.set_clip(self._list_rect.clip(prev) if prev else self._list_rect)
        y = self._list_rect.y - self.list_scroll
        self._trial_rows = []
        line = m.small.get_linesize()
        for sc in sorted(by_sc):
            rs = by_sc[sc]
            r0 = rs[0]
            h = line * 2 + 6
            box = pygame.Rect(rect.x, y, rect.width - 6, h)
            if box.collidepoint(mouse) and self._list_rect.collidepoint(mouse):
                pygame.draw.rect(surface, ROW_HOT, box, border_radius=4)
            world = "grid" if r0["world_kind"] == "grid" else "polygons"
            fog = "fog" if str(r0["fow"]) in ("True", "1", "true") else "full"
            _text(surface, f"#{sc}  {r0['map_kind']} ({world}), {r0['n_agents']} agents, {r0['starts']}, {fog}, seed {r0['seed']}", (box.x + 6, box.y + 3), m.small, PANEL_FG)
            x = box.x + 6
            for r in rs:
                if int(r.get("skipped", 0)):
                    txt, color = f"{_label(r['planner'])}: n/a", PANEL_MUTED
                else:
                    ok = int(r["success"])
                    txt = f"{_label(r['planner'])}: {float(r['makespan']):.1f} s" if ok else f"{_label(r['planner'])}: fail"
                    color = FIT_OK if r["planner"] in win.get(sc, []) else (PANEL_MUTED if ok else FIT_CANNOT)
                img = m.small.render(txt, True, color)
                surface.blit(img, (x, box.y + 3 + line))
                x += img.get_width() + 14
            self._trial_rows.append((box, sc))
            y += h + 2
        self._list_h = y + self.list_scroll - self._list_rect.y
        surface.set_clip(prev)


def _coerce(text, default):
    if isinstance(text, str):
        if isinstance(default, bool):
            return text in ("True", "true", "1")
        if isinstance(default, int):
            return int(float(text))
        if isinstance(default, float):
            return float(text)
    return text

