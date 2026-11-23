"""Popup lab menu: a card over the paused live view.

Tabs:
  1 Scenario  - pick scenario traits on the left, see which planners can run it
                on the right (with how well they fit and why), then Apply.
  2 Planners  - every planner against every trait value, with the notes that
                explain each adaptation, and a card for the selected planner.

M opens it, Esc / a click outside the card closes it, 1-2 switch tabs.
"""

from __future__ import annotations

import random
from typing import Any, Callable

import pygame

from swarmpathfinding.core.capabilities import (
    TRAIT_LABELS,
    TRAITS,
    VALUE_LABELS,
    Fit,
    compatibility,
    matrix,
    scenario_traits,
)
from swarmpathfinding.core.registry import _REGISTRY, ensure_builtin_planners, list_planners
from swarmpathfinding.generators.scenarios import GRID_MAPS, POLY_MAPS, START_LAYOUTS
from swarmpathfinding.sim.simulation import CHANGES
from swarmpathfinding.viz.colors import (
    ACCENT,
    BUTTON,
    BUTTON_DISABLED,
    BUTTON_HOT,
    CARD_BG,
    CARD_EDGE,
    FIT_ADAPTED,
    FIT_CANNOT,
    FIT_DESIGNED,
    FIT_OK,
    MENU_DIM,
    PANEL_BG,
    PANEL_FG,
    PANEL_MUTED,
    ROW_HOT,
    ROW_SELECTED,
    TOGGLE_ON,
)
from swarmpathfinding.viz.panel import Check, Cycle, SidePanel, Slider

TABS = ("Scenario", "Planners", "Benchmark", "Results")
FIT_LABEL = {Fit.DESIGNED: "designed for it", Fit.OK: "works as is", Fit.ADAPTED: "adapted", Fit.CANNOT: "cannot run"}
FIT_COLOR = {Fit.DESIGNED: FIT_DESIGNED, Fit.OK: FIT_OK, Fit.ADAPTED: FIT_ADAPTED, Fit.CANNOT: FIT_CANNOT}
# Short matrix column headers, in TRAITS order.
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
DENSITY_MAPS = {"bernoulli", "clustered", "open", "polygons", "sparse", "clutter"}

AVOID_LABEL = {"none": "none", "orca": "ORCA (agents dodge)"}

# Keys of the scenario form, i.e. the SimConfig fields the menu controls.
FORM_KEYS = (
    "world_kind",
    "map_kind",
    "starts",
    "density",
    "n_agents",
    "seed",
    "fow",
    "vision",
    "vision_radius",
    "los",
    "changes",
    "edit_at",
    "avoidance",
)

ApplyFn = Callable[[dict], None]


# ---------------------------------------------------------------- drawing helpers


def draw_badge(surface: pygame.Surface, fit: Fit, center: tuple[int, int], size: int = 14) -> None:
    """A shape per fit level, so the matrix reads without colour too:
    star = designed for it, check = works, tilde = adapted, cross = cannot."""
    cx, cy = center
    r = size // 2
    color = FIT_COLOR[fit]
    if fit == Fit.DESIGNED:
        import math

        pts = []
        for i in range(10):
            ang = -math.pi / 2 + i * math.pi / 5
            rad = r if i % 2 == 0 else r * 0.45
            pts.append((cx + rad * math.cos(ang), cy + rad * math.sin(ang)))
        pygame.draw.polygon(surface, color, pts)
    elif fit == Fit.OK:
        pygame.draw.lines(surface, color, False, [(cx - r + 1, cy), (cx - 1, cy + r - 2), (cx + r, cy - r + 1)], 3)
    elif fit == Fit.ADAPTED:
        pygame.draw.circle(surface, color, center, r, 2)
        pts = [(cx - r + 3, cy + 1), (cx - 2, cy - 2), (cx + 2, cy + 2), (cx + r - 3, cy - 1)]
        pygame.draw.lines(surface, color, False, pts, 2)
    else:
        pygame.draw.line(surface, color, (cx - r + 2, cy - r + 2), (cx + r - 2, cy + r - 2), 3)
        pygame.draw.line(surface, color, (cx - r + 2, cy + r - 2), (cx + r - 2, cy - r + 2), 3)


def wrap(text: str, font: pygame.font.Font, width: int) -> list[str]:
    lines: list[str] = []
    for para in text.split("\n"):
        line = ""
        for word in para.split():
            trial = f"{line} {word}".strip()
            if font.size(trial)[0] <= width or not line:
                line = trial
            else:
                lines.append(line)
                line = word
        lines.append(line)
    return lines


def _text(surface, text, pos, font, color) -> int:
    img = font.render(text, True, color)
    surface.blit(img, pos)
    return pos[1] + font.get_linesize()


def _button(surface, rect: pygame.Rect, label: str, font, mouse, primary=False, enabled=True) -> None:
    if not enabled:
        color = BUTTON_DISABLED
    elif rect.collidepoint(mouse):
        color = ACCENT if primary else BUTTON_HOT
    else:
        color = BUTTON_HOT if primary else BUTTON
    pygame.draw.rect(surface, color, rect, border_radius=4)
    img = font.render(label, True, PANEL_FG if enabled else PANEL_MUTED)
    surface.blit(img, img.get_rect(center=rect.center))


def planner_class(name: str):
    ensure_builtin_planners()
    return _REGISTRY.get(name)


def planner_label(name: str) -> str:
    base, _, avoid = name.partition("+")  # benchmark variants like "a_star+orca"
    cls = planner_class(base)
    label = getattr(cls, "label", None) or base.replace("_", " ")
    return f"{label} + {avoid.upper()}" if avoid else label


# ---------------------------------------------------------------- tabs


class ScenarioTab:
    """Traits form (left) and the planners that can run it (right)."""

    def __init__(self, menu: "MenuOverlay") -> None:
        self.menu = menu
        self.form = SidePanel(pygame.Rect(0, 0, 300, 400), menu.font, menu.small)
        self.form.title = None
        self.form.button_label = None
        self.form.edge_line = False
        self.form.sections = ["World", "Swarm", "Changes"]
        self.form.widgets = [
            Cycle("world_kind", "World", ["grid", "continuous"], 0, section="World"),
            Cycle("map_kind", "Map", list(GRID_MAPS), 1, section="World"),
            Slider("density", "Density", 0.02, 0.50, 0.15, section="World"),
            Cycle("starts", "Starts", list(START_LAYOUTS), 0, section="World"),
            Slider("seed", "Seed", 1, 50, 1, integer=True, section="World"),
            Slider("n_agents", "Agents", 1, 100, 8, integer=True, section="Swarm"),
            Check("fow", "Fog of war", True, section="Swarm"),
            Cycle("vision", "Vision", ["circle", "cone", "none"], 0, section="Swarm"),
            Slider("vision_radius", "Vision radius", 2.0, 16.0, 6.0, section="Swarm"),
            Check("los", "Vision needs LOS", True, section="Swarm", hint="grid only"),
            Cycle(
                "changes",
                "Changes",
                list(CHANGES),
                0,
                section="Changes",
                display={"none": "none (E = manual)", "slow_edit": "auto edit"},
            ),
            Slider("edit_at", "Edit at (s)", 0.5, 30.0, 3.0, section="Changes"),
            Cycle("avoidance", "Avoidance", ["none", "orca"], 0, section="Swarm", display=AVOID_LABEL),
        ]
        self.form.set_on_change(self._on_change)
        self.planner = "weighted_a_star"
        self.compare: list[str] = []  # 2-4 ticked planners open compare panes
        self._rows: list[tuple[pygame.Rect, str, bool]] = []
        self._ticks: list[tuple[pygame.Rect, str]] = []
        self._apply_rect = pygame.Rect(0, 0, 0, 0)
        self._random_rect = pygame.Rect(0, 0, 0, 0)
        self._cancel_rect = pygame.Rect(0, 0, 0, 0)
        self.list_scroll = 0
        self._list_rect = pygame.Rect(0, 0, 0, 0)
        self._list_h = 0

    # ------------------------------------------------------------ values

    def load(self, config: dict, planner: str) -> None:
        set_form_values(self.form, config)
        self.planner = planner
        self._sync()

    def values(self) -> dict:
        vals = self.form.values()
        return {k: vals[k] for k in FORM_KEYS if k in vals}

    def traits(self) -> dict[str, str]:
        return scenario_traits(self.values())

    def randomise(self, rng: random.Random | None = None) -> None:
        rng = rng or random.Random()
        world = rng.choice(["grid", "continuous"])
        maps = list(GRID_MAPS) if world == "grid" else list(POLY_MAPS)
        vals = {
            "world_kind": world,
            "map_kind": rng.choice(maps),
            "density": round(rng.uniform(0.05, 0.35), 2),
            "starts": rng.choice(list(START_LAYOUTS)),
            "seed": rng.randint(1, 50),
            "n_agents": rng.choice([1, rng.randint(2, 10), rng.randint(11, 60)]),
            "fow": rng.random() < 0.6,
            "changes": rng.choice(list(CHANGES)),
            "edit_at": round(rng.uniform(1.0, 8.0), 1),
        }
        set_form_values(self.form, vals)
        self._sync()

    def _on_change(self, key: str, final: bool) -> None:
        if key == "world_kind":
            swap_map_options(self.form)
        self._sync()

    def _sync(self) -> None:
        vals = self.form.values()
        self.form.get("density").visible = vals["map_kind"] in DENSITY_MAPS
        self.form.get("los").enabled = vals["world_kind"] == "grid"
        self.form.get("vision_radius").enabled = vals["vision"] != "none"
        self.form.get("edit_at").visible = vals["changes"] == "slow_edit"
        if not compatibility(self.planner, self.traits()).runnable:
            runnable = [p for p in list_planners() if compatibility(p, self.traits()).runnable]
            if runnable:
                self.planner = runnable[0]
        self.compare = [p for p in self.compare if compatibility(p, self.traits()).runnable]

    def toggle_compare(self, name: str) -> None:
        if name in self.compare:
            self.compare.remove(name)
        elif len(self.compare) < 4:
            self.compare.append(name)

    def compare_list(self) -> list[str]:
        """Planners for compare panes, the picked planner first (empty = one pane)."""
        if len(self.compare) < 2:
            return []
        return sorted(self.compare, key=lambda p: p != self.planner)

    # ------------------------------------------------------------ events

    def handle(self, event: pygame.event.Event) -> None:
        self.form.handle(event)
        if event.type == pygame.MOUSEWHEEL and self._list_rect.collidepoint(pygame.mouse.get_pos()):
            max_scroll = max(0, self._list_h - self._list_rect.height)
            self.list_scroll = min(max_scroll, max(0, self.list_scroll - event.y * 40))
        if event.type == pygame.KEYDOWN and event.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
            self.apply()
        if event.type != pygame.MOUSEBUTTONDOWN or event.button != 1:
            return
        if self._apply_rect.collidepoint(event.pos):
            self.apply()
        elif self._random_rect.collidepoint(event.pos):
            self.randomise()
        elif self._cancel_rect.collidepoint(event.pos):
            self.menu.close()
        else:
            for rect, name in self._ticks:
                if rect.collidepoint(event.pos):
                    self.toggle_compare(name)
                    return
            for rect, name, runnable in self._rows:
                if runnable and rect.collidepoint(event.pos):
                    self.planner = name

    def apply(self) -> None:
        if not compatibility(self.planner, self.traits()).runnable:
            return
        compare = self.compare_list()
        if compare:
            self.menu.on_apply({**self.values(), "planner": compare[0], "compare": compare})
        else:
            self.menu.on_apply({**self.values(), "planner": self.planner})
        self.menu.close()

    # ------------------------------------------------------------ drawing

    def draw(self, surface: pygame.Surface, area: pygame.Rect, mouse) -> None:
        m = self.menu
        form_w = 300
        footer_h = 44
        self.form.rect = pygame.Rect(area.x, area.y, form_w, area.height - footer_h)
        self.form.draw(surface)
        pygame.draw.line(surface, CARD_EDGE, (area.x + form_w + 8, area.y), (area.x + form_w + 8, area.bottom - footer_h), 1)

        right = pygame.Rect(area.x + form_w + 22, area.y, area.width - form_w - 22, area.height - footer_h)
        y = _text(surface, "THIS SCENARIO", (right.x, right.y + 6), m.small, ACCENT) + 2
        y = self._draw_chips(surface, right.x, y, right.width) + 8
        y = _text(surface, "PLANNERS  (click one to pick it; tick 2-4 to compare them side by side)", (right.x, y), m.small, ACCENT) + 4
        self._list_rect = pygame.Rect(right.x, y, right.width, right.bottom - y)
        self._draw_list(surface, self._list_rect, mouse)

        # footer
        fy = area.bottom - footer_h + 8
        pygame.draw.line(surface, CARD_EDGE, (area.x, fy - 6), (area.right, fy - 6), 1)
        runnable = compatibility(self.planner, self.traits()).runnable
        self._cancel_rect = pygame.Rect(area.right - 110, fy, 110, 30)
        n_cmp = len(self.compare_list())
        label = f"Compare {n_cmp} planners side by side  (Enter)" if n_cmp else f"Apply with {planner_label(self.planner)}  (Enter)"
        aw = m.small.size(label)[0] + 32
        self._apply_rect = pygame.Rect(self._cancel_rect.x - 10 - aw, fy, aw, 30)
        self._random_rect = pygame.Rect(area.x, fy, 150, 30)
        _button(surface, self._random_rect, "Randomise", m.small, mouse)
        _button(surface, self._apply_rect, label, m.small, mouse, primary=True, enabled=runnable)
        _button(surface, self._cancel_rect, "Cancel", m.small, mouse)

    def _draw_chips(self, surface, x, y, width) -> int:
        m = self.menu
        cx = x
        for trait, value in self.traits().items():
            text = f"{TRAIT_LABELS[trait]}: {VALUE_LABELS[value]}"
            img = m.small.render(text, True, PANEL_FG)
            w = img.get_width() + 14
            if cx + w > x + width:
                cx = x
                y += 26
            rect = pygame.Rect(cx, y, w, 22)
            pygame.draw.rect(surface, BUTTON, rect, border_radius=11)
            surface.blit(img, img.get_rect(center=rect.center))
            cx += w + 6
        return y + 26

    def _draw_list(self, surface, area: pygame.Rect, mouse) -> None:
        m = self.menu
        traits = self.traits()
        rows = []
        for name in list_planners():
            comp = compatibility(name, traits)
            rows.append((name, comp))
        # best fit first, then name
        rows.sort(key=lambda r: (-int(r[1].fit), r[0]))
        prev_clip = surface.get_clip()
        surface.set_clip(area)
        self._rows = []
        self._ticks = []
        y = area.y - self.list_scroll
        line = m.small.get_linesize()
        for name, comp in rows:
            notes = comp.notes()
            note_lines = [ln for n in notes for ln in wrap(f"- {n}", m.small, area.width - 44)]
            h = 30 + len(note_lines) * line + (6 if note_lines else 0)
            rect = pygame.Rect(area.x, y, area.width - 8, h)
            selected = name == self.planner
            if selected:
                pygame.draw.rect(surface, ROW_SELECTED, rect, border_radius=5)
                pygame.draw.rect(surface, ACCENT, rect, 1, border_radius=5)
            elif comp.runnable and rect.collidepoint(mouse) and area.collidepoint(mouse):
                pygame.draw.rect(surface, ROW_HOT, rect, border_radius=5)
            draw_badge(surface, comp.fit, (rect.x + 16, rect.y + 15), 16)
            fg = PANEL_FG if comp.runnable else PANEL_MUTED
            _text(surface, planner_label(name), (rect.x + 34, rect.y + 6), m.font, fg)
            tag = m.small.render(FIT_LABEL[comp.fit], True, FIT_COLOR[comp.fit])
            surface.blit(tag, (rect.right - tag.get_width() - 10, rect.y + 9))
            if comp.runnable:
                tick = pygame.Rect(rect.right - 150 - 96, rect.y + 7, 16, 16)
                on = name in self.compare
                pygame.draw.rect(surface, TOGGLE_ON if on else BUTTON, tick, border_radius=3)
                if tick.inflate(70, 8).collidepoint(mouse):
                    pygame.draw.rect(surface, ACCENT, tick, 1, border_radius=3)
                if on:
                    pygame.draw.lines(surface, PANEL_FG, False, [(tick.x + 3, tick.y + 8), (tick.x + 6, tick.y + 12), (tick.x + 13, tick.y + 4)], 2)
                surface.blit(m.small.render("compare", True, PANEL_MUTED), (tick.right + 6, tick.y))
                self._ticks.append((pygame.Rect(tick.x - 4, tick.y - 4, 96, 24), name))
            ny = rect.y + 30
            for ln in note_lines:
                ny = _text(surface, ln, (rect.x + 34, ny), m.small, PANEL_MUTED)
            self._rows.append((rect, name, comp.runnable))
            y += h + 4
        self._list_h = y + self.list_scroll - area.y
        surface.set_clip(prev_clip)


class PlannersTab:
    """Planner x trait-value matrix, hover notes, and a selected-planner card."""

    def __init__(self, menu: "MenuOverlay") -> None:
        self.menu = menu
        self.selected = "weighted_a_star"
        self.filter = Cycle("filter", "Designed for", ["anything", *[v for vals in TRAITS.values() for v in vals]], 0)
        self.filter.display = {"anything": "any trait", **{v: VALUE_LABELS[v] for vals in TRAITS.values() for v in vals}}
        self._cells: list[tuple[pygame.Rect, str, str, str]] = []  # rect, planner, trait, value
        self._rows: list[tuple[pygame.Rect, str]] = []
        self._use_rect = pygame.Rect(0, 0, 0, 0)
        self.hover: tuple[str, str, str] | None = None

    def planners(self) -> list[str]:
        names = list_planners()
        f = self.filter.value
        if f == "anything":
            return names
        trait = next(t for t, vals in TRAITS.items() if f in vals)
        mat = matrix(names)
        return [n for n in names if mat[n][(trait, f)].fit == Fit.DESIGNED]

    def handle(self, event: pygame.event.Event) -> None:
        if event.type == pygame.MOUSEMOTION:
            self.hover = None
            for rect, name, trait, value in self._cells:
                if rect.collidepoint(event.pos):
                    self.hover = (name, trait, value)
        if event.type != pygame.MOUSEBUTTONDOWN or event.button != 1:
            return
        if self.filter.rect.collidepoint(event.pos):
            self.filter.cycle(1 if event.pos[0] >= self.filter.rect.centerx else -1)
            return
        if self._use_rect.collidepoint(event.pos):
            self.use_selected()
            return
        for rect, name in self._rows:
            if rect.collidepoint(event.pos):
                self.selected = name

    def use_selected(self) -> None:
        traits = scenario_traits(self.menu.current_config())
        if compatibility(self.selected, traits).runnable:
            self.menu.on_apply({"planner": self.selected})
            self.menu.close()

    # ------------------------------------------------------------ drawing

    def draw(self, surface: pygame.Surface, area: pygame.Rect, mouse) -> None:
        m = self.menu
        names = self.planners()
        live = scenario_traits(self.menu.current_config())
        mat = matrix(list_planners())

        # filter row
        _text(surface, "Show planners designed for", (area.x, area.y + 8), m.small, PANEL_MUTED)
        self.filter.rect = pygame.Rect(area.x + 230, area.y + 4, 170, 24)
        _button(surface, self.filter.rect, self.filter.shown, m.small, mouse)
        legend_x = self.filter.rect.right + 24
        for fit in (Fit.DESIGNED, Fit.OK, Fit.ADAPTED, Fit.CANNOT):
            draw_badge(surface, fit, (legend_x + 7, area.y + 16), 14)
            legend_x = _text_w(surface, FIT_LABEL[fit], (legend_x + 18, area.y + 8), m.small, PANEL_MUTED) + 18

        # matrix
        name_w = min(160, area.width // 5)
        columns = [(t, v) for t, vals in TRAITS.items() for v in vals]
        card_h = 150
        top = area.y + 44
        col_w = (area.width - name_w) // len(columns)
        avail = (area.bottom - card_h - 34) - (top + 48)
        row_h = max(16, min(28, avail // max(1, len(names)) - 2))
        # trait group headers
        x = area.x + name_w
        for trait, vals in TRAITS.items():
            gw = col_w * len(vals)
            img = m.small.render(TRAIT_LABELS[trait], True, ACCENT)
            surface.blit(img, img.get_rect(midtop=(x + gw // 2, top)))
            pygame.draw.line(surface, CARD_EDGE, (x + 4, top + 18), (x + gw - 4, top + 18), 1)
            x += gw
        x = area.x + name_w
        for trait, value in columns:
            live_col = live.get(trait) == value
            rect = pygame.Rect(x, top + 22, col_w, 20)
            if live_col:
                pygame.draw.rect(surface, ROW_SELECTED, rect.inflate(0, 0), border_radius=3)
            img = m.small.render(SHORT[value], True, PANEL_FG if live_col else PANEL_MUTED)
            surface.blit(img, img.get_rect(center=rect.center))
            x += col_w
        y = top + 48
        self._cells = []
        self._rows = []
        max_rows_bottom = area.bottom - card_h - 34
        for name in names:
            if y + row_h > max_rows_bottom:
                break
            row = pygame.Rect(area.x, y, name_w + col_w * len(columns), row_h)
            if name == self.selected:
                pygame.draw.rect(surface, ROW_SELECTED, row, border_radius=4)
                pygame.draw.rect(surface, ACCENT, row, 1, border_radius=4)
            elif row.collidepoint(mouse):
                pygame.draw.rect(surface, ROW_HOT, row, border_radius=4)
            fits = compatibility(name, live)
            draw_badge(surface, fits.fit, (row.x + 12, row.centery), 12)
            _text(surface, planner_label(name), (row.x + 26, row.y + 6), m.small, PANEL_FG if fits.runnable else PANEL_MUTED)
            x = area.x + name_w
            for trait, value in columns:
                cell = pygame.Rect(x, y, col_w, row_h)
                if live.get(trait) == value:
                    pygame.draw.rect(surface, (34, 44, 58), cell.inflate(-4, -4), border_radius=3)
                draw_badge(surface, mat[name][(trait, value)].fit, cell.center, 14)
                if self.hover == (name, trait, value):
                    pygame.draw.rect(surface, ACCENT, cell.inflate(-2, -2), 1, border_radius=3)
                self._cells.append((cell, name, trait, value))
                x += col_w
            self._rows.append((row, name))
            y += row_h + 2
        if not names:
            _text(surface, "No planner is designed for this yet.", (area.x, y + 4), m.small, PANEL_MUTED)
            y += 26

        # hover note
        note_y = max_rows_bottom + 4
        if self.hover is not None:
            name, trait, value = self.hover
            r = mat[name][(trait, value)]
            note = r.note or {Fit.DESIGNED: "the case it was built for", Fit.OK: "works as is"}.get(r.fit, "")
            text = f"{planner_label(name)} on {VALUE_LABELS[value]} ({TRAIT_LABELS[trait].lower()}): {FIT_LABEL[r.fit]}. {note}"
            for ln in wrap(text, m.small, area.width)[:2]:
                note_y = _text(surface, ln, (area.x, note_y), m.small, PANEL_FG)
        else:
            _text(surface, "Hover a cell to see how the planner handles that case. Highlighted column = current scenario.", (area.x, note_y), m.small, PANEL_MUTED)

        self._draw_card(surface, pygame.Rect(area.x, area.bottom - card_h, area.width, card_h), live, mouse)

    def _draw_card(self, surface, rect: pygame.Rect, live: dict, mouse) -> None:
        m = self.menu
        pygame.draw.rect(surface, PANEL_BG, rect, border_radius=6)
        pygame.draw.rect(surface, CARD_EDGE, rect, 1, border_radius=6)
        cls = planner_class(self.selected)
        x = rect.x + 14
        y = _text(surface, planner_label(self.selected), (x, rect.y + 10), m.font, ACCENT)
        text_w = rect.width - 28 - 230
        for ln in wrap(getattr(cls, "summary", "") or "No summary yet.", m.small, text_w)[:3]:
            y = _text(surface, ln, (x, y), m.small, PANEL_FG)
        params = getattr(cls, "params", ()) or ()
        ptxt = ", ".join(f"{p[0]} = {p[1]} ({p[2]:g} to {p[3]:g})" for p in params) or "none"
        y = _text(surface, f"Parameters: {ptxt}", (x, y + 4), m.small, PANEL_MUTED)
        overlays = getattr(cls, "overlays", ()) or ()
        for ln in wrap(f"Debug views: {', '.join(overlays) or 'none yet'}", m.small, rect.width - 28)[:3]:
            y = _text(surface, ln, (x, y), m.small, PANEL_MUTED)
        comp = compatibility(self.selected, live)
        self._use_rect = pygame.Rect(rect.right - 224, rect.y + 12, 210, 30)
        _button(surface, self._use_rect, "Use in current scenario", m.small, mouse, primary=True, enabled=comp.runnable)
        draw_badge(surface, comp.fit, (rect.right - 210, rect.y + 62), 14)
        _text(surface, f"here: {FIT_LABEL[comp.fit]}", (rect.right - 196, rect.y + 54), m.small, FIT_COLOR[comp.fit])


def _text_w(surface, text, pos, font, color) -> int:
    img = font.render(text, True, color)
    surface.blit(img, pos)
    return pos[0] + img.get_width()


# ---------------------------------------------------------------- overlay


class MenuOverlay:
    def __init__(
        self,
        font: pygame.font.Font,
        small: pygame.font.Font,
        on_apply: ApplyFn,
        current_config: Callable[[], dict[str, Any]],
    ) -> None:
        self.font = font
        self.small = small
        self.on_apply = on_apply
        self.current_config = current_config
        self.is_open = False
        self.tab = 0
        self.card = pygame.Rect(0, 0, 0, 0)
        self._tab_rects: list[pygame.Rect] = []
        self._close_rect = pygame.Rect(0, 0, 0, 0)
        self.scenario = ScenarioTab(self)
        self.planners = PlannersTab(self)
        self.on_close: Callable[[], None] | None = None
        # benchmark state lives here so it survives closing the menu
        self.runner = None  # BenchmarkRunner | None
        self.on_bench_start: Callable[[Any, str], None] | None = None
        self.toast = ""  # one-line note the live view shows after the menu closes
        from swarmpathfinding.viz.menu_bench import BenchmarkTab, ResultsTab

        self.benchmark = BenchmarkTab(self)
        self.results = ResultsTab(self)

    def start_benchmark(self, runner, mode: str) -> None:
        """mode: 'headless' or a visual speed ('x4', 'x16', 'max')."""
        if self.runner is not None and self.runner.running:
            self.runner.cancel()
        self.runner = runner
        if self.on_bench_start:
            self.on_bench_start(runner, mode)

    @property
    def tabs(self) -> list:
        return [self.scenario, self.planners, self.benchmark, self.results]

    def open(self, tab: int | None = None) -> None:
        cfg = self.current_config()
        planner = cfg.get("planner", "weighted_a_star")
        self.scenario.load(cfg, planner)
        self.planners.selected = planner
        if tab is not None:
            self.tab = tab
        self.is_open = True

    def close(self) -> None:
        if not self.is_open:
            return
        self.is_open = False
        if self.on_close:
            self.on_close()

    def layout(self, screen_size: tuple[int, int]) -> None:
        sw, sh = screen_size
        w = max(min(int(sw * 0.8), 1180), min(sw - 20, 760))
        h = max(min(int(sh * 0.82), 760), min(sh - 20, 520))
        self.card = pygame.Rect((sw - w) // 2, (sh - h) // 2, w, h)

    def handle(self, event: pygame.event.Event) -> None:
        if event.type == pygame.KEYDOWN:
            if event.key in (pygame.K_ESCAPE, pygame.K_m):
                self.close()
                return
            if pygame.K_1 <= event.key <= pygame.K_1 + len(TABS) - 1:
                self.tab = event.key - pygame.K_1
                return
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if not self.card.collidepoint(event.pos) or self._close_rect.collidepoint(event.pos):
                self.close()
                return
            for i, rect in enumerate(self._tab_rects):
                if rect.collidepoint(event.pos):
                    self.tab = i
                    return
        self.tabs[self.tab].handle(event)

    def draw(self, surface: pygame.Surface) -> None:
        if not self.is_open:
            return
        self.layout(surface.get_size())
        mouse = pygame.mouse.get_pos()
        dim = pygame.Surface(surface.get_size(), pygame.SRCALPHA)
        dim.fill(MENU_DIM)
        surface.blit(dim, (0, 0))
        card = self.card
        shadow = card.move(0, 6)
        pygame.draw.rect(surface, (0, 0, 0), shadow, border_radius=10)
        pygame.draw.rect(surface, CARD_BG, card, border_radius=10)
        pygame.draw.rect(surface, CARD_EDGE, card, 1, border_radius=10)

        # header: title, tabs, close hint
        x = card.x + 20
        title = self.font.render("Lab menu", True, ACCENT)
        surface.blit(title, (x, card.y + 14))
        tx = x + title.get_width() + 28
        self._tab_rects = []
        for i, name in enumerate(TABS):
            img = self.small.render(f"{i + 1}  {name}", True, PANEL_FG if i == self.tab else PANEL_MUTED)
            rect = pygame.Rect(tx, card.y + 10, img.get_width() + 28, 30)
            if i == self.tab:
                pygame.draw.rect(surface, BUTTON, rect, border_radius=5)
                pygame.draw.line(surface, ACCENT, (rect.x + 6, rect.bottom - 2), (rect.right - 6, rect.bottom - 2), 2)
            elif rect.collidepoint(mouse):
                pygame.draw.rect(surface, ROW_HOT, rect, border_radius=5)
            surface.blit(img, img.get_rect(center=rect.center))
            self._tab_rects.append(rect)
            tx = rect.right + 6
        hint = self.small.render("Esc closes  ·  simulation paused", True, PANEL_MUTED)
        self._close_rect = pygame.Rect(card.right - 40, card.y + 10, 30, 30)
        surface.blit(hint, (self._close_rect.x - hint.get_width() - 14, card.y + 17))
        hot = self._close_rect.collidepoint(mouse)
        pygame.draw.rect(surface, BUTTON_HOT if hot else BUTTON, self._close_rect, border_radius=5)
        c = self._close_rect.center
        pygame.draw.line(surface, PANEL_FG, (c[0] - 6, c[1] - 6), (c[0] + 6, c[1] + 6), 2)
        pygame.draw.line(surface, PANEL_FG, (c[0] - 6, c[1] + 6), (c[0] + 6, c[1] - 6), 2)
        pygame.draw.line(surface, CARD_EDGE, (card.x, card.y + 50), (card.right, card.y + 50), 1)

        footer = 0
        if self.runner is not None and self.tab != 2:
            footer = 34
            from swarmpathfinding.viz.menu_bench import gradient_bar

            p = self.runner.progress
            fy = card.bottom - 30
            pygame.draw.line(surface, CARD_EDGE, (card.x, fy - 6), (card.right, fy - 6), 1)
            state = "done" if self.runner.finished else ("cancelled" if self.runner.cancelled else ("paused" if self.runner.paused else "running"))
            label = self.small.render(f"Benchmark {state}", True, PANEL_MUTED)
            surface.blit(label, (card.x + 20, fy + 2))
            bar = pygame.Rect(card.x + 40 + label.get_width(), fy, card.width - 80 - label.get_width(), 20)
            gradient_bar(surface, bar, p.fraction, self.small, f"{p.done}/{p.total}", dim=self.runner.paused)
        area = pygame.Rect(card.x + 20, card.y + 60, card.width - 40, card.height - 74 - footer)
        prev = surface.get_clip()
        surface.set_clip(area.inflate(4, 4))
        self.tabs[self.tab].draw(surface, area, mouse)
        surface.set_clip(prev)


# ---------------------------------------------------------------- form helpers


def swap_map_options(panel: SidePanel) -> None:
    world = panel.get("world_kind").value
    maps = panel.get("map_kind")
    wanted = list(GRID_MAPS) if world == "grid" else list(POLY_MAPS)
    if maps.options != wanted:
        maps.options = wanted
        maps.index = 0


def set_form_values(panel: SidePanel, values: dict) -> None:
    """Set widgets from a config dict; world_kind first so map options match."""
    keys = {w.key for w in panel.widgets}
    order = sorted(values, key=lambda k: k != "world_kind")
    for key in order:
        if key not in keys:
            continue
        wd = panel.get(key)
        val = values[key]
        if isinstance(wd, Cycle):
            if key == "world_kind":
                wd.index = wd.options.index(val) if val in wd.options else 0
                swap_map_options(panel)
            elif val in wd.options:
                wd.index = wd.options.index(val)
        elif isinstance(wd, Slider):
            wd.value = float(min(wd.hi, max(wd.lo, val)))
        else:
            wd.value = bool(val)
