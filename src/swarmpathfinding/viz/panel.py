from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

import pygame

from swarmpathfinding.viz.colors import (
    ACCENT,
    BUTTON,
    BUTTON_DISABLED,
    BUTTON_HOT,
    PANEL_BG,
    PANEL_FG,
    PANEL_MUTED,
    TOGGLE_ON,
)

ROW_CYCLE = 28
ROW_SLIDER = 32
ROW_CHECK = 22
LABEL_W = 92


@dataclass(slots=True)
class Cycle:
    key: str
    label: str
    options: list[str]
    index: int = 0
    section: str = ""
    visible: bool = True
    enabled: bool = True
    display: dict[str, str] = field(default_factory=dict)  # option -> shown text
    rect: pygame.Rect = field(default_factory=lambda: pygame.Rect(0, 0, 0, 0))

    @property
    def value(self) -> str:
        return self.options[self.index]

    @property
    def shown(self) -> str:
        return self.display.get(self.value, self.value)

    def cycle(self, step: int = 1) -> None:
        self.index = (self.index + step) % len(self.options)


@dataclass(slots=True)
class Slider:
    key: str
    label: str
    lo: float
    hi: float
    value: float
    integer: bool = False
    section: str = ""
    visible: bool = True
    enabled: bool = True
    rect: pygame.Rect = field(default_factory=lambda: pygame.Rect(0, 0, 0, 0))

    def set_from_x(self, x: int) -> bool:
        t = (x - self.rect.x) / max(1, self.rect.width)
        t = min(1.0, max(0.0, t))
        raw = self.lo + t * (self.hi - self.lo)
        new = float(int(round(raw))) if self.integer else round(raw, 2)
        changed = new != self.value
        self.value = new
        return changed

    def text(self) -> str:
        return str(int(self.value)) if self.integer else f"{self.value:.2f}"


@dataclass(slots=True)
class Check:
    key: str
    label: str
    value: bool
    section: str = ""
    visible: bool = True
    enabled: bool = True
    hint: str = ""
    rect: pygame.Rect = field(default_factory=lambda: pygame.Rect(0, 0, 0, 0))


Widget = Cycle | Slider | Check
# on_change(key, final): `final` is False while a slider is being dragged.
ChangeFn = Callable[[str, bool], None]


class SidePanel:
    """Right-hand control column: sections of pickers, sliders and toggles,
    a restart button, live metrics and a key reference. Scrolls with the wheel
    when the window is too short."""

    def __init__(self, rect: pygame.Rect, font: pygame.font.Font, small: pygame.font.Font):
        self.rect = rect
        self.font = font
        self.small = small
        self.widgets: list[Widget] = []
        self.sections: list[str] = []
        self._drag_slider: Slider | None = None
        self._help: list[tuple[str, str]] = []
        self._metrics: list[tuple[str, str]] = []
        self._on_change: ChangeFn | None = None
        self._button_rect = pygame.Rect(0, 0, 0, 0)
        self._top_rect = pygame.Rect(0, 0, 0, 0)
        self.button_label: str | None = "Restart"  # bottom action button; None hides it
        self.top_button_label: str | None = None  # button under the title; None hides it
        self.title: str | None = "SwarmPathfinding"
        self.edge_line = True  # accent rule on the left edge (side panel look)
        self.scroll = 0
        self._content_h = 0

    # ------------------------------------------------------------ data access

    @property
    def cycles(self) -> list[Cycle]:
        return [w for w in self.widgets if isinstance(w, Cycle)]

    @property
    def sliders(self) -> list[Slider]:
        return [w for w in self.widgets if isinstance(w, Slider)]

    @property
    def checks(self) -> list[Check]:
        return [w for w in self.widgets if isinstance(w, Check)]

    def get(self, key: str) -> Widget:
        return next(w for w in self.widgets if w.key == key)

    def set_on_change(self, fn: ChangeFn) -> None:
        self._on_change = fn

    def values(self) -> dict:
        return {w.key: w.value for w in self.widgets}

    def set_help(self, lines: list[tuple[str, str]]) -> None:
        self._help = lines

    def set_metrics(self, lines: list[tuple[str, str]]) -> None:
        self._metrics = lines

    # ------------------------------------------------------------ events

    def handle(self, event: pygame.event.Event) -> str | None:
        """Return 'button' if the action button was clicked."""
        if event.type == pygame.MOUSEWHEEL and self.rect.collidepoint(pygame.mouse.get_pos()):
            self._scroll_by(-event.y * 40)
            return None
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if not self.rect.collidepoint(event.pos):
                return None
            mx, my = event.pos
            if self.button_label and self._button_rect.collidepoint(mx, my):
                return "button"
            if self.top_button_label and self._top_rect.collidepoint(mx, my):
                return "top"
            for w in self.widgets:
                if not (w.visible and w.enabled):
                    continue
                if isinstance(w, Cycle) and w.rect.collidepoint(mx, my):
                    w.cycle(1 if mx >= w.rect.centerx else -1)
                    self._fire(w.key, True)
                    return None
                if isinstance(w, Check) and w.rect.collidepoint(mx, my):
                    w.value = not w.value
                    self._fire(w.key, True)
                    return None
                if isinstance(w, Slider) and w.rect.inflate(0, 14).collidepoint(mx, my):
                    self._drag_slider = w
                    if w.set_from_x(mx):
                        self._fire(w.key, False)
                    return None
        if event.type == pygame.MOUSEBUTTONUP and event.button == 1 and self._drag_slider is not None:
            slider = self._drag_slider
            self._drag_slider = None
            self._fire(slider.key, True)
        if event.type == pygame.MOUSEMOTION and self._drag_slider is not None:
            if self._drag_slider.set_from_x(event.pos[0]):
                self._fire(self._drag_slider.key, False)
        return None

    @property
    def dragging(self) -> bool:
        return self._drag_slider is not None

    def _scroll_by(self, dy: int) -> None:
        max_scroll = max(0, self._content_h - self.rect.height)
        self.scroll = min(max_scroll, max(0, self.scroll + dy))

    def _fire(self, key: str, final: bool) -> None:
        if self._on_change:
            self._on_change(key, final)

    # ------------------------------------------------------------ drawing

    def draw(self, surface: pygame.Surface) -> None:
        pygame.draw.rect(surface, PANEL_BG, self.rect)
        prev_clip = surface.get_clip()
        surface.set_clip(self.rect)
        mouse = pygame.mouse.get_pos()
        x = self.rect.x + 14
        w = self.rect.width - 28
        top = self.rect.y + 10 - self.scroll
        y = top
        if self.title:
            y = self._blit(surface, self.title, x, y, self.font, ACCENT) + 2
        if self.top_button_label:
            self._top_rect = pygame.Rect(x, y + 2, w, 26)
            hot = self._top_rect.collidepoint(mouse)
            pygame.draw.rect(surface, BUTTON_HOT if hot else BUTTON, self._top_rect, border_radius=4)
            self._blit_center(surface, self.top_button_label, self._top_rect, self.small, PANEL_FG)
            y = self._top_rect.bottom + 2
        for section in self.sections:
            items = [wd for wd in self.widgets if wd.section == section and wd.visible]
            if not items:
                continue
            y = self._section(surface, section, x, y, w)
            for wd in items:
                if isinstance(wd, Cycle):
                    y = self._draw_cycle(surface, wd, x, y, w, mouse)
                elif isinstance(wd, Slider):
                    y = self._draw_slider(surface, wd, x, y, w)
                else:
                    y = self._draw_check(surface, wd, x, y, w, mouse)
        if self.button_label:
            self._button_rect = pygame.Rect(x, y + 6, w, 28)
            hot = self._button_rect.collidepoint(mouse)
            pygame.draw.rect(surface, ACCENT if hot else BUTTON_HOT, self._button_rect, border_radius=4)
            self._blit_center(surface, self.button_label, self._button_rect, self.small, PANEL_FG)
            y = self._button_rect.bottom + 8
        if self._metrics:
            y = self._section(surface, "Live", x, y, w)
            y = self._two_columns(surface, self._metrics, x, y, w, PANEL_MUTED, PANEL_FG)
        if self._help:
            y = self._section(surface, "Keys", x, y + 4, w)
            y = self._two_columns(surface, self._help, x, y, w, ACCENT, PANEL_MUTED)
        self._content_h = (y - top) + 14
        self._scroll_by(0)
        surface.set_clip(prev_clip)
        if self.edge_line:
            pygame.draw.line(surface, ACCENT, (self.rect.x, self.rect.y), (self.rect.x, self.rect.bottom), 2)
        if self._content_h > self.rect.height:
            self._scrollbar(surface)

    def _section(self, surface, title: str, x: int, y: int, w: int) -> int:
        y += 6
        img = self.small.render(title.upper(), True, ACCENT)
        surface.blit(img, (x, y))
        line_y = y + img.get_height() // 2
        pygame.draw.line(surface, BUTTON, (x + img.get_width() + 8, line_y), (x + w, line_y), 1)
        return y + img.get_height() + 4

    def _draw_cycle(self, surface, c: Cycle, x, y, w, mouse) -> int:
        self._blit(surface, c.label, x, y + 4, self.small, PANEL_MUTED if c.enabled else BUTTON)
        c.rect = pygame.Rect(x + LABEL_W, y, w - LABEL_W, ROW_CYCLE - 4)
        hot = c.enabled and c.rect.collidepoint(mouse)
        color = BUTTON_DISABLED if not c.enabled else (BUTTON_HOT if hot else BUTTON)
        pygame.draw.rect(surface, color, c.rect, border_radius=4)
        fg = PANEL_FG if c.enabled else PANEL_MUTED
        self._blit_center(surface, c.shown, c.rect, self.small, fg)
        self._arrow(surface, c.rect.x + 9, c.rect.centery, -1, fg)
        self._arrow(surface, c.rect.right - 9, c.rect.centery, 1, fg)
        return y + ROW_CYCLE

    def _draw_slider(self, surface, s: Slider, x, y, w) -> int:
        self._blit(surface, s.label, x, y, self.small, PANEL_MUTED)
        val = self.small.render(s.text(), True, PANEL_FG)
        surface.blit(val, (x + w - val.get_width(), y))
        s.rect = pygame.Rect(x + 4, y + 20, w - 8, 6)
        pygame.draw.rect(surface, BUTTON, s.rect, border_radius=3)
        t = (s.value - s.lo) / max(1e-6, s.hi - s.lo)
        knob_x = s.rect.x + int(t * s.rect.width)
        fill = pygame.Rect(s.rect.x, s.rect.y, knob_x - s.rect.x, s.rect.height)
        pygame.draw.rect(surface, BUTTON_HOT, fill, border_radius=3)
        pygame.draw.circle(surface, ACCENT, (knob_x, s.rect.centery), 7)
        return y + ROW_SLIDER

    def _draw_check(self, surface, c: Check, x, y, w, mouse) -> int:
        c.rect = pygame.Rect(x, y, w, ROW_CHECK - 2)
        box = pygame.Rect(x, y + 2, 16, 16)
        if not c.enabled:
            pygame.draw.rect(surface, BUTTON_DISABLED, box, border_radius=3)
        else:
            on = TOGGLE_ON if c.value else BUTTON
            pygame.draw.rect(surface, on, box, border_radius=3)
            if c.rect.collidepoint(mouse):
                pygame.draw.rect(surface, ACCENT, box, 1, border_radius=3)
            if c.value:
                pygame.draw.lines(surface, PANEL_FG, False, [(x + 4, y + 10), (x + 7, y + 14), (x + 13, y + 6)], 2)
        label = c.label if c.enabled or not c.hint else f"{c.label} ({c.hint})"
        self._blit(surface, label, x + 24, y + 1, self.small, PANEL_FG if c.enabled else PANEL_MUTED)
        return y + ROW_CHECK

    def _two_columns(self, surface, rows: list[tuple[str, str]], x, y, w, c1, c2) -> int:
        col_w = w // 2
        line_h = self.small.get_linesize()
        for i, (a, b) in enumerate(rows):
            cx = x + (i % 2) * col_w
            cy = y + (i // 2) * line_h
            img_a = self.small.render(a, True, c1)
            surface.blit(img_a, (cx, cy))
            surface.blit(self.small.render(b, True, c2), (cx + max(img_a.get_width() + 6, 62), cy))
        return y + ((len(rows) + 1) // 2) * line_h

    def _scrollbar(self, surface) -> None:
        frac = self.rect.height / self._content_h
        bar_h = max(24, int(self.rect.height * frac))
        max_scroll = self._content_h - self.rect.height
        bar_y = self.rect.y + int((self.rect.height - bar_h) * (self.scroll / max(1, max_scroll)))
        pygame.draw.rect(surface, BUTTON_HOT, pygame.Rect(self.rect.right - 5, bar_y, 3, bar_h), border_radius=2)

    def _arrow(self, surface, cx: int, cy: int, direction: int, color) -> None:
        pts = [(cx - 3 * direction, cy - 5), (cx - 3 * direction, cy + 5), (cx + 4 * direction, cy)]
        pygame.draw.polygon(surface, color, pts)

    def _blit(self, surface, text, x, y, font, color) -> int:
        img = font.render(text, True, color)
        surface.blit(img, (x, y))
        return y + img.get_height() + 2

    def _blit_center(self, surface, text, rect, font, color) -> None:
        img = font.render(text, True, color)
        surface.blit(img, img.get_rect(center=rect.center))
