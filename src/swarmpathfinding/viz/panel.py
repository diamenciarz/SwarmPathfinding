from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

import pygame

from swarmpathfinding.viz.colors import (
    ACCENT,
    BUTTON,
    BUTTON_HOT,
    PANEL_BG,
    PANEL_FG,
    PANEL_MUTED,
    TOGGLE_ON,
)


@dataclass(slots=True)
class Cycle:
    key: str
    label: str
    options: list[str]
    index: int = 0
    rect: pygame.Rect = field(default_factory=lambda: pygame.Rect(0, 0, 0, 0))

    @property
    def value(self) -> str:
        return self.options[self.index]

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
    rect: pygame.Rect = field(default_factory=lambda: pygame.Rect(0, 0, 0, 0))

    def set_from_x(self, x: int) -> None:
        t = (x - self.rect.x) / max(1, self.rect.width)
        t = min(1.0, max(0.0, t))
        raw = self.lo + t * (self.hi - self.lo)
        self.value = float(int(round(raw))) if self.integer else raw


@dataclass(slots=True)
class Check:
    key: str
    label: str
    value: bool
    rect: pygame.Rect = field(default_factory=lambda: pygame.Rect(0, 0, 0, 0))


class SidePanel:
    def __init__(self, rect: pygame.Rect, font: pygame.font.Font, small: pygame.font.Font):
        self.rect = rect
        self.font = font
        self.small = small
        self.cycles: list[Cycle] = []
        self.sliders: list[Slider] = []
        self.checks: list[Check] = []
        self._drag_slider: Slider | None = None
        self._help: list[str] = []
        self._metrics: list[str] = []
        self._on_change: Callable[[], None] | None = None
        self._regen_rect = pygame.Rect(0, 0, 0, 0)

    def set_on_change(self, fn: Callable[[], None]) -> None:
        self._on_change = fn

    def values(self) -> dict:
        out: dict = {c.key: c.value for c in self.cycles}
        out.update({s.key: s.value for s in self.sliders})
        out.update({c.key: c.value for c in self.checks})
        return out

    def set_help(self, lines: list[str]) -> None:
        self._help = lines

    def set_metrics(self, lines: list[str]) -> None:
        self._metrics = lines

    def handle(self, event: pygame.event.Event) -> str | None:
        """Return 'regen' if the regenerate button was clicked."""
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            mx, my = event.pos
            if self._regen_rect.collidepoint(mx, my):
                return "regen"
            for cycle in self.cycles:
                if cycle.rect.collidepoint(mx, my):
                    cycle.cycle(1 if mx > cycle.rect.centerx else -1)
                    self._fire()
                    return None
            for check in self.checks:
                if check.rect.collidepoint(mx, my):
                    check.value = not check.value
                    self._fire()
                    return None
            for slider in self.sliders:
                if slider.rect.inflate(0, 10).collidepoint(mx, my):
                    self._drag_slider = slider
                    slider.set_from_x(mx)
                    self._fire()
                    return None
        if event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            self._drag_slider = None
        if event.type == pygame.MOUSEMOTION and self._drag_slider is not None:
            self._drag_slider.set_from_x(event.pos[0])
            self._fire()
        return None

    def _fire(self) -> None:
        if self._on_change:
            self._on_change()

    def draw(self, surface: pygame.Surface) -> None:
        pygame.draw.rect(surface, PANEL_BG, self.rect)
        pygame.draw.line(
            surface,
            ACCENT,
            (self.rect.x, self.rect.y),
            (self.rect.x, self.rect.bottom),
            2,
        )
        x = self.rect.x + 14
        y = self.rect.y + 12
        w = self.rect.width - 28
        y = self._blit(surface, "SwarmPathfinding", x, y, self.font, ACCENT)
        y += 8
        for cycle in self.cycles:
            y = self._blit(surface, cycle.label, x, y, self.small, PANEL_MUTED)
            cycle.rect = pygame.Rect(x, y, w, 26)
            pygame.draw.rect(surface, BUTTON, cycle.rect, border_radius=4)
            self._blit_center(surface, f"◀  {cycle.value}  ▶", cycle.rect, self.small, PANEL_FG)
            y += 32
        for slider in self.sliders:
            shown = f"{slider.label}: {int(slider.value) if slider.integer else f'{slider.value:.2f}'}"
            y = self._blit(surface, shown, x, y, self.small, PANEL_MUTED)
            slider.rect = pygame.Rect(x, y + 6, w, 8)
            pygame.draw.rect(surface, BUTTON, slider.rect, border_radius=3)
            t = (slider.value - slider.lo) / max(1e-6, slider.hi - slider.lo)
            knob_x = slider.rect.x + int(t * slider.rect.width)
            pygame.draw.circle(surface, ACCENT, (knob_x, slider.rect.centery), 7)
            y += 24
        for check in self.checks:
            check.rect = pygame.Rect(x, y, w, 22)
            box = pygame.Rect(x, y, 16, 16)
            pygame.draw.rect(surface, TOGGLE_ON if check.value else BUTTON, box, border_radius=3)
            self._blit(surface, check.label, x + 24, y - 2, self.small, PANEL_FG)
            y += 26
        self._regen_rect = pygame.Rect(x, y + 6, w, 28)
        pygame.draw.rect(surface, BUTTON_HOT, self._regen_rect, border_radius=4)
        self._blit_center(surface, "Regenerate map", self._regen_rect, self.small, PANEL_FG)
        y = self._regen_rect.bottom + 16
        y = self._blit(surface, "Live", x, y, self.font, ACCENT)
        for line in self._metrics:
            y = self._blit(surface, line, x, y, self.small, PANEL_FG)
        y += 10
        y = self._blit(surface, "Keys", x, y, self.font, ACCENT)
        for line in self._help:
            y = self._blit(surface, line, x, y, self.small, PANEL_MUTED)

    def _blit(self, surface, text, x, y, font, color) -> int:
        img = font.render(text, True, color)
        surface.blit(img, (x, y))
        return y + img.get_height() + 2

    def _blit_center(self, surface, text, rect, font, color) -> None:
        img = font.render(text, True, color)
        surface.blit(img, img.get_rect(center=rect.center))
