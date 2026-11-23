from __future__ import annotations

from dataclasses import dataclass, field

import pygame

from swarmpathfinding.core.geom import Vec2


@dataclass(slots=True)
class Camera:
    x: float = 0.0
    y: float = 0.0
    zoom: float = 16.0  # pixels per world unit
    viewport: pygame.Rect = field(default_factory=lambda: pygame.Rect(0, 0, 800, 600))

    def world_to_screen(self, point: Vec2) -> tuple[int, int]:
        sx = self.viewport.x + (point.x - self.x) * self.zoom
        sy = self.viewport.bottom - (point.y - self.y) * self.zoom
        return int(sx), int(sy)

    def local(self, point: Vec2) -> tuple[int, int]:
        """Pixel position relative to the viewport's top-left corner, for drawing
        on a subsurface of the viewport."""
        sx = (point.x - self.x) * self.zoom
        sy = self.viewport.height - (point.y - self.y) * self.zoom
        return int(sx), int(sy)

    def screen_to_world(self, sx: int, sy: int) -> Vec2:
        x = (sx - self.viewport.x) / self.zoom + self.x
        y = (self.viewport.bottom - sy) / self.zoom + self.y
        return Vec2(x, y)

    def fit(self, width: float, height: float, padding: float = 0.5) -> None:
        """Zoom so the whole world fits, centred in the viewport."""
        if self.viewport.width <= 0 or self.viewport.height <= 0:
            return
        zx = self.viewport.width / max(width + 2 * padding, 1e-6)
        zy = self.viewport.height / max(height + 2 * padding, 1e-6)
        self.zoom = max(4.0, min(zx, zy))
        self.x = width / 2 - self.viewport.width / (2 * self.zoom)
        self.y = height / 2 - self.viewport.height / (2 * self.zoom)

    def set_viewport(self, viewport: pygame.Rect) -> None:
        """Change the viewport, keeping the world point at its centre in place."""
        if self.viewport.width > 0 and self.viewport.height > 0:
            center = self.screen_to_world(*self.viewport.center)
        else:
            center = None
        self.viewport = viewport
        if center is not None:
            self.x = center.x - viewport.width / (2 * self.zoom)
            self.y = center.y - viewport.height / (2 * self.zoom)

    def pan(self, dx_px: float, dy_px: float) -> None:
        self.x -= dx_px / self.zoom
        self.y += dy_px / self.zoom

    def zoom_at(self, sx: int, sy: int, factor: float) -> None:
        before = self.screen_to_world(sx, sy)
        self.zoom = min(80.0, max(4.0, self.zoom * factor))
        after = self.screen_to_world(sx, sy)
        self.x += before.x - after.x
        self.y += before.y - after.y

    def contains_screen(self, sx: int, sy: int) -> bool:
        return bool(self.viewport.collidepoint(sx, sy))
