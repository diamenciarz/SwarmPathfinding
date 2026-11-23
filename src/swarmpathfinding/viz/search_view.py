"""Step-through replay of a recorded search, its overlays and the cell inspector.

`SearchPlayer` rebuilds the planner's state (open / closed sets, g, h, f,
parents, expansion order) at any point of a `SearchTrace`. The live lab draws
that state over the world and lets you play, step and scrub through it.
"""

from __future__ import annotations

import math

import numpy as np
import pygame

from swarmpathfinding.core.geom import Vec2
from swarmpathfinding.core.trace import EXPAND, PUSH, UPDATE, SearchTrace
from swarmpathfinding.viz.camera import Camera
from swarmpathfinding.viz.colors import (
    ACCENT,
    BANNER_BG,
    BUTTON,
    BUTTON_HOT,
    PANEL_BG,
    PANEL_FG,
    PANEL_MUTED,
    SEARCH,
    SEARCH_CURRENT,
    SEARCH_OPEN,
    SEARCH_PARENT,
)
from swarmpathfinding.viz.draw import blit_cells

UNSEEN, OPEN, CLOSED = 0, 1, 2
SPEEDS = (5, 20, 100, 500, 2000)  # expansions per second
HEATMAPS = ("off", "g", "h", "f")

# a short perceptual ramp (dark blue -> teal -> yellow), like viridis
_RAMP = np.array(
    [(68, 1, 84), (59, 82, 139), (33, 145, 140), (94, 201, 98), (253, 231, 37)],
    dtype=np.float32,
)


def ramp(t: np.ndarray) -> np.ndarray:
    """Map values in [0, 1] to RGB (float array, shape (..., 3))."""
    t = np.clip(t, 0.0, 1.0) * (len(_RAMP) - 1)
    i = np.minimum(t.astype(int), len(_RAMP) - 2)
    frac = (t - i)[..., None]
    return _RAMP[i] * (1 - frac) + _RAMP[i + 1] * frac


class SearchPlayer:
    def __init__(self) -> None:
        self.trace: SearchTrace | None = None
        self.shape: tuple[int, int] = (0, 0)
        self.n = 0  # events applied
        self.playing = False
        self.speed_index = 1
        self._acc = 0.0
        self.frozen = False  # True while replaying: newer searches don't replace this one
        self._alloc((0, 0))

    # ------------------------------------------------------------ state

    def _alloc(self, shape: tuple[int, int]) -> None:
        self.shape = shape
        self.state = np.zeros(shape, dtype=np.uint8)
        self.g = np.full(shape, np.nan, dtype=np.float32)
        self.h = np.full(shape, np.nan, dtype=np.float32)
        self.f = np.full(shape, np.nan, dtype=np.float32)
        self.parent = np.full((*shape, 2), -1, dtype=np.int32)
        self.order = np.full(shape, -1, dtype=np.int32)
        self.expanded = 0
        self.current: tuple[int, int] | None = None
        self.n = 0

    @property
    def total(self) -> int:
        return len(self.trace) if self.trace is not None else 0

    @property
    def total_expansions(self) -> int:
        if self.trace is None:
            return 0
        return sum(1 for k in self.trace.kinds if k == EXPAND)

    @property
    def at_end(self) -> bool:
        return self.n >= self.total

    @property
    def speed(self) -> int:
        return SPEEDS[self.speed_index]

    def offer(self, trace: SearchTrace | None, shape: tuple[int, int] | None) -> None:
        """Called every frame with the focus agent's latest trace. A new trace is
        adopted (shown complete) unless the user is replaying the current one."""
        if trace is self.trace or self.frozen:
            return
        self.load(trace, shape)

    def load(self, trace: SearchTrace | None, shape: tuple[int, int] | None, at_end: bool = True) -> None:
        self.trace = trace
        self.playing = False
        self._alloc(shape or (0, 0))
        if trace is not None and at_end:
            self.seek(len(trace))

    def _apply(self, i: int) -> None:
        tr = self.trace
        assert tr is not None
        k, x, y = tr.kinds[i], tr.xs[i], tr.ys[i]
        if not (0 <= y < self.shape[0] and 0 <= x < self.shape[1]):
            return
        if k == EXPAND:
            self.state[y, x] = CLOSED
            self.order[y, x] = self.expanded
            self.expanded += 1
            self.current = (x, y)
        elif k == PUSH and self.state[y, x] != CLOSED:
            self.state[y, x] = OPEN
        if k in (PUSH, UPDATE, EXPAND):
            self.g[y, x] = tr.g[i]
            self.f[y, x] = tr.f[i]
            if i < len(tr.h):
                self.h[y, x] = tr.h[i]
                if tr.px[i] >= 0:
                    self.parent[y, x] = (tr.px[i], tr.py[i])

    def seek(self, n: int) -> None:
        n = max(0, min(self.total, n))
        if n < self.n:
            self._alloc(self.shape)
        while self.n < n:
            self._apply(self.n)
            self.n += 1

    def _event_after_expansions(self, k: int) -> int:
        """Event count right after the `k`-th expansion (k counted from 1)."""
        tr = self.trace
        if tr is None or k <= 0:
            return 0
        seen = 0
        for i, kind in enumerate(tr.kinds):
            if kind == EXPAND:
                seen += 1
                if seen == k:
                    # include the pushes that expansion caused
                    j = i + 1
                    while j < len(tr.kinds) and tr.kinds[j] != EXPAND:
                        j += 1
                    return j
        return len(tr.kinds)

    def step(self, expansions: int) -> None:
        """Move by whole expansions (negative = back)."""
        self.frozen = True
        self.seek(self._event_after_expansions(self.expanded + expansions))

    def restart(self) -> None:
        self.frozen = True
        self.seek(0)

    def to_end(self) -> None:
        self.seek(self.total)
        self.playing = False
        self.frozen = False

    def toggle_play(self) -> None:
        if self.trace is None:
            return
        if self.at_end:
            self.seek(0)
        self.playing = not self.playing
        self.frozen = True

    def tick(self, dt: float) -> None:
        if not self.playing:
            return
        self._acc += self.speed * dt
        k = int(self._acc)
        if k:
            self._acc -= k
            self.seek(self._event_after_expansions(self.expanded + k))
        if self.at_end:
            self.playing = False

    def scrub(self, frac: float) -> None:
        self.frozen = True
        self.seek(int(round(frac * self.total)))

    # ------------------------------------------------------------ queries

    def cell_info(self, x: int, y: int) -> list[tuple[str, str]] | None:
        if not (0 <= y < self.shape[0] and 0 <= x < self.shape[1]):
            return None
        st = int(self.state[y, x])
        rows = [("cell", f"({x}, {y})"), ("state", ("not reached", "open", "closed")[st])]
        if st:
            rows += [("g", _f(self.g[y, x])), ("h", _f(self.h[y, x])), ("f", _f(self.f[y, x]))]
            px, py = self.parent[y, x]
            rows.append(("parent", f"({px}, {py})" if px >= 0 else "start"))
        if self.order[y, x] >= 0:
            rows.append(("expanded", f"#{int(self.order[y, x]) + 1}"))
        return rows


def _f(v: float) -> str:
    return "-" if v is None or math.isnan(v) else f"{v:.2f}"


# ---------------------------------------------------------------- drawing


def draw_search_state(
    surface: pygame.Surface,
    camera: Camera,
    player: SearchPlayer,
    cell_size: float,
    heatmap: str = "off",
    show_open: bool = True,
    show_closed: bool = True,
    arrows: bool = False,
) -> None:
    if player.trace is None or player.shape == (0, 0):
        return
    st = player.state
    rgba = np.zeros((*st.shape, 4), dtype=np.uint8)
    if heatmap in ("g", "h", "f"):
        vals = getattr(player, heatmap)
        mask = (st > 0) & ~np.isnan(vals)
        if mask.any():
            lo, hi = float(np.nanmin(vals[mask])), float(np.nanmax(vals[mask]))
            t = (vals - lo) / max(1e-9, hi - lo)
            rgb = ramp(np.nan_to_num(t))
            rgba[mask, :3] = rgb[mask].astype(np.uint8)
            rgba[mask, 3] = 170
    else:
        if show_closed:
            rgba[st == CLOSED] = SEARCH
        if show_open:
            rgba[st == OPEN] = SEARCH_OPEN
    if heatmap in ("g", "h", "f") and show_open:
        # keep the frontier readable on top of the heatmap: brighten open cells
        rgba[st == OPEN, 3] = 230
    blit_cells(surface, camera, rgba, cell_size)
    px_per_cell = camera.zoom * cell_size
    if arrows and px_per_cell >= 7:
        _draw_arrows(surface, camera, player, cell_size)
    if player.current is not None:
        cx, cy = player.current
        center = camera.local(Vec2((cx + 0.5) * cell_size, (cy + 0.5) * cell_size))
        r = max(4, int(px_per_cell * 0.7))
        pygame.draw.circle(surface, SEARCH_CURRENT, center, r, 2)


def _draw_arrows(surface, camera: Camera, player: SearchPlayer, cell_size: float) -> None:
    vp = camera.viewport
    tl = camera.screen_to_world(vp.left, vp.top)
    br = camera.screen_to_world(vp.right, vp.bottom)
    h, w = player.shape
    x0, x1 = max(0, int(tl.x / cell_size)), min(w, int(br.x / cell_size) + 1)
    y0, y1 = max(0, int(br.y / cell_size)), min(h, int(tl.y / cell_size) + 1)
    sub = player.state[y0:y1, x0:x1]
    ys, xs = np.nonzero(sub)
    if len(xs) > 6000:
        return
    for yy, xx in zip(ys + y0, xs + x0):
        px, py = player.parent[yy, xx]
        if px < 0:
            continue
        a = camera.local(Vec2((xx + 0.5) * cell_size, (yy + 0.5) * cell_size))
        b = camera.local(Vec2((xx + 0.5 + (px - xx) * 0.6) * cell_size, (yy + 0.5 + (py - yy) * 0.6) * cell_size))
        pygame.draw.line(surface, SEARCH_PARENT, a, b, 1)
        pygame.draw.circle(surface, SEARCH_PARENT, b, 2)


class SearchBar:
    """Transport controls along the bottom of the world view."""

    H = 40

    def __init__(self, font: pygame.font.Font) -> None:
        self.font = font
        self.rect = pygame.Rect(0, 0, 0, 0)
        self.buttons: dict[str, pygame.Rect] = {}
        self.track = pygame.Rect(0, 0, 0, 0)
        self.dragging = False

    def layout(self, viewport: pygame.Rect) -> None:
        self.rect = pygame.Rect(viewport.x + 8, viewport.bottom - self.H - 8, viewport.width - 16, self.H)

    def handle(self, event: pygame.event.Event, player: SearchPlayer) -> bool:
        """True if the event was used."""
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1 and self.rect.collidepoint(event.pos):
            for name, r in self.buttons.items():
                if r.collidepoint(event.pos):
                    {"start": player.restart, "back": lambda: player.step(-1), "play": player.toggle_play,
                     "fwd": lambda: player.step(1), "end": player.to_end,
                     "speed": lambda: setattr(player, "speed_index", (player.speed_index + 1) % len(SPEEDS))}[name]()
                    return True
            if self.track.inflate(0, 16).collidepoint(event.pos):
                self.dragging = True
                player.playing = False
                player.scrub((event.pos[0] - self.track.x) / max(1, self.track.width))
            return True
        if event.type == pygame.MOUSEMOTION and self.dragging:
            player.scrub((event.pos[0] - self.track.x) / max(1, self.track.width))
            return True
        if event.type == pygame.MOUSEBUTTONUP and event.button == 1 and self.dragging:
            self.dragging = False
            return True
        return False

    def draw(self, surface: pygame.Surface, player: SearchPlayer, title: str) -> None:
        r = self.rect
        panel = pygame.Surface(r.size, pygame.SRCALPHA)
        panel.fill((*PANEL_BG, 225))
        surface.blit(panel, r.topleft)
        pygame.draw.rect(surface, BUTTON, r, 1, border_radius=6)
        mouse = pygame.mouse.get_pos()
        x = r.x + 8
        self.buttons = {}
        for name in ("start", "back", "play", "fwd", "end"):
            b = pygame.Rect(x, r.y + 7, 30, 26)
            pygame.draw.rect(surface, BUTTON_HOT if b.collidepoint(mouse) else BUTTON, b, border_radius=4)
            _icon(surface, name, b.center, player.playing)
            self.buttons[name] = b
            x = b.right + 4
        sp = pygame.Rect(x + 4, r.y + 7, 74, 26)
        pygame.draw.rect(surface, BUTTON_HOT if sp.collidepoint(mouse) else BUTTON, sp, border_radius=4)
        img = self.font.render(f"{player.speed}/s", True, PANEL_FG)
        surface.blit(img, img.get_rect(center=sp.center))
        self.buttons["speed"] = sp
        info = f"{title}: expansion {player.expanded}/{player.total_expansions}"
        info_img = self.font.render(info, True, PANEL_FG)
        info_w = info_img.get_width()
        self.track = pygame.Rect(sp.right + 12, r.centery - 3, max(40, r.right - sp.right - 24 - info_w - 12), 6)
        pygame.draw.rect(surface, BUTTON, self.track, border_radius=3)
        frac = player.n / player.total if player.total else 0
        fill = pygame.Rect(self.track.x, self.track.y, int(self.track.width * frac), 6)
        pygame.draw.rect(surface, ACCENT, fill, border_radius=3)
        pygame.draw.circle(surface, PANEL_FG, (fill.right, self.track.centery), 7)
        surface.blit(info_img, (self.track.right + 12, r.centery - info_img.get_height() // 2))


def _icon(surface, name: str, c: tuple[int, int], playing: bool) -> None:
    x, y = c
    col = PANEL_FG
    if name == "play":
        if playing:
            pygame.draw.rect(surface, col, (x - 6, y - 6, 4, 12))
            pygame.draw.rect(surface, col, (x + 2, y - 6, 4, 12))
        else:
            pygame.draw.polygon(surface, col, [(x - 5, y - 7), (x - 5, y + 7), (x + 7, y)])
    elif name in ("fwd", "back"):
        d = 1 if name == "fwd" else -1
        pygame.draw.polygon(surface, col, [(x - 4 * d, y - 6), (x - 4 * d, y + 6), (x + 4 * d, y)])
        pygame.draw.rect(surface, col, (x + 4 * d - (2 if d > 0 else 0), y - 6, 2, 12))
    else:
        d = 1 if name == "end" else -1
        for off in (-4, 3):
            pygame.draw.polygon(surface, col, [(x + (off - 3) * d, y - 6), (x + (off - 3) * d, y + 6), (x + (off + 4) * d, y)])


def draw_inspector(surface: pygame.Surface, font: pygame.font.Font, pos: tuple[int, int], rows: list[tuple[str, str]], bounds: pygame.Rect) -> None:
    line = font.get_linesize()
    w = max(font.size(f"{a}  {b}")[0] for a, b in rows) + 24
    h = line * len(rows) + 10
    box = pygame.Rect(pos[0] + 16, pos[1] + 16, w, h)
    if box.right > bounds.right:
        box.x = pos[0] - w - 12
    if box.bottom > bounds.bottom:
        box.y = pos[1] - h - 12
    pygame.draw.rect(surface, BANNER_BG, box, border_radius=5)
    pygame.draw.rect(surface, ACCENT, box, 1, border_radius=5)
    y = box.y + 5
    for a, b in rows:
        surface.blit(font.render(a, True, PANEL_MUTED), (box.x + 8, y))
        img = font.render(b, True, PANEL_FG)
        surface.blit(img, (box.right - img.get_width() - 8, y))
        y += line
