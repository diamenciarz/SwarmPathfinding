"""Debug view for ORCA: the focus agent's velocity space drawn on the map.

Velocities are drawn as arrows from the agent, scaled so one second of travel
at full speed spans SCALE seconds of movement. Each neighbour adds one ORCA
line: the allowed velocities are on the side without the hatching. The thin
ring is the speed limit, the shaded green area is every velocity that is
allowed (empty when the constraints conflict and ORCA has to compromise), the
grey arrow is the velocity the agent wants (to its next waypoint) and the green
one is what ORCA lets it do.
"""

from __future__ import annotations

import math

import pygame

from swarmpathfinding.core.geom import Vec2
from swarmpathfinding.viz.colors import ORCA_ALLOWED, ORCA_CHOSEN, ORCA_LINE, ORCA_NEIGHBOR, ORCA_PREF, ORCA_RING

SCALE = 0.5  # world units drawn per (unit/s) of velocity


def _chord(px, py, dx, dy, r):
    """Where the line p + t d crosses the circle |v| = r (None if it misses)."""
    b = px * dx + py * dy
    disc = b * b - (px * px + py * py - r * r)
    if disc <= 0:
        return None
    sq = math.sqrt(disc)
    return -b - sq, -b + sq


def allowed_region(lines, r: float, segments: int = 48) -> list[tuple[float, float]]:
    """The speed disc clipped by every half-plane (Sutherland-Hodgman)."""
    poly = [(r * math.cos(2 * math.pi * k / segments), r * math.sin(2 * math.pi * k / segments)) for k in range(segments)]
    for ln in lines:
        def inside(q):
            # allowed = left of the direction: det(d, p - q) <= 0
            return ln.dx * (ln.py - q[1]) - ln.dy * (ln.px - q[0]) <= 0.0

        def cut(a, b):
            ax, ay = a[0] - ln.px, a[1] - ln.py
            bx, by = b[0] - ln.px, b[1] - ln.py
            fa = ln.dx * ay - ln.dy * ax
            fb = ln.dx * by - ln.dy * bx
            t = fa / (fa - fb) if fa != fb else 0.0
            return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)

        out = []
        for k, cur in enumerate(poly):
            prev = poly[k - 1]
            if inside(cur):
                if not inside(prev):
                    out.append(cut(prev, cur))
                out.append(cur)
            elif inside(prev):
                out.append(cut(prev, cur))
        poly = out
        if not poly:
            break
    return poly


def _arrow(surface, a, b, color, width=2):
    pygame.draw.line(surface, color, a, b, width)
    ang = math.atan2(b[1] - a[1], b[0] - a[0])
    if math.hypot(b[0] - a[0], b[1] - a[1]) < 4:
        return
    for s in (2.6, -2.6):
        tip = (b[0] + 7 * math.cos(ang + s), b[1] + 7 * math.sin(ang + s))
        pygame.draw.line(surface, color, b, tip, width)


def draw_orca(surface, camera, sim) -> None:
    dbg = getattr(sim, "orca_debug", None)
    sc = sim.scenario
    if dbg is None or sc is None or dbg.agent >= len(sc.agents):
        return
    agents = sc.agents
    me = agents[dbg.agent]
    origin = me.pos

    def at(vx, vy):
        return camera.local(Vec2(origin.x + vx * SCALE, origin.y + vy * SCALE))

    for j in dbg.neighbors:
        if j < len(agents):
            c = camera.local(agents[j].pos)
            pygame.draw.circle(surface, ORCA_NEIGHBOR, c, max(3, int(agents[j].radius * camera.zoom) + 3), 1)
    c0 = camera.local(origin)
    ring = max(2, int(dbg.max_speed * SCALE * camera.zoom))
    r = dbg.max_speed
    region = allowed_region(dbg.lines, r)
    if len(region) >= 3:
        pts = [at(*q) for q in region]
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        x0, y0 = min(xs), min(ys)
        w, h = max(xs) - x0 + 2, max(ys) - y0 + 2
        if 0 < w < 4000 and 0 < h < 4000:
            layer = pygame.Surface((w, h), pygame.SRCALPHA)
            pygame.draw.polygon(layer, ORCA_ALLOWED, [(p[0] - x0, p[1] - y0) for p in pts])
            surface.blit(layer, (x0, y0))
    pygame.draw.circle(surface, ORCA_RING, c0, ring, 1)
    for ln in dbg.lines:
        ts = _chord(ln.px, ln.py, ln.dx, ln.dy, r)
        if ts is None:
            continue
        a = at(ln.px + ts[0] * ln.dx, ln.py + ts[0] * ln.dy)
        b = at(ln.px + ts[1] * ln.dx, ln.py + ts[1] * ln.dy)
        pygame.draw.line(surface, ORCA_LINE, a, b, 1)
        # short ticks on the forbidden side (right of the direction)
        nx, ny = ln.dy, -ln.dx
        n = 6
        for k in range(1, n):
            t = ts[0] + (ts[1] - ts[0]) * k / n
            vx, vy = ln.px + t * ln.dx, ln.py + t * ln.dy
            pygame.draw.line(surface, ORCA_LINE, at(vx, vy), at(vx + nx * 0.35, vy + ny * 0.35), 1)
    _arrow(surface, c0, at(*dbg.pref), ORCA_PREF)
    _arrow(surface, c0, at(*dbg.new), ORCA_CHOSEN)
