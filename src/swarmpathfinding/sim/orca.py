"""ORCA: Optimal Reciprocal Collision Avoidance (van den Berg et al. 2011).

Each agent turns every nearby agent into a half-plane of velocities that
avoid a collision within `tau` seconds (1 s here: longer horizons make
symmetric crowds lock up in rings), assuming the other agent takes half
the responsibility. It then picks the allowed velocity closest to the one it
wants (towards its next waypoint) with a small 2D linear program.

This is a direct port of the agent-agent part of the RVO2 library
(computeNewVelocity, linearProgram1/2/3). Static obstacles are left to the
global planner's route and to physics.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

EPS = 1e-5
PERTURB = 1e-2  # velocity units


@dataclass(slots=True)
class Line:
    px: float
    py: float
    dx: float
    dy: float


def _det(ax, ay, bx, by) -> float:
    return ax * by - ay * bx


def _lp1(lines: list[Line], no: int, radius: float, opt: tuple[float, float], direction_opt: bool):
    ln = lines[no]
    dot = ln.px * ln.dx + ln.py * ln.dy
    disc = dot * dot + radius * radius - (ln.px * ln.px + ln.py * ln.py)
    if disc < 0.0:
        return None
    sq = math.sqrt(disc)
    t_left, t_right = -dot - sq, -dot + sq
    for i in range(no):
        li = lines[i]
        den = _det(ln.dx, ln.dy, li.dx, li.dy)
        num = _det(li.dx, li.dy, ln.px - li.px, ln.py - li.py)
        if abs(den) <= EPS:
            if num < 0.0:
                return None
            continue
        t = num / den
        if den >= 0.0:
            t_right = min(t_right, t)
        else:
            t_left = max(t_left, t)
        if t_left > t_right:
            return None
    if direction_opt:
        t = t_right if opt[0] * ln.dx + opt[1] * ln.dy > 0.0 else t_left
    else:
        t = ln.dx * (opt[0] - ln.px) + ln.dy * (opt[1] - ln.py)
        t = min(max(t, t_left), t_right)
    return (ln.px + t * ln.dx, ln.py + t * ln.dy)


def _lp2(lines: list[Line], radius: float, opt: tuple[float, float], direction_opt: bool, result: list[float]) -> int:
    if direction_opt:
        result[0], result[1] = opt[0] * radius, opt[1] * radius
    elif opt[0] * opt[0] + opt[1] * opt[1] > radius * radius:
        n = math.hypot(*opt)
        result[0], result[1] = opt[0] / n * radius, opt[1] / n * radius
    else:
        result[0], result[1] = opt
    for i, li in enumerate(lines):
        if _det(li.dx, li.dy, li.px - result[0], li.py - result[1]) > 0.0:
            r = _lp1(lines, i, radius, opt, direction_opt)
            if r is None:
                return i
            result[0], result[1] = r
    return len(lines)


def _lp3(lines: list[Line], n_obst: int, begin: int, radius: float, result: list[float]) -> None:
    """Agent lines that cannot all hold are relaxed equally; the first `n_obst`
    lines (walls) are never relaxed."""
    distance = 0.0
    for i in range(begin, len(lines)):
        li = lines[i]
        if _det(li.dx, li.dy, li.px - result[0], li.py - result[1]) > distance:
            proj: list[Line] = list(lines[:n_obst])
            for j in range(n_obst, i):
                lj = lines[j]
                det = _det(li.dx, li.dy, lj.dx, lj.dy)
                if abs(det) <= EPS:
                    if li.dx * lj.dx + li.dy * lj.dy > 0.0:
                        continue
                    px, py = 0.5 * (li.px + lj.px), 0.5 * (li.py + lj.py)
                else:
                    t = _det(lj.dx, lj.dy, li.px - lj.px, li.py - lj.py) / det
                    px, py = li.px + t * li.dx, li.py + t * li.dy
                dx, dy = lj.dx - li.dx, lj.dy - li.dy
                n = math.hypot(dx, dy) or 1.0
                proj.append(Line(px, py, dx / n, dy / n))
            temp = list(result)
            if _lp2(proj, radius, (-li.dy, li.dx), True, result) < len(proj):
                result[0], result[1] = temp
            distance = _det(li.dx, li.dy, li.px - result[0], li.py - result[1])


def orca_lines(pos, vel, others, radius: float, tau: float, dt: float) -> list[Line]:
    """Half-planes (point, direction; allowed side on the left) for one agent.
    `others` is a list of (position, velocity, radius)."""
    lines = []
    inv_tau = 1.0 / tau
    px, py = pos
    vx, vy = vel
    for (ox, oy), (ovx, ovy), orad in others:
        rpx, rpy = ox - px, oy - py
        rvx, rvy = vx - ovx, vy - ovy
        dist_sq = rpx * rpx + rpy * rpy
        cr = radius + orad
        cr_sq = cr * cr
        if dist_sq > cr_sq:
            wx, wy = rvx - inv_tau * rpx, rvy - inv_tau * rpy
            w_len_sq = wx * wx + wy * wy
            dot1 = wx * rpx + wy * rpy
            if dot1 < 0.0 and dot1 * dot1 > cr_sq * w_len_sq:
                w_len = math.sqrt(w_len_sq)
                ux_, uy_ = wx / w_len, wy / w_len
                dx, dy = uy_, -ux_
                ux, uy = (cr * inv_tau - w_len) * ux_, (cr * inv_tau - w_len) * uy_
            else:
                leg = math.sqrt(dist_sq - cr_sq)
                if _det(rpx, rpy, wx, wy) > 0.0:
                    dx = (rpx * leg - rpy * cr) / dist_sq
                    dy = (rpx * cr + rpy * leg) / dist_sq
                else:
                    dx = -(rpx * leg + rpy * cr) / dist_sq
                    dy = -(-rpx * cr + rpy * leg) / dist_sq
                dot2 = rvx * dx + rvy * dy
                ux, uy = dot2 * dx - rvx, dot2 * dy - rvy
        else:
            inv_dt = 1.0 / dt
            wx, wy = rvx - inv_dt * rpx, rvy - inv_dt * rpy
            w_len = math.hypot(wx, wy) or EPS
            ux_, uy_ = wx / w_len, wy / w_len
            dx, dy = uy_, -ux_
            ux, uy = (cr * inv_dt - w_len) * ux_, (cr * inv_dt - w_len) * uy_
        lines.append(Line(vx + 0.5 * ux, vy + 0.5 * uy, dx, dy))
    return lines


def wall_lines(pos, radius: float, points, tau: float) -> list[Line]:
    """One half-plane per nearby wall point: never close in on it faster than
    you could stop within `tau` seconds (RVO2 builds these from obstacle edges;
    the closest point per obstacle is a simpler, slightly stricter version)."""
    lines = []
    px, py = pos
    for cx, cy in points:
        nx, ny = cx - px, cy - py
        d = math.hypot(nx, ny)
        if d < EPS:
            continue
        nx, ny = nx / d, ny / d
        s = (d - radius) / tau
        # allowed: v . n <= s, i.e. the left side of direction (-ny, nx)
        lines.append(Line(nx * s, ny * s, -ny, nx))
    return lines


def solve(lines: list[Line], pref: tuple[float, float], max_speed: float, n_obst: int = 0) -> tuple[float, float]:
    result = [0.0, 0.0]
    fail = _lp2(lines, max_speed, pref, False, result)
    if fail < len(lines):
        _lp3(lines, n_obst, fail, max_speed, result)
    return result[0], result[1]


@dataclass(slots=True)
class OrcaDebug:
    agent: int
    lines: list[Line]
    pref: tuple[float, float]
    new: tuple[float, float]
    max_speed: float
    neighbors: list[int]
    n_walls: int = 0


def orca_step(
    pos: np.ndarray,
    vel: np.ndarray,
    pref: np.ndarray,
    active: np.ndarray,
    radius: float,
    max_speed: float,
    dt: float,
    tau: float = 1.0,
    neighbor_dist: float = 5.0,
    max_neighbors: int = 10,
    debug_agent: int | None = None,
    walls=None,
    tau_wall: float = 0.6,
) -> tuple[np.ndarray, OrcaDebug | None]:
    """New velocities for every agent (rows of `pos`). Inactive agents (already
    arrived) keep their preferred velocity and are not avoided. `walls(i)`, if
    given, returns the nearby wall points of agent i."""
    n = len(pos)
    out = pref.copy()
    dbg = None
    if n == 0:
        return out, dbg
    # Agents stacked on the same spot have no direction to part in; a tiny fixed
    # per-agent offset breaks the tie deterministically.
    k = np.arange(n)
    pos = pos + 1e-3 * np.stack([np.cos(k * 2.399963), np.sin(k * 2.399963)], axis=1)
    diff = pos[None, :, :] - pos[:, None, :]
    d2 = (diff**2).sum(-1)
    for i in range(n):
        if not active[i]:
            continue
        cand = np.nonzero(active & (d2[i] < neighbor_dist * neighbor_dist))[0]
        cand = cand[cand != i]
        if len(cand) > max_neighbors:
            cand = cand[np.argsort(d2[i, cand])[:max_neighbors]]
        others = [((pos[j, 0], pos[j, 1]), (vel[j, 0], vel[j, 1]), radius) for j in cand]
        me = (pos[i, 0], pos[i, 1])
        obst = wall_lines(me, radius, walls(i), tau_wall) if walls is not None else []
        lines = obst + orca_lines(me, (vel[i, 0], vel[i, 1]), others, radius, tau, dt)
        p = (float(pref[i, 0]), float(pref[i, 1]))
        if lines:
            # Like RVO2's demos: nudge the wish by a hair so perfectly symmetric
            # crowds do not freeze. Hash of the state, so runs stay reproducible.
            h = math.sin(i * 12.9898 + me[0] * 78.233 + me[1] * 37.719) * 43758.5453
            a = (h - math.floor(h)) * 2 * math.pi
            p = (p[0] + PERTURB * math.cos(a), p[1] + PERTURB * math.sin(a))
        new = solve(lines, p, max_speed, len(obst)) if lines else p
        out[i] = new
        if i == debug_agent:
            dbg = OrcaDebug(i, lines, p, (float(new[0]), float(new[1])), max_speed, [int(j) for j in cand], len(obst))
    return out, dbg
