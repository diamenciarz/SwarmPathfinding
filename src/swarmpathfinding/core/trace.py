"""Search event recording, so a search can be replayed step by step.

A planner asked to record (`Budget.record_trace`) appends one event per action
on a grid cell. The live lab will replay these with play / step / scrub
controls; tests use them to check search order.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

PUSH = 0  # cell added to (or improved in) the open set
EXPAND = 1  # cell taken off the open set and expanded (closed)
UPDATE = 2  # a cost value changed without a push (e.g. D* Lite rhs)
SCAN = 3  # a cell looked at without being queued (e.g. a JPS jump scan)
LOS = 4  # a line-of-sight test from (px, py) to (x, y); g = 1 if clear, 0 if blocked
KIND_NAMES = {PUSH: "push", EXPAND: "expand", UPDATE: "update", SCAN: "scan", LOS: "line of sight"}


@dataclass(slots=True)
class SearchTrace:
    cell_size: float = 1.0
    kinds: list[int] = field(default_factory=list)
    xs: list[int] = field(default_factory=list)
    ys: list[int] = field(default_factory=list)
    g: list[float] = field(default_factory=list)
    f: list[float] = field(default_factory=list)
    h: list[float] = field(default_factory=list)  # unweighted heuristic
    px: list[int] = field(default_factory=list)  # parent cell (-1 = none)
    py: list[int] = field(default_factory=list)

    def add(self, kind: int, x: int, y: int, g: float, f: float, h: float = float("nan"), px: int = -1, py: int = -1) -> None:
        self.kinds.append(kind)
        self.xs.append(x)
        self.ys.append(y)
        self.g.append(g)
        self.f.append(f)
        self.h.append(h)
        self.px.append(px)
        self.py.append(py)

    def __len__(self) -> int:
        return len(self.kinds)

    def expansions(self) -> list[tuple[int, int]]:
        return [(x, y) for k, x, y in zip(self.kinds, self.xs, self.ys) if k == EXPAND]

    def state_at(self, n: int, shape: tuple[int, int]) -> tuple[np.ndarray, np.ndarray]:
        """(open mask, closed mask) after the first `n` events."""
        open_ = np.zeros(shape, dtype=bool)
        closed = np.zeros(shape, dtype=bool)
        for k, x, y in zip(self.kinds[:n], self.xs[:n], self.ys[:n]):
            if k == PUSH and not closed[y, x]:
                open_[y, x] = True
            elif k == EXPAND:
                open_[y, x] = False
                closed[y, x] = True
        return open_, closed
