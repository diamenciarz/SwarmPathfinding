from __future__ import annotations

from dataclasses import dataclass, field
from typing import Hashable


@dataclass(slots=True)
class DirtyTracker:
    """Sector / tile dirty flags for flow-field caches (Phase 4 fills this in)."""

    dirty: set[Hashable] = field(default_factory=set)
    hits: int = 0
    misses: int = 0

    def mark(self, key: Hashable) -> None:
        self.dirty.add(key)

    def take(self) -> set[Hashable]:
        out = self.dirty
        self.dirty = set()
        return out

    def hit(self) -> None:
        self.hits += 1

    def miss(self) -> None:
        self.misses += 1

    def reset_counts(self) -> None:
        self.hits = 0
        self.misses = 0
