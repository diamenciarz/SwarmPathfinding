"""Turn benchmark rows into the numbers the Results tab shows."""

from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Any, Iterable

from swarmpathfinding.bench.experiment import METRICS
from swarmpathfinding.core.capabilities import TRAITS

LOWER_IS_BETTER = {k: low for k, _, low in METRICS}
# Metric that decides who "wins" a scenario: fastest to get everyone home.
WIN_METRIC = "makespan"


def ran(rows: Iterable[dict]) -> list[dict]:
    """Rows of trials that actually ran (not skipped as incompatible)."""
    return [r for r in rows if not int(r.get("skipped", 0)) and "success" in r]


def planners_of(rows: Iterable[dict]) -> list[str]:
    seen: list[str] = []
    for r in rows:
        if r["planner"] not in seen:
            seen.append(r["planner"])
    return seen


def summary(rows: list[dict]) -> dict[str, dict[str, float]]:
    """planner -> {trials, skipped, wins, <metric mean>...}."""
    out: dict[str, dict[str, float]] = {}
    won = wins(rows)
    for p in planners_of(rows):
        mine = [r for r in rows if r["planner"] == p]
        done = ran(mine)
        s: dict[str, float] = {"trials": len(done), "skipped": len(mine) - len(done), "wins": won.get(p, 0)}
        for key, _, _ in METRICS:
            vals = [float(r[key]) for r in done if r.get(key) not in (None, "")]
            s[key] = sum(vals) / len(vals) if vals else math.nan
        out[p] = s
    return out


def winners(rows: list[dict]) -> dict[int, list[str]]:
    """scenario -> planners with the best makespan among those that succeeded
    (ties share the win). Scenarios nobody solved have no winner."""
    best: dict[int, tuple[float, list[str]]] = {}
    for r in ran(rows):
        if not int(r["success"]):
            continue
        sc = int(r["scenario"])
        v = float(r[WIN_METRIC])
        cur = best.get(sc)
        if cur is None or v < cur[0] - 1e-9:
            best[sc] = (v, [r["planner"]])
        elif abs(v - cur[0]) <= 1e-9:
            cur[1].append(r["planner"])
    return {sc: names for sc, (_, names) in best.items()}


def wins(rows: list[dict]) -> dict[str, float]:
    out: dict[str, float] = {}
    for names in winners(rows).values():
        for n in names:
            out[n] = out.get(n, 0) + 1 / len(names)
    return out


def win_matrix(rows: list[dict]) -> dict[tuple[str, str], dict[str, tuple[float, int]]]:
    """(trait, value) -> planner -> (share of those scenarios it won, scenarios).

    Only scenarios with that trait value count, so each cell answers
    "when the scenario is like this, how often is this planner the best?"."""
    win_by_sc = winners(rows)
    traits_by_sc: dict[int, dict[str, str]] = {}
    for r in rows:
        traits_by_sc[int(r["scenario"])] = {t: r.get(f"trait_{t}") for t in TRAITS}
    planners = planners_of(rows)
    out: dict[tuple[str, str], dict[str, tuple[float, int]]] = {}
    for trait, values in TRAITS.items():
        for value in values:
            scs = [sc for sc, tr in traits_by_sc.items() if tr.get(trait) == value]
            cell: dict[str, tuple[float, int]] = {}
            for p in planners:
                share = sum(1 / len(win_by_sc[sc]) for sc in scs if p in win_by_sc.get(sc, []))
                cell[p] = (share / len(scs) if scs else math.nan, len(scs))
            out[(trait, value)] = cell
    return out


def box_stats(values: list[float]) -> tuple[float, float, float, float, float] | None:
    """(min, q1, median, q3, max) or None when empty."""
    v = sorted(x for x in values if not math.isnan(x))
    if not v:
        return None

    def q(p: float) -> float:
        k = (len(v) - 1) * p
        lo, hi = math.floor(k), math.ceil(k)
        return v[lo] + (v[hi] - v[lo]) * (k - lo)

    return v[0], q(0.25), q(0.5), q(0.75), v[-1]


def metric_values(rows: list[dict], planner: str, key: str) -> list[float]:
    return [float(r[key]) for r in ran(rows) if r["planner"] == planner and r.get(key) not in (None, "")]


def write_summary_csv(rows: list[dict], path: Path | str) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    summ = summary(rows)
    keys = ["planner", "trials", "skipped", "wins", *[k for k, _, _ in METRICS]]
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, keys)
        w.writeheader()
        for p, s in summ.items():
            w.writerow({"planner": p, **{k: (round(v, 4) if isinstance(v, float) else v) for k, v in s.items()}})
    return path


def read_rows(path: Path | str) -> list[dict[str, Any]]:
    with Path(path).open(newline="") as f:
        return list(csv.DictReader(f))
