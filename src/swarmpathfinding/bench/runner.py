"""Runs an experiment's trials and keeps the progress the lab draws.

Three modes:
  "headless"  - worker processes (CPU count - 1, one core left free). `poll()`
                collects finished trials and submits more; nothing blocks.
  "inline"    - same, but in this process within a time budget per poll
                (tests, or a machine with one core).
  "visual"    - the lab draws each trial while `visual_step()` advances it
                several sim steps per frame (x4, x16, max).

All modes step the same `Simulation` with all replanning done synchronously,
so a trial gives the same numbers whichever mode ran it. Each finished trial
is appended to the CSV straight away.
"""

from __future__ import annotations

import csv
import multiprocessing
import os
import time
from concurrent.futures import Future, ProcessPoolExecutor
from concurrent.futures.process import BrokenProcessPool
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from swarmpathfinding.bench.experiment import DT, ExperimentSpec, Trial, _worker, row_for, run_trial, trial_metrics
from swarmpathfinding.sim.simulation import Simulation

OUT_DIR = Path("benches") / "out" / "lab"
SPEEDS = {"x4": 4, "x16": 16, "max": 10_000}


def default_workers() -> int:
    return max(1, (os.cpu_count() or 2) - 1)


@dataclass
class Progress:
    total: int = 0
    done: int = 0
    skipped: int = 0
    per_planner: dict[str, list[int]] = field(default_factory=dict)  # name -> [done, total]
    started: float = 0.0
    finished: float | None = None

    @property
    def fraction(self) -> float:
        return self.done / self.total if self.total else 0.0

    def eta_s(self, now: float | None = None) -> float | None:
        if not self.done or self.finished is not None:
            return None
        elapsed = (now or time.perf_counter()) - self.started
        return elapsed / self.done * (self.total - self.done)


class BenchmarkRunner:
    def __init__(
        self,
        spec: ExperimentSpec,
        mode: str = "headless",
        workers: int | None = None,
        out_dir: Path | str | None = OUT_DIR,
    ) -> None:
        self.spec = spec
        self.mode = mode
        self.workers = workers or default_workers()
        self.trials: list[Trial] = spec.trials()
        self.rows: list[dict[str, Any]] = []
        self.progress = Progress(total=len(self.trials), started=time.perf_counter())
        for t in self.trials:
            self.progress.per_planner.setdefault(t.planner, [0, 0])[1] += 1
        self.paused = False
        self.cancelled = False
        self.error = ""
        self._next = 0
        self._pool: ProcessPoolExecutor | None = None
        self._inflight: dict[Future, Trial] = {}
        # visual mode
        self.current: Trial | None = None
        self.current_sim: Simulation | None = None
        self._t0 = 0.0
        self.csv_path: Path | None = None
        if out_dir is not None:
            stamp = time.strftime("%Y%m%d-%H%M%S")
            safe = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in spec.name) or "experiment"
            self.csv_path = Path(out_dir) / f"{stamp}_{safe}.csv"
            self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self._csv_fields: list[str] | None = None

    # ------------------------------------------------------------------ state

    @property
    def running(self) -> bool:
        return not self.cancelled and self.progress.done < self.progress.total

    @property
    def finished(self) -> bool:
        return self.progress.finished is not None

    def pause(self, paused: bool = True) -> None:
        self.paused = paused

    def cancel(self) -> None:
        self.cancelled = True
        for fut in self._inflight:
            fut.cancel()
        self._inflight.clear()
        self.current = None
        self.current_sim = None
        self._shutdown()
        if self.progress.finished is None:
            self.progress.finished = time.perf_counter()

    def _shutdown(self) -> None:
        if self._pool is not None:
            self._pool.shutdown(wait=False, cancel_futures=True)
            self._pool = None

    # ------------------------------------------------------------------ recording

    def _record(self, trial: Trial, metrics: dict[str, Any] | None) -> None:
        row = row_for(trial, metrics)
        self.rows.append(row)
        p = self.progress
        p.done += 1
        p.skipped += int(trial.skipped)
        p.per_planner[trial.planner][0] += 1
        if self.csv_path is not None:
            if self._csv_fields is None:
                self._csv_fields = list(row_for(self.trials[0], _EMPTY_METRICS).keys())
                with self.csv_path.open("w", newline="") as f:
                    csv.DictWriter(f, self._csv_fields).writeheader()
            with self.csv_path.open("a", newline="") as f:
                csv.DictWriter(f, self._csv_fields, extrasaction="ignore").writerow(row)
        if p.done >= p.total and p.finished is None:
            p.finished = time.perf_counter()
            self._shutdown()

    def _take_next(self) -> Trial | None:
        """Next trial to run; skipped trials are recorded on the way."""
        while self._next < len(self.trials):
            t = self.trials[self._next]
            self._next += 1
            if t.skipped:
                self._record(t, None)
                continue
            return t
        return None

    # ------------------------------------------------------------------ headless / inline

    def poll(self, budget_s: float = 0.02) -> None:
        """Call every frame. Collects finished trials and starts new ones."""
        if self.cancelled or self.finished:
            return
        if self.mode == "visual":
            return
        if self.mode == "inline":
            self._poll_inline(budget_s)
            return
        self._poll_pool()

    def _poll_inline(self, budget_s: float) -> None:
        end = time.perf_counter() + budget_s
        while not self.paused and not self.cancelled and time.perf_counter() < end:
            t = self._take_next()
            if t is None:
                return
            self._record(t, run_trial(t.config, self.spec.time_limit))

    def _poll_pool(self) -> None:
        if self._pool is None:
            ctx = multiprocessing.get_context("spawn")  # same behaviour on Windows and Linux
            self._pool = ProcessPoolExecutor(max_workers=self.workers, mp_context=ctx)
        for fut in [f for f in self._inflight if f.done()]:
            trial = self._inflight.pop(fut)
            try:
                metrics = fut.result()
            except BrokenProcessPool:
                self._next = min(self._next, trial.trial_id)
                continue
            except Exception as exc:  # a crashing planner is a result too
                metrics = {"success": 0, "error": repr(exc)[:200]}
            self._record(trial, metrics)
        if self.paused or self.cancelled:
            return
        while len(self._inflight) < self.workers * 2:
            t = self._take_next()
            if t is None:
                break
            try:
                fut = self._pool.submit(_worker, (t.config.as_dict(), self.spec.time_limit))
            except BrokenProcessPool:
                # workers could not start (e.g. no importable __main__): finish in-process
                self._next -= 1
                self._shutdown()
                self.mode = "inline"
                self.error = "worker processes failed; running in the lab process"
                return
            self._inflight[fut] = t

    def run_all(self) -> list[dict[str, Any]]:
        """Blocking run (scripts / notebooks)."""
        while not self.finished and not self.cancelled:
            self.poll(0.5)
            if self.mode == "headless":
                time.sleep(0.01)
            elif self.mode == "visual":
                self.visual_step(SPEEDS["max"])
        return self.rows

    # ------------------------------------------------------------------ visual

    def visual_step(self, steps: int, budget_s: float = 1 / 60) -> Simulation | None:
        """Advance the visible trial by up to `steps` sim steps (or the time budget).
        Returns the simulation to draw."""
        if self.cancelled or self.finished:
            return self.current_sim
        if self.current_sim is None:
            if self.paused:
                return None
            self.current = self._take_next()
            if self.current is None:
                return None
            self._t0 = time.perf_counter()
            self.current_sim = Simulation(self.current.config)
        if self.paused:
            return self.current_sim
        sim = self.current_sim
        end = time.perf_counter() + budget_s
        limit = self.spec.time_limit
        for _ in range(steps):
            if sim.now >= limit or _sim_finished(sim):
                assert self.current is not None
                self._record(self.current, trial_metrics(sim, limit, time.perf_counter() - self._t0))
                self.current_sim = None
                self.current = None
                break
            sim.step(DT, sync_plans=True)
            if time.perf_counter() > end:
                break
        return sim


def _sim_finished(sim: Simulation) -> bool:
    sc = sim.scenario
    return bool(sim.done and (sim.config.changes != "slow_edit" or sc is None or sc.pending_edit is None))


_EMPTY_METRICS = {
    "success": 0,
    "arrived_frac": 0,
    "makespan": 0,
    "mean_arrival": 0,
    "path_len": 0,
    "plan_ms": 0,
    "expansions": 0,
    "plans": 0,
    "plan_failures": 0,
    "plan_ms_after_edit": 0,
    "sim_time": 0,
    "wall_s": 0,
    "error": "",
}
