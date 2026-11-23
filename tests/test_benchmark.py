"""Benchmark engine: sampling, seed sync, determinism across modes, analysis."""

import csv

from swarmpathfinding.bench import analysis as A
from swarmpathfinding.bench.experiment import ExperimentSpec, run_trial
from swarmpathfinding.bench.runner import BenchmarkRunner

SPEC = dict(n_scenarios=4, worlds=["grid", "continuous"], agents=(1, 4), time_limit=15, changes=["none", "slow_edit"])


def test_scenarios_are_reproducible_and_seed_synced():
    spec = ExperimentSpec(**SPEC)
    a, b = spec.scenarios(), ExperimentSpec(**SPEC).scenarios()
    assert a == b
    assert [c.seed for c in a] == [1, 2, 3, 4]
    trials = spec.trials()
    assert len(trials) == 4 * len(spec.planners)
    for i in range(4):
        cfgs = [t.config for t in trials if t.scenario == i]
        assert len({c.with_(planner="x") == cfgs[0].with_(planner="x") for c in cfgs}) == 1


def test_sampling_respects_chips_and_ranges():
    spec = ExperimentSpec(n_scenarios=60, worlds=["continuous"], poly_maps=["sparse"], agents=(3, 5), density=(0.1, 0.2), fog=[False])
    for c in spec.scenarios():
        assert c.world_kind == "continuous" and c.map_kind == "sparse" and not c.fow
        assert 3 <= c.n_agents <= 5 and 0.1 <= c.density <= 0.2


def test_problems_reported():
    assert ExperimentSpec(planners=[]).problems()
    assert ExperimentSpec(worlds=["grid"], grid_maps=[]).problems()
    assert not ExperimentSpec().problems()


def test_run_trial_metrics():
    spec = ExperimentSpec(**SPEC)
    m = run_trial(spec.trials()[0].config, 15)
    for key in ("success", "makespan", "path_len", "plan_ms", "expansions", "wall_s"):
        assert key in m


def _key(rows):
    return sorted((r["trial"], r["makespan"], r["expansions"], r["path_len"]) for r in rows)


def test_inline_and_visual_modes_agree(tmp_path):
    spec = ExperimentSpec(**SPEC)
    inline = BenchmarkRunner(spec, mode="inline", out_dir=tmp_path).run_all()
    visual = BenchmarkRunner(spec, mode="visual", out_dir=None).run_all()
    assert _key(inline) == _key(visual)


def test_headless_pool_matches_inline(tmp_path):
    spec = ExperimentSpec(**{**SPEC, "n_scenarios": 3})
    r = BenchmarkRunner(spec, mode="headless", workers=2, out_dir=tmp_path)
    rows = r.run_all()
    inline = BenchmarkRunner(spec, mode="inline", out_dir=None).run_all()
    assert _key(rows) == _key(inline)
    assert r.progress.done == r.progress.total == len(rows)
    with r.csv_path.open() as f:
        assert len(list(csv.DictReader(f))) == len(rows)


def test_pause_and_cancel():
    spec = ExperimentSpec(**SPEC)
    r = BenchmarkRunner(spec, mode="inline", out_dir=None)
    r.pause()
    r.poll(0.5)
    assert r.progress.done == 0
    r.pause(False)
    r.poll(0.0001)
    r.cancel()
    assert not r.running
    done = r.progress.done
    r.poll(0.5)
    assert r.progress.done == done


def test_analysis():
    rows = []
    for sc, (a, b) in enumerate([(5.0, 7.0), (6.0, 6.0), (60.0, 9.0)]):
        for p, v in (("a", a), ("b", b)):
            rows.append({"scenario": sc, "planner": p, "success": int(v < 60), "makespan": v, "skipped": 0,
                         "trait_world": "grid", "trait_knowledge": "fog" if sc else "full", "trait_changes": "static",
                         "trait_agents": "few", "trait_goals": "shared", "path_len": 1, "plan_ms": 1, "expansions": 1})
    assert A.winners(rows) == {0: ["a"], 1: ["a", "b"], 2: ["b"]}
    assert A.wins(rows) == {"a": 1.5, "b": 1.5}
    hm = A.win_matrix(rows)
    assert hm[("knowledge", "full")]["a"] == (1.0, 1)
    assert hm[("knowledge", "fog")]["b"] == (0.75, 2)
    assert A.box_stats([1, 2, 3, 4, 5]) == (1, 2, 3, 4, 5)
    assert A.summary(rows)["a"]["success"] == 2 / 3
