from pathlib import Path

from swarmpathfinding.bench.harness import run_pack, run_spec, write_csv
from swarmpathfinding.generators.packs import build_spec, list_packs, pack_specs
from swarmpathfinding.generators.scenarios import apply_pending_edit, make_scenario
from swarmpathfinding.physics.bodies import make_space_for_world


def test_pack_names():
    names = list_packs()
    assert "single_static" in names
    assert "slow_edit" in names
    assert "fow_ghost" in names
    assert "opposite" in names


def test_single_static_builds():
    specs = pack_specs("single_static", seeds=(0,))
    assert len(specs) >= 10
    sc = build_spec(specs[0])
    assert len(sc.agents) == 1
    assert sc.scenario_id


def test_spawn_layouts():
    clump = make_scenario(n_agents=6, start_layout="clump", seed=2, fow=False, map_kind="empty")
    xs = [a.pos.x for a in clump.agents]
    assert max(xs) - min(xs) < 8
    flanks = make_scenario(n_agents=8, start_layout="flanks", seed=3, fow=False, map_kind="empty")
    left = sum(1 for a in flanks.agents if a.pos.x < 24)
    right = sum(1 for a in flanks.agents if a.pos.x > 24)
    assert left >= 2 and right >= 2
    opp = make_scenario(n_agents=8, start_layout="opposite", seed=4, fow=False, map_kind="corridor")
    goals = {a.goal for a in opp.agents if a.goal is not None}
    assert len(goals) >= 2


def test_slow_edit_deletes_wall():
    specs = [s for s in pack_specs("slow_edit", seeds=(1,)) if s.world_kind == "grid"]
    sc = build_spec(specs[0])
    assert sc.pending_edit is not None
    gen = sc.world.generation
    assert apply_pending_edit(sc)
    assert sc.world.generation > gen
    assert sc.pending_edit is None


def test_kinematic_movers():
    sc = make_scenario(world_kind="continuous", map_kind="movers", n_agents=2, seed=1, fow=False)
    space, movers = make_space_for_world(sc.world)
    assert len(movers) >= 1
    assert space is not None


def test_harness_csv(tmp_path: Path):
    rows = run_pack("slow_edit", "straight_line", seeds=(0,), limit=3)
    assert len(rows) == 3
    assert any(row.slow_edit for row in rows)
    path = write_csv(tmp_path / "out.csv", rows)
    text = path.read_text(encoding="utf-8")
    assert "scenario_id" in text
    assert "replan_ms" in text


def test_run_spec_straight_line():
    spec = pack_specs("single_static", seeds=(0,))[0]
    row = run_spec(spec, "none")
    assert row.algorithm == "none"
    assert not row.success
