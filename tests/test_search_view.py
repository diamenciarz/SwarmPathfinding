"""Step-through replay, overlays, inspector and compare panes (headless)."""

import os

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

import numpy as np  # noqa: E402
import pygame  # noqa: E402

from swarmpathfinding.core.registry import create_planner  # noqa: E402
from swarmpathfinding.core.results import Budget  # noqa: E402
from swarmpathfinding.generators.scenarios import make_scenario  # noqa: E402
from swarmpathfinding.viz.live import LiveApp  # noqa: E402
from swarmpathfinding.viz.search_view import CLOSED, OPEN, SearchPlayer  # noqa: E402
from swarmpathfinding.viz.settings import LabSettings  # noqa: E402


def _trace():
    sc = make_scenario(world_kind="grid", map_kind="bernoulli", density=0.2, n_agents=1, seed=5, fow=False)
    r = create_planner("dijkstra").plan(sc.belief, [sc.agents[0].pos], sc.goal, Budget(record_trace=True))
    return r, r.extra["trace"], r.extra["closed"].shape


def test_player_matches_trace_state_any_order():
    r, tr, shape = _trace()
    p = SearchPlayer()
    p.load(tr, shape)
    assert p.expanded == r.expansions
    assert np.array_equal(p.state == CLOSED, r.extra["closed"])
    for n in (len(tr) // 3, 5, len(tr) - 1, 0, len(tr) // 2):
        p.seek(n)
        open_, closed = tr.state_at(n, shape)
        assert np.array_equal(p.state == CLOSED, closed)
        assert np.array_equal(p.state == OPEN, open_)


def test_player_steps_by_expansion_and_plays():
    _, tr, shape = _trace()
    p = SearchPlayer()
    p.load(tr, shape)
    p.restart()
    p.step(10)
    assert p.expanded == 10 and p.frozen
    p.step(-3)
    assert p.expanded == 7
    info = dict(p.cell_info(*p.current))
    assert info["state"] == "closed" and info["expanded"] == "#7"
    p.toggle_play()
    p.tick(1.0)  # 20 expansions per second by default
    assert p.expanded == 27
    p.to_end()
    assert p.at_end and not p.frozen


def test_frozen_player_keeps_old_trace():
    _, tr, shape = _trace()
    _, tr2, _ = _trace()
    p = SearchPlayer()
    p.offer(tr, shape)
    p.step(1)
    p.offer(tr2, shape)
    assert p.trace is tr
    p.to_end()
    p.offer(tr2, shape)
    assert p.trace is tr2


def _app(**kw):
    app = LiveApp(LabSettings(**{"n_agents": 1, "fow": False, "map_kind": "clustered", **kw}))
    app.panel.get("show_search").value = True
    app._on_panel_change("show_search", True)
    app.draw()
    return app


def _key(app, key):
    app.handle(pygame.event.Event(pygame.KEYDOWN, key=key))
    app.handle(pygame.event.Event(pygame.KEYUP, key=key))


def test_replay_keys_pause_and_step():
    app = _app()
    app.paused = False
    assert app.search_active()
    _key(app, pygame.K_HOME)
    assert app.paused and app.players[0].expanded == 0
    _key(app, pygame.K_RIGHT)
    _key(app, pygame.K_RIGHT)
    assert app.players[0].expanded == 2
    _key(app, pygame.K_p)
    app.tick(0.5)
    assert app.players[0].expanded > 2
    _key(app, pygame.K_END)
    assert app.players[0].at_end
    for hm in range(4):
        app.panel.get("heatmap").index = hm
        app.panel.get("arrows").value = bool(hm % 2)
        app.draw()


def test_view_toggles_do_not_rebuild():
    app = _app()
    sim = app.sim
    seed_cells = app.scenario.world.blocked.copy()
    app.panel.get("heatmap").index = 2
    app._on_panel_change("heatmap", True)
    assert app.sim is sim and app.sim.now == 0
    assert (app.scenario.world.blocked == seed_cells).all()


def test_search_bar_buttons_and_scrub():
    app = _app()
    bar = app.search_bar
    app.draw()

    def click(pos):
        app.handle(pygame.event.Event(pygame.MOUSEBUTTONDOWN, pos=pos, button=1))
        app.handle(pygame.event.Event(pygame.MOUSEBUTTONUP, pos=pos, button=1))

    click(bar.buttons["start"].center)
    assert app.players[0].n == 0
    click(bar.buttons["fwd"].center)
    assert app.players[0].expanded == 1
    click((bar.track.centerx, bar.track.centery))
    assert abs(app.players[0].n - app.players[0].total / 2) <= 2
    click(bar.buttons["speed"].center)
    assert app.players[0].speed_index == 2


def test_compare_panes_mirror_edits_and_steps():
    app = _app(n_agents=2)
    app.apply_config({"planner": "dijkstra", "compare": ["dijkstra", "a_star", "greedy_best_first"]})
    assert [s.config.planner for s in app.panes] == ["dijkstra", "a_star", "greedy_best_first"]
    assert len(app.pane_rects()) == 3
    app.draw()
    seeds = {s.config.seed for s in app.panes}
    assert len(seeds) == 1
    # an edit in pane 2 lands in every pane at the same cell
    rect = app.pane_rects()[2]
    pos = rect.center
    app.handle(pygame.event.Event(pygame.MOUSEBUTTONDOWN, pos=pos, button=1))
    blocked = [s.scenario.world.blocked.copy() for s in app.panes]
    assert all((b == blocked[0]).all() for b in blocked)
    app.paused = False
    for _ in range(10):
        app.tick(1 / 30)
    assert len({round(s.now, 6) for s in app.panes}) == 1
    # replay runs in lock-step by expansion count
    _key(app, pygame.K_HOME)
    for _ in range(5):
        _key(app, pygame.K_RIGHT)
    assert {p.expanded for p in app.players if p.trace is not None} == {5}
    app.draw()
    _key(app, pygame.K_c)
    assert len(app.panes) == 1


def test_menu_compare_ticks_apply_panes():
    app = _app()
    app.open_menu()
    app.draw()
    tab = app.menu.scenario
    for name in ("a_star", "dijkstra"):
        rect = next(r for r, n in tab._ticks if n == name)
        app.handle(pygame.event.Event(pygame.MOUSEBUTTONDOWN, pos=rect.center, button=1))
    assert tab.compare == ["a_star", "dijkstra"]
    tab.planner = "dijkstra"
    tab.apply()
    assert [s.config.planner for s in app.panes] == ["dijkstra", "a_star"]
    app.open_menu()
    assert app.menu.scenario.compare == ["dijkstra", "a_star"]
