"""Headless tests for the popup lab menu."""

import os

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

import pygame  # noqa: E402

from swarmpathfinding.core.capabilities import Fit  # noqa: E402
from swarmpathfinding.viz.live import LiveApp  # noqa: E402
from swarmpathfinding.viz.menu import draw_badge, wrap  # noqa: E402
from swarmpathfinding.viz.settings import LabSettings  # noqa: E402


def _key(app, key):
    app.handle(pygame.event.Event(pygame.KEYDOWN, key=key))
    app.handle(pygame.event.Event(pygame.KEYUP, key=key))


def _click(app, pos):
    app.handle(pygame.event.Event(pygame.MOUSEBUTTONDOWN, pos=pos, button=1))
    app.handle(pygame.event.Event(pygame.MOUSEBUTTONUP, pos=pos, button=1))


def _app(**kw):
    app = LiveApp(LabSettings(**kw))
    app.draw()
    return app


def test_m_opens_and_pauses_esc_restores():
    app = _app(n_agents=4)
    app.paused = False
    _key(app, pygame.K_m)
    assert app.menu.is_open and app.paused
    t = app.sim.now
    app.tick(1 / 30)
    assert app.sim.now == t  # paused while open
    app.draw()
    _key(app, pygame.K_ESCAPE)
    assert not app.menu.is_open and not app.paused


def test_esc_with_menu_open_does_not_quit_and_click_outside_closes():
    app = _app()
    _key(app, pygame.K_m)
    assert app.handle(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_ESCAPE)) is True
    _key(app, pygame.K_m)
    app.draw()
    _click(app, (2, 2))
    assert not app.menu.is_open


def test_panel_top_button_opens_menu():
    app = _app()
    _click(app, app.panel._top_rect.center)
    assert app.menu.is_open


def test_tabs_switch_by_key_and_click():
    app = _app()
    app.open_menu()
    app.draw()
    _key(app, pygame.K_2)
    assert app.menu.tab == 1
    app.draw()
    _click(app, app.menu._tab_rects[0].center)
    assert app.menu.tab == 0


def test_menu_clicks_do_not_edit_the_world():
    app = _app(map_kind="empty", fow=False)
    before = app.scenario.world.blocked.copy()
    app.open_menu()
    app.draw()
    _click(app, app.menu.card.center)
    assert (app.scenario.world.blocked == before).all()


def test_apply_loads_form_into_live_view():
    app = _app(n_agents=4)
    app.open_menu()
    tab = app.menu.scenario
    tab.form.get("n_agents").value = 20
    tab.form.get("changes").index = tab.form.get("changes").options.index("slow_edit")
    tab.form.get("edit_at").value = 7.5
    tab.planner = "straight_line"
    app.draw()
    _key(app, pygame.K_RETURN)
    assert not app.menu.is_open
    assert app.sim.config.n_agents == 20
    assert app.sim.config.changes == "slow_edit"
    assert app.sim.config.edit_at == 7.5
    assert app.sim.config.planner == "straight_line"
    assert app.panel.get("planner").value == "straight_line"
    assert len(app.scenario.agents) == 20


def test_apply_continuous_world_swaps_map_options():
    app = _app()
    app.open_menu()
    tab = app.menu.scenario
    tab.form.get("world_kind").index = 1
    tab._on_change("world_kind", True)
    assert tab.form.get("map_kind").value == "polygons"
    tab.apply()
    assert app.sim.config.world_kind == "continuous"
    assert app.panel.get("map_kind").options[0] == "polygons"


def test_compatibility_rows_select_planner():
    app = _app()
    app.open_menu()
    app.draw()
    tab = app.menu.scenario
    rect, name, _ = next(r for r in tab._rows if r[1] == "none")
    _click(app, rect.center)
    assert tab.planner == "none"


def test_randomise_is_valid_and_seedable():
    import random

    app = _app()
    app.open_menu()
    tab = app.menu.scenario
    for i in range(20):
        tab.randomise(random.Random(i))
        vals = tab.values()
        maps = tab.form.get("map_kind").options
        assert vals["map_kind"] in maps
        app.draw()


def test_planners_tab_matrix_and_use_button():
    app = _app()
    app.open_menu(tab=1)
    app.draw()
    tab = app.menu.planners
    assert len(tab._cells) == len(tab._rows) * 12
    row = next(r for r, n in tab._rows if n == "straight_line")
    _click(app, row.center)
    assert tab.selected == "straight_line"
    app.draw()
    seed = app.sim.config.seed
    _click(app, tab._use_rect.center)
    assert not app.menu.is_open
    assert app.sim.config.planner == "straight_line" and app.sim.config.seed == seed


def test_planners_filter_designed_for():
    app = _app()
    app.open_menu(tab=1)
    tab = app.menu.planners
    tab.filter.index = tab.filter.options.index("grid")
    assert tab.planners() == ["weighted_a_star"]
    tab.filter.index = tab.filter.options.index("movers")
    assert tab.planners() == []
    app.draw()


def test_hover_note_draws():
    app = _app()
    app.open_menu(tab=1)
    app.draw()
    cell = next(c for c in app.menu.planners._cells if c[1] == "weighted_a_star" and c[3] == "fog")
    app.handle(pygame.event.Event(pygame.MOUSEMOTION, pos=cell[0].center, rel=(0, 0), buttons=(0, 0, 0)))
    assert app.menu.planners.hover == ("weighted_a_star", "knowledge", "fog")
    app.draw()


def test_small_window_menu_fits():
    app = _app()
    app.handle(pygame.event.Event(pygame.VIDEORESIZE, w=800, h=560, size=(800, 560)))
    app.open_menu()
    app.draw()
    assert app.screen.get_rect().contains(app.menu.card)


def test_badges_and_wrap():
    surf = pygame.Surface((40, 40))
    for fit in Fit:
        draw_badge(surf, fit, (20, 20), 16)
    pygame.font.init()
    font = pygame.font.Font(None, 18)
    lines = wrap("one two three four five six seven eight nine ten", font, 60)
    assert len(lines) > 1 and all(font.size(ln)[0] <= 60 or " " not in ln for ln in lines)
