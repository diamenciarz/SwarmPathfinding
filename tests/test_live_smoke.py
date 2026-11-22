"""Headless smoke tests for the Pygame lab: build, step, click, edit, resize."""

import os

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

import pygame  # noqa: E402
import pytest  # noqa: E402

from swarmpathfinding.core.geom import Vec2  # noqa: E402
from swarmpathfinding.viz.live import LiveApp  # noqa: E402
from swarmpathfinding.viz.settings import PRESETS, LabSettings  # noqa: E402
from swarmpathfinding.worlds.continuous import ContinuousWorld  # noqa: E402


def _click(app, pos, button=1):
    app.handle(pygame.event.Event(pygame.MOUSEBUTTONDOWN, pos=pos, button=button))
    app.handle(pygame.event.Event(pygame.MOUSEBUTTONUP, pos=pos, button=button))


def _key(app, key):
    app.handle(pygame.event.Event(pygame.KEYDOWN, key=key))
    app.handle(pygame.event.Event(pygame.KEYUP, key=key))


def _run(app, steps=30):
    app.paused = False
    for _ in range(steps):
        app.step(1 / 30)
    app.draw()


@pytest.mark.parametrize("preset", sorted(PRESETS))
def test_presets_run(preset):
    app = LiveApp(PRESETS[preset])
    _run(app)
    assert app.scenario is not None


def test_agents_reach_goal_on_grid():
    app = LiveApp(LabSettings(map_kind="bernoulli", n_agents=4, fow=False))
    _run(app, 900)
    assert all(app.results[a.id].success for a in app.scenario.agents)
    assert all(a.pos.dist(app._goal_for(a)) < 0.6 for a in app.scenario.agents)


def test_agents_never_enter_walls_on_grid():
    app = LiveApp(LabSettings(map_kind="clustered", n_agents=10, fow=True, starts="scattered"))
    app.paused = False
    world = app.scenario.world
    for _ in range(300):
        app.step(1 / 30)
        for a in app.scenario.agents:
            assert not world.is_blocked(world.world_to_cell(a.pos))


def test_panel_change_rebuilds_and_planner_switch_keeps_map():
    app = LiveApp(LabSettings(n_agents=5))
    app.draw()
    agents = app.panel.get("n_agents")
    agents.value = 12
    app._on_panel_change("n_agents", True)
    assert len(app.scenario.agents) == 12
    world_before = app.scenario.world
    app.panel.get("planner").index = app.panel.get("planner").options.index("straight_line")
    app._on_panel_change("planner", True)
    assert app.scenario.world is world_before
    assert app.planner.name == "straight_line"
    assert not app.panel.get("w").visible


def test_world_toggle_swaps_maps_and_greys_los():
    app = LiveApp(LabSettings())
    app.draw()
    world = app.panel.get("world_kind")
    _click(app, (world.rect.right - 5, world.rect.centery))
    assert isinstance(app.scenario.world, ContinuousWorld)
    assert app.panel.get("map_kind").value == "polygons"
    assert not app.panel.get("los").enabled


def test_goal_mode_moves_every_agents_goal():
    app = LiveApp(LabSettings(world_kind="grid", map_kind="empty", n_agents=3))
    app.draw()
    target = app.camera.world_to_screen(Vec2(5.5, 5.5))
    app.handle(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_g))
    _click(app, target)
    app.handle(pygame.event.Event(pygame.KEYUP, key=pygame.K_g))
    for a in app.scenario.agents:
        assert a.goal.dist(Vec2(5.5, 5.5)) < 1.0


def test_continuous_add_and_delete_keep_physics_in_sync():
    app = LiveApp(LabSettings(world_kind="continuous", map_kind="sparse", fow=False, n_agents=2))
    app.draw()
    world = app.scenario.world
    n0 = len(world.obstacles)
    shapes0 = len(app.space.shapes)
    pos = app.camera.world_to_screen(Vec2(3.0, 3.0))
    _click(app, pos, 1)
    assert len(world.obstacles) == n0 + 1
    assert len(app.space.shapes) == shapes0 + 1
    new_id = world.obstacles[-1].id
    assert new_id in app.scenario.belief.obstacles  # fog off: planner knows at once
    _click(app, pos, 3)
    assert len(world.obstacles) == n0
    assert len(app.space.shapes) == shapes0
    assert new_id not in app.scenario.belief.obstacles


def test_pending_edit_continuous_removes_body():
    app = LiveApp(LabSettings(world_kind="continuous", map_kind="polygons", fow=False, n_agents=1))
    shapes0 = len(app.space.shapes)
    assert app.scenario.pending_edit is not None
    _key(app, pygame.K_e)
    assert app.scenario.pending_edit is None
    assert len(app.space.shapes) == shapes0 - 1


def test_weight_changes_expansions():
    app = LiveApp(LabSettings(map_kind="bernoulli", n_agents=1, fow=False, seed=3))
    e1 = app.results[app._focus_id()].expansions
    app.panel.get("w").value = 3.0
    app._on_panel_change("w", True)
    e3 = app.results[app._focus_id()].expansions
    assert e3 < e1


def test_resize_keeps_zoom():
    app = LiveApp(LabSettings())
    app.camera.zoom_at(300, 300, 1.5)
    zoom = app.camera.zoom
    app.handle(pygame.event.Event(pygame.VIDEORESIZE, w=1000, h=700, size=(1000, 700)))
    app.draw()
    assert app.camera.zoom == zoom
    assert app.panel.rect.right == 1000


def test_half_visible_obstacle_keeps_its_hidden_part_drawn():
    """An obstacle that sticks out of view must stay drawn (dimmed) outside the
    vision circle, not get cut off at its edge."""
    from shapely.geometry import box

    from swarmpathfinding.worlds.continuous import PolygonObstacle

    app = LiveApp(
        LabSettings(world_kind="continuous", map_kind="sparse", fow=True, n_agents=1, vision_radius=4.0)
    )
    world = app.scenario.world
    world.obstacles = [PolygonObstacle(id="wall", polygon=box(20.0, 10.0, 30.0, 12.0))]
    world.invalidate()
    app._sync_physics()
    agent = app.scenario.agents[0]
    agent.pos = Vec2(20.0, 16.0)  # sees the left end of the wall only
    if agent.body is not None:
        agent.body.position = (20.0, 16.0)
    app.scenario.belief.obstacles.clear()
    app._reveal()
    assert "wall" in app.seen_ids
    remembered, ghosts = app._fog_memory()
    assert [r.id for r in remembered] == ["wall"] and not ghosts

    app.draw()
    hidden = app.camera.world_to_screen(Vec2(28.0, 11.0))  # far end, out of view
    floor = app.camera.world_to_screen(Vec2(28.0, 14.0))  # out of view, no wall
    wall_px = app.screen.get_at(hidden)[:3]
    floor_px = app.screen.get_at(floor)[:3]
    assert sum(wall_px) > sum(floor_px) + 30
