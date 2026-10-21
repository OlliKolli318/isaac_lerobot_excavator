"""Launcher lifecycle checks using a synthetic Isaac application."""
import sys
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock

import pytest

import launch_splat_simulation as launcher


@pytest.fixture
def runtime(monkeypatch, tmp_path):
    scene = tmp_path / 'scene.usd'
    scene.touch()
    monkeypatch.setattr(sys, 'argv', ['launcher', '--scene', str(scene)])
    monkeypatch.setattr(launcher, 'isaac_root', lambda: tmp_path)
    monkeypatch.setattr(launcher, 'configure_simulation_transport', Mock())
    monkeypatch.setattr(launcher, 'configure_camera_delivery', Mock())
    monkeypatch.setattr(launcher.os, 'environ', {})
    app, timeline, context = Mock(), Mock(), Mock()
    app.is_running.side_effect = [True, True, False]
    manager = Mock()
    modules = {name: ModuleType(name) for name in
               ('isaacsim', 'omni', 'omni.kit', 'omni.kit.app', 'omni.timeline', 'omni.usd')}
    modules['isaacsim'].SimulationApp = Mock(return_value=app)
    modules['omni'].kit = modules['omni.kit']
    modules['omni'].timeline = modules['omni.timeline']
    modules['omni'].usd = modules['omni.usd']
    modules['omni.kit'].app = modules['omni.kit.app']
    modules['omni.kit.app'].get_app = lambda: SimpleNamespace(get_extension_manager=lambda: manager)
    modules['omni.timeline'].get_timeline_interface = lambda: timeline
    modules['omni.usd'].get_context = lambda: context
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)
    return SimpleNamespace(app=app, timeline=timeline, context=context, manager=manager)


def test_normal_launch_stops_and_closes(runtime):
    launcher.main()
    runtime.timeline.play.assert_called_once()
    runtime.timeline.stop.assert_called_once()
    runtime.app.close.assert_called_once()


def test_headless_launch_keeps_scene_playback(runtime, monkeypatch):
    monkeypatch.setattr(sys, 'argv', [*sys.argv, '--headless'])
    launcher.main()
    settings = sys.modules['isaacsim'].SimulationApp.call_args.args[0]
    assert settings['headless'] is True
    assert settings['hide_ui'] is True
    runtime.timeline.play.assert_called_once()
    runtime.app.close.assert_called_once()


def test_stage_failure_closes_application(runtime):
    runtime.context.open_stage.side_effect = RuntimeError('stage failure')
    with pytest.raises(RuntimeError, match='stage failure'):
        launcher.main()
    runtime.timeline.play.assert_not_called()
    runtime.app.close.assert_called_once()


def test_close_during_stage_loading_does_not_start_timeline(runtime):
    runtime.app.is_running.side_effect = [False, False]
    launcher.main()
    runtime.timeline.play.assert_not_called()
    runtime.app.close.assert_called_once()


def test_stop_failure_still_closes_application(runtime):
    runtime.timeline.stop.side_effect = RuntimeError('stop failure')
    with pytest.raises(RuntimeError, match='stop failure'):
        launcher.main()
    runtime.app.close.assert_called_once()
