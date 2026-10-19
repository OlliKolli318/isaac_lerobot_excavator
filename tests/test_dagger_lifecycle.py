"""DAgger cleanup tests that never start a participant or upload data."""
import ast
from contextlib import ExitStack
import copy
import logging
from pathlib import Path
import queue
import secrets
import sys
from types import ModuleType, SimpleNamespace as NS
from unittest.mock import Mock

import pytest

SOURCE = Path(__file__).resolve().parents[1] / 'lerobot_robot_isaac_excavator'


def definitions(filename, names, namespace):
    tree = ast.parse((SOURCE / filename).read_text())
    nodes = [node for node in tree.body if isinstance(node, (ast.FunctionDef, ast.ClassDef)) and node.name in names]
    exec(compile(ast.Module(body=nodes, type_ignores=[]), filename, 'exec'), namespace)
    return namespace


@pytest.mark.parametrize('failure', ['run', 'hold', 'engine', 'finalize', None])
def test_dagger_cleanup_releases_all_resources_on_failure(failure):
    dataset = Mock(num_episodes=1, has_pending_frames=Mock(return_value=True))
    robot, link, listener, runner = Mock(is_connected=True), Mock(), Mock(), Mock(active=True)
    failing = {'run': runner.run, 'hold': link.phase, 'engine': runner.engine.stop,
               'finalize': dataset.finalize}
    if failure:
        failing[failure].side_effect = RuntimeError('injected failure')
    cfg = NS(continue_after_success=False, fps=30, task='synthetic task',
             robot=NS(manage_control_mode=False, start_mode='unchanged'),
             teleop=NS(type='isaac_excavator_action'),
             strategy=NS(record_autonomous=False, input_device='keyboard', num_episodes=1,
                         target_video_file_size_mb=None, keyboard=NS()),
             dataset=NS(fps=30, episode_time_s=1, repo_id='local/rollout_test', video=True,
                        push_to_hub=True, tags=[], private=True))
    ctx = NS(runtime=NS(cfg=cfg), hardware=NS(robot_wrapper=NS(inner=robot)),
             data=NS(dataset_features={}))
    namespace = definitions('simulation_dagger.py', {'run_dagger', '_discard_and_finalize'}, dict(
        copy=copy, ExitStack=ExitStack, BaseStrategyConfig=NS, TIME_FEATURES=('sim_time_ns',),
        build_rollout_context=lambda *args: ctx, _make_dataset=lambda *args: dataset,
        DaggerLink=lambda **kwargs: link, DAggerEvents=NS,
        init_simulation_dagger_keyboard=lambda *args: listener, SimulationDagger=lambda *args: runner))
    if failure:
        with pytest.raises(RuntimeError, match='injected failure'):
            namespace['run_dagger'](cfg, Mock())
        dataset.push_to_hub.assert_not_called()
    else:
        assert namespace['run_dagger'](cfg, Mock()) is dataset
        dataset.push_to_hub.assert_called_once_with(tags=[], private=True)
    for call in (runner.engine.stop, listener.stop, dataset.clear_episode_buffer,
                 dataset.finalize, link.close, robot.disconnect):
        call.assert_called_once()


def test_rollout_cleanup_survives_inference_stop_failure():
    robot, teleop = Mock(is_connected=True), Mock(is_connected=True)
    robot.timeline.time.return_value = (0, 0)
    robot.timeline.run_id = None
    engine = Mock(start=Mock(side_effect=RuntimeError('start failure')),
                  stop=Mock(side_effect=RuntimeError('stop failure')))
    ctx = NS(runtime=NS(cfg=NS(), shutdown_event=Mock()),
             hardware=NS(robot_wrapper=NS(inner=robot), teleop=teleop))
    namespace = definitions('kaivuri_rollout.py', {'run_simulation_rollout'},
                            dict(ExitStack=ExitStack, SimulationPolicy=lambda _: engine))
    with pytest.raises(RuntimeError, match='stop failure'):
        namespace['run_simulation_rollout'](ctx)
    robot.disconnect.assert_called_once()
    teleop.disconnect.assert_called_once()


def test_dagger_link_constructor_releases_partial_ros_connection(monkeypatch):
    messages, clock = ModuleType('std_msgs.msg'), ModuleType('rclpy.clock')
    messages.String = NS
    clock.Clock, clock.ClockType = Mock(), NS(STEADY_TIME=1)
    monkeypatch.setitem(sys.modules, 'std_msgs.msg', messages)
    monkeypatch.setitem(sys.modules, 'rclpy.clock', clock)
    ros = Mock()
    ros.node.create_publisher.side_effect = RuntimeError('publisher failure')
    namespace = definitions('simulation_dagger.py', {'DaggerLink'}, dict(
        ExitStack=ExitStack, RosNodeContext=lambda _: ros, queue=queue, secrets=secrets,
        time=NS(monotonic=lambda: 0.), logging=logging))
    with pytest.raises(RuntimeError, match='publisher failure'):
        namespace['DaggerLink']()
    ros.disconnect.assert_called_once()
