"""Recorder failure handling with local dependency doubles."""
import ast
from contextlib import ExitStack
from dataclasses import dataclass
import logging
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

SOURCE = Path(__file__).resolve().parents[1] / 'lerobot_robot_isaac_excavator'


def load_record(namespace):
    tree = ast.parse((SOURCE / 'kaivuri_record.py').read_text())
    function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'record')
    function.decorator_list = []
    exec(compile(ast.Module(body=[function], type_ignores=[]), 'kaivuri_record.py', 'exec'), namespace)
    return namespace['record']


@pytest.fixture
def recorder():
    order = []
    dataset = Mock(num_episodes=1, has_pending_frames=Mock(return_value=True))
    dataset.finalize.side_effect = lambda: order.append('dataset')
    robot, teleop, bridge, listener = Mock(is_connected=True), Mock(is_connected=True), Mock(), Mock()
    for name, resource, method in [('robot', robot, 'disconnect'), ('teleop', teleop, 'disconnect'),
                                   ('bridge', bridge, 'stop'), ('listener', listener, 'stop')]:
        getattr(resource, method).side_effect = lambda name=name: order.append(name)
    bridge.wait_for_start.return_value = True
    bridge.finish_episode.return_value = 'end_save'
    cfg = NS(time_source='simulation', robot=NS(), teleop=NS(), display_data=False,
             display_compressed_images=False, display_mode=None, play_sounds=False,
             end_save_command='end_save', end_discard_command='end_discard',
             saved_status='saved', discarded_status='discarded',
             dataset=NS(fps=30, push_to_hub=True, tags=[], private=True, video=False,
                        num_episodes=1, episode_time_s=1, single_task='synthetic task'))
    loop = Mock()
    namespace = dict(
        ExitStack=ExitStack, logging=logging, KaivuriRecordConfig=NS, RobotProcessorPipeline=object,
        LeRobotDataset=object, init_logging=Mock(), make_robot_from_config=lambda _: robot,
        make_teleoperator_from_config=lambda _: teleop,
        combine_feature_dicts=lambda *args: {}, aggregate_pipeline_dataset_features=lambda **kw: {},
        upstream=NS(create_initial_features=lambda **kw: {}), TIME_FEATURES=('sim_time_ns',),
        _make_dataset=lambda *args: dataset,
        init_keyboard_listener=lambda: (listener, dict(stop_recording=False, exit_early=False, rerecord_episode=False)),
        EpisodeCommandBridge=lambda *args: bridge, VideoEncodingManager=lambda _: ExitStack(),
        log_say=Mock(), simulation_record_loop=loop,
        ClockReset=type('ClockReset', (RuntimeError,), {}), MissingSample=type('MissingSample', (RuntimeError,), {}))
    function = load_record(namespace)
    return NS(run=lambda: function(cfg, lambda v: v, lambda v: v, lambda v: v),
              dataset=dataset, loop=loop, order=order)


def test_success_preserves_explicit_upload_and_shutdown_order(recorder):
    assert recorder.run() is recorder.dataset
    assert recorder.order == ['dataset', 'bridge', 'robot', 'teleop', 'listener']
    recorder.dataset.push_to_hub.assert_called_once_with(tags=[], private=True)


def test_recording_failure_cleans_up_without_upload(recorder):
    recorder.loop.side_effect = RuntimeError('record failure')
    with pytest.raises(RuntimeError, match='record failure'):
        recorder.run()
    assert recorder.order == ['dataset', 'bridge', 'robot', 'teleop', 'listener']
    recorder.dataset.push_to_hub.assert_not_called()


def test_finalization_failure_still_stops_ros_and_listener(recorder):
    recorder.dataset.finalize.side_effect = RuntimeError('finalize failure')
    with pytest.raises(RuntimeError, match='finalize failure'):
        recorder.run()
    assert recorder.order == ['bridge', 'robot', 'teleop', 'listener']
    recorder.dataset.push_to_hub.assert_not_called()


def test_local_dataset_defaults_and_explicit_upload():
    @dataclass
    class UpstreamDataset:
        repo_id: str
        push_to_hub: bool = True

    tree = ast.parse((SOURCE / 'kaivuri_record.py').read_text())
    config = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'LocalDatasetRecordConfig')
    namespace = {'dataclass': dataclass, 'upstream': NS(DatasetRecordConfig=UpstreamDataset)}
    exec(compile(ast.Module(body=[config], type_ignores=[]), 'kaivuri_record.py', 'exec'), namespace)
    local = namespace['LocalDatasetRecordConfig']
    assert not local('local/example').push_to_hub
    assert local('local/example', push_to_hub=True).push_to_hub
    # Both CLI configuration classes must select the same local default.
    for filename, name in [('kaivuri_record.py', 'KaivuriRecordConfig'),
                           ('kaivuri_rollout.py', 'SimulationRolloutConfig')]:
        tree = ast.parse((SOURCE / filename).read_text())
        config = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == name)
        field = next(n for n in config.body if isinstance(n, ast.AnnAssign) and n.target.id == 'dataset')
        assert 'LocalDatasetRecordConfig' in ast.unparse(field.annotation)
