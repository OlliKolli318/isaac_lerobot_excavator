"""ROS lifecycle and image conversion without creating a ROS participant."""
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS
from unittest.mock import Mock

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'lerobot_robot_isaac_excavator'))
import ros_bridge


@pytest.fixture
def ros(monkeypatch):
    module = ModuleType('rclpy')
    running = False

    def init(**kwargs):
        nonlocal running
        running = True

    def shutdown():
        nonlocal running
        running = False

    module.ok = lambda: running
    module.init = Mock(side_effect=init)
    module.shutdown = Mock(side_effect=shutdown)
    module.create_node = Mock(side_effect=lambda name: Mock())
    executors = ModuleType('rclpy.executors')
    executors.MultiThreadedExecutor = Mock(side_effect=lambda **kwargs: Mock())
    monkeypatch.setitem(sys.modules, 'rclpy', module)
    monkeypatch.setitem(sys.modules, 'rclpy.executors', executors)
    monkeypatch.setattr(ros_bridge, '_ACTIVE_CONTEXTS', 0)
    monkeypatch.setattr(ros_bridge, '_RCLPY_STARTED_BY_PLUGIN', False)
    monkeypatch.setattr(ros_bridge.threading, 'Thread', Mock(side_effect=lambda **kwargs: Mock(ident=1)))
    return NS(module=module, executors=executors)


def test_context_lifetime_is_shared_and_disconnect_is_idempotent(ros):
    first, second = ros_bridge.RosNodeContext('first'), ros_bridge.RosNodeContext('second')
    first.connect()
    second.connect()
    first.connect()
    assert ros_bridge._ACTIVE_CONTEXTS == 2
    ros.module.init.assert_called_once()
    first.disconnect()
    first.disconnect()
    ros.module.shutdown.assert_not_called()
    second.disconnect()
    ros.module.shutdown.assert_called_once()
    assert ros_bridge._ACTIVE_CONTEXTS == 0


def test_external_context_is_never_shutdown(ros):
    ros.module.init()
    context = ros_bridge.RosNodeContext('test')
    context.connect()
    context.disconnect()
    ros.module.shutdown.assert_not_called()


@pytest.mark.parametrize('failure', ['node', 'executor', 'thread'])
def test_partial_initialization_releases_all_resources_and_can_retry(ros, failure):
    context = ros_bridge.RosNodeContext('test')
    node = Mock()
    ros.module.create_node.side_effect = None
    ros.module.create_node.return_value = node
    if failure == 'node':
        ros.module.create_node.side_effect = RuntimeError('node failure')
    elif failure == 'executor':
        ros.executors.MultiThreadedExecutor.side_effect = RuntimeError('executor failure')
    else:
        ros_bridge.threading.Thread.side_effect = lambda **kwargs: Mock(
            ident=None, start=Mock(side_effect=RuntimeError('thread failure')))
    with pytest.raises(RuntimeError, match='failure'):
        context.connect()
    assert context.node is context.executor is context.thread is None
    assert ros_bridge._ACTIVE_CONTEXTS == 0
    ros.module.shutdown.assert_called_once()
    if failure != 'node':
        node.destroy_node.assert_called_once()
    ros.module.create_node.side_effect = lambda name: Mock()
    ros.executors.MultiThreadedExecutor.side_effect = lambda **kwargs: Mock()
    ros_bridge.threading.Thread.side_effect = lambda **kwargs: Mock(ident=1)
    context.connect()
    context.disconnect()
    assert ros_bridge._ACTIVE_CONTEXTS == 0


def test_cleanup_failure_still_destroys_node_and_releases_context(ros):
    context = ros_bridge.RosNodeContext('test')
    context.connect()
    node = context.node
    context.executor.shutdown.side_effect = RuntimeError('shutdown failure')
    with pytest.raises(RuntimeError, match='shutdown failure'):
        context.disconnect()
    node.destroy_node.assert_called_once()
    ros.module.shutdown.assert_called_once()
    assert ros_bridge._ACTIVE_CONTEXTS == 0


@pytest.mark.parametrize('encoding,pixel,rgb', [
    ('rgb8', [10, 20, 30], [10, 20, 30]), ('bgr8', [30, 20, 10], [10, 20, 30]),
    ('rgba8', [10, 20, 30, 255], [10, 20, 30]),
    ('bgra8', [30, 20, 10, 255], [10, 20, 30]), ('mono8', [10], [10, 10, 10]),
])
def test_image_encodings_support_padded_rows_and_own_their_data(encoding, pixel, rgb):
    data = bytearray((pixel + [99]) * 2)
    message = NS(height=2, width=1, step=len(pixel) + 1, encoding=encoding, data=data)
    result = ros_bridge.image_to_numpy(message, (2, 1, 3), '/image')
    np.testing.assert_array_equal(result, [[rgb], [rgb]])
    assert result.flags.c_contiguous
    data[0] = 0
    np.testing.assert_array_equal(result, [[rgb], [rgb]])


@pytest.mark.parametrize('step,data', [(2, b'12'), (3, b'12'), (3, b'1234')])
def test_bad_image_layout_is_rejected(step, data):
    message = NS(height=1, width=1, step=step, encoding='rgb8', data=data)
    with pytest.raises(ValueError, match='stride or buffer length'):
        ros_bridge.image_to_numpy(message, (1, 1, 3), '/image')


@pytest.mark.parametrize('value', [float('nan'), float('inf'), None, 'private-payload', 10**400])
def test_bad_numeric_samples_are_rejected_without_payload(value):
    with pytest.raises(ValueError) as caught:
        ros_bridge.float_multi_array_to_list(NS(data=[value]), 1, '/test')
    assert 'private-payload' not in str(caught.value)


def test_numeric_values_and_length_contract():
    assert ros_bridge.float_multi_array_to_list(NS(data=[1, '2']), 2, '/test') == [1., 2.]
    with pytest.raises(ValueError, match='expected 2'):
        ros_bridge.float_multi_array_to_list(NS(data=[1]), 2, '/test')
