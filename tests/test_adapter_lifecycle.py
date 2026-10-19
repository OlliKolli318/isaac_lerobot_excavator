"""Exercise adapter connection boundaries without importing deployment packages."""
import ast
from functools import cached_property
import logging
from pathlib import Path
import threading
import time
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from ros_bridge import finite_float_values

SOURCE = Path(__file__).resolve().parents[1] / 'lerobot_robot_isaac_excavator'


def adapter_class(filename, name):
    tree = ast.parse((SOURCE / filename).read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == name)
    namespace = dict(Robot=object, Teleoperator=object, IsaacExcavatorConfig=NS,
                     IsaacExcavatorActionConfig=NS, RobotAction=dict, RobotObservation=dict,
                     cached_property=cached_property, threading=threading, time=time,
                     logger=logging.getLogger('adapter_test'), Any=object,
                     check_if_already_connected=lambda method: method,
                     check_if_not_connected=lambda method: method,
                     finite_float_values=finite_float_values)
    exec(compile(ast.Module(body=[cls], type_ignores=[]), filename, 'exec'), namespace)
    return namespace[name]


@pytest.mark.parametrize('filename,name', [('isaac_excavator.py', 'IsaacExcavator'),
                                         ('isaac_excavator_action.py', 'IsaacExcavatorAction')])
def test_connection_failure_disconnects_and_can_retry(filename, name):
    cls = adapter_class(filename, name)
    robot = cls.__new__(cls)
    robot.config = NS(start_mode='unchanged')
    robot._ros = Mock()
    robot._connected = False
    robot._connect_ros = Mock(side_effect=TimeoutError('missing messages'))
    with pytest.raises(TimeoutError):
        robot.connect()
    robot._ros.disconnect.assert_called_once()
    assert not robot.is_connected
    robot._connect_ros.side_effect = None
    robot.connect()
    assert robot.is_connected
    robot.disconnect()
    assert not robot.is_connected
    if name == 'IsaacExcavator':
        assert robot._stamped_action_publisher is None


def test_state_contract_and_legacy_conversion_reject_nonfinite_values():
    cls = adapter_class('isaac_excavator.py', 'IsaacExcavator')
    robot = cls.__new__(cls)
    robot.config = NS(state_names=['a', 'b', 'c', 'd', 'claw'], state_topic='/state',
                      allow_legacy_six_value_state=True, legacy_claw_closed_threshold_rad=0.2)
    robot._legacy_state_warning_emitted = False
    assert robot._normalize_state(NS(data=[1, 2, 3, 4, 0])) == [1, 2, 3, 4, 0]
    assert robot._normalize_state(NS(data=[1, 2, 3, 4, 0.1, -0.1])) == [1, 2, 3, 4, 1]
    assert robot._normalize_state(NS(data=[1, 2, 3, 4, 0.3, -0.3])) == [1, 2, 3, 4, 0]
    for values in ([float('nan'), 0, 0, 0, 1], [0, 0, 0, 0, float('inf'), 0]):
        with pytest.raises(ValueError, match='finite'):
            robot._normalize_state(NS(data=values))
