"""Run real callback methods against synthetic messages and publishers."""
import ast
import __future__
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from kaivuri_lerobot_bridge.action_contract import InvalidAction, PolicyWatchdog, normalize_action
from kaivuri_lerobot_bridge.dagger_control import DaggerLease, decode_control


@pytest.fixture
def bridge():
    path = Path(__file__).resolve().parents[1] / 'kaivuri_lerobot_bridge/control_bridge.py'
    tree = ast.parse(path.read_text())
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'ControlBridge')
    namespace = dict(Node=object, InvalidAction=InvalidAction, PolicyWatchdog=PolicyWatchdog,
                     normalize_action=normalize_action, DaggerLease=DaggerLease, decode_control=decode_control,
                     time=NS(monotonic=lambda: 100.), Float32MultiArray=NS, Bool=NS, String=NS)
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(path), 'exec',
                 flags=__future__.annotations.compiler_flag), namespace)
    cls = namespace['ControlBridge']
    node = cls.__new__(cls)
    node._action_names = ['slew.pos', 'arm.pos', 'bucket.pos', 'boom.pos', 'claw.pos']
    node._safe_action, node._axis_count = (0, 0, 0, 0, 1), 4
    node._watchdog = node._new_watchdog(0.4)
    node._dagger = DaggerLease()
    node._simulation, node._enabled, node._policy_mode = True, True, True
    node._sim_time, node._run_id = 1., 'run1'
    node._retired_runs, node._run_arrived_before_reset = set(), False
    node._last_packet_ns, node._last_packet_wall = -1, None
    node._last_status, node._last_rejection = '', ''
    for name in ('_stamped_pub', '_scene_action_pub', '_simulation_pub', '_manual_mode_pub',
                 '_dagger_pub', '_enabled_pub', '_status_pub'):
        setattr(node, name, Mock())
    node.get_logger = Mock(return_value=Mock())
    node.get_parameter = lambda name: NS(value=10.)
    return node


def action(bridge, *, ns=1_000_000_000, run='run1'):
    sec, nanosec = divmod(ns, 1_000_000_000)
    return NS(header=NS(stamp=NS(sec=sec, nanosec=nanosec), frame_id=run),
              name=list(bridge._action_names), position=[2, 0, 0, 0, 0])


def test_valid_stamped_action_and_duplicate_rejection(bridge):
    message = action(bridge)
    bridge._on_stamped_policy(message)
    assert message.position == [1, 0, 0, 0, 0]
    bridge._on_stamped_policy(action(bridge))
    assert bridge._stamped_pub.publish.call_count == 1


@pytest.mark.parametrize('run,ns', [('other', 1_000_000_000), ('run1', 800_000_000),
                                  ('run1', 2_100_000_000)])
def test_invalid_stamped_action_is_never_forwarded(bridge, run, ns):
    bridge._on_stamped_policy(action(bridge, ns=ns, run=run))
    bridge._stamped_pub.publish.assert_not_called()


def test_disabled_bridge_does_not_forward(bridge):
    bridge._enabled = False
    bridge._on_stamped_policy(action(bridge))
    bridge._stamped_pub.publish.assert_not_called()


def test_clock_reset_invalidates_packet_and_run(bridge):
    bridge._on_stamped_policy(action(bridge))
    bridge._on_clock(NS(clock=NS(sec=0, nanosec=0)))
    assert bridge._last_packet_ns == -1
    assert bridge._last_packet_wall is None
    assert bridge._run_id is None
    assert 'run1' in bridge._retired_runs
    assert bridge._watchdog.select(0, True, True).reason == 'policy_waiting'


def test_sample_run_change_invalidates_transport_freshness(bridge):
    bridge._on_stamped_policy(action(bridge))
    bridge._on_sample(action(bridge, ns=0, run='run2'))
    assert bridge._run_id == 'run2'
    assert bridge._last_packet_wall is None


def test_simulation_heartbeat_does_not_publish_wall_time_action(bridge):
    bridge._publish_tick()
    bridge._scene_action_pub.publish.assert_not_called()
    assert bridge._enabled_pub.publish.call_args.args[0].data is False
