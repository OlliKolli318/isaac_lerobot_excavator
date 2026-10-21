"""Command compatibility and offline defaults without starting ROS or LeRobot."""
import json
import sys
from types import ModuleType
from unittest.mock import Mock

import pytest

import splat_tools
import simulation_transport


@pytest.fixture
def commands(monkeypatch):
    monkeypatch.setattr(simulation_transport, 'configure_simulation_transport', Mock())
    modules = {}
    names = ('kaivuri_lerobot_bridge', 'kaivuri_lerobot_bridge.control_bridge',
             'lerobot_robot_isaac_excavator', 'lerobot_robot_isaac_excavator.kaivuri_record',
             'lerobot_robot_isaac_excavator.kaivuri_rollout',
             'lerobot', 'lerobot.scripts', 'lerobot.scripts.lerobot_train')
    for name in names:
        module = ModuleType(name)
        module.__file__ = name
        module.main = Mock(return_value=None)
        modules[name] = module
        monkeypatch.setitem(sys.modules, name, module)
        if '.' in name:
            parent, child = name.rsplit('.', 1)
            setattr(modules[parent], child, module)
    imports = Mock()
    monkeypatch.setattr(splat_tools.importlib, 'import_module', imports)
    return modules, imports


@pytest.mark.parametrize('command', ['record', 'dagger'])
def test_collection_defaults_to_local_output(monkeypatch, commands, command):
    monkeypatch.setattr(sys, 'argv', ['tools', command])
    splat_tools.main()
    assert '--dataset.push_to_hub=false' in sys.argv
    target = 'kaivuri_record' if command == 'record' else 'kaivuri_rollout'
    commands[0]['lerobot_robot_isaac_excavator.' + target].main.assert_called_once()


def test_explicit_options_are_preserved(monkeypatch, commands):
    monkeypatch.setattr(sys, 'argv', ['tools', 'dagger', '--dataset.push_to_hub=true',
                                    '--return_to_initial_position=true'])
    splat_tools.main()
    assert '--dataset.push_to_hub=false' not in sys.argv
    assert '--return_to_initial_position=false' not in sys.argv


def test_dagger_rejects_a_different_strategy(monkeypatch, commands):
    monkeypatch.setattr(sys, 'argv', ['tools', 'dagger', '--strategy.type=base'])
    with pytest.raises(SystemExit, match='requires'):
        splat_tools.main()


def test_check_actually_imports_dagger_and_aggregation(monkeypatch, commands):
    monkeypatch.setattr(sys, 'argv', ['tools', 'check'])
    splat_tools.main()
    assert [call.args[0] for call in commands[1].call_args_list] == [
        'rclpy', 'cv2', 'lerobot_robot_isaac_excavator.simulation_dagger',
        'lerobot_robot_isaac_excavator.dagger_aggregate',
        'lerobot.policies.smolvla.modeling_smolvla']


@pytest.mark.parametrize('state,action', [([6], [5]), ([5], [6]), (None, None)])
def test_incompatible_checkpoints_are_rejected(tmp_path, state, action):
    (tmp_path / 'config.json').write_text(json.dumps({
        'input_features': {'observation.state': {'shape': state}},
        'output_features': {'action': {'shape': action}}}))
    with pytest.raises(SystemExit, match='Incompatible checkpoint'):
        splat_tools.check_policy(tmp_path)


def test_compatible_checkpoint(tmp_path):
    (tmp_path / 'config.json').write_text(json.dumps({
        'input_features': {'observation.state': {'shape': [5]}},
        'output_features': {'action': {'shape': [5]}}}))
    splat_tools.check_policy(tmp_path)


@pytest.mark.parametrize('explicit', [None, 'true', 'false'])
def test_training_defaults_to_local_output_without_overriding_explicit_choice(monkeypatch, commands, explicit):
    arguments = ['tools', 'train']
    if explicit is not None:
        arguments.append('--policy.push_to_hub=' + explicit)
    monkeypatch.setattr(sys, 'argv', arguments)
    splat_tools.main()
    options = [arg for arg in sys.argv if arg.startswith('--policy.push_to_hub=')]
    assert options == ['--policy.push_to_hub=' + (explicit or 'false')]
    commands[0]['lerobot.scripts.lerobot_train'].main.assert_called_once()
