"""Dispatch the splat scene command-line tools."""
import importlib
import json
import os
from pathlib import Path
import sys
from typing import List

_dll_handles: List[object] = []
if sys.platform == 'win32':
    for folder in [Path(sys.executable).parent / 'Library/bin']:
        if folder.is_dir():
            _dll_handles.append(os.add_dll_directory(str(folder)))


def check_policy(path):
    path = Path(path) / 'config.json'
    config = json.loads(path.read_text(encoding='utf-8'))
    state = config.get('input_features', {}).get('observation.state', {}).get('shape')
    action = config.get('output_features', {}).get('action', {}).get('shape')
    if state != [5] or action != [5]:
        raise SystemExit(f'Incompatible checkpoint: state={state}, action={action}; this scene/plugin expects [5] and [5]. Train with the splat dataset or implement a deliberate legacy adapter first.')
    print('PASS: checkpoint dimensions are state=5, action=5. Camera names and action semantics must also match training.')


def _default_option(name, value):
    if not any(arg.startswith(name + '=') for arg in sys.argv[1:]):
        sys.argv.append(f'{name}={value}')


def main():
    entry_point = 'splat_tools.cmd' if sys.platform == 'win32' else './splat_tools.sh'
    if len(sys.argv) < 2 or sys.argv[1] in ('-h', '--help'):
        print(f'Usage: {entry_point} {{bridge|record|rollout|dagger|dagger-aggregate|episode-preview|train|ros2|check|check-policy}} [arguments]')
        print('Direct Python entry point: python scripts/splat_tools.py <command> [arguments]')
        print('bridge starts disabled; use ros2 service calls to enable manual/policy control.')
        return
    command = sys.argv.pop(1)
    if command == 'dagger':
        _default_option('--strategy.type', 'dagger')
        if '--strategy.type=dagger' not in sys.argv:
            raise SystemExit('The dagger command requires --strategy.type=dagger.')
        command = 'rollout'
    if command == 'check-policy':
        if len(sys.argv) != 2:
            raise SystemExit(f'Usage: {entry_point} check-policy <pretrained_model_directory>')
        check_policy(sys.argv[1])
        return
    if command == 'episode-preview':
        from lerobot_robot_ros2.episode_preview import main as run
        run()
        return
    # Every participant, including diagnostic/service ros2 commands, must use
    # the same process-local Fast DDS profile as Isaac Sim.
    from simulation_transport import configure_simulation_transport
    configure_simulation_transport()
    if command in ('check', 'record', 'rollout', 'dagger-aggregate', 'train'):
        # Load ROS and OpenCV before torch/LeRobot to avoid the Windows DLL conflict.
        for module in ('rclpy', 'cv2'):
            importlib.import_module(module)
    if command == 'check':
        from kaivuri_lerobot_bridge import control_bridge
        from lerobot_robot_isaac_excavator import kaivuri_record
        from lerobot_robot_isaac_excavator import kaivuri_rollout
        for module in ('simulation_dagger', 'dagger_aggregate'):
            importlib.import_module('lerobot_robot_isaac_excavator.' + module)
        importlib.import_module('lerobot.policies.smolvla.modeling_smolvla')
        print('PASS: ROS2, OpenCV, simulation bridge, recorder, SmolVLA rollout, DAgger and aggregation import successfully.')
        print('Python:', sys.executable)
        print('Recorder:', kaivuri_record.__file__)
        print('Bridge:', control_bridge.__file__)
        print('Rollout:', kaivuri_rollout.__file__)
        return
    if command == 'bridge':
        from kaivuri_lerobot_bridge.control_bridge import main as run
    elif command == 'record':
        _default_option('--dataset.push_to_hub', 'false')
        from lerobot_robot_isaac_excavator.kaivuri_record import main as run
    elif command == 'rollout':
        if '--strategy.type=dagger' in sys.argv:
            _default_option('--dataset.push_to_hub', 'false')
        for arg in sys.argv[1:]:
            if arg.startswith('--policy.path='):
                checkpoint = Path(arg.split('=', 1)[1])
                if checkpoint.is_dir():
                    check_policy(checkpoint)
        # This robot consumes joystick actions, not absolute joint-position targets.
        _default_option('--return_to_initial_position', 'false')
        from lerobot_robot_isaac_excavator.kaivuri_rollout import main as run
    elif command == 'ros2':
        from ros2cli.cli import main as run
    elif command == 'dagger-aggregate':
        from lerobot_robot_isaac_excavator.dagger_aggregate import main as run
    elif command == 'train':
        _default_option('--policy.push_to_hub', 'false')
        from lerobot.scripts.lerobot_train import main as run
    else:
        raise SystemExit('Unknown command: ' + command)
    result = run()
    if isinstance(result, int):
        raise SystemExit(result)


if __name__ == '__main__':
    main()
