"""Dispatch the splat scene command-line tools."""
import os
import json
from pathlib import Path
import sys

_dll_handles = []
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


def main():
    if len(sys.argv) < 2 or sys.argv[1] in ('-h', '--help'):
        print('Usage: splat_tools.cmd {bridge|record|rollout|dagger|dagger-aggregate|episode-preview|train|ros2|check|check-policy} [arguments]')
        print('bridge starts disabled; use ros2 service calls to enable manual/policy control.')
        return
    command = sys.argv.pop(1)
    if command == 'dagger':
        if not any(arg.startswith('--strategy.type=') for arg in sys.argv):
            sys.argv.append('--strategy.type=dagger')
        if '--strategy.type=dagger' not in sys.argv:
            raise SystemExit('The dagger command requires --strategy.type=dagger.')
        command = 'rollout'
    if command == 'check-policy':
        if len(sys.argv) != 2:
            raise SystemExit('Usage: splat_tools.cmd check-policy C:/path/to/pretrained_model')
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
    if command == 'check':
        import rclpy
        import cv2
        from kaivuri_lerobot_bridge import control_bridge
        from lerobot_robot_isaac_excavator import kaivuri_record
        from lerobot_robot_isaac_excavator import kaivuri_rollout
        print('PASS: ROS2, OpenCV, simulation bridge, recorder, rollout, DAgger and aggregation import successfully.')
        print('Recorder:', kaivuri_record.__file__)
        print('Bridge:', control_bridge.__file__)
        print('Rollout:', kaivuri_rollout.__file__)
        return
    if command in ('record', 'rollout', 'dagger-aggregate', 'train'):
        # Load ROS and OpenCV before torch/LeRobot to avoid the Windows DLL conflict.
        import rclpy
        import cv2
    if command == 'bridge':
        from kaivuri_lerobot_bridge.control_bridge import main as run
    elif command == 'record':
        if not any(arg.startswith('--dataset.push_to_hub=') for arg in sys.argv):
            sys.argv.append('--dataset.push_to_hub=false')
        from lerobot_robot_isaac_excavator.kaivuri_record import main as run
    elif command == 'rollout':
        if '--strategy.type=dagger' in sys.argv and not any(arg.startswith('--dataset.push_to_hub=') for arg in sys.argv):
            sys.argv.append('--dataset.push_to_hub=false')
        for arg in sys.argv[1:]:
            if arg.startswith('--policy.path='):
                checkpoint = Path(arg.split('=', 1)[1])
                if checkpoint.is_dir():
                    check_policy(checkpoint)
        # This robot consumes joystick actions, not absolute joint-position targets.
        if not any(arg.startswith('--return_to_initial_position=') for arg in sys.argv):
            sys.argv.append('--return_to_initial_position=false')
        from lerobot_robot_isaac_excavator.kaivuri_rollout import main as run
    elif command == 'ros2':
        from ros2cli.cli import main as run
    elif command == 'dagger-aggregate':
        from lerobot_robot_isaac_excavator.dagger_aggregate import main as run
    elif command == 'train':
        from lerobot.scripts.lerobot_train import main as run
    else:
        raise SystemExit('Unknown command: ' + command)
    result = run()
    if isinstance(result, int):
        raise SystemExit(result)


if __name__ == '__main__':
    main()
