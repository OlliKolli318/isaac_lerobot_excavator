"""Exercise Bash wrappers with fake runtimes; no ROS participants or GPU."""
import json
import os
from pathlib import Path
import shutil
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]
pytestmark = pytest.mark.skipif(shutil.which('bash') is None, reason='Bash required')


@pytest.fixture
def workspace(tmp_path):
    scene = tmp_path / 'workspace with spaces' / 'scene'
    (scene / 'scripts').mkdir(parents=True)
    for filename in ('launch_splat_simulation.sh', 'splat_tools.sh'):
        shutil.copyfile(ROOT / filename, scene / filename)
    env = os.environ.copy()
    for key in ('AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH', 'CONDA_PREFIX',
                'VIRTUAL_ENV', 'PYTHONEXE', 'PYTHONHOME', 'ROS_DISTRO',
                'ISAAC_SIM_ROOT', 'ISAAC_ROS_DISTRO', 'ROS_SETUP',
                'ROS_LOCAL_SETUP', 'LEROBOT_VENV'):
        env.pop(key, None)
    env['RESULT_FILE'] = str(tmp_path / 'result.json')
    return scene, env


def fake_python(path):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        '#!/usr/bin/env python3\n'
        'import json, os, sys\n'
        'with open(os.environ["RESULT_FILE"], "w") as f:\n'
        '    json.dump({"args": sys.argv[1:], "env": dict(os.environ)}, f)\n'
        'sys.exit(7)\n')
    path.chmod(0o755)


def test_isaac_wrapper_uses_bundled_runtime_and_preserves_arguments(workspace):
    scene, env = workspace
    isaac = scene.parent / 'Isaac Sim'
    fake_python(isaac / 'python.sh')
    lib = isaac / 'exts/isaacsim.ros2.core/jazzy/lib'
    lib.mkdir(parents=True)
    env.update(ISAAC_SIM_ROOT=str(isaac), VIRTUAL_ENV='/unused/policy',
               PYTHONEXE='/must/not/be/used', LD_LIBRARY_PATH='/existing/lib')
    result = subprocess.run(
        ['bash', str(scene / 'launch_splat_simulation.sh'), '--headless',
         '--scene', '/a scene/env.usd'], env=env, cwd='/tmp', capture_output=True)
    assert result.returncode == 7
    captured = json.loads(Path(env['RESULT_FILE']).read_text())
    assert captured['args'] == [
        str(scene / 'scripts/launch_splat_simulation.py'), '--headless',
        '--scene', '/a scene/env.usd']
    assert captured['env']['LD_LIBRARY_PATH'] == f'{lib}:/existing/lib'
    assert 'PYTHONEXE' not in captured['env']
    assert 'VIRTUAL_ENV' not in captured['env']


def test_isaac_wrapper_rejects_system_ros_environment(workspace):
    scene, env = workspace
    fake_python(scene / 'isaac/python.sh')
    env.update(ISAAC_SIM_ROOT=str(scene / 'isaac'), AMENT_PREFIX_PATH='/opt/ros/jazzy')
    result = subprocess.run(['bash', str(scene / 'launch_splat_simulation.sh')],
                            env=env, text=True, capture_output=True)
    assert result.returncode == 1
    assert 'fresh terminal' in result.stderr
    assert not Path(env['RESULT_FILE']).exists()


@pytest.mark.parametrize('isaac_distro', [None, 'jazzy', 'humble'])
def test_launchers_select_ros_distributions_independently(workspace, isaac_distro):
    scene, env = workspace
    isaac = scene / 'isaac'
    fake_python(isaac / 'python.sh')
    expected_distro = isaac_distro or 'jazzy'
    lib = isaac / f'exts/isaacsim.ros2.core/{expected_distro}/lib'
    lib.mkdir(parents=True)
    venv = scene.parent / 'policy venv'
    fake_python(venv / 'bin/python')
    (venv / 'bin/activate').write_text('export POLICY_ENV_LOADED=yes\n')
    ros = scene / 'lyrical.bash'
    ros.write_text('export ROS_ENV_LOADED=yes\n')
    settings = (
        f'export ISAAC_SIM_ROOT="{isaac}"\n'
        'export ROS_DISTRO=lyrical\n'
        f'export ROS_SETUP="{ros}"\n'
        f'export LEROBOT_VENV="{venv}"\n')
    if isaac_distro:
        settings += f'export ISAAC_ROS_DISTRO={isaac_distro}\n'
    (scene / 'splat.local.sh').write_text(settings)

    result = subprocess.run(['bash', str(scene / 'launch_splat_simulation.sh')],
                            env=env, capture_output=True)
    assert result.returncode == 7, result.stderr
    captured = json.loads(Path(env['RESULT_FILE']).read_text())
    assert captured['env']['ROS_DISTRO'] == expected_distro
    assert captured['env']['LD_LIBRARY_PATH'].split(os.pathsep)[0] == str(lib)
    assert 'ROS_ENV_LOADED' not in captured['env']

    result = subprocess.run(['bash', str(scene / 'splat_tools.sh'), 'bridge'],
                            env=env, capture_output=True)
    assert result.returncode == 7, result.stderr
    captured = json.loads(Path(env['RESULT_FILE']).read_text())
    assert captured['env']['ROS_DISTRO'] == 'lyrical'
    assert captured['env']['ROS_ENV_LOADED'] == 'yes'
    assert captured['env']['POLICY_ENV_LOADED'] == 'yes'


@pytest.mark.parametrize('runtime_exists', [False, True])
def test_sourcing_isaac_launcher_preserves_calling_shell(workspace, runtime_exists):
    scene, env = workspace
    isaac = scene / 'isaac'
    if runtime_exists:
        fake_python(isaac / 'python.sh')
        (isaac / 'exts/isaacsim.ros2.core/jazzy/lib').mkdir(parents=True)
    env['ISAAC_SIM_ROOT'] = str(isaac)
    result = subprocess.run(
        ['bash', '-e', '-c',
         'source "$1" --headless; printf "shell is still running\\n"; '
         '[[ -z "${ROS_DISTRO+x}" ]]',
         'bash', str(scene / 'launch_splat_simulation.sh')],
        env=env, text=True, capture_output=True)
    assert result.returncode == 0
    assert 'shell is still running' in result.stdout
    assert 'Execute this launcher instead of sourcing it' in result.stderr
    assert not Path(env['RESULT_FILE']).exists()


def test_tools_wrapper_sources_environment_and_forwards_arguments(workspace):
    scene, env = workspace
    venv = scene.parent / 'policy venv'
    fake_python(venv / 'bin/python')
    (venv / 'bin/activate').write_text('export POLICY_ENV_LOADED=yes\n')
    ros = scene / 'ros.bash'
    overlay = scene / 'bridge.bash'
    ros.write_text('export ROS_ENV_LOADED=yes\n')
    overlay.write_text('export BRIDGE_ENV_LOADED=yes\n')
    (scene / 'splat.local.sh').write_text(
        f'export ROS_SETUP="{ros}"\nexport ROS_LOCAL_SETUP="{overlay}"\n'
        f'export LEROBOT_VENV="{venv}"\nexport ROS_DOMAIN_ID=42\n')
    result = subprocess.run(
        ['bash', str(scene / 'splat_tools.sh'), 'record', '--task=a task with spaces'],
        env=env, cwd='/tmp', capture_output=True)
    assert result.returncode == 7
    captured = json.loads(Path(env['RESULT_FILE']).read_text())
    assert captured['args'] == [
        '-X', 'utf8', '-B', str(scene / 'scripts/splat_tools.py'),
        'record', '--task=a task with spaces']
    for key in ('POLICY_ENV_LOADED', 'ROS_ENV_LOADED', 'BRIDGE_ENV_LOADED'):
        assert captured['env'][key] == 'yes'
    assert captured['env']['ROS_DOMAIN_ID'] == '42'


@pytest.mark.parametrize('use_settings_template', [False, True])
def test_tools_launcher_runs_without_colcon_workspace(workspace, use_settings_template):
    scene, env = workspace
    venv = scene.parent / '.venv'
    fake_python(venv / 'bin/python')
    (venv / 'bin/activate').write_text('export POLICY_ENV_LOADED=yes\n')
    ros = scene / 'ros.bash'
    ros.write_text('export ROS_ENV_LOADED=yes\n')
    env['ROS_SETUP'] = str(ros)
    if use_settings_template:
        shutil.copyfile(ROOT / 'splat.local.sh.example', scene / 'splat.local.sh')
    result = subprocess.run(
        ['bash', str(scene / 'splat_tools.sh'), 'bridge', '--ros-args',
         '-p', 'initial_mode:=manual'],
        env=env, cwd='/tmp', capture_output=True)
    assert result.returncode == 7
    captured = json.loads(Path(env['RESULT_FILE']).read_text())
    assert captured['args'][-4:] == ['bridge', '--ros-args', '-p', 'initial_mode:=manual']
    assert captured['env']['ROS_ENV_LOADED'] == 'yes'
    assert captured['env']['POLICY_ENV_LOADED'] == 'yes'
    assert 'BRIDGE_ENV_LOADED' not in captured['env']
    assert not (scene.parent / 'kaivuri-control-bridge').exists()


def test_tools_launcher_rejects_missing_explicit_overlay(workspace):
    scene, env = workspace
    ros = scene / 'ros.bash'
    ros.write_text('export ROS_ENV_LOADED=yes\n')
    env.update(ROS_SETUP=str(ros), ROS_LOCAL_SETUP=str(scene / 'missing.bash'))
    result = subprocess.run(
        ['bash', str(scene / 'splat_tools.sh'), 'bridge'],
        env=env, text=True, capture_output=True)
    assert result.returncode == 1
    assert 'Configured ROS_LOCAL_SETUP not found' in result.stderr
    assert not Path(env['RESULT_FILE']).exists()
