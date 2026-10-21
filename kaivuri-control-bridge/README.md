# kaivuri-control-bridge

ROS 2 safety/control boundary for LeRobot policy input and the Isaac scene.
The ROS package is in `src/kaivuri_lerobot_bridge`. It owns the scene-facing
policy command stream, starts disabled by default, validates action length and
values, and drops stale commands. The Isaac scene publishes episode commands
directly to `/kaivuri/episode_command`; the recorder or DAgger collector
subscribes to that topic. It can be used without Isaac if another simulator
implements the same topics and message contract.

Install in a ROS 2 workspace with `rclpy`, `std_msgs`, `sensor_msgs`,
`rosgraph_msgs`, and `std_srvs` available:

```bat
cd C:\path\to\kaivuri-control-bridge
colcon build --packages-select kaivuri_lerobot_bridge --base-paths src
call install\local_setup.bat
ros2 run kaivuri_lerobot_bridge control_bridge --ros-args -p initial_mode:=manual
```

The command enables manual control for the gamepad scene. Omit
`--ros-args -p initial_mode:=manual` to start with control disabled.
Policy control requires an explicit service call to
`/kaivuri_lerobot_control_bridge/set_policy_mode` and a live stream of valid
actions. Never launch a second bridge on the same topic set.

Defaults match the splat scene: 5 actions (4 normalized axes and one binary
claw), `/kaivuri/control/policy_action` input,
`/kaivuri/policy_action` output, `/kaivuri/sim_sample` atomic samples, and
stamped policy actions for simulation-time inference. The ROS parameters
`action_names`, `axis_count`, `safe_action`, and the topic parameters in
`control_bridge.py` can adapt the bridge to another numeric-action simulator.
The simulator must implement matching action semantics; changing parameter
names alone does not make a different actuator safe. DAgger's phase protocol
and the episode command strings remain Kaivuri-specific.

For local Python development, install the package in editable mode in a
ROS 2-enabled environment:

```bat
python -m pip install -e src\kaivuri_lerobot_bridge
```

From the repository root, run `python -m pytest -q`, `python -m flake8`, and
`python -m mypy`. The unit tests exercise action validation and DAgger leases
without starting a ROS participant. Integration with Isaac still requires a
separate simulation test.
