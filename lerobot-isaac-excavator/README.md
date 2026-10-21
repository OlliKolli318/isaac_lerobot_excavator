# lerobot-isaac-excavator

LeRobot's excavator-specific robot/teleoperator profile and the project-owned
record, rollout, simulation-time DAgger, and dataset-aggregation commands.
This repository is independent of the old simulation checkout. It expects the
ROS topic contract of `isaac-splat-scene` and the safety bridge from
`kaivuri-control-bridge`; neither is imported through a checkout path.

Install in the Python environment that contains LeRobot, ROS 2 `rclpy`,
OpenCV, NumPy, PyArrow and the ROS message packages:

```bat
python -m pip install -e C:\path\to\lerobot-isaac-excavator
```

Start Isaac from `isaac-splat-scene`, then the control bridge, then run the
recorder in the same ROS domain:

```bat
kaivuri-lerobot-record --robot.type=isaac_excavator --robot.id=splat ^
  --teleop.type=isaac_excavator_action --teleop.id=gamepad ^
  --dataset.repo_id=local/splat_demo --dataset.root=C:/data/splat_demo ^
  --dataset.single_task="Pick up the log" --dataset.num_episodes=10 ^
  --dataset.episode_time_s=120 --dataset.fps=30 --dataset.push_to_hub=false
```

The recorder waits for the scene's `start` command (LB), saves on `end_save`
(RB or task success), and discards on `end_discard` (X). In simulation-time
mode it records the state and applied action from the same `/kaivuri/sim_sample`
message, plus synchronized camera frames. In wall-time mode it reads the
teleoperator's latest `/kaivuri/action` value. `kaivuri-lerobot-rollout`
performs policy rollout; it needs a checkpoint trained on this scene's 5-state,
5-action, two-camera schema and an enabled policy-mode bridge. DAgger and
aggregation are project-specific and should be used only after a dry run with
the same LeRobot version as the source checkout.

Action order is slew, arm, bucket, boom, claw-closed. The first four values
are normalized axes; the last is binary. Change the ROS topic names via the
robot and teleoperator configuration classes, not by editing checkout paths.
The scene and the bridge need matching topic names and action order.

The project recorder and DAgger rollout default to local datasets, including
when launched directly through their console scripts. Upload requires an
explicit `--dataset.push_to_hub=true` (or the equivalent configuration value)
and a successful run. Confirm permission for the dataset and destination
before enabling it; a private destination does not establish permission.

Run `python -m pytest tests -q` in the full LeRobot environment. The metadata
aggregation tests require LeRobot and PyArrow; the other unit tests use
synthetic messages and dependency doubles and do not start ROS participants.
Run `python -m flake8` for correctness lint and `python -m mypy` for local type
checks. These checks do not replace an Isaac/ROS recording and rollout test.

The reusable, hardware-independent LeRobot/ROS 2 adapter lives in the
separate `lerobot-ros2` repository. This package retains the excavator's
specific schema and collector behavior for compatibility. It still has some
utility-code overlap with `lerobot-ros2`; do not install both as competing
robot configurations for one recording without selecting the intended type.

Origin: copied from
`isaac_excavator_gripper_simulation/lerobot_robot_isaac_excavator`.
