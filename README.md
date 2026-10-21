# Isaac excavator simulation and LeRobot workflow

This project combines the Isaac Sim scene, its LeRobot recording and policy
tools, and the ROS 2 control bridge in one repository. Together they support
simulation, gamepad demonstrations, policy rollout, and DAgger.
Clone the repository once; the component folders are included in the same
checkout.

## Project folders

| Folder | Purpose |
| --- | --- |
| [`isaac-splat-scene/`](isaac-splat-scene/) | Isaac Sim scene, assets, launchers, and command dispatcher. |
| [`lerobot-isaac-excavator/`](lerobot-isaac-excavator/) | LeRobot robot and gamepad integration, recording, policy rollout, and DAgger. |
| [`kaivuri-control-bridge/`](kaivuri-control-bridge/) | ROS 2 safety and control bridge between policy input and the scene. |

The scene publishes simulation time, synchronized camera frames, and state
samples. The bridge validates and gates actions before forwarding them to the
scene. The LeRobot tools record demonstrations or send policy actions through
the bridge. Keep the action order, camera names, ROS domain, and DDS settings
consistent across all three components.

## Requirements

Install these external programs once on the computer that will run the
simulation and policy workflow:

- **NVIDIA Isaac Sim 6.0.1** with NuRec volume rendering support and the ROS 2
  Bridge extension enabled. `isaac-splat-scene/env.usd` is the active scene;
  keep it with the `isaac-splat-scene/assets/` directory.
- **A GPU supported by Isaac Sim and a compatible NVIDIA driver.** Headless
  mode still renders cameras and needs a GPU.
- **ROS 2 with Python bindings and Fast DDS** (`rmw_fastrtps_cpp`). The external
  ROS environment used to develop the project is ROS 2 Lyrical. Isaac Sim uses
  its own bundled ROS libraries; the Linux scene launcher defaults to the
  bundled Jazzy libraries. Use a matching ROS domain and compatible DDS setup
  for all running processes.
- **Python 3.10 or newer**, compatible with the chosen ROS Python packages,
  plus pip and `venv`.
- **LeRobot** with the `smolvla` and `core_scripts` extras, and **FFmpeg** for
  recording and decoding dataset videos.

For GPU policy inference or training, also use a supported NVIDIA GPU,
compatible driver, and CUDA-enabled PyTorch build. Training additionally needs
LeRobot's `training` extra. Building Python dependencies from source may need a
compiler toolchain, Python development headers, CMake, pkg-config, and FFmpeg
development headers; prebuilt wheels do not need these tools.

The adapter requires a LeRobot release that provides
`lerobot.rollout.RolloutConfig`, `build_rollout_context`, and
`lerobot.utils.feature_utils`. The declared minimum version alone does not
guarantee those APIs; use a compatible release or source checkout, preferably
the revision used to train your checkpoint.

## Set up on Linux

The examples assume this repository is at `~/isaac_excavator_env`. Run the
ROS and LeRobot tools from a Python virtual environment that can see the ROS
Python packages. Create it with `--system-site-packages` so it can use the ROS
packages installed by your ROS distribution:

```bash
cd "$HOME/isaac_excavator_env"
python3 -m venv --system-site-packages .venv
source /opt/ros/lyrical/setup.bash
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install 'lerobot[smolvla,core_scripts]'
python -m pip install -e ./lerobot-isaac-excavator
python -m pip install -e ./kaivuri-control-bridge/src/kaivuri_lerobot_bridge
```

Install FFmpeg with your operating system's package manager if it is not
already available. The bridge is installed directly into the same environment;
the supplied tools do not require a `colcon build` or a generated bridge
`install/` folder.

The virtual environment can be stored elsewhere. The default used by the
scene tools is the repository-root `.venv`; set `LEROBOT_VENV` in
`isaac-splat-scene/splat.local.sh` to select another environment.

Copy the scene settings template and adjust the paths for this computer:

```bash
cd "$HOME/isaac_excavator_env/isaac-splat-scene"
cp -n splat.local.sh.example splat.local.sh
nano splat.local.sh
```

The settings file can be created before the virtual environment; it only sets
paths and variables. It is ignored by Git. For the scene, `ISAAC_SIM_ROOT`
must point to an Isaac Sim installation containing `python.sh` and `apps/`.
The scene launcher defaults to Isaac's bundled Jazzy ROS libraries. If your
Isaac installation has those libraries, launch the scene in a fresh terminal
without system ROS or Conda sourced:

```bash
cd "$HOME/isaac_excavator_env"
ROS_DISTRO=jazzy ./isaac-splat-scene/launch_splat_simulation.sh
```

The tools launcher loads ROS and the selected venv. For an external Lyrical
installation, set `ROS_SETUP=/opt/ros/lyrical/setup.bash` and
`ROS_DISTRO=lyrical` in `splat.local.sh` (or use the paths for your ROS
installation). Keep the scene and tools on the same `ROS_DOMAIN_ID` and use
compatible Fast DDS settings. The launcher wrappers work from any current
directory; no `colcon` build is needed for the bridge.

Check the installed Python packages and ROS imports:

```bash
cd "$HOME/isaac_excavator_env"
./isaac-splat-scene/splat_tools.sh check
```

## Set up on Windows

Install Isaac Sim with NuRec and the ROS 2 Bridge enabled, then copy
`isaac-splat-scene/splat.local.cmd.example` to `splat.local.cmd` and set
`ISAAC_SIM_ROOT`. The scene launcher uses Isaac's bundled Python and ROS
libraries.

The tools launcher also needs Pixi and a ROS-enabled Pixi manifest configured
as `PIXI_MANIFEST`. This repository does not include that machine-specific
manifest. Copy `splat.local.cmd.example` to `splat.local.cmd` and set both
`ISAAC_SIM_ROOT` and `PIXI_MANIFEST`. From the project root, install the Python
packages in the manifest's environment:

```bat
call isaac-splat-scene\splat.local.cmd
pixi run --manifest-path "%PIXI_MANIFEST%" python -m pip install "lerobot[smolvla,core_scripts]"
pixi run --manifest-path "%PIXI_MANIFEST%" python -m pip install -e .\lerobot-isaac-excavator
pixi run --manifest-path "%PIXI_MANIFEST%" python -m pip install -e .\kaivuri-control-bridge\src\kaivuri_lerobot_bridge
```

The manifest's environment must expose the ROS Python modules, use Fast DDS,
and provide FFmpeg. Then run `splat_tools.cmd check` from the
`isaac-splat-scene` folder.

## Run the project

### Start the scene

On Linux, run `./isaac-splat-scene/launch_splat_simulation.sh` from the project
root. On Windows, run `launch_splat_simulation.cmd` from
`isaac-splat-scene`. Add `--headless` to run without a GUI; camera rendering
still requires a compatible GPU. Use `--scene /absolute/path/to/scene.usd` to
open another scene. Execute the Linux launcher; do not source it.

### Start the bridge

In a ROS-enabled terminal, start one bridge in manual mode:

```bash
./isaac-splat-scene/splat_tools.sh bridge --ros-args -p initial_mode:=manual
```

On Windows, use `splat_tools.cmd bridge --ros-args -p initial_mode:=manual`.
Omit the `initial_mode` argument to start with controls disabled. Do not start
more than one bridge for the same topic set.

Enable policy mode from another terminal when starting a policy rollout:

```bash
./isaac-splat-scene/splat_tools.sh ros2 service call \
  /kaivuri_lerobot_control_bridge/set_policy_mode std_srvs/srv/SetBool \
  "{data: true}"
```

Use `{data: false}` to return to manual mode. Disable controls with:

```bash
./isaac-splat-scene/splat_tools.sh ros2 service call \
  /kaivuri_lerobot_control_bridge/disable std_srvs/srv/Trigger "{}"
```

### Record demonstrations

Start Isaac, start the bridge in manual mode, then record from another
ROS-enabled terminal. Adjust the paths, task, and episode count as needed:

```bash
./isaac-splat-scene/splat_tools.sh record \
  --robot.type=isaac_excavator --robot.id=splat \
  --teleop.type=isaac_excavator_action --teleop.id=gamepad \
  --dataset.repo_id=local/splat_demo --dataset.root="$HOME/data/splat_demo" \
  --dataset.single_task="Pick up the log" --dataset.num_episodes=10 \
  --dataset.episode_time_s=120 --dataset.fps=30 \
  --dataset.push_to_hub=false
```

The recorder starts on the gamepad `start` command (LB), saves on `end_save`
(RB or task success), and discards on `end_discard` (X). Recording and DAgger
commands default to local datasets. Upload only by explicitly enabling
`--dataset.push_to_hub=true` for a destination where you are authorized to
upload.

### Run a trained policy

Use a checkpoint trained for this scene's five state values, five actions,
and cabin/gripper camera features. Start Isaac and the bridge, enable policy
mode on the bridge, then run:

```bash
./isaac-splat-scene/splat_tools.sh rollout \
  --robot.type=isaac_excavator --robot.id=splat \
  --strategy.type=base \
  --policy.path=/path/to/checkpoint/pretrained_model \
  --policy.device=cuda \
  --task="Pick up the log" \
  --fps=30 --duration=120 \
  --display_data=false --use_torch_compile=false \
  --interpolation_multiplier=1 --return_to_initial_position=false
```

The policy process can run on a separate computer from Isaac when both can
communicate over ROS. Model or tokenizer downloads need network access on the
first run unless their files are already cached. Use `splat_tools.sh dagger`
for a DAgger rollout; aggregation is available through
`splat_tools.sh dagger-aggregate`.

## Available tools

Run commands through `isaac-splat-scene/splat_tools.sh` on Linux or
`splat_tools.cmd` on Windows:

| Command | Purpose |
| --- | --- |
| `check` | Check ROS, bridge, recording, rollout, and DAgger imports. |
| `bridge` | Run the control bridge. |
| `record` | Record demonstration episodes. |
| `rollout` | Run a trained policy. |
| `dagger` | Run rollout with the DAgger strategy. |
| `dagger-aggregate` | Aggregate DAgger data. |
| `train` | Launch LeRobot training. |
| `episode-preview` | Preview episodes; additionally requires `lerobot-ros2`. |
| `ros2` | Run ROS diagnostics and service commands with the configured DDS profile. |
| `check-policy <directory>` | Check that a policy has five-value state and action dimensions. |

The `train` command defaults to `policy.push_to_hub=false`. The adapter also
provides `kaivuri-lerobot-record` and `kaivuri-lerobot-rollout` entry points
after installation. Run `--help` after any command to see its options.

## Development and validation

The scene tests can run without Isaac or ROS:

```bash
cd isaac-splat-scene
python -m pytest tests -q
```

The adapter tests are in `lerobot-isaac-excavator/tests` and the bridge tests
are in `kaivuri-control-bridge/src/kaivuri_lerobot_bridge/test`. Adapter
metadata aggregation tests need LeRobot and PyArrow. Unit tests do not start
an Isaac simulation; validate scene behavior, cameras, `/clock`, and
`/kaivuri/sim_sample` in Isaac after scene changes.

The control contract uses five actions in this order: slew, arm, bucket, boom,
claw-closed. The first four values are normalized axes and the last is binary.
The scene and bridge use `/kaivuri/sim_sample`, `/cabin/rgb`, and
`/gripper/rgb`; keep topic names and action meanings consistent when changing
the robot configuration.

## Third-party asset attribution

- `isaac-splat-scene/assets/props/Garbage_Container_Blue.usdz`: **usedm**,
  [Sketchfab source](https://sketchfab.com/3d-models/garbage-container-blue-0a5a85b055a447b7955d959e30861468),
  [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/).
- `isaac-splat-scene/assets/props/Low_Poly_Log.usdz`: **Theballinballer408**,
  [Sketchfab source](https://sketchfab.com/3d-models/low-poly-log-6ec7c9f05bf24752ac02da51f047381e),
  [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/).

The USDZ files contain the asset attribution metadata. Retain the attribution
and license links when redistributing these assets, and indicate further
modifications. These credits apply to the identified assets only.
