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

The recording, rollout, DAgger, and training tools require **CPython 3.12.x**
and **ROS 2 Python bindings built for CPython 3.12 on the same OS and
architecture**. These tools import ROS, OpenCV, and LeRobot into one Python
process. Installing them into separate environments does not make the current
launcher compatible.

### Required OS, ROS 2, and Python combination

| Operating system | External ROS 2 installation | Python used by ROS bindings | Use for this workflow |
| --- | --- | --- | --- |
| **Ubuntu 24.04 LTS (Noble)** | **Jazzy**, official Ubuntu packages | **3.12** | Recommended Linux setup. |
| **Windows 11, x86_64** | **Lyrical**, Windows binaries and their matching Pixi environment (VS2022 build) | **3.12** | Windows setup used by this project. |
| Ubuntu 22.04 LTS (Jammy) | Humble, official Ubuntu packages | 3.10 | Incompatible with the pinned LeRobot 0.6.0 in the same process. |
| Ubuntu 26.04 LTS (Resolute) | Lyrical, official Ubuntu packages | 3.14 | Does not provide the Python 3.12 bindings required by this setup. |
| Ubuntu 24.04 LTS (Noble) | Lyrical, built from source | 3.12 | Upstream Tier 3; requires a source build, outside the standard setup below. |

ROS binary packages are specific to the OS release and Python ABI. Jazzy's
Ubuntu packages target **24.04**, not 22.04. Lyrical targets **Windows 11** on
Windows; its primary Ubuntu binary target is **26.04**, with a different Python
version. A ROS distribution name alone does not establish Python compatibility.
See the upstream [ROS platform matrix (REP 2000)](https://github.com/ros-infrastructure/rep/blob/master/rep-2000.rst),
[Jazzy Ubuntu installation guide](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Installation/Ubuntu-Install-Debs.rst),
and [Lyrical platform and dependency requirements](https://github.com/ros2/ros2_documentation/blob/lyrical/source/Releases/lyrical/supported-platforms.rst).

Install these external programs in the OS/environment that runs each process:

- **NVIDIA Isaac Sim 6.0.1** with NuRec volume rendering support and the ROS 2
  Bridge extension enabled. `isaac-splat-scene/env.usd` is the active scene;
  keep it with the `isaac-splat-scene/assets/` directory.
- **A GPU supported by Isaac Sim and a compatible NVIDIA driver.** Headless
  mode still renders cameras and needs a GPU.
- **ROS 2 with Python 3.12 bindings and Fast DDS** (`rmw_fastrtps_cpp`), using
  a compatible OS/ROS combination from the table above. Source its setup script
  before running the tools. Required ROS packages include `rclpy`, `sensor_msgs`,
  `rosgraph_msgs`, `std_msgs`, and `std_srvs`.
- **CPython 3.12.x**, plus pip. [uv](https://docs.astral.sh/uv/getting-started/installation/)
  can install Python with `uv python install 3.12` and create virtual
  environments. For Ubuntu's ROS packages, use uv with the **system interpreter**
  as shown below; Windows tools use the matching **Pixi interpreter**.
- **LeRobot 0.6.0** with the `smolvla` and `core_scripts` extras, **NumPy
  2.2.6** (the adapter's current dependency pins), and **FFmpeg** for recording
  and decoding dataset videos. Published [LeRobot 0.6.0 requires Python
  >=3.12](https://pypi.org/project/lerobot/0.6.0/); Python 3.10/3.11 cannot run
  this dependency set.

`uv python install 3.12` installs an interpreter; it does not install or rebuild
ROS. `--system-site-packages` exposes packages from the selected base
interpreter; it does not convert Python 3.10 compiled extensions into Python
3.12 extensions. A uv-managed Python also does not automatically inherit
Ubuntu's `/usr/lib/python3/dist-packages`. Do not replace Ubuntu's system
Python. See [uv's interpreter selection documentation](https://docs.astral.sh/uv/concepts/python-versions/).

Isaac Sim runs in a separate process with its own Python and bundled ROS
libraries. The external tools' Python requirement does not require replacing
Isaac's interpreter. For a single x86_64 computer running both processes, use
Ubuntu 24.04 or Windows 11; see [Isaac Sim's OS requirements](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/installation/requirements.html).
The Linux scene launcher defaults to bundled Jazzy. Using external Lyrical
with Isaac's Humble/Jazzy bridge crosses ROS distributions: matching domain IDs
and DDS settings is necessary, but does not guarantee interoperability. Verify
camera/state reception, clock, services, and actions before rollout. The
matching Jazzy setup on Ubuntu 24.04 avoids that distribution mismatch.

For GPU policy inference or training, also use a supported NVIDIA GPU,
compatible driver, and CUDA-enabled PyTorch build. Training additionally needs
LeRobot's `training` extra. Building Python dependencies from source may need a
compiler toolchain, Python development headers, CMake, pkg-config, and FFmpeg
development headers; prebuilt wheels do not need these tools.

The adapter requires a LeRobot release that provides
`lerobot.rollout.RolloutConfig`, `build_rollout_context`, and
`lerobot.utils.feature_utils`. Use the pinned release and run the import check
below. If your checkpoint needs a source checkout, record its exact revision
and verify these APIs; a version label alone does not establish compatibility.

## Set up on Linux

Use **Ubuntu 24.04 LTS + ROS 2 Jazzy** for this setup. In a container, the
container's OS release and ROS packages must match this combination even if the
host uses another OS. Preserve an existing Ubuntu 22.04/Humble container and
create a separate 24.04/Jazzy container for these tools.

First install Jazzy using the upstream Ubuntu installation guide linked above
(including its ROS apt repository configuration). Then install the runtime
packages and create the venv. The examples assume this repository is at
`~/isaac_excavator_env`. Use a fresh tools terminal, and use an unused `.venv`
path if an older environment already exists:

```bash
sudo apt update
sudo apt install python3-venv ffmpeg ros-jazzy-ros-base ros-jazzy-rmw-fastrtps-cpp \
  ros-jazzy-sensor-msgs ros-jazzy-rosgraph-msgs ros-jazzy-std-msgs ros-jazzy-std-srvs
# Install uv (or use an existing installation).
curl -LsSf https://astral.sh/uv/install.sh | sh

cd "$HOME/isaac_excavator_env"
/usr/bin/python3 --version  # Must report 3.12.x on Ubuntu 24.04.
/usr/bin/python3 -c 'import sys; assert sys.version_info[:2] == (3, 12), sys.version'
source /opt/ros/jazzy/setup.bash
"$HOME/.local/bin/uv" venv --python /usr/bin/python3 --system-site-packages --seed .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install 'lerobot[smolvla,core_scripts]==0.6.0'
python3 -m pip install -e ./lerobot-isaac-excavator
python3 -m pip install -e ./kaivuri-control-bridge/src/kaivuri_lerobot_bridge
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
The Linux scene launcher uses `ISAAC_ROS_DISTRO` (default: `jazzy`) to select
Isaac's bundled ROS libraries, then sets `ROS_DISTRO` for the Isaac process.
It does not load the external installation specified by `ROS_SETUP`. Isaac
Sim 6.0.1 bundles Humble and Jazzy, not Lyrical. Launch the scene in a fresh
terminal without system ROS or Conda sourced:

```bash
cd "$HOME/isaac_excavator_env"
./isaac-splat-scene/launch_splat_simulation.sh
```

The tools launcher loads the external ROS installation and the selected venv.
Use these settings in `splat.local.sh` for the Ubuntu 24.04/Jazzy setup; these
match the current Linux template and launcher defaults:

```bash
export ISAAC_ROS_DISTRO="${ISAAC_ROS_DISTRO:-jazzy}"
export ROS_DISTRO="${ROS_DISTRO:-jazzy}"
export ROS_SETUP="${ROS_SETUP:-/opt/ros/$ROS_DISTRO/setup.bash}"
```

`ISAAC_ROS_DISTRO` controls the scene's bundled libraries independently of the
tools' `ROS_DISTRO` and `ROS_SETUP`. No per-command override is needed. Existing
`splat.local.sh` files without `ISAAC_ROS_DISTRO` also default to Jazzy for the
scene. Selecting `ISAAC_ROS_DISTRO=humble` only changes Isaac's bundled
libraries; the external tools still require ROS bindings built for Python 3.12.

Keep the scene and tools on the same `ROS_DOMAIN_ID` and use compatible Fast
DDS settings. The launcher wrappers work from any current directory; no
`colcon` build is needed for the bridge.

Check the installed Python packages and ROS imports:

```bash
cd "$HOME/isaac_excavator_env"
python -c "import os, sys, rclpy; from rclpy.impl.implementation_singleton import rclpy_implementation; print(sys.executable, sys.version); print('ROS_DISTRO:', os.environ.get('ROS_DISTRO')); print(rclpy_implementation.__file__)"
./isaac-splat-scene/splat_tools.sh check
```

Confirm Python 3.12, `ROS_DISTRO=jazzy`, and a Python 3.12 `rclpy` extension.
An empty `ROS_DISTRO` means the shell is not configured; setting that variable
alone does not source ROS libraries. The `check` command additionally verifies
ROS, OpenCV, LeRobot, and project imports together in the tools interpreter.
It does not test communication with the running scene.

## Set up on Windows

Use **Windows 11 x86_64 + ROS 2 Lyrical Windows binaries built for Python
3.12**, with the matching Pixi dependency environment. Upstream lists VS2022
for this ROS build; retain its matching compiler/runtime dependencies. The
working development environment uses Python 3.12.3. A Pixi environment named
`pixi_ros2_lyrical` is not sufficient evidence of the installed ROS version:
verify its ROS setup and imports below.

Install Isaac Sim with NuRec and the ROS 2 Bridge enabled, then copy
`isaac-splat-scene/splat.local.cmd.example` to `splat.local.cmd` and set
`ISAAC_SIM_ROOT`. The scene launcher uses Isaac's bundled Python and ROS
libraries.

The tools launcher also needs Pixi and a ROS-enabled Pixi manifest configured
as `PIXI_MANIFEST`. This repository does not include that machine-specific
manifest. Copy `splat.local.cmd.example` to `splat.local.cmd` and set both
`ISAAC_SIM_ROOT` and `PIXI_MANIFEST`. The Pixi environment must load the actual
ROS installation's `local_setup.bat` through its activation setup, or you must
call that script in the terminal before running the tools. `PIXI_MANIFEST`
alone does not source ROS. From the project root, install the Python packages
in the manifest's environment:

```bat
call isaac-splat-scene\splat.local.cmd
pixi run --manifest-path "%PIXI_MANIFEST%" python -m pip install "lerobot[smolvla,core_scripts]==0.6.0"
pixi run --manifest-path "%PIXI_MANIFEST%" python -m pip install -e .\lerobot-isaac-excavator
pixi run --manifest-path "%PIXI_MANIFEST%" python -m pip install -e .\kaivuri-control-bridge\src\kaivuri_lerobot_bridge
```

The manifest's environment must expose the ROS Python modules, use Fast DDS,
and provide FFmpeg. Verify the actual interpreter, sourced ROS distribution,
and compiled binding from the same project-root terminal:

```bat
pixi run --manifest-path "%PIXI_MANIFEST%" python -c "import os, sys, rclpy; from rclpy.impl.implementation_singleton import rclpy_implementation; from importlib.metadata import version; print(sys.executable, sys.version); print('ROS_DISTRO:', os.environ.get('ROS_DISTRO')); print(rclpy_implementation.__file__); print('LeRobot:', version('lerobot'))"
```

Expect Python 3.12.x, `ROS_DISTRO=lyrical`, a `cp312` ROS binding, and LeRobot
0.6.0. Then run `splat_tools.cmd check` from the `isaac-splat-scene` folder.
Keep Python and ROS in this Pixi environment: installing another Python with
uv does not connect it to the Pixi/ROS DLLs. For an intentionally separate
Python setup, uv can install 3.12, but all ROS binary dependencies still need
to match that interpreter and OS.

## Run the project

### Start the scene

On Linux, run this from the project root in a fresh terminal without system
ROS or Conda sourced:

```bash
./isaac-splat-scene/launch_splat_simulation.sh
```

The launcher selects `ISAAC_ROS_DISTRO` (default: `jazzy`) independently of
the external tools' `ROS_DISTRO` and `ROS_SETUP`. On Windows, run
`launch_splat_simulation.cmd` from
`isaac-splat-scene`. Add `--headless` to run without a GUI; camera rendering
still requires a compatible GPU. Use `--scene /absolute/path/to/scene.usd` to
open another scene. Execute the Linux launcher; do not source it.

### Xbox controller input on Linux

Connect the controller before starting Isaac and use the project's scene
launcher. It automatically loads
[`config/gamepad_mappings.txt`](isaac-splat-scene/config/gamepad_mappings.txt)
into Isaac's own GLFW library before controllers are registered. No manual
mapping step is needed for these included Xbox Series S|X identities:

| Connection | Controller GUID |
| --- | --- |
| USB | `030000005e040000120b000009050000` |
| Bluetooth | `050000005e040000130b000009050000` |

Other Xbox models and firmware revisions may expose different identities and
layouts. If the controller does not work, follow these checks in order:

1. **Check Linux device detection.** Run:

   ```bash
   cat /proc/bus/input/devices
   ls -l /dev/input/js*
   ```

   Find the Xbox/Microsoft controller block and its `Handlers` line. A `js0`
   entry corresponds to `/dev/input/js0`; another controller may use `js1`.
   If the controller is missing from the device list, check Bluetooth pairing
   or the USB data cable first. A missing `js*` device alone does not prove
   that Linux cannot detect the controller; inspect the device list too.
   If `jstest` is installed, run `jstest /dev/input/js0` (adjust the index) and
   move both sticks and press buttons. Changing values confirm input reaches
   Linux. GLFW reads `/dev/input/event*`, so a working `js0` test alone is
   insufficient. Use the controller block's `Handlers` line to find its event
   device, then check it (replace `event7` with the actual name):

   ```bash
   test -r /dev/input/event7 && echo "Controller event device is readable" || echo "Controller event device access denied"
   ```

   Local desktop sessions normally receive event-device access automatically.
   RDP/remote sessions can lack it while the local seat belongs to the login
   screen. If access is denied on Ubuntu, the optional
   [`99-splat-xbox.rules`](isaac-splat-scene/config/99-splat-xbox.rules) grants
   the `plugdev` group access specifically to the supported USB/Bluetooth Xbox
   controller event devices. From the project root:

   ```bash
   id -nG
   sudo install -m 0644 isaac-splat-scene/config/99-splat-xbox.rules /etc/udev/rules.d/99-splat-xbox.rules
   sudo udevadm control --reload-rules
   ```

   Confirm `plugdev` appears in your groups. If it does not, an administrator
   can add your user with `sudo usermod -aG plugdev "$USER"`; log out and back
   in afterward so new processes receive that group. Reconnect the controller
   (USB unplug/replug or Bluetooth disconnect/reconnect), repeat the event-device
   readability check, and restart Isaac. Other distributions may use different
   device-access groups. Run Isaac as your normal user. This applies to a
   controller physically connected to the Isaac computer; forwarding a controller
   attached to the RDP client is a separate setup.

2. **Capture the Isaac startup messages.** In a fresh terminal without system
   ROS or Conda sourced, run from the project root:

   ```bash
   ./isaac-splat-scene/launch_splat_simulation.sh 2>&1 | tee /tmp/splat-controller.log
   ```

   `Loaded Linux gamepad mappings` confirms the project loaded its mapping
   file; it does not prove that the connected controller has a matching entry.
   `Joystick with unknown remapping detected (will be ignored)` means Isaac
   sees the device but cannot map it. Copy the controller name and the GUID
   in square brackets from that warning. `No Gamepad!` alone means the graph
   cannot access its selected gamepad; check the preceding startup messages.
   A mapping-loader warning such as `Multiple cached windowing plugins found`
   means the mappings were not loaded and needs investigation first.

3. **Check the graph while simulation is playing.** Open
   `/World/BucketExcavatorActionGraph` and inspect `outputs:value` on
   `GamepadAxis0` through `GamepadAxis3` while moving both sticks. These nodes
   select gamepad index `0`; if multiple controllers are connected, disconnect
   the others and restart Isaac for a simple single-controller check. If the
   raw axis values change but the excavator does not move, continue with the
   bridge/manual-mode instructions below. The controller mapping is already
   delivering those axis values.

4. **Add a mapping only if the GUID is unsupported.** Look for the exact
   Linux GUID in the maintained
   [SDL GameController database](https://github.com/mdqinc/SDL_GameControllerDB/blob/master/gamecontrollerdb.txt),
   which uses the mapping format GLFW accepts. Verify the axis/button layout
   against the connected device before adding an entry to
   `isaac-splat-scene/config/gamepad_mappings.txt`. Keep each mapping on one
   line, including its GUID and `platform:Linux` field. Do not substitute a new
   GUID into an existing row without checking its layout: USB and Bluetooth
   can number sticks, triggers, and buttons differently.

   Close and restart Isaac after editing the file, then repeat the graph check
   and verify triggers and buttons as well as sticks. If there is no matching
   entry, report the controller model, connection type, GUID, Linux/Isaac
   versions, startup warnings, and whether the Linux input test works. A new
   mapping must be measured and validated for that device. Report these details
   too if the GUID already matches an included mapping but input still fails.

Windows Xbox controllers normally use GLFW's standard XInput mapping. Linux
uses device-specific GUID mappings, so the same controller can work on Windows
while needing an additional mapping on Linux. See the
[GLFW gamepad mapping documentation](https://www.glfw.org/docs/latest/input_guide.html#gamepad_mapping).

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
  --robot.type=isaac_excavator \
  --robot.id=excavator \
  --teleop.type=isaac_excavator_action \
  --teleop.id=gamepad \
  --dataset.repo_id=local/splat_demo \
  --dataset.root="$HOME/data/splat_demo" \
  --dataset.single_task="Pick up the log and place it in the blue container" \
  --dataset.num_episodes=10 \
  --dataset.episode_time_s=120 \
  --dataset.fps=30 \
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
  --strategy.type=base \
  --inference.type=rtc \
  --inference.rtc.execution_horizon=10 \
  --inference.rtc.max_guidance_weight=10 \
  --inference.queue_threshold=30 \
  --robot.type=isaac_excavator --robot.id=excavator \
  --robot.connect_timeout_s=60 \
  --policy.path=/path/to/checkpoint/pretrained_model \
  --device=cuda \
  --task="Pick up the log and place it in the blue container" \
  --fps=30 --duration=6430 \
  --return_to_initial_position=false
```

This example enables real-time chunking (RTC) with an execution horizon of 10
and queue threshold of 30. Adjust the checkpoint path and duration as needed;
the duration is measured in simulation seconds.

The policy process can run on a separate computer from Isaac when both can
communicate over ROS. Model or tokenizer downloads need network access on the
first run unless their files are already cached. Use `splat_tools.sh dagger`
for a DAgger rollout and change the --strategy.type=dagger; aggregation is available through
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
