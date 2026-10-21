# isaac-splat-scene

An Isaac Sim excavator scene with a splat environment, a movable log, cabin
and gripper cameras, and ROS 2 controls for recording and policy execution.
`env.usd` is the active scene. Its control graphs and ScriptNode programs are
stored in the USD; the Python files under `scripts/` launch the scene and
configure the surrounding processes.

## Project layout

```text
isaac-splat-scene/
├── README.md
├── .gitignore
├── splat.local.cmd.example
├── splat.local.cmd           # Your paths; ignored by Git
├── env.usd
├── launch_splat_simulation.cmd
├── splat_tools.cmd
├── assets/
│   ├── robots/
│   │   └── excavator_gripper.usd
│   ├── environment/
│   │   └── splat.usdz
│   └── props/
│       ├── Low_Poly_Log.usdz
│       └── Garbage_Container_Blue.usdz
├── scripts/
│   ├── launch_splat_simulation.py
│   ├── splat_tools.py
│   ├── scene_environment.py
│   ├── simulation_camera_delivery.py
│   └── simulation_transport.py
├── config/
│   └── fastdds_simulation.xml
├── tests/                       # Local only; ignored by Git
│   ├── test_camera_delivery.py
│   ├── test_scene_assets.py
│   └── test_simulation_transport.py
└── dist/                         # Generated output; ignored by Git
```

The excavator is referenced from `assets/robots/excavator_gripper.usd`.
The splat, log, and container are payloads under `assets/`. Keep these files
together with `env.usd`: asset paths are relative to the layer containing
them. `dist/` is optional and may be absent in a fresh checkout.

## Requirements

On Windows, install Isaac Sim with support for NuRec volume rendering and
enable its ROS 2 Bridge extension. The included `splat.usdz` contains NuRec
volume fields; NuRec is rendering support in Isaac Sim, not a separate
package you need to download for this scene.

In your ROS 2/Pixi environment, install `lerobot-isaac-excavator`
and build/source the `kaivuri-control-bridge` ROS package. Install
`lerobot-ros2` if you want the optional `episode-preview` command.

Isaac Sim supplies the material library used by `OmniGlass.mdl`. The scene
assets do not replace that installation, the ROS bridge, or the LeRobot
packages. This repository does not include a Pixi manifest; use the manifest
for your installed ROS/LeRobot environment.

## Launch the scene

Copy the local settings template once and edit the paths to match your
installation. The resulting `splat.local.cmd` is ignored by Git:

```bat
cd /d C:\path\to\isaac-splat-scene
copy splat.local.cmd.example splat.local.cmd
notepad splat.local.cmd
```

Then start the scene with:

```bat
launch_splat_simulation.cmd
```

The launcher opens the repository's `env.usd` and starts playback. It finds
the scene and DDS configuration relative to its own file location, so calling
the launcher by its full path also works from another directory. To open a
different scene, pass `--scene "C:\path\to\scene.usd"`.

The launcher defaults to `ROS_DISTRO=jazzy` and uses Isaac's bundled ROS
libraries. Set `ROS_DISTRO` explicitly if your isaac sim installation uses a different
bundled distribution. The launcher also applies synchronous camera delivery
and the Fast DDS profile in `config/fastdds_simulation.xml` to its process.

## Run the ROS and recording tools

In another terminal, go to this repository and run the ROS tools. Both
launch commands load the same `splat.local.cmd`, including `ROS_DOMAIN_ID`:

```bat
splat_tools.cmd check
splat_tools.cmd bridge --ros-args -p initial_mode:=manual
```

The bridge starts with controls disabled. Enable control through the bridge's
ROS services as documented by `kaivuri-control-bridge`.

In a third terminal, use
`splat_tools.cmd record` or `splat_tools.cmd rollout` with the options documented
by `lerobot-isaac-excavator`. The available commands are:

| Command | Purpose |
| --- | --- |
| `check` | Check imports for ROS, the bridge, recording, rollout, and DAgger. |
| `bridge` | Run the excavator control bridge. |
| `record` | Record demonstration episodes. |
| `rollout` | Run a trained policy. |
| `dagger` | Run rollout with the DAgger strategy. |
| `dagger-aggregate` | Aggregate DAgger data. |
| `train` | Launch LeRobot training. |
| `episode-preview` | Preview episodes using the optional `lerobot-ros2` tools. |
| `ros2` | Run ROS diagnostics and service commands. |
| `check-policy <directory>` | Check that a policy has five-value state and action dimensions. |

`scripts/splat_tools.py` dispatches these commands to the installed
packages. Recordings and DAgger rollouts default to `dataset.push_to_hub=false`.
Training through `splat_tools.cmd train` defaults to `policy.push_to_hub=false`.
Enable uploads explicitly only after confirming permission for the data/model
and the destination. Upstream commands and other integrations can have their
own network settings.
Keep the action order, camera names, ROS domain, and DDS settings consistent
across this scene, `kaivuri-control-bridge`, and `lerobot-isaac-excavator`.

Both launch commands use `config/fastdds_simulation.xml`. Set
`KAIVURI_FASTDDS_PROFILE` to another XML file to override it, or to `inherit`
if your environment already manages DDS configuration.

## Edit and validate

Edit scene properties and embedded control graphs in `env.usd`, or edit the
referenced robot in `assets/robots/excavator_gripper.usd`. Close the scene in
Isaac before changing its files externally. When moving assets, update their
reference or payload paths in the owning USD layer.

The `tests/` directory checks launcher cleanup, command dispatch,
DDS configuration, and the embedded scene contracts. Run it from the
repository root with Python and pytest installed:

```text
python -m pytest tests -q
```

These tests do not require Isaac or ROS and do not exercise simulation.
Run `python -m flake8` for correctness lint and `python -m mypy` for local
type checks; deployment libraries are outside that type-checking scope.
After scene changes, launch Isaac and check robot/log behavior, both camera
streams (`/cabin/rgb` and `/gripper/rgb`), `/clock`, and `/kaivuri/sim_sample`.

Historical authoring, migration, and packaging scripts are not part of the
active project. Keep local archives and checkpoints outside the repository;
generated output can go in the Git-ignored `dist/` directory. The folder
reorganization preserved the remaining old helpers and original tests in a
timestamped sibling `isaac-splat-scene-local-archive/` directory. That archive
is local only and is not needed to launch the scene.

## Third-party asset attribution

### Garbage Container Blue

- **Asset file:** [`assets/props/Garbage_Container_Blue.usdz`](assets/props/Garbage_Container_Blue.usdz)
- **Creator:** [usedm](https://sketchfab.com/usedm)
- **Source:** [Garbage Container Blue on Sketchfab](https://sketchfab.com/3d-models/garbage-container-blue-0a5a85b055a447b7955d959e30861468)
- **License:** [Creative Commons Attribution 4.0 International (CC BY 4.0)](https://creativecommons.org/licenses/by/4.0/)
- **Scene adaptations:** `env.usd` applies scale, orientation and placement
  overrides, configures collisions, and adds `Goal_plane` geometry under the
  container for the simulation.

### Low Poly Log

- **Asset file:** [`assets/props/Low_Poly_Log.usdz`](assets/props/Low_Poly_Log.usdz)
- **Creator:** [Theballinballer408](https://sketchfab.com/Theballinballer408)
- **Source:** [Low Poly Log on Sketchfab](https://sketchfab.com/3d-models/low-poly-log-6ec7c9f05bf24752ac02da51f047381e)
- **License:** [Creative Commons Attribution 4.0 International (CC BY 4.0)](https://creativecommons.org/licenses/by/4.0/)
- **Scene adaptations:** `env.usd` applies scale, orientation and placement
  overrides, enables rigid-body simulation, and configures convex-hull
  collisions for the log.

The creators, sources and licenses above are recorded in the respective
USDZ files' embedded metadata. Retain the relevant attribution and license
link when redistributing either asset, and indicate any further modifications.
These credits apply to the identified assets; they do not assign a license
to the rest of the project.

Origin: scene assets copied from `splat_env_proto`; graph and transport helpers
originally copied from `isaac_excavator_gripper_simulation`.
