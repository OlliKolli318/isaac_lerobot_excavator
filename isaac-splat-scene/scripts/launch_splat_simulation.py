"""Open env.usd with synchronous camera delivery for simulation-clock tools."""
import argparse
import os
from pathlib import Path

from scene_environment import isaac_root
from simulation_camera_delivery import configure_camera_delivery
from simulation_transport import configure_simulation_transport


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', type=Path, default=Path(__file__).resolve().parent.parent / 'env.usd',
                        help='USD or USDZ scene to open; defaults to env.usd')
    parser.add_argument('--headless', action='store_true',
                        help='Run without a GUI window (camera rendering still requires a GPU)')
    args = parser.parse_args()
    isaac = isaac_root()
    scene = args.scene.resolve()
    if not scene.is_file():
        parser.error(f'Scene does not exist: {scene}')

    os.environ.setdefault('ROS_DISTRO', 'jazzy')
    os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    ros_lib = isaac / 'exts' / 'isaacsim.ros2.core' / os.environ['ROS_DISTRO'] / 'lib'
    os.environ['PATH'] = str(ros_lib) + os.pathsep + os.environ.get('PATH', '')
    configure_simulation_transport()
    try:
        from isaacsim import SimulationApp
    except ImportError:
        from omni.isaac.kit import SimulationApp
    app = SimulationApp({'headless': args.headless, 'hide_ui': args.headless},
                        experience=str(isaac / 'apps' / 'isaacsim.exp.full.kit'))
    timeline = None
    try:
        # Isaac extensions are available only after SimulationApp starts.
        import omni.kit.app
        import omni.timeline
        import omni.usd

        manager = omni.kit.app.get_app().get_extension_manager()
        if not manager.is_extension_enabled('isaacsim.ros2.nodes'):
            manager.set_extension_enabled_immediate('isaacsim.ros2.nodes', True)
        configure_camera_delivery()
        context = omni.usd.get_context()
        context.open_stage(str(scene))
        while app.is_running() and context.get_stage() is None:
            app.update()
        if not app.is_running():
            return
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        print('Splat scene playing with simulation-time camera delivery.', flush=True)
        while app.is_running():
            app.update()
    finally:
        try:
            if timeline is not None:
                timeline.stop()
        finally:
            app.close()


if __name__ == '__main__':
    main()
