from dataclasses import dataclass, field

from lerobot.robots import RobotConfig


@RobotConfig.register_subclass("isaac_excavator")
@dataclass
class IsaacExcavatorConfig(RobotConfig):
    state_topic: str = "/kaivuri/observation_state"
    # This is an input to the ROS control bridge, not the Isaac scene sink.
    # The bridge is the sole publisher of /kaivuri/policy_action.
    action_topic: str = "/kaivuri/control/policy_action"
    recording_mode_topic: str = "/kaivuri/recording_mode"
    cabin_rgb_topic: str = "/cabin/rgb"
    gripper_rgb_topic: str = "/gripper/rgb"
    # Enabled by the project recording/rollout entry points.
    simulation_time: bool = False
    clock_topic: str = "/clock"
    sample_topic: str = "/kaivuri/sim_sample"
    stamped_action_topic: str = "/kaivuri/control/policy_action_stamped"
    sync_tolerance_s: float = 0.002
    sync_grace_s: float = 0.5
    sync_buffer_size: int = 64
    # Clock messages are small; retain more history without buffering more RGB images.
    sync_clock_buffer_size: int = 1024

    state_names: list[str] = field(
        default_factory=lambda: [
            "revolute_carriage",
            "revolute_tilt",
            "revolute_tool",
            "revolute_lift",
            "claw_closed",
        ]
    )
    action_names: list[str] = field(
        default_factory=lambda: [
            "left_stick_x_slew",
            "left_stick_y_arm",
            "right_stick_x_bucket",
            "right_stick_y_boom",
            "claw_closed",
        ]
    )

    camera_height: int = 512
    camera_width: int = 512
    camera_channels: int = 3
    node_name: str = "lerobot_isaac_excavator_robot"
    connect_timeout_s: float = 10.0
    stale_timeout_s: float = 1.0
    # Isaac RGB rendering can be appreciably slower than its joint-state graph.
    # Keep control state strict while allowing a camera frame to span several
    # control ticks.
    camera_stale_timeout_s: float = 5.0
    allow_legacy_six_value_state: bool = True
    legacy_claw_closed_threshold_rad: float = 0.2

    # Kept for compatibility with existing command lines. New deployments let
    # kaivuri_lerobot_bridge own mode changes through ROS services.
    start_mode: str = "unchanged"
    manage_control_mode: bool = False

    # LeRobot rollout currently routes robot actions whose keys end in ".pos".
    action_key_suffix: str = ".pos"
