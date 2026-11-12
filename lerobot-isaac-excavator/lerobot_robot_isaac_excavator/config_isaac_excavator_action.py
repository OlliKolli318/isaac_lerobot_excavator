from dataclasses import dataclass, field

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("isaac_excavator_action")
@dataclass
class IsaacExcavatorActionConfig(TeleoperatorConfig):
    action_topic: str = "/kaivuri/action"
    recording_mode_topic: str = "/kaivuri/recording_mode"
    node_name: str = "lerobot_isaac_excavator_action_teleop"
    connect_timeout_s: float = 10.0
    stale_timeout_s: float = 1.0
    set_manual_mode: bool = True
    manage_control_mode: bool = False
    action_key_suffix: str = ".pos"
    action_names: list[str] = field(
        default_factory=lambda: [
            "revolute_carriage",
            "revolute_tilt",
            "revolute_tool",
            "revolute_lift",
            "claw_closed",
        ]
    )
