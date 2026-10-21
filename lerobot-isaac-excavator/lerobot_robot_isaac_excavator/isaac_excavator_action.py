from __future__ import annotations

import logging
import threading
import time
from functools import cached_property
from typing import Any

from lerobot.teleoperators.teleoperator import Teleoperator
try:
    from lerobot.lerobot_types import RobotAction
except ModuleNotFoundError as exc:
    if exc.name != "lerobot.lerobot_types":
        raise
    from lerobot.types import RobotAction
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from .config_isaac_excavator_action import IsaacExcavatorActionConfig
from .ros_bridge import (
    LatestValue,
    RosNodeContext,
    check_not_stale,
    float_multi_array_to_list,
    make_recording_mode_qos,
    publish_recording_mode,
    wait_for_value,
)

logger = logging.getLogger(__name__)


class IsaacExcavatorAction(Teleoperator):
    config_class = IsaacExcavatorActionConfig
    name = "isaac_excavator_action"

    def __init__(self, config: IsaacExcavatorActionConfig) -> None:
        super().__init__(config)
        self.config = config
        self._ros = RosNodeContext(config.node_name)
        self._lock = threading.Lock()
        self._action = LatestValue()
        self._subscription = None
        self._recording_mode_publisher = None
        self._connected = False

    @property
    def _action_keys(self) -> list[str]:
        return [f"{name}{self.config.action_key_suffix}" for name in self.config.action_names]

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {key: float for key in self._action_keys}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_calibrated(self) -> bool:
        return True

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        del calibrate
        try:
            self._connect_ros()
        except Exception:
            self._disconnect_ros()
            raise
        self._connected = True
        logger.info("%s connected.", self)

    def _connect_ros(self) -> None:
        from std_msgs.msg import Bool, Float32MultiArray

        self._ros.connect()
        node = self._ros.node
        self._subscription = node.create_subscription(
            Float32MultiArray,
            self.config.action_topic,
            self._action_callback,
            10,
        )
        if self.config.manage_control_mode and self.config.set_manual_mode:
            self._recording_mode_publisher = node.create_publisher(
                Bool,
                self.config.recording_mode_topic,
                make_recording_mode_qos(),
            )
            publish_recording_mode(node, self._recording_mode_publisher, manual_mode=True)

        wait_for_value(self._action, self._lock, self.config.connect_timeout_s, self.config.action_topic)

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    @check_if_not_connected
    def get_action(self) -> RobotAction:
        with self._lock:
            check_not_stale(self._action, self.config.stale_timeout_s, self.config.action_topic)
            values = list(self._action.value)
        return dict(zip(self._action_keys, values, strict=True))

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        del feedback

    @check_if_not_connected
    def disconnect(self) -> None:
        self._disconnect_ros()
        logger.info("%s disconnected.", self)

    def _disconnect_ros(self) -> None:
        self._subscription = None
        self._recording_mode_publisher = None
        self._connected = False
        self._ros.disconnect()

    def _action_callback(self, msg) -> None:
        try:
            values = float_multi_array_to_list(msg, len(self.config.action_names), self.config.action_topic)
        except ValueError as exc:
            logger.warning("Ignoring invalid teleoperator action on %s: %s", self.config.action_topic, exc)
            return
        with self._lock:
            # connect() waits on this instance, so mutate rather than replace it.
            self._action.value = values
            self._action.stamp = time.monotonic()
