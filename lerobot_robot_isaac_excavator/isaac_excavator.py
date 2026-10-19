from __future__ import annotations

import logging
import threading
import time
from functools import cached_property
from typing import Any

from lerobot.robots import Robot
from lerobot.types import RobotAction, RobotObservation
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from .config_isaac_excavator import IsaacExcavatorConfig
from .simulation_time import MissingSample, SimTimeline, stamp_ns
from .ros_bridge import (
    LatestValue,
    RosNodeContext,
    check_not_stale,
    finite_float_values,
    image_to_numpy,
    make_recording_mode_qos,
    make_sensor_qos,
    publish_recording_mode,
    wait_for_value,
)

logger = logging.getLogger(__name__)


class IsaacExcavator(Robot):
    config_class = IsaacExcavatorConfig
    name = "isaac_excavator"

    def __init__(self, config: IsaacExcavatorConfig) -> None:
        super().__init__(config)
        self.config = config
        self.cameras = {
            "camera2": (config.cabin_rgb_topic, self._camera_shape),
            "camera3": (config.gripper_rgb_topic, self._camera_shape),
        }
        self._ros = RosNodeContext(config.node_name)
        self._lock = threading.Lock()
        self._state = LatestValue()
        self._images = {camera: LatestValue() for camera in self.cameras}
        self._subscriptions: list[Any] = []
        self._action_publisher: Any = None
        self._recording_mode_publisher = None
        self._connected = False
        self._legacy_state_warning_emitted = False
        self.timeline = SimTimeline(config.sync_buffer_size, config.sync_tolerance_s, config.sync_grace_s,
                                    clock_capacity=config.sync_clock_buffer_size)
        self._stamped_action_publisher: Any = None

    @property
    def _camera_shape(self) -> tuple[int, int, int]:
        return (self.config.camera_height, self.config.camera_width, self.config.camera_channels)

    @property
    def _state_keys(self) -> list[str]:
        return [f"{name}.pos" for name in self.config.state_names]

    @property
    def _action_keys(self) -> list[str]:
        return [f"{name}{self.config.action_key_suffix}" for name in self.config.action_names]

    @cached_property
    def observation_features(self) -> dict[str, type | tuple[int, int, int]]:
        return {
            **{key: float for key in self._state_keys},
            "camera2": self._camera_shape,
            "camera3": self._camera_shape,
        }

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {key: float for key in self._action_keys}

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_calibrated(self) -> bool:
        return True

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        del calibrate
        if self.config.start_mode not in {"policy", "manual", "unchanged"}:
            raise ValueError("start_mode must be one of: policy, manual, unchanged")
        try:
            self._connect_ros()
        except Exception:
            self._disconnect_ros()
            raise
        self._connected = True
        logger.info("%s connected.", self)

    def _connect_ros(self) -> None:
        from sensor_msgs.msg import Image
        from std_msgs.msg import Bool, Float32MultiArray

        self._ros.connect()
        node = self._ros.node
        # Fixed simulation-grid recording needs every source frame. Best-effort
        # depth=1 can lose an image before it reaches the stamped buffer.
        image_qos = (make_sensor_qos(depth=self.config.sync_buffer_size, reliable=True)
                     if self.config.simulation_time else make_sensor_qos())

        self._subscriptions = [
            node.create_subscription(
                Float32MultiArray,
                self.config.state_topic,
                self._state_callback,
                10,
            ),
            node.create_subscription(
                Image,
                self.config.cabin_rgb_topic,
                lambda msg: self._image_callback("camera2", msg),
                image_qos,
            ),
            node.create_subscription(
                Image,
                self.config.gripper_rgb_topic,
                lambda msg: self._image_callback("camera3", msg),
                image_qos,
            ),
        ]
        self._action_publisher = node.create_publisher(Float32MultiArray, self.config.action_topic, 10)
        if self.config.simulation_time:
            from rosgraph_msgs.msg import Clock
            from sensor_msgs.msg import JointState
            self._subscriptions.extend([
                node.create_subscription(Clock, self.config.clock_topic,
                                         lambda msg: self.timeline.update_clock(stamp_ns(msg.clock)),
                                         make_sensor_qos(depth=self.config.sync_clock_buffer_size, reliable=True)),
                node.create_subscription(JointState, self.config.sample_topic, self._sample_callback,
                                         self.config.sync_buffer_size),
            ])
            self._stamped_action_publisher = node.create_publisher(JointState, self.config.stamped_action_topic, 10)
            try:
                self.timeline.wait_ready(self.config.connect_timeout_s)
            except MissingSample as exc:
                from rclpy.qos import ReliabilityPolicy
                publishers = node.get_publishers_info_by_topic(self.config.clock_topic)
                if publishers and all(p.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
                                      for p in publishers):
                    raise MissingSample(
                        f"{self.config.clock_topic} is still published as BEST_EFFORT, but this recorder "
                        "requires RELIABLE clock delivery. Restart Isaac with the updated "
                        "env.usd and launch_splat_simulation.cmd."
                    ) from exc
                raise
        if self.config.manage_control_mode:
            self._recording_mode_publisher = node.create_publisher(
                Bool,
                self.config.recording_mode_topic,
                make_recording_mode_qos(),
            )
            if self.config.start_mode == "policy":
                publish_recording_mode(node, self._recording_mode_publisher, manual_mode=False)
            elif self.config.start_mode == "manual":
                publish_recording_mode(node, self._recording_mode_publisher, manual_mode=True)

        wait_for_value(self._state, self._lock, self.config.connect_timeout_s, self.config.state_topic)
        for camera, (topic_name, _) in self.cameras.items():
            wait_for_value(self._images[camera], self._lock, self.config.connect_timeout_s, topic_name)

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def _sample_callback(self, msg):
        import math
        expected = ["state/" + k for k in self._state_keys] + ["action/" + k for k in self._action_keys]
        if list(msg.name) != expected or len(msg.position) != 10 or not all(math.isfinite(v) for v in msg.position):
            logger.warning("Rejecting sim_sample with incorrect names, dimensions, or non-finite values.")
            return
        self.timeline.put("sample", stamp_ns(msg.header.stamp),
                          (tuple(msg.position[:5]), tuple(msg.position[5:]), msg.header.frame_id,
                           tuple(msg.effort)),
                          run_id=msg.header.frame_id)

    def observation_from_sample(self, sample):
        return {**dict(zip(self._state_keys, sample.state, strict=True)),
                **{key: value.copy() for key, value in sample.images.items()}}

    def action_from_sample(self, sample):
        return dict(zip(self._action_keys, sample.action, strict=True))

    def send_action_at_sim_time(self, action, ns, run_id):
        from sensor_msgs.msg import JointState
        msg = JointState()
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(int(ns), 1_000_000_000)
        msg.header.frame_id = run_id
        msg.name = self._action_keys
        msg.position = [float(action[k]) for k in self._action_keys]
        self._stamped_action_publisher.publish(msg)
        return action

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        with self._lock:
            check_not_stale(self._state, 0 if self.config.simulation_time else self.config.stale_timeout_s,
                            self.config.state_topic)
            state_values = list(self._state.value)

            images = {}
            for camera, (topic_name, _) in self.cameras.items():
                latest = self._images[camera]
                check_not_stale(latest, 0 if self.config.simulation_time else self.config.camera_stale_timeout_s,
                                topic_name)
                images[camera] = latest.value.copy()

        obs: RobotObservation = dict(zip(self._state_keys, state_values, strict=True))
        obs.update(images)
        return obs

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        from std_msgs.msg import Float32MultiArray

        ordered_values = []
        sent: RobotAction = {}
        for raw_name, key in zip(self.config.action_names, self._action_keys, strict=True):
            if key in action:
                value = float(action[key])
            elif raw_name in action:
                value = float(action[raw_name])
            else:
                raise KeyError(f"Missing action key {key!r} (legacy alias {raw_name!r}).")
            ordered_values.append(value)
            sent[key] = value

        msg = Float32MultiArray()
        msg.data = ordered_values
        self._action_publisher.publish(msg)
        return sent

    @check_if_not_connected
    def disconnect(self) -> None:
        self._disconnect_ros()
        logger.info("%s disconnected.", self)

    def _disconnect_ros(self) -> None:
        self._subscriptions = []
        self._action_publisher = None
        self._stamped_action_publisher = None
        self._recording_mode_publisher = None
        self._connected = False
        self._ros.disconnect()

    def _state_callback(self, msg) -> None:
        try:
            values = self._normalize_state(msg)
        except ValueError as exc:
            # ROS callback exceptions stop the executor thread in rclpy. A bad
            # message must therefore be rejected locally, not propagated.
            logger.warning("Ignoring invalid state message on %s: %s", self.config.state_topic, exc)
            return
        with self._lock:
            # Keep this object stable: connect() is waiting on this exact
            # instance while ROS callbacks arrive on the executor thread.
            self._state.value = values
            self._state.stamp = time.monotonic()

    def _image_callback(self, camera: str, msg) -> None:
        topic_name, shape = self.cameras[camera]
        try:
            image = image_to_numpy(msg, shape, topic_name)
        except ValueError as exc:
            logger.warning("Ignoring invalid image on %s: %s", topic_name, exc)
            return
        with self._lock:
            latest = self._images[camera]
            latest.value = image
            latest.stamp = time.monotonic()
        if self.config.simulation_time:
            self.timeline.put(camera, stamp_ns(msg.header.stamp), image)

    def _normalize_state(self, msg) -> list[float]:
        """Accept the current five-value contract and the pre-bridge USD format."""
        values = finite_float_values(msg.data, self.config.state_topic)
        expected_size = len(self.config.state_names)
        if len(values) == expected_size:
            return values

        if self.config.allow_legacy_six_value_state and len(values) == expected_size + 1:
            claw_1, claw_2 = values[-2:]
            threshold = float(self.config.legacy_claw_closed_threshold_rad)
            claw_closed = float(max(abs(claw_1), abs(claw_2)) <= threshold)
            if not self._legacy_state_warning_emitted:
                logger.warning(
                    "Using legacy six-value observation state; regenerate the Isaac USD to publish "
                    "[slew, arm, bucket, boom, claw_closed] directly."
                )
                self._legacy_state_warning_emitted = True
            return values[: expected_size - 1] + [claw_closed]

        raise ValueError(
            f"expected {expected_size} values (or legacy {expected_size + 1}), got {len(values)}"
        )
