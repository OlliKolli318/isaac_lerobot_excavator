"""Own the command boundary between LeRobot and the Isaac Action Graph."""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32MultiArray, String
from std_srvs.srv import SetBool, Trigger
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from rclpy.clock import Clock as RclClock, ClockType

from .action_contract import InvalidAction, PolicyWatchdog, normalize_action
from .dagger_control import DaggerLease, decode_control


def transient_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class ControlBridge(Node):
    """Publish exactly one scene command stream and fail closed by default."""

    def __init__(self) -> None:
        super().__init__("kaivuri_lerobot_control_bridge")
        self.declare_parameter("policy_input_topic", "/kaivuri/control/policy_action")
        self.declare_parameter("scene_action_topic", "/kaivuri/policy_action")
        self.declare_parameter("recording_mode_topic", "/kaivuri/recording_mode")
        self.declare_parameter("control_enabled_topic", "/kaivuri/control_enabled")
        self.declare_parameter("status_topic", "/kaivuri/control/status")
        self.declare_parameter("action_timeout_s", 0.4)
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("initial_mode", "disabled")
        self.declare_parameter("time_source", "simulation")
        self.declare_parameter("clock_topic", "/clock")
        self.declare_parameter("sample_topic", "/kaivuri/sim_sample")
        self.declare_parameter("stamped_input_topic", "/kaivuri/control/policy_action_stamped")
        self.declare_parameter("stamped_scene_topic", "/kaivuri/policy_action_stamped")
        self.declare_parameter("transport_timeout_s", 10.0)
        self.declare_parameter("action_names", ["left_stick_x_slew", "left_stick_y_arm",
                                                "right_stick_x_bucket", "right_stick_y_boom", "claw_closed"])
        self.declare_parameter("axis_count", 4)
        self.declare_parameter("safe_action", [0.0, 0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("simulation_mode_topic", "/kaivuri/simulation_mode")
        self.declare_parameter("dagger_state_topic", "/kaivuri/dagger/state")
        self.declare_parameter("dagger_request_topic", "/kaivuri/dagger/request")
        self._action_names = [str(n) + ".pos" for n in self.get_parameter("action_names").value]
        self._safe_action = tuple(self.get_parameter("safe_action").value)
        self._axis_count = int(self.get_parameter("axis_count").value)
        if len(self._action_names) != len(self._safe_action) or len(set(self._action_names)) != len(self._action_names):
            raise ValueError("action_names and safe_action must have equal length and unique names")
        normalize_action(self._safe_action, action_size=len(self._safe_action), axis_count=self._axis_count)
        source = self.get_parameter("time_source").value
        if source not in ("simulation", "wall"):
            raise ValueError("time_source must be simulation or wall")
        self._simulation = source == "simulation"
        self._sim_time = None
        self._run_id = None
        self._retired_runs: set[str] = set()
        self._run_arrived_before_reset = False
        self._last_packet_ns = -1
        self._last_packet_wall = None

        timeout_s = max(0.01, float(self.get_parameter("action_timeout_s").value))
        self._watchdog = self._new_watchdog(timeout_s)
        self._policy_mode = False
        self._enabled = False
        self._last_status = ""
        self._last_rejection = ""
        self._dagger = DaggerLease()

        initial_mode = str(self.get_parameter("initial_mode").value).lower()
        if initial_mode == "manual":
            self._policy_mode = False
            self._enabled = True
        elif initial_mode == "policy":
            self._policy_mode = True
            self._enabled = True
        elif initial_mode != "disabled":
            raise ValueError("initial_mode must be manual, policy, or disabled")

        policy_topic = str(self.get_parameter("policy_input_topic").value)
        scene_topic = str(self.get_parameter("scene_action_topic").value)
        mode_topic = str(self.get_parameter("recording_mode_topic").value)
        enabled_topic = str(self.get_parameter("control_enabled_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)

        self.create_subscription(Float32MultiArray, policy_topic, self._on_policy, 10)
        self._scene_action_pub = self.create_publisher(Float32MultiArray, scene_topic, 10)
        self._manual_mode_pub = self.create_publisher(Bool, mode_topic, transient_qos())
        self._enabled_pub = self.create_publisher(Bool, enabled_topic, transient_qos())
        self._status_pub = self.create_publisher(String, status_topic, transient_qos())
        self._simulation_pub = self.create_publisher(Bool, self.get_parameter("simulation_mode_topic").value, transient_qos())
        self._dagger_pub = self.create_publisher(String, self.get_parameter("dagger_state_topic").value, transient_qos())
        self.create_subscription(String, self.get_parameter("dagger_request_topic").value, self._on_dagger, 10)
        self._stamped_pub = self.create_publisher(JointState, self.get_parameter("stamped_scene_topic").value, 10)
        self.create_subscription(JointState, self.get_parameter("stamped_input_topic").value, self._on_stamped_policy, 10)
        self.create_subscription(JointState, self.get_parameter("sample_topic").value, self._on_sample, 10)
        self.create_subscription(Clock, self.get_parameter("clock_topic").value, self._on_clock,
                                 QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.create_service(SetBool, "~/set_policy_mode", self._set_policy_mode)
        self.create_service(Trigger, "~/disable", self._disable)
        self.create_service(Trigger, "~/enable", self._enable)
        self.create_service(Trigger, "~/status", self._status_service)

        rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        # This is a mode/safety heartbeat. Actions are forwarded once per
        # received packet and consumed on their timestamp inside Isaac.
        self.create_timer(1.0 / rate_hz, self._publish_tick, clock=RclClock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(
            f"Control bridge ready: starts {self._mode_name()}; "
            f"time_source={source}; "
            f"policy input={policy_topic}; scene sink={scene_topic}"
        )

    def _on_policy(self, msg: Float32MultiArray) -> None:
        if self._simulation:
            return
        try:
            self._watchdog.update(msg.data, time.monotonic())
            self._last_rejection = ""
        except InvalidAction as exc:
            self._last_rejection = str(exc)
            self.get_logger().warning(f"Rejected policy action: {exc}", throttle_duration_sec=1.0)

    def _new_watchdog(self, timeout_s: float) -> PolicyWatchdog:
        return PolicyWatchdog(timeout_s, self._safe_action, self._axis_count)

    def _reset_policy(self) -> None:
        self._watchdog = self._new_watchdog(self._watchdog.timeout_s)
        self._last_packet_ns = -1
        self._last_packet_wall = None

    def _on_dagger(self, msg):
        if not self._simulation:
            return
        try:
            previous = self._dagger.state
            if not self._dagger.accept(msg.data, time.monotonic()):
                return
            if previous != self._dagger.state:
                _, _, phase = decode_control(self._dagger.state)
                self._enabled = phase in ("autonomous", "correcting", "armed")
                self._policy_mode = phase == "autonomous"
                self._reset_policy()
            self._publish_tick()
        except ValueError as exc:
            self.get_logger().warning(str(exc))

    def _now(self):
        return (self._sim_time or 0.0) if self._simulation else time.monotonic()

    def _on_clock(self, msg):
        now = msg.clock.sec + msg.clock.nanosec / 1e9
        if self._sim_time is not None and now < self._sim_time:
            if self._run_id and not self._run_arrived_before_reset:
                self._retired_runs.add(self._run_id)
            self._run_arrived_before_reset = False
            self._run_id = None
            self._reset_policy()
        self._sim_time = now

    def _on_sample(self, msg):
        run_id = msg.header.frame_id
        if not run_id or run_id in self._retired_runs:
            return
        if run_id != self._run_id:
            if self._run_id is not None:
                self._retired_runs.add(self._run_id)
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                self._run_arrived_before_reset = self._sim_time is not None and stamp < self._sim_time
            self._run_id = run_id
            self._reset_policy()

    def _on_stamped_policy(self, msg):
        if not self._simulation or self._sim_time is None or not self._run_id:
            return
        names = self._action_names
        ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        try:
            values = normalize_action(msg.position, action_size=len(self._safe_action), axis_count=self._axis_count)
            dagger = self._dagger.effective(time.monotonic())
            expected_run = self._run_id + ("#" + dagger if dagger else "")
            if dagger and decode_control(dagger)[2] != "autonomous":
                return
            if list(msg.name) != names or msg.header.frame_id != expected_run:
                raise InvalidAction("stamped action names or simulation run ID do not match")
            if ns <= self._last_packet_ns or not -0.1 <= ns / 1e9 - self._sim_time <= 1.0:
                raise InvalidAction("action is out of order, expired, or too far in the future")
            if not self._enabled or not self._policy_mode:
                return
            msg.position = list(values)
            self._last_packet_ns = ns
            self._last_packet_wall = time.monotonic()
            self._watchdog.update(values, self._sim_time)
            self._last_rejection = ""
            self._stamped_pub.publish(msg)
        except InvalidAction as exc:
            self._last_rejection = str(exc)
            self.get_logger().warning(f"Rejected stamped policy action: {exc}", throttle_duration_sec=1.0)

    def _set_policy_mode(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        if self._dagger.effective(time.monotonic()):
            response.success = False
            response.message = "DAgger owns control; use its pause/correction controls or exit the collector."
            return response
        self._policy_mode = bool(request.data)
        self._enabled = True
        response.success = True
        response.message = f"control enabled in {self._mode_name()} mode"
        return response

    def _disable(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self._enabled = False
        response.success = True
        response.message = "control disabled; Isaac holds its current articulation positions"
        return response

    def _enable(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        if self._dagger.effective(time.monotonic()):
            response.success = False
            response.message = "DAgger owns control; request a new phase in the collector."
            return response
        self._enabled = True
        response.success = True
        response.message = f"control enabled in {self._mode_name()} mode"
        return response

    def _status_service(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        response.success = True
        response.message = self._format_status(self._watchdog.select(self._now(), self._enabled, self._policy_mode).reason)
        return response

    def _publish_tick(self) -> None:
        selection = self._watchdog.select(self._now(), self._enabled, self._policy_mode)
        if not self._simulation:
            command = Float32MultiArray()
            command.data = list(selection.action)
            self._scene_action_pub.publish(command)
        self._simulation_pub.publish(Bool(data=self._simulation))
        self._manual_mode_pub.publish(Bool(data=not self._policy_mode))
        active = self._enabled
        if self._simulation and self._policy_mode:
            transport_ok = (self._last_packet_wall is not None and time.monotonic() - self._last_packet_wall
                            < float(self.get_parameter("transport_timeout_s").value))
            active = active and selection.reason == "policy" and transport_ok
        dagger = self._dagger.effective(time.monotonic())
        if dagger and decode_control(dagger)[2] in ("idle", "paused"):
            active = False
        self._dagger_pub.publish(String(data=dagger))
        self._enabled_pub.publish(Bool(data=active))

        status = self._format_status(selection.reason)
        if status != self._last_status:
            self._status_pub.publish(String(data=status))
            self._last_status = status
            self.get_logger().info(status)

    def _mode_name(self) -> str:
        if not self._enabled:
            return "disabled"
        return "policy" if self._policy_mode else "manual"

    def _format_status(self, reason: str) -> str:
        status = f"mode={self._mode_name()} command={reason}"
        if self._last_rejection:
            status += f" rejected={self._last_rejection}"
        return status


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = ControlBridge()
        rclpy.spin(node)
    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            rclpy.shutdown()
