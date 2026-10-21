from __future__ import annotations

from contextlib import ExitStack
import math
import threading
import time
from dataclasses import dataclass
from typing import Any

import numpy as np


_CONTEXT_LOCK = threading.Lock()
_ACTIVE_CONTEXTS = 0
_RCLPY_STARTED_BY_PLUGIN = False


@dataclass
class LatestValue:
    value: Any = None
    stamp: float = 0.0


class RosNodeContext:
    def __init__(self, node_name: str) -> None:
        self.node_name = node_name
        # ROS is loaded lazily and may have no installed typing stubs.
        self.node: Any = None
        self.executor: Any = None
        self.thread: threading.Thread | None = None
        self._registered = False

    def connect(self) -> None:
        global _ACTIVE_CONTEXTS, _RCLPY_STARTED_BY_PLUGIN

        if self.node is not None:
            return

        import rclpy
        from rclpy.executors import MultiThreadedExecutor

        with _CONTEXT_LOCK:
            if not rclpy.ok():
                rclpy.init(args=None)
                _RCLPY_STARTED_BY_PLUGIN = True
            _ACTIVE_CONTEXTS += 1
            self._registered = True

        try:
            self.node = rclpy.create_node(self.node_name)
            self.executor = MultiThreadedExecutor(num_threads=2)
            self.executor.add_node(self.node)
            self.thread = threading.Thread(target=self.executor.spin, daemon=True)
            self.thread.start()
        except Exception:
            self.disconnect()
            raise

    def disconnect(self) -> None:
        if not self._registered:
            return

        node, executor, thread = self.node, self.executor, self.thread
        self.node = None
        self.executor = None
        self.thread = None
        # Attempt every cleanup even when a partially initialized resource fails.
        with ExitStack() as cleanup:
            cleanup.callback(self._release_context)
            if node is not None:
                cleanup.callback(node.destroy_node)
            if thread is not None and thread.ident is not None:
                cleanup.callback(thread.join, timeout=2.0)
            if executor is not None:
                cleanup.callback(executor.shutdown)

    def _release_context(self) -> None:
        global _ACTIVE_CONTEXTS, _RCLPY_STARTED_BY_PLUGIN

        import rclpy

        with _CONTEXT_LOCK:
            if not self._registered:
                return
            self._registered = False
            _ACTIVE_CONTEXTS -= 1
            if _ACTIVE_CONTEXTS == 0 and _RCLPY_STARTED_BY_PLUGIN:
                _RCLPY_STARTED_BY_PLUGIN = False
                if rclpy.ok():
                    rclpy.shutdown()


def make_sensor_qos(depth=1, reliable=False):
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    return QoSProfile(
        depth=depth,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def make_recording_mode_qos():
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    return QoSProfile(
        depth=1,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def publish_recording_mode(node, publisher, manual_mode: bool) -> None:
    from std_msgs.msg import Bool

    del node
    msg = Bool()
    msg.data = bool(manual_mode)
    publisher.publish(msg)
    # Give transient-local mode updates a chance to leave before the first action.
    time.sleep(0.05)


def wait_for_value(
    latest: LatestValue,
    lock: threading.Lock,
    timeout_s: float,
    name: str,
) -> Any:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        with lock:
            if latest.value is not None:
                return latest.value
        time.sleep(0.01)
    raise TimeoutError(f"Timed out waiting for ROS topic {name!r}. Is Isaac Sim playing?")


def check_not_stale(latest: LatestValue, stale_timeout_s: float, name: str) -> None:
    age = time.monotonic() - latest.stamp
    if latest.value is None:
        raise RuntimeError(f"No message has been received yet from ROS topic {name!r}.")
    if stale_timeout_s > 0 and age > stale_timeout_s:
        raise RuntimeError(f"Latest ROS topic {name!r} message is stale ({age:.3f}s old).")


def finite_float_values(values, topic_name: str) -> list[float]:
    """Reject malformed numeric samples without logging their payloads."""
    try:
        result = [float(value) for value in values]
    except (TypeError, ValueError, OverflowError):
        raise ValueError(f"ROS topic {topic_name!r} must contain numeric values") from None
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"ROS topic {topic_name!r} must contain finite values")
    return result


def float_multi_array_to_list(msg, expected_len: int, topic_name: str) -> list[float]:
    values = finite_float_values(msg.data, topic_name)
    if len(values) != expected_len:
        raise ValueError(
            f"ROS topic {topic_name!r} produced {len(values)} values, expected {expected_len}"
        )
    return values


def image_to_numpy(msg, expected_shape: tuple[int, int, int], topic_name: str) -> np.ndarray:
    height, width, channels = expected_shape
    if msg.height != height or msg.width != width:
        raise ValueError(
            f"ROS topic {topic_name!r} image shape is {(msg.height, msg.width)}, expected {(height, width)}"
        )

    encoding = msg.encoding.lower()
    pixel_channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}.get(encoding)
    if pixel_channels is None:
        raise ValueError(f"Unsupported image encoding on ROS topic {topic_name!r}.")
    if msg.step < width * pixel_channels or len(msg.data) != height * msg.step:
        raise ValueError(f"ROS topic {topic_name!r} image row stride or buffer length is invalid")
    raw = bytes(msg.data)
    row = np.frombuffer(raw, dtype=np.uint8).reshape((msg.height, msg.step))

    if encoding == "rgb8":
        image = row[:, : msg.width * 3].reshape((msg.height, msg.width, 3))
    elif encoding == "bgr8":
        image = row[:, : msg.width * 3].reshape((msg.height, msg.width, 3))[:, :, ::-1]
    elif encoding == "rgba8":
        image = row[:, : msg.width * 4].reshape((msg.height, msg.width, 4))[:, :, :3]
    elif encoding == "bgra8":
        image = row[:, : msg.width * 4].reshape((msg.height, msg.width, 4))[:, :, [2, 1, 0]]
    else:  # mono8
        gray = row[:, : msg.width].reshape((msg.height, msg.width))
        image = np.repeat(gray[:, :, None], 3, axis=2)

    if image.shape != (height, width, channels):
        raise ValueError(f"ROS topic {topic_name!r} converted to {image.shape}, expected {expected_shape}")
    return np.ascontiguousarray(image)
