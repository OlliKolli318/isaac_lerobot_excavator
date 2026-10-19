"""Pure, testable validation for the scene's five-value joystick action."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


ACTION_SIZE = 5
AXIS_COUNT = 4
SAFE_ACTION = (0.0, 0.0, 0.0, 0.0, 1.0)


class InvalidAction(ValueError):
    """An action cannot safely be forwarded to the simulation."""


def normalize_action(values: Sequence[float], *, action_size: int = ACTION_SIZE,
                     axis_count: int = AXIS_COUNT) -> tuple[float, ...]:
    """Clamp normalized axes and binarize remaining values."""
    if not 0 <= axis_count <= action_size:
        raise InvalidAction("axis_count must be between zero and action_size")
    if len(values) != action_size:
        raise InvalidAction(f"expected {action_size} action values, got {len(values)}")

    result = []
    for index, raw_value in enumerate(values):
        try:
            value = float(raw_value)
        except (TypeError, ValueError, OverflowError):
            raise InvalidAction(f"action[{index}] is not numeric") from None
        if not math.isfinite(value):
            raise InvalidAction(f"action[{index}] must be finite, got {value!r}")
        if index < axis_count:
            result.append(max(-1.0, min(1.0, value)))
        else:
            result.append(1.0 if value > 0.5 else 0.0)
    return tuple(result)


@dataclass(frozen=True)
class Selection:
    action: tuple[float, ...]
    reason: str


class PolicyWatchdog:
    """Stores the last valid policy command and makes staleness explicit."""

    def __init__(self, timeout_s: float, safe_action: Sequence[float] = SAFE_ACTION,
                 axis_count: int = AXIS_COUNT) -> None:
        if not math.isfinite(timeout_s) or timeout_s <= 0.0:
            raise ValueError("timeout_s must be positive and finite")
        self.timeout_s = timeout_s
        self.axis_count = axis_count
        self.safe_action = normalize_action(safe_action, action_size=len(safe_action), axis_count=axis_count)
        self._latest: tuple[float, ...] | None = None
        self._received_at = 0.0

    def update(self, values: Sequence[float], received_at: float) -> None:
        if not math.isfinite(received_at):
            raise InvalidAction("received_at must be finite")
        self._latest = normalize_action(values, action_size=len(self.safe_action), axis_count=self.axis_count)
        self._received_at = received_at

    def select(self, now: float, enabled: bool, policy_mode: bool) -> Selection:
        if not enabled:
            return Selection(self.safe_action, "disabled")
        if not policy_mode:
            return Selection(self.safe_action, "manual")
        if self._latest is None:
            return Selection(self.safe_action, "policy_waiting")
        if not math.isfinite(now) or not 0.0 <= now - self._received_at <= self.timeout_s:
            return Selection(self.safe_action, "policy_stale")
        return Selection(self._latest, "policy")
