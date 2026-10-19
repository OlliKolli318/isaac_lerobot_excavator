import math

import pytest

from kaivuri_lerobot_bridge.action_contract import InvalidAction, PolicyWatchdog, SAFE_ACTION, normalize_action


def test_axes_clamp_and_claw_threshold():
    assert normalize_action([-2, 2, -0.5, 0.5, 0.5]) == (-1, 1, -0.5, 0.5, 0)
    assert normalize_action([0, 0, 0, 0, 0.51])[-1] == 1


def test_custom_action_dimensions():
    assert normalize_action([2, 0.6, 0.2], action_size=3, axis_count=1) == (1, 1, 0)


@pytest.mark.parametrize('value', [math.nan, math.inf, -math.inf, None, 'invalid', 10**400])
def test_invalid_values_are_rejected_without_echoing_payload(value):
    with pytest.raises(InvalidAction):
        normalize_action([value, 0, 0, 0, 1])
    if value == 'invalid':
        with pytest.raises(InvalidAction) as caught:
            normalize_action([value, 0, 0, 0, 1])
        assert value not in str(caught.value)


@pytest.mark.parametrize('values,axes', [([0] * 4, 4), ([0] * 5, 6), ([0] * 5, -1)])
def test_wrong_dimensions_are_rejected(values, axes):
    with pytest.raises(InvalidAction):
        normalize_action(values, axis_count=axes)


@pytest.mark.parametrize('timeout', [0, -1, math.nan, math.inf, -math.inf])
def test_timeout_must_be_finite_and_positive(timeout):
    with pytest.raises(ValueError):
        PolicyWatchdog(timeout)


def test_watchdog_modes_and_timeout_boundary():
    watchdog = PolicyWatchdog(0.5)
    assert watchdog.select(10, False, True).reason == 'disabled'
    assert watchdog.select(10, True, False).reason == 'manual'
    assert watchdog.select(10, True, True).reason == 'policy_waiting'
    watchdog.update([0.2, 0, 0, 0, 0], 10)
    assert watchdog.select(10.5, True, True).action == (0.2, 0, 0, 0, 0)
    assert watchdog.select(10.5001, True, True).action == SAFE_ACTION
    assert watchdog.select(9, True, True).reason == 'policy_stale'


@pytest.mark.parametrize('now', [math.nan, math.inf, -math.inf])
def test_nonfinite_clock_cannot_keep_policy_alive(now):
    watchdog = PolicyWatchdog(0.5)
    watchdog.update([1, 0, 0, 0, 0], 10)
    with pytest.raises(InvalidAction):
        watchdog.update([0, 1, 0, 0, 0], now)
    assert watchdog.select(now, True, True).action == SAFE_ACTION
    assert watchdog.select(10, True, True).action == (1, 0, 0, 0, 0)


def test_invalid_action_does_not_refresh_watchdog():
    watchdog = PolicyWatchdog(0.5)
    watchdog.update([1, 0, 0, 0, 0], 10)
    with pytest.raises(InvalidAction):
        watchdog.update([math.nan, 0, 0, 0, 1], 10.4)
    assert watchdog.select(10.6, True, True).reason == 'policy_stale'
