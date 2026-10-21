import math

import pytest

from kaivuri_lerobot_bridge.dagger_control import DaggerLease, decode_control


@pytest.mark.parametrize('value', ['bad', 'private-payload:0:idle', '0:0:idle',
                                 '1:-1:idle', '1:0:unknown', '1:0:idle:extra',
                                 f'{2**52}:0:idle', f'1:{2**31}:idle'])
def test_invalid_protocol_does_not_echo_payload(value):
    with pytest.raises(ValueError, match='^Invalid DAgger control state$'):
        decode_control(value)


def test_live_owner_cannot_be_replaced_or_replayed():
    lease = DaggerLease()
    assert lease.accept('1:2:autonomous', 10)
    assert not lease.accept('2:0:autonomous', 11)
    assert not lease.accept('1:1:autonomous', 11)
    assert not lease.accept('1:2:correcting', 11)
    assert lease.accept('1:2:autonomous', 11)
    assert lease.effective(12) == '1:2:autonomous'


def test_expired_heartbeat_cannot_restart_motion():
    lease = DaggerLease()
    lease.accept('1:0:autonomous', 10)
    assert lease.effective(12) == '1:0:autonomous'
    assert lease.effective(12.1) == '1:0:idle'
    assert not lease.accept('1:0:autonomous', 12.1)
    assert lease.accept('1:1:autonomous', 12.2)


def test_new_owner_retires_old_session():
    lease = DaggerLease()
    lease.accept('1:0:autonomous', 10)
    assert lease.accept('2:0:idle', 13)
    assert not lease.accept('1:1:autonomous', 14)


def test_off_releases_ownership():
    lease = DaggerLease()
    lease.accept('1:0:off', 10)
    assert lease.effective(10) == ''
    assert lease.accept('2:0:idle', 10.1)


@pytest.mark.parametrize('now', [math.nan, math.inf, -math.inf, 9])
def test_invalid_time_cannot_extend_lease(now):
    lease = DaggerLease()
    lease.accept('1:0:autonomous', 10)
    with pytest.raises(ValueError):
        lease.accept('1:1:autonomous', now)
    assert lease.effective(now) == '1:0:idle'
    assert lease.state == '1:0:autonomous'


@pytest.mark.parametrize('timeout', [0, -1, math.nan, math.inf])
def test_invalid_timeout(timeout):
    with pytest.raises(ValueError):
        DaggerLease(timeout)
