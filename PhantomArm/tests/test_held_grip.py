import pytest
from phantomarm.held_grip import HeldGrip
from phantomarm.windows_keys import held_gripper_keys


def test_hold_close_progressively():
    grip = HeldGrip()
    assert grip.step(0.5, True, False) == pytest.approx(0.018)
    assert grip.step(0.5, True, False) == pytest.approx(0.011)


def test_release_holds_current_position():
    grip = HeldGrip()
    grip.step(0.7, True, False)
    old = grip.target
    assert grip.step(0.3, False, False) == old


def test_hold_open_slowly():
    grip = HeldGrip(target=0.003)
    assert grip.step(0.5, False, True) == pytest.approx(0.0075)


def test_held_keys_saturate_and_simultaneous_no_move():
    grip = HeldGrip()
    for _ in range(30): grip.step(0.1, True, False)
    assert grip.target == 0.0
    assert grip.step(0.3, True, True) == 0.0
    for _ in range(30): grip.step(0.1, False, True)
    assert grip.target == .025


def test_bad_dt_rejected():
    with pytest.raises(ValueError): HeldGrip().step(-.1, True, False)


def test_windows_keys_noop_outside_windows(monkeypatch):
    from phantomarm import windows_keys
    monkeypatch.setattr(windows_keys.sys, 'platform', 'linux')
    assert held_gripper_keys() == (False, False)
