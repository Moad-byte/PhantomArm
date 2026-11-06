"""Checks that stationary mode holds horizontal target and sweep is opt-in."""
import pytest
from phantomarm.sources import DemoSource
from phantomarm.controller import Controller, State


def test_still_source_has_fixed_orientation():
    src=DemoSource('still')
    src.t0=100.0
    samples=[src.poll(100.10 + i*0.04) for i in range(12)]
    assert all(s is not None for s in samples)
    assert all(s.roll_deg == 0 and s.pitch_deg == 0 for s in samples)
    ctrl=Controller()
    for i,s in enumerate(samples):
        out=ctrl.consume(s, at=100.1+i*0.04)
        assert out.state is State.READY
        assert out.target.x == pytest.approx(0.38)
        assert out.target.y == pytest.approx(0.0)
    ctrl.set_height(0.015)
    out=ctrl.consume(src.poll(101), at=101)
    assert out.target.z == pytest.approx(0.365)
    assert out.target.x == pytest.approx(0.38)
    ctrl.toggle_gripper()
    out=ctrl.consume(src.poll(102),at=102)
    assert out.target.x == pytest.approx(0.38)
    assert out.target.y == pytest.approx(0.0)
    assert out.gripper_open is False


def test_sweep_is_explicit_and_changes_both_axes():
    src=DemoSource('sweep')
    src.t0=100.0
    a=src.poll(100.1)
    b=src.poll(101.0)
    assert a is not None and b is not None
    assert abs(a.roll_deg-b.roll_deg)>1
    assert abs(a.pitch_deg-b.pitch_deg)>1


def test_reject_unsupported_pattern():
    with pytest.raises(ValueError):
        DemoSource('random')
