import pytest
from phantomarm.controller import Controller,State,clamp
from phantomarm.telemetry import Sample

def s(seq,roll=0,pitch=0,button=False,valid=True):
    return Sample(seq*20,seq,roll,pitch,button,valid)

def test_initial_hold():
    c=Controller(); assert c.tick(0).state==State.HOLD

def test_neutral_auto_capture():
    c=Controller(); x=c.consume(s(1,15,-10),1)
    assert x.state==State.READY and x.target.x==pytest.approx(.38) and x.target.y==0

def test_roll_changes_y_and_pitch_changes_x():
    c=Controller(alpha=1);c.consume(s(1),0)
    x=c.consume(s(2,35,35),.02)
    assert x.target.x==pytest.approx(.27) and x.target.y==pytest.approx(.17)

def test_clamps_tilt():
    c=Controller(alpha=1);c.consume(s(1),0)
    x=c.consume(s(2,170,-170),.02)
    assert x.target.x==pytest.approx(.49) and x.target.y==pytest.approx(.17)

def test_watchdog():
    c=Controller();c.consume(s(1),0)
    assert c.tick(.39).state==State.READY
    assert c.tick(.41).state==State.HOLD

def test_invalid_sample():
    c=Controller();c.consume(s(1),0)
    assert c.consume(s(2,valid=False),.02).state==State.HOLD

def test_repeated_frame_rejected():
    c=Controller();c.consume(s(1),0)
    assert c.consume(s(1),.02).state==State.HOLD

def test_button_edge():
    c=Controller();c.consume(s(1),0)
    assert c.gripper_open
    c.consume(s(2,button=True),.02);assert not c.gripper_open
    c.consume(s(3,button=True),.04);assert not c.gripper_open
    c.consume(s(4,button=False),.06)
    c.consume(s(5,button=True),.08);assert c.gripper_open

def test_estop_blocks_motion():
    c=Controller();c.consume(s(1),0);c.emergency_stop()
    assert c.consume(s(2,button=True),.02).state==State.ESTOP
    assert c.gripper_open

def test_reset_neutral():
    c=Controller();c.consume(s(1),0);c.emergency_stop();c.reset_emergency()
    x=c.consume(s(2,20,20),.04)
    assert x.state==State.READY and x.target.x==pytest.approx(.38)

def test_calibration_on_command():
    c=Controller(alpha=1);c.consume(s(1),0)
    c.consume(s(2,10,10),.02);c.calibrate_next()
    x=c.consume(s(3,10,10),.04)
    assert x.target.x==pytest.approx(.38) and x.target.y==pytest.approx(0)

def test_height_boundaries():
    c=Controller();c.set_height(100);assert c.height==.52
    c.set_height(-100);assert c.height==.30
