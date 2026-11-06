"""v1.4 manual Cartesian jogging and calibration regression tests."""
import pytest
from phantomarm.controller import Controller,State
from phantomarm.kinematics import inverse_kinematics
from phantomarm.telemetry import Sample


def sample(seq, roll=0, pitch=0):
    return Sample(seq*20,seq,roll,pitch,False,True)


def test_jog_three_cartesian_directions():
    c=Controller(alpha=1)
    c.consume(sample(1),0.0)
    c.set_height(+.015)
    c.nudge_xy(dy=.015,dx=.015)
    p=c.consume(sample(2),.02).target
    assert p.x==pytest.approx(.395)
    assert p.y==pytest.approx(.015)
    assert p.z==pytest.approx(.365)
    assert inverse_kinematics(p)


def test_jog_stable_across_new_demo_sensor_samples():
    c=Controller(alpha=1)
    c.consume(sample(1),0.0)
    c.nudge_xy(dy=-.03)
    p1=c.consume(sample(2),.02).target
    p2=c.consume(sample(3),.04).target
    assert p1==p2
    assert p2.y==pytest.approx(-.03)


def test_jog_clamping_and_reachability():
    c=Controller(alpha=1)
    c.consume(sample(1),0.0)
    for _ in range(50):c.nudge_xy(dx=.015,dy=.015)
    p=c.consume(sample(2),.02).target
    assert p.x==pytest.approx(.49)
    assert p.y==pytest.approx(.17)
    assert inverse_kinematics(p)


def test_stopped_controller_ignores_manual_jog():
    c=Controller(alpha=1);c.consume(sample(1),0)
    c.emergency_stop()
    before=c.last_target
    c.nudge_xy(dx=.015,dy=.015)
    c.set_height(.015)
    assert c.last_target==before
    assert c.height==pytest.approx(.35)


def test_home_resets_manual_offsets_and_height():
    c=Controller(alpha=1);c.consume(sample(1),0)
    c.nudge_xy(dx=.03,dy=-.06);c.set_height(.07)
    c.reset_manual_target()
    p=c.consume(sample(2),.02).target
    assert (p.x,p.y,p.z)==pytest.approx((.38,0,.35))


def test_calibration_feedback_event():
    c=Controller(alpha=1)
    c.consume(sample(1,5,-5),0)
    assert c.calibration_count==1
    c.calibrate_next()
    c.consume(sample(2,20,14),.02)
    assert c.calibration_count==2
    assert c.last_calibration==(20,14)
    assert c.last_target.x==pytest.approx(.38)
    assert c.last_target.y==pytest.approx(0)
