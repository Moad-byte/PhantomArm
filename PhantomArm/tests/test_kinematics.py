import math
import pytest
from phantomarm.kinematics import Pose,inverse_kinematics,forward_kinematics,UnreachableTarget

@pytest.mark.parametrize('p',[
    Pose(.38,0,.35),Pose(.30,.12,.34),Pose(.43,-.1,.4),
    Pose(.28,.03,.44),Pose(.48,0,.30),Pose(.34,-.09,.46)])
def test_fk_ik_roundtrip(p):
    out=forward_kinematics(inverse_kinematics(p))
    assert math.dist((p.x,p.y,p.z),(out.x,out.y,out.z))<1e-8

def test_unreachable_far():
    with pytest.raises(UnreachableTarget):inverse_kinematics(Pose(3,0,.5))

def test_nonfinite():
    with pytest.raises(UnreachableTarget):inverse_kinematics(Pose(float('nan'),0,.35))

def test_joint_limit():
    with pytest.raises(UnreachableTarget):inverse_kinematics(Pose(-.30,0,.4))
