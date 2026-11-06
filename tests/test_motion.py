import pytest
from phantomarm.motion import slew

def test_no_jump():
    p=slew((0,0,0,0),(1,1,-1,1),.02)
    assert p==pytest.approx((.032,.028,-.032,.04))

def test_reaches_target():
    q=(0,0,0,0)
    for _ in range(100):q=slew(q,(.5,.5,-.5,.5),.02)
    assert q==pytest.approx((.5,.5,-.5,.5))

def test_clamps_joint_target():
    q=slew((0,0,0,0),(99,99,99,99),1)
    assert all(v<=bound for v,bound in zip(q,(1.55,1.65,2.50,2.80)))

def test_reject_large_step():
    with pytest.raises(ValueError):slew((0,0,0,0),(0,0,0,0),1.5)
