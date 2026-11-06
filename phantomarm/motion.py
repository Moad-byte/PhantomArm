"""Rate-limited joint transitions. Avoids instantaneous discontinuities."""
from .kinematics import JOINT_BOUNDS

def slew(current, target, dt, rates=(1.6,1.4,1.6,2.0)):
    if not 0<=dt<=1:
        raise ValueError('Unreasonable integration time')
    result=[]
    for curr,wanted,rate,(lo,hi) in zip(current,target,rates,JOINT_BOUNDS):
        desired=max(lo,min(hi,wanted))
        step=max(-rate*dt,min(rate*dt,desired-curr))
        result.append(curr+step)
    return tuple(result)
