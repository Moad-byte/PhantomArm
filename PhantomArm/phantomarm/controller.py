"""Host-side calibration, filtering, motion envelope, and interlocks.

HOLD freezes the most recent joint targets if the source becomes stale.
ESTOP is a software simulation interlock, not a certified hardware stop.
"""
from dataclasses import dataclass
from enum import Enum
import math
from .kinematics import Pose, inverse_kinematics, UnreachableTarget
from .telemetry import Sample

class State(str,Enum):
    READY='READY'
    HOLD='HOLD'
    ESTOP='ESTOP'

@dataclass(frozen=True)
class MotionResult:
    state: State
    target: Pose | None
    gripper_open: bool
    reason: str


def clamp(x, lo, hi):
    return max(lo,min(hi,x))

class Controller:
    def __init__(self, alpha=0.25, stale_after=0.40):
        if not (0 < alpha <= 1):
            raise ValueError('EMA alpha must be in (0,1]')
        self.alpha=alpha
        self.stale_after=stale_after
        self.neutral=None
        self.smoothed=(0.0,0.0)
        self.last_seen=None
        self.last_seq=None
        self.state=State.HOLD
        self.estop=False
        self.gripper_open=True
        self.prev_button=False
        self.height=0.35
        self.manual_dx=0.0
        self.manual_dy=0.0
        self.calibration_count=0
        self.last_calibration=None
        self.last_target=Pose(0.38,0,self.height)
        self._want_calibration=False

    def calibrate_next(self):
        self._want_calibration=True
        self.state=State.HOLD

    def set_height(self,dz):
        if not self.estop:
            new_h = clamp(self.height+dz,0.30,0.52)
            candidate = Pose(self.last_target.x,self.last_target.y,new_h)
            try:
                inverse_kinematics(candidate)
                self.height = new_h
                self.last_target = candidate
            except UnreachableTarget:
                pass

    def nudge_xy(self, dx=0.0, dy=0.0):
        """Keyboard jog offsets supplement IMU-driven X/Y input.

        Offsets and the resulting target are limited to the same conservative
        Cartesian workspace as the original IMU mapping and checked for IK reachability.
        """
        if self.estop:
            return
        candidate = Pose(
            clamp(self.last_target.x+dx,0.27,0.49),
            clamp(self.last_target.y+dy,-0.17,0.17),self.height)
        try:
            inverse_kinematics(candidate)
            self.manual_dx=clamp(self.manual_dx+dx,-0.11,0.11)
            self.manual_dy=clamp(self.manual_dy+dy,-0.17,0.17)
            self.last_target=candidate
        except UnreachableTarget:
            pass

    def reset_manual_target(self, height=0.35):
        """Restore neutral XY; optionally choose a safe hover height for grasp scenes."""
        if not math.isfinite(height) or not 0.30 <= height <= 0.52:
            raise ValueError("Reset height must be inside the controller workspace")
        self.manual_dx=0.0
        self.manual_dy=0.0
        self.height=height
        self.last_target=Pose(0.38,0.0,height)

    def emergency_stop(self):
        self.estop=True
        self.state=State.ESTOP

    def reset_emergency(self):
        self.estop=False
        self.state=State.HOLD
        self.last_seen=None
        self.last_seq=None
        self.calibrate_next()

    def toggle_gripper(self):
        if not self.estop:
            self.gripper_open=not self.gripper_open

    def consume(self, s: Sample, at: float) -> MotionResult:
        if not s.valid:
            self.state=State.HOLD if not self.estop else State.ESTOP
            return self.status('Invalid sensor sample')
        # Device millis/sequence may reset after reboot. Reset calibration explicitly
        # rather than silently accepting replayed/repeated values.
        if self.last_seq is not None and s.seq <= self.last_seq:
            self.state=State.HOLD if not self.estop else State.ESTOP
            return self.status('Out-of-order or restarted sample: recalibrate')
        self.last_seq=s.seq
        self.last_seen=at
        if self.neutral is None or self._want_calibration:
            self.neutral=(s.roll_deg,s.pitch_deg)
            self.smoothed=(0.0,0.0)
            self._want_calibration=False
            self.calibration_count+=1
            self.last_calibration=self.neutral
        roll=clamp(s.roll_deg-self.neutral[0],-35,35)
        pitch=clamp(s.pitch_deg-self.neutral[1],-35,35)
        self.smoothed=(self.alpha*roll+(1-self.alpha)*self.smoothed[0],
                       self.alpha*pitch+(1-self.alpha)*self.smoothed[1])
        if s.button and not self.prev_button and not self.estop:
            self.toggle_gripper()
        self.prev_button=s.button
        if self.estop:
            self.state=State.ESTOP
            return self.status('Emergency stop active')
        self.last_target=Pose(
            clamp(0.38-0.11*self.smoothed[1]/35+self.manual_dx,0.27,0.49),
            clamp(0.17*self.smoothed[0]/35+self.manual_dy,-0.17,0.17),
            self.height)
        self.state=State.READY
        return self.status('Tracking')

    def tick(self,at:float) -> MotionResult:
        if self.estop:
            self.state=State.ESTOP
            return self.status('Emergency stop active')
        if self.last_seen is None or at-self.last_seen>self.stale_after:
            self.state=State.HOLD
            return self.status('Telemetry timeout / waiting for valid sample')
        return self.status('Tracking')

    def status(self,reason:str)->MotionResult:
        return MotionResult(self.state,self.last_target,self.gripper_open,reason)
