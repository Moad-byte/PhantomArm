"""Analytic IK for the project's original yaw + two-link + wrist arm.

The model uses MuJoCo local +Y pitch axes. Positive joint angles move
an +X link downward. We negate conventional planar elevation angles.
"""
from dataclasses import dataclass
import math

SHOULDER_Z = 0.22
UPPER = 0.30
FOREARM = 0.26
WRIST = 0.10
JOINT_BOUNDS = ((-1.55, 1.55), (-1.65, 1.65), (-2.50, 2.50), (-2.80, 2.80))

class UnreachableTarget(ValueError):
    """Requested end-effector target falls outside reachable joint limits."""

@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    z: float


def forward_kinematics(angles: tuple[float, float, float, float]) -> Pose:
    yaw, sh, el, wr = angles
    # Pitch rotation about local +Y maps +X upward for negative angles.
    r = UPPER * math.cos(sh) + FOREARM * math.cos(sh + el) + WRIST * math.cos(sh + el + wr)
    z = SHOULDER_Z - UPPER * math.sin(sh) - FOREARM * math.sin(sh + el) - WRIST * math.sin(sh + el + wr)
    return Pose(r * math.cos(yaw), r * math.sin(yaw), z)


def inverse_kinematics(target: Pose) -> tuple[float, float, float, float]:
    if not all(math.isfinite(k) for k in (target.x,target.y,target.z)):
        raise UnreachableTarget("Non-finite target")
    r = math.hypot(target.x,target.y)
    yaw = math.atan2(target.y,target.x)
    u = r - WRIST  # horizontal wrist is maintained using the last revolute joint
    v = target.z - SHOULDER_Z
    D = (u*u + v*v - UPPER*UPPER - FOREARM*FOREARM) / (2 * UPPER * FOREARM)
    if not -1 <= D <= 1:
        raise UnreachableTarget("Outside two-link reach")
    # Elbow-up branch; beta is positive in the conventional upward-vertical frame.
    beta = math.acos(max(-1.0,min(1.0,D)))
    alpha = math.atan2(v,u) - math.atan2(FOREARM*math.sin(beta),UPPER+FOREARM*math.cos(beta))
    q = (yaw, -alpha, -beta, alpha+beta)
    if any(not (lo-1e-8 <= angle <= hi+1e-8) for angle,(lo,hi) in zip(q,JOINT_BOUNDS)):
        raise UnreachableTarget("Joint limit exceeded")
    return q
