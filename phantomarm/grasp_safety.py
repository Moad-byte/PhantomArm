"""Conservative Cartesian keep-out above PhantomArm's two pedestal tops."""
from .kinematics import Pose


def clamp_grasp_target(pose: Pose) -> Pose:
    """Don't command the gripper's tool center inside a pedestal.

    The table top is Z=0.327m. With a ~24mm pad half-height and pad +5mm
    offset, the tool target should stay >= 0.350m near either table. Actual
    geometry is still checked by MuJoCo contacts; this is not collision-free
    motion planning and does not cover all robot links.
    """
    near_x = 0.36 <= pose.x <= 0.50
    near_pickup = -0.07 <= pose.y <= 0.07
    near_drop = 0.05 <= pose.y <= 0.19
    if near_x and (near_pickup or near_drop) and pose.z < 0.35:
        return Pose(pose.x, pose.y, 0.35)
    return pose
