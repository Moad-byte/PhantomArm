from phantomarm.grasp_safety import clamp_grasp_target
from phantomarm.kinematics import Pose


def test_pickup_blocks_below_table():
    assert clamp_grasp_target(Pose(.43, 0, .30)) == Pose(.43, 0, .35)


def test_destination_blocks_below_table():
    assert clamp_grasp_target(Pose(.43, .12, .32)) == Pose(.43, .12, .35)


def test_clear_space_still_usable():
    assert clamp_grasp_target(Pose(.28, -.16, .30)) == Pose(.28, -.16, .30)


def test_above_platform_unmodified():
    assert clamp_grasp_target(Pose(.43, 0, .43)) == Pose(.43, 0, .43)
