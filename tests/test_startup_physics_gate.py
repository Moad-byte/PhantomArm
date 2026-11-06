"""Real MuJoCo automated headless physics validation tests.

Verifies:
1. Cube remains stable on its platform at startup for >= 3 seconds.
2. Cube-only reset leaves arm state unchanged and blocks when obstructed.
3. Gripper closes around the cube with actual physical contact (2 contacts).
4. Cube is lifted >= 3 cm (lifts ~5.8 cm) and held for >= 3 seconds.
5. Robot transports and releases the cube successfully onto drop pedestal.
6. The complete pick-transport-release-reset sequence succeeds 3 times in a row.
"""
from pathlib import Path
import numpy as np
import pytest

XML = Path(__file__).resolve().parents[1] / "assets" / "phantom_arm_grasp.xml"


def test_cube_stays_stable_before_touching_arm():
    mujoco = pytest.importorskip("mujoco")
    from phantomarm.kinematics import Pose, inverse_kinematics
    from phantomarm.grasp import restore_grasp_world

    m = mujoco.MjModel.from_xml_path(str(XML))
    d = mujoco.MjData(m)
    q = inverse_kinematics(Pose(0.38, 0, 0.43))
    restore_grasp_world(mujoco, m, d, q)
    body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "pickup_cube")
    start = float(d.xpos[body][2])

    # Run for 3.0 seconds under gravity with no arm motion
    steps = int(3.0 / m.opt.timestep)
    for _ in range(steps):
        d.ctrl[:4] = q
        d.ctrl[4:6] = (0.025, 0.025)
        mujoco.mj_step(m, d)

    final_z = float(d.xpos[body][2])
    assert abs(final_z - start) <= 0.008, f"Cube sank or dropped: start={start}, final={final_z}"
    assert final_z >= 0.342, f"Cube fell off platform: final_z={final_z}"


def test_cube_only_reset_leaves_arm_unchanged_and_blocks_obstruction():
    mujoco = pytest.importorskip("mujoco")
    from phantomarm.kinematics import Pose, inverse_kinematics
    from phantomarm.grasp import restore_grasp_world, restore_cube_only, cube_spawn_is_clear

    m = mujoco.MjModel.from_xml_path(str(XML))
    d = mujoco.MjData(m)
    q_hover = inverse_kinematics(Pose(0.38, 0, 0.43))
    restore_grasp_world(mujoco, m, d, q_hover)

    # 1. Arm at safe hover: spawn must be clear
    assert cube_spawn_is_clear(mujoco, m, d) is True

    # 2. Arm at pickup grasp pose: spawn must be detected as obstructed
    q_pickup = inverse_kinematics(Pose(0.38, 0, 0.355))
    d.qpos[:4] = q_pickup
    d.ctrl[:4] = q_pickup
    mujoco.mj_forward(m, d)
    assert cube_spawn_is_clear(mujoco, m, d) is False

    # 3. Arm at destination pedestal: spawn must be clear
    q_drop = inverse_kinematics(Pose(0.38, 0.12, 0.43))
    d.qpos[:4] = q_drop
    d.ctrl[:4] = q_drop
    mujoco.mj_forward(m, d)
    assert cube_spawn_is_clear(mujoco, m, d) is True

    # 4. Modify cube pose and velocity
    cube_jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "cube_free")
    q_cube = int(m.jnt_qposadr[cube_jid])
    v_cube = int(m.jnt_dofadr[cube_jid])
    d.qpos[q_cube : q_cube + 3] = [0.80, 0.40, 0.02]
    d.qvel[v_cube : v_cube + 6] = [0.1, -0.2, 0.3, 0.0, 0.1, -0.1]

    # Set arbitrary non-zero robot states
    d.qpos[:6] = [0.15, -0.25, 0.35, -0.10, 0.012, 0.018]
    d.qvel[:6] = [0.01, -0.02, 0.03, -0.04, 0.001, -0.001]
    d.ctrl[:6] = [0.15, -0.25, 0.35, -0.10, 0.012, 0.018]

    robot_q = d.qpos[:6].copy()
    robot_v = d.qvel[:6].copy()
    robot_ctrl = d.ctrl[:6].copy()

    restore_cube_only(mujoco, m, d)

    # Robot states must be strictly identical
    assert np.allclose(d.qpos[:6], robot_q, atol=1e-12)
    assert np.allclose(d.qvel[:6], robot_v, atol=1e-12)
    assert np.allclose(d.ctrl[:6], robot_ctrl, atol=1e-12)

    # Cube freejoint must be back at MJCF spawn with zero velocities
    assert np.allclose(d.qpos[q_cube : q_cube + 7], m.qpos0[q_cube : q_cube + 7], atol=1e-5)
    assert np.allclose(d.qvel[v_cube : v_cube + 6], 0.0, atol=1e-5)


def test_full_grasp_lift_hold_transport_release_three_trials():
    mujoco = pytest.importorskip("mujoco")
    from phantomarm.kinematics import Pose, inverse_kinematics
    from phantomarm.motion import slew
    from phantomarm.grasp import restore_grasp_world, restore_cube_only, cube_spawn_is_clear

    m = mujoco.MjModel.from_xml_path(str(XML))
    d = mujoco.MjData(m)

    cube_body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "pickup_cube")
    cube_geom = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "cube_collision")
    left_finger = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "left_finger")
    right_finger = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "right_finger")
    drop_geom = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "drop_pedestal")

    def count_finger_contacts():
        touching = set()
        for i in range(d.ncon):
            c = d.contact[i]
            if cube_geom not in (c.geom1, c.geom2) or c.efc_address < 0:
                continue
            other = c.geom2 if c.geom1 == cube_geom else c.geom1
            body = int(m.geom_bodyid[other])
            if body in (left_finger, right_finger):
                touching.add(body)
        return len(touching)

    def is_on_destination():
        for i in range(d.ncon):
            c = d.contact[i]
            if {c.geom1, c.geom2} == {cube_geom, drop_geom}:
                return True
        return False

    hover_pose = Pose(0.38, 0, 0.43)
    hover_joints = inverse_kinematics(hover_pose)
    restore_grasp_world(mujoco, m, d, hover_joints)
    start_ref_z = float(d.xpos[cube_body][2])

    for trial in range(1, 4):
        cmd = tuple(hover_joints)
        finger = 0.025

        def step_trajectory(target_pose, target_grip, duration_sec, min_contacts=None):
            nonlocal cmd, finger
            q_target = inverse_kinematics(target_pose)
            steps = int(duration_sec / m.opt.timestep)
            for _ in range(steps):
                cmd = slew(cmd, q_target, m.opt.timestep)
                df = 0.014 * m.opt.timestep
                finger += max(-df, min(df, target_grip - finger))
                d.ctrl[:4] = cmd
                d.ctrl[4:6] = finger
                mujoco.mj_step(m, d)
                if min_contacts is not None:
                    assert count_finger_contacts() >= min_contacts

        # 1. Startup stability for 3s
        step_trajectory(hover_pose, 0.025, 3.0)
        assert abs(float(d.xpos[cube_body][2]) - start_ref_z) <= 0.008

        # 2. Approach
        step_trajectory(Pose(0.38, 0, 0.355), 0.025, 3.0)

        # 3. Close with actual contact (2 contacts)
        step_trajectory(Pose(0.38, 0, 0.355), 0.0, 3.0)
        assert count_finger_contacts() == 2

        # 4. Lift >= 3 cm (gain is ~5.8 cm)
        step_trajectory(Pose(0.38, 0, 0.43), 0.0, 3.0, min_contacts=2)
        lift_height = float(d.xpos[cube_body][2])
        assert lift_height - start_ref_z >= 0.030

        # 5. Hold for 3.0 seconds maintaining contact
        step_trajectory(Pose(0.38, 0, 0.43), 0.0, 3.0, min_contacts=2)

        # 6. Transport sideways to destination platform
        step_trajectory(Pose(0.38, 0.12, 0.43), 0.0, 3.5, min_contacts=2)

        # 7. Lower to drop pedestal
        step_trajectory(Pose(0.38, 0.12, 0.360), 0.0, 3.0, min_contacts=2)

        # 8. Release
        step_trajectory(Pose(0.38, 0.12, 0.360), 0.025, 2.5)

        # 9. Retract and verify cube rests on drop pedestal
        step_trajectory(Pose(0.38, 0.12, 0.43), 0.025, 2.0)
        assert is_on_destination()

        # 10. Cube-only reset
        step_trajectory(hover_pose, 0.025, 2.0)
        assert cube_spawn_is_clear(mujoco, m, d) is True
        restore_cube_only(mujoco, m, d)
        assert abs(float(d.xpos[cube_body][2]) - start_ref_z) <= 0.008
