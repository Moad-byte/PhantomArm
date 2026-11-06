import xml.etree.ElementTree as ET
from pathlib import Path
import pytest
XML = Path(__file__).resolve().parents[1]/'assets'/'phantom_arm_grasp.xml'


def test_collision_filters_bidirectional():
    root=ET.parse(XML).getroot()
    table=root.find(".//geom[@name='pickup_pedestal']")
    pad=root.find(".//geom[@name='left_pad_collision']")
    cube=root.find(".//geom[@name='cube_collision']")
    assert table is not None and pad is not None and cube is not None
    def collide(a,b):
        return bool((int(a.get('contype')) & int(b.get('conaffinity'))) or
                    (int(b.get('contype')) & int(a.get('conaffinity'))))
    assert collide(table,pad)
    assert collide(table,cube)
    assert collide(pad,cube)


def test_cube_reset_preserves_robot_states_with_engine():
    mujoco=pytest.importorskip('mujoco')
    from phantomarm.grasp import restore_cube_only
    m=mujoco.MjModel.from_xml_path(str(XML)); d=mujoco.MjData(m)
    d.qpos[:6]=(.12,-.1,-.3,.1,.008,.009)
    d.qvel[:6]=(.01,.02,.03,.04,.05,.06)
    d.ctrl[:6]=(.12,-.1,-.3,.1,.008,.009)
    cube_jid=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,'cube_free')
    q=int(m.jnt_qposadr[cube_jid]); v=int(m.jnt_dofadr[cube_jid])
    d.qpos[q:q+3]=(.95,.5,.04)
    d.qvel[v:v+6]=1
    before_q=d.qpos[:6].copy(); before_v=d.qvel[:6].copy(); before_ctrl=d.ctrl[:6].copy()
    restore_cube_only(mujoco,m,d)
    assert list(d.qpos[:6])==list(before_q)
    assert list(d.qvel[:6])==list(before_v)
    assert list(d.ctrl[:6])==list(before_ctrl)
    assert list(d.qpos[q:q+7])==list(m.qpos0[q:q+7])
    assert list(d.qvel[v:v+6])==[0.0]*6


def test_cube_reset_away_from_home_arm_does_not_move_or_fall():
    """Verify that resetting the cube while arm is away from home preserves
    arm state, does not drive the arm into the table, and allows immediate manual control.
    """
    mujoco = pytest.importorskip('mujoco')
    import numpy as np
    from phantomarm.controller import Controller
    from phantomarm.kinematics import Pose, inverse_kinematics
    from phantomarm.motion import slew
    from phantomarm.grasp import restore_cube_only, cube_spawn_is_clear, restore_grasp_world

    m = mujoco.MjModel.from_xml_path(str(XML))
    d = mujoco.MjData(m)
    tip_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, 'tool_tip')
    cube_jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'cube_free')
    q_cube = int(m.jnt_qposadr[cube_jid])
    v_cube = int(m.jnt_dofadr[cube_jid])

    home_pose = Pose(0.38, 0, 0.43)
    initial = inverse_kinematics(home_pose)
    restore_grasp_world(mujoco, m, d, initial)

    ctrl = Controller()
    ctrl.reset_manual_target(height=home_pose.z)
    cmd = tuple(initial)

    # 3 repetitions
    for trial in range(1, 4):
        # Reset controller target to home at the start of each trial
        ctrl.reset_manual_target(height=home_pose.z)
        # Move arm away from home using manual jog
        ctrl.nudge_xy(dx=0.03 * (1 if trial % 2 == 1 else -1), dy=0.04 * trial)
        ctrl.set_height(0.02)
        target = ctrl.last_target

        for _ in range(int(1.0 / m.opt.timestep)):
            goal = inverse_kinematics(ctrl.last_target)
            cmd = slew(cmd, goal, 0.009)
            d.ctrl[:4] = cmd
            d.ctrl[4:6] = (0.025, 0.025)
            mujoco.mj_step(m, d)

        # Move cube away
        d.qpos[q_cube:q_cube+3] = [0.43, 0.12, 0.350]
        mujoco.mj_forward(m, d)

        tip_before = np.array(d.site_xpos[tip_id])
        q_before = d.qpos[:6].copy()
        v_before = d.qvel[:6].copy()
        c_before = d.ctrl[:6].copy()

        # Perform cube-only reset
        assert cube_spawn_is_clear(mujoco, m, d) is True
        restore_cube_only(mujoco, m, d)

        # Assert arm state unchanged
        assert np.allclose(d.qpos[:6], q_before, atol=1e-12)
        assert np.allclose(d.qvel[:6], v_before, atol=1e-12)
        assert np.allclose(d.ctrl[:6], c_before, atol=1e-12)
        assert ctrl.last_target == target

        # Step without inputs for 0.3s; verify arm does not drop or drift
        for _ in range(int(0.3 / m.opt.timestep)):
            goal = inverse_kinematics(ctrl.last_target)
            cmd = slew(cmd, goal, 0.009)
            d.ctrl[:4] = cmd
            d.ctrl[4:6] = (0.025, 0.025)
            mujoco.mj_step(m, d)

        tip_drift = np.linalg.norm(np.array(d.site_xpos[tip_id]) - tip_before)
        assert tip_drift < 0.002, f"Arm moved after reset: {tip_drift*1000} mm"
        assert d.site_xpos[tip_id][2] > 0.38, "Arm dropped towards table"

        # Test manual movement immediately afterward
        ctrl.nudge_xy(dx=0.03, dy=-0.03)
        for _ in range(int(1.0 / m.opt.timestep)):
            goal = inverse_kinematics(ctrl.last_target)
            cmd = slew(cmd, goal, 0.009)
            d.ctrl[:4] = cmd
            d.ctrl[4:6] = (0.025, 0.025)
            mujoco.mj_step(m, d)

        displacement = np.linalg.norm(np.array(d.site_xpos[tip_id]) - tip_before)
        assert displacement > 0.020, "Arm did not follow manual command after reset"
