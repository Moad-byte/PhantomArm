"""Headless MuJoCo grasp diagnostic for users of PhantomArm.

Verifies:
1. Cube remains stable on its platform at startup for >= 3 seconds.
2. Cube-only reset leaves arm state unchanged and detects obstructions.
3. Gripper closes around the cube with actual contact (2 contacts).
4. Cube is lifted >= 5 cm (>= 3 cm required) and held for >= 3 seconds.
5. Robot transports and releases the cube successfully onto drop pedestal.
6. Repeat the complete sequence successfully three times.

Run: python -m phantomarm.grasp_diagnostic
"""
from pathlib import Path
import math
import numpy as np
from .kinematics import Pose, inverse_kinematics
from .motion import slew
from .grasp import restore_grasp_world, restore_cube_only, cube_spawn_is_clear

MODEL = Path(__file__).resolve().parents[1] / "assets" / "phantom_arm_grasp.xml"


def main():
    import mujoco
    m = mujoco.MjModel.from_xml_path(str(MODEL))
    d = mujoco.MjData(m)

    cube_body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "pickup_cube")
    cube_geom = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "cube_collision")
    left = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "left_finger")
    right = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "right_finger")
    drop_geom = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "drop_pedestal")
    tip = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "tool_tip")
    hand = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "grip_center")
    cube_jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "cube_free")
    q_cube = int(m.jnt_qposadr[cube_jid])
    v_cube = int(m.jnt_dofadr[cube_jid])

    print("=" * 65)
    print("PhantomArm: HEADLESS RIGOROUS PHYSICS GRASP & RESET DIAGNOSTIC")
    print("=" * 65)

    def contacts():
        touching = set()
        active_names = set()
        for i in range(d.ncon):
            c = d.contact[i]
            if cube_geom not in (c.geom1, c.geom2) or c.efc_address < 0:
                continue
            other = c.geom2 if c.geom1 == cube_geom else c.geom1
            body = int(m.geom_bodyid[other])
            if body in (left, right):
                touching.add(body)
                active_names.add(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, int(other)) or str(other))
        return len(touching), sorted(active_names)

    def is_on_drop_pedestal():
        for i in range(d.ncon):
            c = d.contact[i]
            if {c.geom1, c.geom2} == {cube_geom, drop_geom}:
                return True
        return False

    hover_pose = Pose(0.38, 0.0, 0.43)
    hover_joints = inverse_kinematics(hover_pose)
    restore_grasp_world(mujoco, m, d, hover_joints)
    reset_cube = tuple(float(v) for v in d.xpos[cube_body])
    print(f"Spawn cube reference: ({reset_cube[0]:.3f}, {reset_cube[1]:.3f}, {reset_cube[2]:.3f}) m")

    TRIALS = 3
    for trial in range(1, TRIALS + 1):
        print(f"\n--- TRIAL {trial} of {TRIALS} ---")
        command = tuple(hover_joints)
        finger = 0.025

        def run_phase(label, seconds, pose, grip=0.025, require_min_contacts=None):
            nonlocal command, finger
            q = inverse_kinematics(pose)
            steps = int(seconds / m.opt.timestep)
            max_contacts = 0
            for _ in range(steps):
                command = slew(command, q, m.opt.timestep)
                df = 0.014 * m.opt.timestep
                finger += max(-df, min(df, grip - finger))
                d.ctrl[:4] = command
                d.ctrl[4:6] = finger
                mujoco.mj_step(m, d)
                c_cnt, _ = contacts()
                max_contacts = max(max_contacts, c_cnt)
                if require_min_contacts is not None and c_cnt < require_min_contacts:
                    print(f"  [FAIL] Contacts dropped to {c_cnt} (required >= {require_min_contacts}) during {label}")
                    return None, c_cnt

            xyz = tuple(round(float(x), 4) for x in d.xpos[cube_body])
            tip_xyz = tuple(round(float(x), 4) for x in d.site_xpos[tip])
            hand_xyz = tuple(round(float(x), 4) for x in d.site_xpos[hand])
            pair_count, geom_names = contacts()
            finger_qpos = tuple(round(float(v), 4) for v in d.qpos[4:6])
            print(f"{label:>14s} | cube={xyz} wrist={tip_xyz} HAND={hand_xyz} | contacts={pair_count} | pads={geom_names} | slide={finger_qpos}")
            return xyz, pair_count

        # 1. Startup stability for >= 3.0 seconds
        stable_xyz, _ = run_phase("hover 3s", 3.0, hover_pose, grip=0.025)
        if abs(stable_xyz[2] - reset_cube[2]) > 0.008:
            print("RESULT: FAIL — cube was not stable on pickup platform at startup.")
            return 3

        # 2. Approach
        run_phase("approach", 3.0, Pose(0.38, 0.0, 0.355), grip=0.025)

        # 3. Close around cube
        _, close_contacts = run_phase("close", 3.0, Pose(0.38, 0.0, 0.355), grip=0.0)
        if close_contacts < 2:
            print(f"RESULT: FAIL — expected 2 finger contacts upon closing, got {close_contacts}.")
            return 2

        # 4. Lift >= 5 cm (>= 3 cm required)
        lift_xyz, lift_contacts = run_phase("lift", 3.0, Pose(0.38, 0.0, 0.43), grip=0.0, require_min_contacts=2)
        if lift_xyz is None:
            print("RESULT: FAIL — lost grip during lifting.")
            return 2
        height_gain = lift_xyz[2] - reset_cube[2]
        print(f"               -> Lifted height gain: {height_gain * 100:.1f} cm (contacts={lift_contacts})")
        if height_gain < 0.030:
            print(f"RESULT: FAIL — lift height {height_gain:.3f}m was less than 3 cm.")
            return 2

        # 5. Hold for >= 3.0 seconds
        hold_xyz, hold_contacts = run_phase("hold 3s", 3.0, Pose(0.38, 0.0, 0.43), grip=0.0, require_min_contacts=2)
        if hold_xyz is None or hold_contacts < 2:
            print("RESULT: FAIL — lost grip during 3-second hold.")
            return 2

        # 6. Transport sideways to drop pedestal (y = 0.12 m)
        trans_xyz, trans_contacts = run_phase("translate", 3.5, Pose(0.38, 0.12, 0.43), grip=0.0, require_min_contacts=2)
        if trans_xyz is None:
            print("RESULT: FAIL — lost grip during transport.")
            return 2

        # 7. Lower to drop pedestal
        run_phase("lower", 3.0, Pose(0.38, 0.12, 0.360), grip=0.0, require_min_contacts=2)

        # 8. Release
        run_phase("release", 2.5, Pose(0.38, 0.12, 0.360), grip=0.025)

        # 9. Retract
        run_phase("retract", 2.0, Pose(0.38, 0.12, 0.43), grip=0.025)
        if not is_on_drop_pedestal():
            print("RESULT: FAIL — cube did not settle on destination pedestal.")
            return 2
        print("               -> Cube successfully delivered to destination platform!")

        # 10. Obstruction detection and Cube-only reset verification
        # 10a. Retract to neutral hover above pickup pedestal at safe height z=0.43
        run_phase("hover_mid", 2.0, hover_pose, grip=0.025)
        # 10b. Move arm down into pickup grasp pose (occupying empty cube spawn)
        run_phase("check_block", 2.5, Pose(0.38, 0.0, 0.355), grip=0.025)
        if cube_spawn_is_clear(mujoco, m, d):
            print("RESULT: FAIL — cube_spawn_is_clear failed to detect arm obstructing cube spawn.")
            return 4
        print("               -> Obstruction detection verified: reset blocked when arm occupies spawn.")

        # 10c. Return to safe hover above pickup platform
        run_phase("return_hover", 2.5, hover_pose, grip=0.025)
        if not cube_spawn_is_clear(mujoco, m, d):
            print("RESULT: FAIL — cube_spawn_is_clear false negative while arm was at hover.")
            return 4

        # 10d. Perform cube-only reset
        robot_q_before = d.qpos[:6].copy()
        robot_v_before = d.qvel[:6].copy()
        robot_ctrl_before = d.ctrl[:6].copy()
        restore_cube_only(mujoco, m, d)

        # Assert arm state unchanged
        if not np.allclose(d.qpos[:6], robot_q_before, atol=1e-12):
            print("RESULT: FAIL — arm qpos changed during restore_cube_only.")
            return 4
        if not np.allclose(d.qvel[:6], robot_v_before, atol=1e-12):
            print("RESULT: FAIL — arm qvel changed during restore_cube_only.")
            return 4
        if not np.allclose(d.ctrl[:6], robot_ctrl_before, atol=1e-12):
            print("RESULT: FAIL — arm ctrl changed during restore_cube_only.")
            return 4
        if not np.allclose(d.qpos[q_cube:q_cube + 7], m.qpos0[q_cube:q_cube + 7], atol=1e-5):
            print("RESULT: FAIL — cube qpos was not restored to spawn.")
            return 4
        if not np.allclose(d.qvel[v_cube:v_cube + 6], 0.0, atol=1e-5):
            print("RESULT: FAIL — cube qvel was not zeroed.")
            return 4
        print("               -> Cube-only reset verified: arm state preserved, cube restored to spawn.")
        print(f"Trial {trial} completed successfully!")

    print("\n" + "=" * 65)
    print("RESULT: PASS — All 3 trials completed successfully!")
    print("Startup stability, real-contact grasp, 3s hold, sideways transport,")
    print("destination release, and cube-only reset verified physically.")
    print("=" * 65)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
