"""Reproducible grasp-scene reset without teleporting the cube during a grasp.

Used only on an explicit user reset. Normal pick-and-place stays contact-based.
"""


def restore_grasp_world(mujoco, model, data, initial_joints):
    """Reset all physics state, then park the arm at its known startup pose.

    MuJoCo restores the cube free-joint's initial pose from MJCF (qpos0).
    Finger slide joints open; all velocities and previous contacts are cleared.
    Must be called from the simulation update thread, with viewer.lock held.
    """
    mujoco.mj_resetData(model, data)
    data.qpos[:4] = initial_joints
    data.qpos[4:6] = (0.025, 0.025)
    data.ctrl[:4] = initial_joints
    data.ctrl[4:6] = (0.025, 0.025)
    mujoco.mj_forward(model, data)


def cube_spawn_is_clear(mujoco, model, data, clearance=0.006):
    """Check collision-enabled robot geometry against cube spawn volume.

    Uses oriented/AABB volume overlap check in world frame to prevent false
    positives while strictly blocking resets if any robot link occupies spawn.
    This test MUST NOT modify any robot, controller or target state.
    """
    import numpy as np
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'cube_free')
    if jid < 0:
        return True
    q = int(model.jnt_qposadr[jid])
    spawn = np.array(model.qpos0[q:q+3], dtype=float)
    cube_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cube_collision')
    if cube_geom < 0:
        return True
    cube_size = np.array(model.geom_size[cube_geom][:3], dtype=float) + clearance
    spawn_min = spawn - cube_size
    spawn_max = spawn + cube_size

    for gid in range(model.ngeom):
        if not (int(model.geom_contype[gid]) & 2):
            continue
        gtype = int(model.geom_type[gid])
        dims = np.array(model.geom_size[gid][:3], dtype=float)
        c = np.array(data.geom_xpos[gid], dtype=float)
        mat = np.array(data.geom_xmat[gid], dtype=float).reshape(3, 3)
        if gtype == int(mujoco.mjtGeom.mjGEOM_CAPSULE):
            h = dims[1]
            p1 = c - h * mat[:, 2]
            p2 = c + h * mat[:, 2]
            r = dims[0]
            g_min = np.minimum(p1, p2) - r
            g_max = np.maximum(p1, p2) + r
        elif gtype == int(mujoco.mjtGeom.mjGEOM_BOX):
            corners = [
                c + mat @ (dims * np.array([sx, sy, sz]))
                for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)
            ]
            corners = np.array(corners)
            g_min = np.min(corners, axis=0)
            g_max = np.max(corners, axis=0)
        else:
            r = float(np.max(dims))
            g_min = c - r
            g_max = c + r

        if np.all(g_min <= spawn_max) and np.all(g_max >= spawn_min):
            return False
    return True


def restore_cube_only(mujoco, model, data):
    """Restore only the cube free-joint 7 qpos and 6 qvel coordinates.

    Does not reset arm qpos, velocities, controls, or gripper targets. Call
    under viewer.lock() and only after cube_spawn_is_clear returns True.
    """
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'cube_free')
    if jid < 0:
        raise ValueError('Cube freejoint not found')
    q = int(model.jnt_qposadr[jid])
    v = int(model.jnt_dofadr[jid])
    data.qpos[q:q+7] = model.qpos0[q:q+7]
    data.qvel[v:v+6] = 0.0
    mujoco.mj_forward(model, data)
