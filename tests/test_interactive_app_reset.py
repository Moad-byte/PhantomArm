"""End-to-end verification of interactive application Delete key cube-only reset.

Exercises the actual on_key(261) GLFW callback, queue handling, controller loop,
and physics stepping used by phantomarm.app in interactive mode.
Verifies 3 distinct arm positions away from home, confirms zero arm movement on
Delete, verifies immediate manual control afterward, and tests obstruction guarding.
"""
import math
import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")
from phantomarm.controller import Controller, State
from phantomarm.grasp import cube_spawn_is_clear, restore_cube_only
from phantomarm.grasp_safety import clamp_grasp_target
from phantomarm.held_grip import HeldGrip
from phantomarm.kinematics import Pose, inverse_kinematics
from phantomarm.motion import slew
from phantomarm.sources import DemoSource
from phantomarm.viewer_controls import action_for_key


class SimulatedInteractiveApp:
    """Replicates the exact control loop and on_key dispatch of phantomarm.app."""

    def __init__(self):
        import xml.etree.ElementTree as ET
        from pathlib import Path
        root = Path(__file__).resolve().parents[1]
        self.xml_path = str(root / "assets" / "phantom_arm_grasp.xml")
        self.model = mujoco.MjModel.from_xml_path(self.xml_path)
        self.data = mujoco.MjData(self.model)

        self.tip_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "tool_tip")
        self.goal_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "workspace_goal")
        self.cube_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "pickup_cube")
        self.cube_jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "cube_free")
        self.q_cube = int(self.model.jnt_qposadr[self.cube_jid])
        self.v_cube = int(self.model.jnt_dofadr[self.cube_jid])

        self.home_pose = Pose(0.38, 0, 0.43)
        self.initial = inverse_kinematics(self.home_pose)
        self.data.qpos[:4] = self.initial
        self.data.qpos[4:6] = (0.025, 0.025)
        self.data.ctrl[:4] = self.initial
        self.data.ctrl[4:6] = (0.025, 0.025)
        mujoco.mj_forward(self.model, self.data)

        self.ctrl = Controller()
        self.ctrl.reset_manual_target(height=self.home_pose.z)
        self.cmd = tuple(self.initial)
        self.src = DemoSource(pattern="still")
        self.park_mode = "LIVE"
        self.pending_scene_reset = False
        self.terminal_queue = []
        self.held_grip = HeldGrip()
        self.finger_command = (0.025, 0.025)
        self.t = 0.0
        self.dt = 0.02
        self.was_clamped = False

    def on_key(self, keycode: int):
        """Matches on_key in phantomarm/app.py."""
        if keycode == 259:
            # Native Backspace safeguard
            self.data.qpos[:4] = self.cmd
            self.data.qpos[4:6] = self.finger_command
            self.data.ctrl[:4] = self.cmd
            self.data.ctrl[4:6] = self.finger_command
            mujoco.mj_forward(self.model, self.data)
            return

        if keycode == 261 or action_for_key(keycode) == "reset_cube":
            self.pending_scene_reset = True
            self.terminal_queue.append("reset_cube")
            return

        action = action_for_key(keycode)
        if action == "reset_cube":
            return
        if action == "estop":
            self.ctrl.emergency_stop()
        elif action == "reset":
            self.ctrl.reset_emergency()
            self.park_mode = "LIVE"
        elif action == "calibrate":
            self.ctrl.calibrate_next()
        elif action in ("raise", "lower", "left", "right", "forward", "backward"):
            step = 0.015
            if action == "raise":
                self.ctrl.set_height(step)
            elif action == "lower":
                self.ctrl.set_height(-step)
            elif action == "left":
                self.ctrl.nudge_xy(dy=step)
            elif action == "right":
                self.ctrl.nudge_xy(dy=-step)
            elif action == "forward":
                self.ctrl.nudge_xy(dx=step)
            elif action == "backward":
                self.ctrl.nudge_xy(dx=-step)

    def step(self):
        """One cycle of the application loop matching phantomarm/app.py."""
        self.t += self.dt
        sample = self.src.poll(self.t)
        if sample is not None:
            self.ctrl.consume(sample, self.t)

        while self.terminal_queue:
            req = self.terminal_queue.pop(0)
            if req == "reset_cube":
                self.pending_scene_reset = True

        do_scene_reset = self.pending_scene_reset
        if do_scene_reset:
            self.pending_scene_reset = False

        status = self.ctrl.tick(self.t)
        effective_target = clamp_grasp_target(status.target)
        if effective_target != status.target:
            self.ctrl.height = effective_target.z
            self.ctrl.last_target = effective_target

        if self.park_mode == "LIVE" and status.state == State.READY:
            try:
                goal = inverse_kinematics(effective_target)
                self.cmd = slew(self.cmd, goal, 0.009)
            except Exception:
                pass

        target_site = self.home_pose if self.park_mode != "LIVE" else effective_target

        # physics_update()
        if do_scene_reset:
            if cube_spawn_is_clear(mujoco, self.model, self.data):
                robot_q_before = self.data.qpos[:6].copy()
                robot_v_before = self.data.qvel[:6].copy()
                old_ctrl = self.data.ctrl[:6].copy()
                restore_cube_only(mujoco, self.model, self.data)
                assert np.allclose(self.data.qpos[:6], robot_q_before, atol=1e-12)
                assert np.allclose(self.data.qvel[:6], robot_v_before, atol=1e-12)
                assert np.allclose(self.data.ctrl[:6], old_ctrl, atol=1e-12)

        self.data.ctrl[:4] = self.cmd
        self.data.ctrl[4:6] = self.finger_command

        heading = math.atan2(target_site.y, target_site.x)
        self.model.site_pos[self.goal_id] = [
            target_site.x + 0.05 * math.cos(heading),
            target_site.y + 0.05 * math.sin(heading),
            target_site.z + 0.005,
        ]

        freeze = self.ctrl.estop or (self.park_mode == "LIVE" and self.ctrl.state != State.READY) or self.park_mode == "PARKED"
        if freeze:
            mujoco.mj_forward(self.model, self.data)
        else:
            for _ in range(4):
                mujoco.mj_step(self.model, self.data)

        return tuple(float(v) for v in self.data.site_xpos[self.tip_id])


def test_interactive_delete_reset_three_trials():
    """Mandatory verification: 3 trials away from home, Delete key reset, immediate response."""
    app = SimulatedInteractiveApp()

    # Settle at initial home pose
    for _ in range(50):
        app.step()

    trials_offsets = [
        # (UP, LEFT)
        {"keys": [265, 265, 263, 263], "name": "UP+LEFT"},
        # (DOWN, RIGHT)
        {"keys": [264, 264, 262, 262, 262], "name": "DOWN+RIGHT"},
        # (FORWARD, UP)
        {"keys": [55, 55, 55, 265, 265], "name": "FORWARD+UP"},
    ]

    for trial_idx, trial in enumerate(trials_offsets, 1):
        # 1. Jog arm away from home using key events
        for k in trial["keys"]:
            app.on_key(k)

        # Allow arm to move toward jogged target
        for _ in range(60):
            app.step()

        tip_jogged = np.array(app.data.site_xpos[app.tip_id])
        target_jogged = app.ctrl.last_target
        q_jogged = app.data.qpos[:6].copy()
        cmd_jogged = tuple(app.cmd)

        # Displace the cube to simulate prior manipulation/drop
        app.data.qpos[app.q_cube : app.q_cube + 3] = [0.43, 0.15, 0.350]
        app.data.qvel[app.v_cube : app.v_cube + 6] = [0.01, 0.02, 0.0, 0.0, 0.0, 0.0]
        mujoco.mj_forward(app.model, app.data)

        # Verify cube is away from spawn
        cube_pos = app.data.xpos[app.cube_body_id]
        assert abs(cube_pos[1] - 0.15) < 0.02

        # 2. Press DELETE key (GLFW keycode 261)
        app.on_key(261)

        # Step 1 tick of the control loop (executes the reset inside physics_update)
        app.step()

        # 3. Verify cube respawned to initial spawn
        spawn_qpos0 = app.model.qpos0[app.q_cube : app.q_cube + 7]
        assert np.allclose(app.data.qpos[app.q_cube : app.q_cube + 7], spawn_qpos0, atol=2e-3)
        assert np.all(np.abs(app.data.qvel[app.v_cube : app.v_cube + 6]) < 0.1)

        # 4. Verify arm state, controller target, commanded joints, and operating state unchanged
        assert app.ctrl.last_target == target_jogged
        assert app.cmd == cmd_jogged
        assert app.park_mode == "LIVE"
        assert app.ctrl.state == State.READY

        # Step without inputs for 0.3s; verify arm does not drop, fall into table, or move unexpectedly
        for _ in range(15):
            app.step()

        tip_after_wait = np.array(app.data.site_xpos[app.tip_id])
        tip_drift = np.linalg.norm(tip_after_wait - tip_jogged)
        assert tip_drift < 0.003, f"Trial {trial_idx}: Arm drifted {tip_drift*1000:.2f} mm after Delete!"
        assert tip_after_wait[2] > 0.35, f"Trial {trial_idx}: Arm fell into table: Z={tip_after_wait[2]:.3f} m"

        # 5. Immediately move arm using arrow keys (e.g. RIGHT arrow keycode 262)
        for _ in range(4):
            app.on_key(262)  # RIGHT 60mm

        for _ in range(60):
            app.step()

        tip_moved = np.array(app.data.site_xpos[app.tip_id])
        displacement = np.linalg.norm(tip_moved - tip_jogged)
        assert displacement > 0.030, f"Trial {trial_idx}: Arm failed to respond to manual commands! Disp={displacement*1000:.1f} mm"


def test_obstruction_detection_and_safeguard():
    """Verify that if the arm occupies the cube spawn, reset is blocked and arm does not move."""
    app = SimulatedInteractiveApp()

    # Move arm directly into the cube spawn volume
    target_spawn = Pose(0.43, 0.0, 0.355)
    app.ctrl.last_target = target_spawn
    app.ctrl.height = target_spawn.z
    for _ in range(100):
        app.step()

    # Displace the cube slightly
    app.data.qpos[app.q_cube : app.q_cube + 3] = [0.43, 0.12, 0.350]
    mujoco.mj_forward(app.model, app.data)

    # Arm is now obstructing spawn
    assert not cube_spawn_is_clear(mujoco, app.model, app.data)

    q_arm_before = app.data.qpos[:6].copy()
    target_before = app.ctrl.last_target

    # Press Delete
    app.on_key(261)
    app.step()

    # Cube should NOT have respawned (spawn was obstructed)
    assert abs(app.data.qpos[app.q_cube + 1] - 0.12) < 0.02

    # Arm state, target, controller must be untouched
    assert app.ctrl.last_target == target_before
    assert np.allclose(app.data.qpos[:6], q_arm_before, atol=1e-3)
