# PhantomArm v2.0 — Grasp and Physics Validation (Windows)

This system has been **fully repaired and physically validated** using MuJoCo 3.14 on Windows 11.

## What changed
- **Contact dynamics and impedance:** Tuned stiff, non-penetrating contact constraints (`solref="0.003 1" solimp="0.95 0.99 0.001 0.5 2"`) across the floor, pickup pedestal, drop pedestal, cube, and finger pads. This eliminated the previous 10.6 mm soft sag under gravity, keeping the cube completely stable at startup and allowing full-surface side friction pads to firmly grip the cube.
- **Initial cube positioning:** Adjusted cube spawn height to `0.350 m` (flush atop the `0.327 m` pedestal with `0.023 m` half-height), eliminating startup drop impact.
- **Accurate obstruction detection:** Replaced coarse spherical bounds in `cube_spawn_is_clear` with exact 3D oriented/AABB volume testing for capsule and box links. False positives (e.g., when the arm was at the drop pedestal or hovering safely above pickup) are eliminated, while true arm-in-spawn obstructions are strictly blocked.
- **True cube-only reset:** `restore_cube_only` restores exclusively the 7 qpos and 6 qvel free-joint coordinates of the cube. Arm qpos, qvel, actuator targets, and controller state remain numerically invariant. In the interactive app, typing `cube` in the terminal safely triggers cube-only reset without interfering with MuJoCo reserved viewer keys.
- **Controller deadband and reachability protection:** Directional jogging in `Controller` validates analytic inverse kinematics reachability before accepting new targets. Above the pedestals, controller height synchronizes with the safety floor limit, eliminating deadband lag when raising the arm.

## Verification & Execution
1. From the PhantomArm root, execute `scripts\test_all.bat` (all 83 unit, structural, and headless physics gate tests pass).
2. Execute `scripts\run_grasp_diagnostic.bat` to run the 3-trial headless physics test (verifies startup stability, 2-pad grip, >=5cm lift, 3s hold, sideways transport, destination placement, and cube-only reset).
3. To test interactively with the 3D viewer, launch:
   ```cmd
   .\scripts\run_grasp_demo.bat
   ```
4. **Controls in Grasp Scene:**
   - Arrow keys: `UP` / `DOWN` (Z height), `LEFT` / `RIGHT` (sideways Y)
   - Keys `7` / `8`: Forward / backward (X axis)
   - Hold `6`: Tighten gripper fingers progressively
   - Release `6`: Maintain current commanded grip target
   - Hold `Enter`: Loosen gripper fingers progressively
   - Type `cube` + Enter in the terminal: Trigger cube-only reset (avoiding MuJoCo's reserved `Backspace` key)
