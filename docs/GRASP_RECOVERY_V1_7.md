# PhantomArm v1.7 — safe reset and grip tracking

## Why v1.6 misbehaved

- The original reset sent the wrist / open fingers **into the space occupied by the start platform** at Z=0.35 m. The resulting contacts could deflect the physical arm while the orange **requested target marker** continued to move. The marker is not the robot.
- The model let the tall table collide with robot geometry that should not have been part of this simplified gripping experiment. The cube should rest on the pedestal, but the pedestal should not block the grasping pads or arm's wrist.
- The finger-close command was abrupt. Close now ramps gradually, and pad geometry is moved slightly above the tabletop.

## Install on Windows

Close any running MuJoCo windows. From a PowerShell terminal at the **inner** `PhantomArm` folder:

```powershell
Expand-Archive -Path "$HOME\Downloads\PhantomArm_v1_7_Grasp_Recovery_Patch.zip" -DestinationPath (Get-Location).Path -Force
.\scripts\test_all.bat
.\scripts\run_grasp_demo.bat
```

`venv` and Python packages are unaffected.

## New start / reset sequence

1. **Page Up** (recommended; `F7` alternative) returns the cube to the left/start pedestal and the OPEN gripper to a safe hover **Z=0.43 m** (instead of Z=0.35 m, which was too close to the table).
2. Press **DOWN** about **five times** to reach the cube level (Z~0.355 m). Give the arm time to settle. Do **not** close the fingers while still hovering above the cube.
3. Press **6** to close the fingers slowly; wait about two seconds. Monitor `[CUBE] finger contacts` in the terminal.
4. Press **UP** once or twice, wait, and verify whether the actual orange cube rises. Slowly move **LEFT** toward the green destination.
5. **PAGE UP** resets again from anywhere. It intentionally restores the **whole** scene, not just the cube.

If the orange square moves but the arm does not, check the `[TRACKING WARNING]` and `[t=...] joint-tip-error` lines in the terminal. The orange square marks the **commanded** tool-tip position; the green tool-tip site is the **actual** position. There can be a lag because of motor inertia and friction, and if the difference persists, the arm is blocked or the target cannot be tracked.

## Headless physics diagnostics

Run `scripts\run_grasp_diagnostic.bat` to automatically reproduce approach, close, lift, and hold using MuJoCo contacts. The command reports cube position, fingertip contacts, and lift height. **If it fails, capture the phase log**—we will adjust the physical model using measurements rather than increasing forces blindly.

## Engineering honesty / limitations

This simplified scene intentionally disables robot-to-pedestal contact so that we can isolate cube-to-gripper friction. The cube still has gravity and genuine contact with both the pedestal and fingers. This does **not** validate whole-robot collision avoidance or real hardware grip force. The simulator was not available to execute in the authoring environment; these corrections need to be verified in the Windows MuJoCo 3.14 installation before being marked successful.

Official MuJoCo docs: https://mujoco.readthedocs.io/en/latest/programming/simulation.html and https://mujoco.readthedocs.io/en/latest/computation/
