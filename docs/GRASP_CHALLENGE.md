# PhantomArm v1.6 — Real contact pick-and-place challenge (experimental)

## What's new?

The original 3D arm demo is preserved. `scripts/run_grasp_demo.bat` loads a separate MJCF scene with a **dynamic cube**, an initial support stand and a green destination stand.
The cube has a **free joint**, so gravity, contact, inertia and friction—not scripted parenting—determine whether it moves with the gripper.
The orange square site is still a *nonphysical goal marker*. The orange block is the graspable object.

## Quick start (Windows)

1. Close all MuJoCo viewers.
2. Apply the patch into the nested `PhantomArm` project folder containing `requirements.txt`. Preserve the `.venv` folder.
3. Run `scripts/test_all.bat`, then `scripts/run_grasp_demo.bat`.
4. Start with the robot at its home pose, gripper OPEN. The orange cube is between the fingers on the grey stand.
5. Press `6` to close; watch for the hands to make contact. Wait 1–2 seconds.
6. Press **UP** several times to lift. If the cube slips, note that outcome—don't claim a grasp.
7. Press **LEFT** about eight times to travel toward the green stand at Y=+0.12 m.
8. Press **DOWN** to lower gently, then `6` to open. The cube should settle on the destination if the physical grasp holds.
9. Use **HOME** to return the robot home and **END** to return to live control.

## What to verify

- A block is visibly present and rests on the source stand, unlike the goal marker.
- `6` closes the fingers and they physically contact the cube (watch `[CUBE]` diagnostics).
- The cube rises above Z=0.395 m *because of contact forces* and stays up while moving.
- The cube moves toward Y=+0.12 m and lands on the destination platform after release.
- It does **not** ghost through the fingers, leap to the gripper, or magically follow without contact.

## Troubleshooting

- If `test_all.bat` fails with a MuJoCo compile error, send the complete error before proceeding. The XML was syntax-checked, but MuJoCo execution could not be run in the file-generation environment.
- The original model restricts motion to X=0.27–0.49, Y=±0.17 and Z=0.30–0.52 m. Fully straightening is **not necessary** for this initial task.
- If a cube slips or is ejected during finger closure, we will tune finger speed, friction, stiffness and contact geometry from **observed** behavior, rather than simulating a magnetic snap.
- `9` is a **simulation software stop**, not a hardware safety-certified emergency stop.

## Implementation details

- `assets/phantom_arm_grasp.xml`: Adds cube free joint, two stands and collision-enabled finger surfaces.
- `phantomarm/app.py`: Adds `--scene grasp` flag and contact/height diagnostics, preserving other modes.
- `tests/test_grasp_scene.py`: XML invariants + MuJoCo compilation check.
- `scripts/run_grasp_demo.bat`: Windows launcher.

## Source

MuJoCo contact computation documentation: https://mujoco.readthedocs.io/en/latest/computation/

## v1.6 retry/reset

Press **BACKSPACE** or **PAGE UP** when the cube falls to reset the whole grasp scene (arm, fingers, cube) without restarting. See `docs/GRASP_TUNING_AND_RESET.md`.
