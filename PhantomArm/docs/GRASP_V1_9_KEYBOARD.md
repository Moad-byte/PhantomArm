# PhantomArm v1.9 — Grasping and cube-only reset

## Controls in `run_grasp_demo.bat`

| Key | Action |
| --- | --- |
| Up / Down | Arm target Z + / - 15 mm per press |
| Left / Right | Arm target Y + / - 15 mm per press |
| 7 / 8 | Arm target X + / - 15 mm per press |
| **Hold 6** | **Gradually tighten** the gripper. Release to hold the requested aperture. |
| **Hold Enter** | **Gradually loosen** the gripper. Release to hold. |
| **Page Up** (or F7) | **CUBE ONLY reset**, keeping all arm joint values, velocities, commands, gripper aperture and controller targets untouched. |
| 9 / Space | Pause / release software stop |
| Home / End | Park / resume |
| F11 | Recalibrate (no visible effect in still-demo) |
| F12 | Exit |

**NEVER press Backspace to reset the cube.** In MuJoCo's built-in viewer, Backspace can reset **the whole physics world**, overriding our Python cube-only reset intention. We have removed Backspace from our custom reset mapping; the native viewer may still handle it.

### Why the gripping fixture looks different

The narrow pickup post supports the 46 mm cube while leaving an overhang under the left/right edges. Small silver support lips attached to the moving jaws fit under these edges when the fingers close. This is *physical collision-based support*; the cube is a free joint with gravity and **no weld/attachment**. You should approach with the wrist target around x=0.38, y=0, z=0.355 m (not x=0.43: the gripper center is 50mm ahead of the wrist site).

### Sequence

1. Launch `.\scripts\run_grasp_demo.bat`.
2. At start, the robot is above the narrow pickup post. Lower the arm one step at a time until the pads align around the cube.
3. Hold `6` until gripping fingers close **and small silver lips slide under the cube overhang**, then release `6`.
4. Press Up once and wait for the robot to settle before continuing lifting.
5. Move sideways slowly; hold `Enter` to release gradually above the green table.
6. To retry, move the arm clear of the pickup area, then press **Page Up**. Only the cube will return. A reset may be blocked if any robot geometry overlaps its spawn zone; in that case move the arm aside. There is no arm repositioning during cube reset.

### Debugging: don't trust the orange marker alone

The orange goal visualizes a commanded position. The actual robot's green wrist site, cyan grip-center site, finger positions and simulation contacts are the ground truth. When the cube slips, run `.\scripts\run_grasp_diagnostic.bat` and send the full output; it now prints **HAND** and **pads** contacts, plus actual finger joint coordinates.

### Verification status

Python/unit and XML structural tests can be run without MuJoCo. The *actual contact-physics grasp must pass on a machine with MuJoCo installed.* An executed `RESULT: PASS` from the physics diagnostic plus a visible manual lift is required before calling this task complete. Neither physical reliability nor manipulation success should be claimed based solely on structural tests.
