# PhantomArm v1.8: Windows grasping controls and validation

## Changed controls in grasp mode ONLY

- **Hold `6`**: gradually tighten both fingers (takes ~1.8 seconds from fully open to full close).
- **Release `6`**: keep the last requested grip position (no automatic release).
- **Hold `Enter`**: gradually open both fingers (~2.8 seconds from fully closed to fully open).
- **Release `Enter`**: hold the current opening. You can simultaneously move the robot using arrows while holding a cube, and ease the grip open when ready to place it.
- **Page Up**, **Backspace**, or **F7**: reset only the cube; robot and fingers remain where they are. If the robot occupies the cube's spawn position, reset is refused until you move away.
- **Arrow Up/Down**: raise/lower (Z).
- **Arrow Left/Right**: robot-left/right (Y).
- **7/8**: forward/backward (X).
- **9**: emergency pause; **Space** releases it.
- **Home / End**: park / resume robot movement.
- **F11**: calibrate orientation; **F12**: quit.

### Important
MuJoCo's passive viewer callback does not expose key-up events. For the grasp scene, a Windows-specific keyboard polling module detects **physically held keys** while MuJoCo has focus. Click the **MuJoCo window** first. On non-Windows systems, the hold controls are disabled until a suitable key-up handler is implemented.

### A safe pick-and-place test

1. Open `run_grasp_demo.bat`; move arm clear of the start stand.
2. Press Page Up to reset **only the cube**; confirm `[CUBE RESET OK]`.
3. Position open fingers around the middle of the cube using arrow and 7/8 keys, staying above the pedestal surface. `6` must be held to close, not tapped.
4. Hold `6` until pads contact the cube (~1–2 sec). Release `6` to keep gripping.
5. Lift *slowly* using Up arrow and wait between presses.
6. Move left toward the green drop stand.
7. Lower, then hold Enter to **gradually** ease open. Release Enter at any intermediate opening to keep its position.
8. If cube falls behind stands, move the arm away first, then Page Up. The arm will not teleport.

### Collision limitation
The scene now enables collisions between robot links/fingers and pedestals as well as the cube, instead of filtering them out. A conservative tool-tip workspace guard clips requested Z to 0.350m over either pedestal. This makes through-table motion less likely, but high actuator forces can still cause small contact penetration and undesired movement. Physics-based gripping is an experimental feature; do not call it verified until the Windows grasp diagnostic and an interactive pick-and-place trial succeed.

### Commands

```powershell
.\scripts\test_all.bat
.\scripts\run_grasp_demo.bat
.\scripts\run_grasp_diagnostic.bat
```

The existing `run_demo.bat` and `run_sweep_demo.bat` standard modes preserve their earlier gripper toggle (`6`) and Enter calibration behavior. New press-and-hold controls apply only to grasp mode.
