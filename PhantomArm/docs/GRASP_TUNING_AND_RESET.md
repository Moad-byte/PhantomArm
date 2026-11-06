# PhantomArm v1.6 — Gripper stability and reset

## Why the cube used to fall

The v1.5 gripper had relatively short pads (26 mm tall) and modest position-servo stiffness (65). A 46 mm-tall cube could slip, particularly when accelerating the arm with arrow keys. The revised grasp scene has 50 mm-high pads, constrained stronger finger position servos (`kp=320`, `forcerange=-8 8`), enhanced contact friction, and slower joint target changes while the gripper is closed.

These are *model-tuning parameters*, **not proof of a reliable real-world grip**. Successful pick-and-place must be demonstrated in MuJoCo; there is no scripted cube attachment.

## Updated controls in grasp scene

- `6`: open/close both gripper fingers
- `UP`/`DOWN`: move tool vertically
- `LEFT`/`RIGHT`: move tool sideways
- `7`/`8`: move forward/backward
- `BACKSPACE`, `PAGE UP`, or `F7`: **reset cube, arm, and gripper to the initial grasp scene**
- `HOME`: return arm home, without resetting cube
- `END`: resume after park
- `9`: software stop; `SPACE`: release stop

The reset clears the full MuJoCo physics state. The cube is restored to the starting pedestal regardless of where it fell, the arm returns to its home pose, and fingers open. It is an explicit simulation reset, not an automatic cube attachment. In the *standard* scene, Backspace still acts as a HOME shortcut.

## Recommended test sequence

1. Run `scripts\run_grasp_demo.bat`.
2. Press `BACKSPACE` once to get a known starting state.
3. Press `6` to close; allow 1–2 seconds to establish contact.
4. Tap `UP` **once or twice**, observing the cube. Wait for the arm to settle before the next tap.
5. Look for nonzero `[CUBE] ... finger contacts=` messages and verify the cube lifts without slipping for several seconds.
6. Tap `LEFT` cautiously to approach the green platform. Lower (`DOWN`), then release (`6`).
7. If the cube falls, press `BACKSPACE` and repeat without closing the program.

If cube still falls, report the observed finger contact count and how quickly it drops. We should adjust physics parameters against those observations instead of faking attachment.

## Engineering notes

- The finger improvements affect only `assets/phantom_arm_grasp.xml`. Original stationary and sweep scenes are unchanged.
- Robot joint targets move slower when carrying (`slew` step 0.006 versus 0.02).
- MuJoCo contact is still fully simulated. Object grasping is a **physics challenge**, not guaranteed by software unit tests.
- Reset implementation uses `mj_resetData` and `mj_forward` to clear stale velocities/contacts, followed by explicit robot home configuration.

Official MuJoCo references: https://mujoco.readthedocs.io/en/latest/programming/simulation.html and https://mujoco.readthedocs.io/en/latest/computation/index.html
