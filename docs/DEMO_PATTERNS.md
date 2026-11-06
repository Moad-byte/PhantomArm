# PhantomArm v1.4 — Manual vs Sweep Simulation

The older demo used a continuously changing synthetic orientation:

`roll = 25*sin(0.85*t)` and `pitch = 20*sin(0.54*t)`. This made horizontal movement continue while independently testing the gripper or height. The base rotating was expected in that mode, not a stuck key.

## Recommended first: stationary manual test

From the repository folder in VS Code PowerShell:

```powershell
.\scripts\run_demo.bat
```

The synthetic IMU is **stationary**, with roll=0 and pitch=0. A neutral-reference calibration keeps the target at (X=0.38m, Y=0m). Press the arrow keys to move vertically and sideways, 7/8 to move forward/backward, or 6 to operate the gripper without automatic sweeping. Do not press keys `0–5`, which MuJoCo reserves.

## Automatic demo / sweeping motion

```powershell
.\scripts\run_sweep_demo.bat
```

The old continuously varying tilt pattern is preserved for demonstrating horizontal joint movement without physical hardware.


## Keys — Windows laptop-friendly (v1.4)

See [Arrow Control Update](ARROW_CONTROL_UPDATE.md) for the current mappings, calibration feedback, and simulation mode details.
