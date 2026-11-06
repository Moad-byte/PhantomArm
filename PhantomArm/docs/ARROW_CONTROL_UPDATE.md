# PhantomArm v1.4 — 3D manual jog controls (Windows)

The **stationary** demo starts with neutral IMU input, so keyboard jogging
moves the end-effector target in 3D without automatic horizontal motion.
The **sweep** demo adds manual jog offsets to a continuously generated IMU sweep.
The actual robot joints rotate as needed to follow the commanded Cartesian target.

| Key | Command | Robot-frame direction |
| --- | --- | --- |
| Up / Down arrow | Vertical movement | Z + / Z - |
| Left / Right arrow | Sideways movement | Y + / Y - |
| 7 / 8 | Forward / backward | X + / X - |
| 6 | Toggle gripper | Fingers open / close |
| 9 | Simulation stop | Freezes physics at current pose |
| Space or F8 | Reset stop | Resume when calibrated |
| Home / Backspace / F9 | Return home and park | X=0.38, Y=0, Z=0.35 |
| End / Delete / F10 | Resume tracking | Exit parked mode |
| Enter / F11 | Calibrate neutral IMU tilt | Affects future sensor input |
| F12 | Quit | On some laptops use Fn+F12 |

**Step size:** 0.015 m (1.5 cm) per key press; targets are limited to
X=0.27–0.49 m, Y=-0.17–0.17 m and Z=0.30–0.52 m.
Left/right refers to the **robot's coordinate frame**, not necessarily the
left/right side of your current camera viewpoint. Rotate the camera if needed.

## Calibration explained

Enter requests **new IMU neutral orientation**. The next valid synthetic or
physical sample becomes the reference. The console prints:

`[CAL] Waiting for next valid sensor frame...`
`[CAL OK] Neutral roll=+0.00°, pitch=+0.00° ...`

In `run_demo.bat`, synthetic roll and pitch are always 0°; calibrating produces
no visible arm movement. In `run_serial.bat`, hold the Pico/MPU6050 controller
still in the desired neutral position, press Enter, and check `[CAL OK]` in
VS Code's terminal. Calibration does not move the arm home; use Home for that.

Keyboard jog offsets complement (and do not replace) IMU motion control.
The existing `run_demo.bat` and `run_sweep_demo.bat` scripts are unchanged.

## Apply patch on Windows

1. Close the MuJoCo viewer.
2. Open PowerShell **inside the existing `PhantomArm` folder**.
3. Run:

```powershell
Expand-Archive -Path "$HOME\Downloads\PhantomArm_v1_4_Arrow_Controls_Patch.zip" -DestinationPath (Get-Location).Path -Force
.\scripts\test_all.bat
.\scripts\run_demo.bat
```

This patch does not touch `.venv`, model geometry, or firmware.
