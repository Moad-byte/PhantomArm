# One-page sprint instruction (Windows)

**First hour**: use your verified Python 3.13 environment (or any supported project setup); extract PhantomArm; open VS Code; run `scripts/setup_windows.bat`; run `scripts/test_all.bat` and save passing output.

**Second hour**: run `scripts/run_demo.bat`. Check Up/Down for Z, Left/Right for Y, 7/8 for X, 6 for gripper, 9 to stop, Space to reset, Home to park, and End to resume. Run `run_sweep_demo.bat` separately for automatic X/Y movement. Capture the first simulator screen recording.

**Hours 3–5**: verify wiring from `docs/HARDWARE_WIRING.md`, upload sketch `firmware/PhantomArm_Pico2W/PhantomArm_Pico2W.ino`, check live CSV frames in Arduino Serial Monitor.

**Hours 5–7**: close Serial Monitor, run `scripts/run_serial.bat` and enter actual COM port; neutral calibrate while holding Pico still, tilt sensor, toggle physical button.

**Hours 7–9**: test failed sensor, USB disconnection, ESTOP, gripper, height and calibration. Record hardware clip and test results.

**Hours 9–12**: record `recordings/hardware_session.jsonl`; replay; inspect `logs/last_session.csv`; collect 3–4 best photos/screenshots.

**Hours 12–16**: complete validation checklist, refine README with *actual* data, create personal GitHub repo, first release and LinkedIn project post.

**Time estimate:** This is a best-case guided workflow, not a guaranteed schedule. USB drivers, I²C wiring and simulator installation can take longer.


**v1.4 controls:** Up/Down = Z height; Left/Right = Y lateral; 7/8 = X forward/back; 6 = gripper; 9 = stop; Space = reset; Home/Backspace = park; End/Delete = live; Enter = recapture the IMU neutral (no visible change in stationary demo). See `docs/ARROW_CONTROL_UPDATE.md`.
