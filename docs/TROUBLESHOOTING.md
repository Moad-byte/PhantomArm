# Troubleshooting

| Symptom | Diagnosis / fix |
|---|---|
| `py -3.11` not recognized | Install CPython 3.11 64-bit from python.org with launcher. Restart VS Code. |
| `No matching distribution` for mujoco | Check `py -3.11 -c "import platform;print(platform.architecture())"` outputs 64bit. Use `.venv` Python, not Store Python. |
| C++ / Visual Studio Build Tools requested | Stop; prebuilt wheel should install with `--only-binary=:all:`. Repair Python installation, don't compile MuJoCo from source. |
| Viewer crashes/blank screen | Update GPU drivers, close overlay tools, run headless test; if only VM has issue, prefer Windows native. |
| Robot does not move sideways | `run_demo.bat` is stationary by design; use `run_sweep_demo.bat` for automatic two-axis movement. |
| Robot does not move when pressing 6/7/8 | Click the viewer to focus it; confirm the viewer is not stopped or parked, then check terminal output. |
| `Model ... compile` fails | Ensure `assets/phantom_arm.xml` is unmodified; run `verify_model.py` and copy full traceback. |
| `ModuleNotFoundError` | VS Code Python interpreter must point to `PhantomArm/.venv/Scripts/python.exe`; rerun setup. |
| No COM port | Data-capable USB cable, board powered, correct USB connector, firmware successfully uploaded. |
| COM port access denied | Close Arduino Serial Monitor, another Python process, or other serial terminal. |
| Only invalid `PA1` messages | Check I²C wires GP4/GP5 + 3V3/GND; scan 0x68 address; unplug safely and reseat wires. |
| Tilt axes reversed | Rotate IMU or reverse orientation mapping intentionally after testing; do not rewrite gyro math blindly. |
| Arm briefly holds on reconnect | 400 ms telemetry watchdog; restart serial program and allow neutral calibration. |
| `PermissionError` on Ubuntu USB | Add user to dialout group and log out/in; for VMware attach USB device to guest explicitly. |
| Headless works but GUI fails | Graphics/OpenGL issue, not core IK. Try updated Windows graphics driver and built-in GPU preference. |

Reference: https://mujoco.readthedocs.io/en/stable/python.html , https://arduino-pico.readthedocs.io/en/latest/wire.html
