# PhantomArm — Complete Windows A–Z setup

## Why Windows (not Ubuntu in VMware)?

Windows 10/11 64-bit can run MuJoCo's official Python wheels and 3D viewer directly. It also simplifies the Pico 2 W USB COM-port connection, reduces virtualization graphics issues, and fits a fast project schedule. **Do not install VMware or Ubuntu for this project unless Windows OpenGL actually fails.**

## Requirements before starting

- Windows x64, Python **3.11 (64-bit)**; install with **py launcher**, not Python 3.13 from the Microsoft Store.
- VS Code, Microsoft Python extension, Arduino IDE 2, Git (optional until publishing).
- GPU supporting OpenGL and updated display drivers; ordinary integrated graphics often works.
- A data-capable USB cable, Raspberry Pi Pico 2 W, MPU6050/GY-521, breadboard and four jumper wires. Button + LEDs optional.
- ~2 GB free storage and an internet connection for initial installs.

Links: https://www.python.org/downloads/ , https://code.visualstudio.com/ , https://www.arduino.cc/en/software , https://git-scm.com/downloads

## Step 1 — Extract project

1. Unzip `PhantomArm-Full-Project.zip` to `Documents\EngineeringProjects\PhantomArm`.
2. Open **this exact directory** in VS Code: File > Open Folder.
3. Do **not** upload or include `.venv` in GitHub.

## Step 2 — Install software and Python libraries

From File Explorer double-click `scripts\setup_windows.bat` or run inside VS Code terminal:

```powershell
py -3.11 --version
py -3.11 -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install --only-binary=:all: -r requirements.txt
.\.venv\Scripts\python.exe -m pip check
```

`--only-binary=:all:` prevents unexpected multi-hour C++ source compilation. If it cannot find a wheel, verify you installed **64-bit Python 3.11** rather than trying Ubuntu immediately.

Choose VS Code Python Interpreter: `Ctrl+Shift+P` > Python: Select Interpreter > `.venv\Scripts\python.exe`.

## Step 3 — Verify analytical code and 3D model

```powershell
.\.venv\Scripts\python.exe -m pytest -v tests
.\.venv\Scripts\python.exe -m phantomarm.verify_model
.\.venv\Scripts\python.exe -m phantomarm.app --mode demo --headless --duration 5
```

The model verifier should print four IK/FK comparisons and `PASS`. The headless run logs `READY` updates and saves `logs/last_session.csv` without opening a window. **Do not proceed if the verifier fails.**

## Step 4 — Launch 3D model in stationary manual or sweep demo mode

```powershell
.\.venv\Scripts\python.exe -m phantomarm.app --mode demo
```

Or double-click `scripts\run_demo.bat`. The stationary demo holds X/Y while you test the gripper and height. Use `scripts\run_sweep_demo.bat` for automatic two-axis movement. A bright orange marker is the commanded end-effector target. The tip of the arm is green. The base is dark and cyan.

Viewer keyboard shortcuts (click inside the viewer window first):

- `E`: software emergency stop — freezes current joint position
- `R`: reset interlock and recalibrate on next valid input
- `C`: neutral calibration on next valid input
- `O`: toggle gripper open/closed
- `W` / `S`: raise/lower target Z within safe height range
- `Q`: quit

## Step 5 — Wire and program real hardware

Follow `docs/HARDWARE_WIRING.md`, then install Arduino IDE board and MPU6050 libraries. Open `firmware/PhantomArm_Pico2W/PhantomArm_Pico2W.ino`; select **Raspberry Pi Pico 2 W** and the correct COM port, then Upload.

Use **Arduino-Pico board core from Earle Philhower** (same as StabilityGuard). Board manager instructions: https://arduino-pico.readthedocs.io/en/latest/install.html

To verify firmware, open Serial Monitor at 115200 and tilt gently. Expect frames like:

`PA1,1250,42,2.410,-8.120,0,1`

For serial operation, **close Serial Monitor first**: Windows only allows one program to own the COM port at a time.

## Step 6 — Connect physical controller to the 3D arm

List ports:

```powershell
.\.venv\Scripts\python.exe -m phantomarm.list_ports
```

Run (replace COM5):

```powershell
.\.venv\Scripts\python.exe -m phantomarm.app --mode serial --port COM5 --record recordings/hardware_session.jsonl
```

Keep sensor stationary while connecting; first valid reading becomes neutral. Tip left/right with roll, forward/back with pitch. Press the physical button to toggle the gripper. Press `C` to re-zero orientation. Motion is intentionally restricted and filtered for repeatability.

## Step 7 — Replay a recording

```powershell
.\.venv\Scripts\python.exe -m phantomarm.app --mode replay --file recordings/hardware_session.jsonl
```

Or test included sample: `scripts\run_replay.bat`. At the end of replay the watchdog causes HOLD.

## Step 8 — Evidence and publishing

Complete `docs/VALIDATION_CHECKLIST.md` with real passes/failures; capture the real board, a screen recording of the 3D arm, a green/red indicator demonstration, test output, and a graph from `logs/last_session.csv` (optional plotting command `python tools/plot_log.py` after installing matplotlib).

Create your own new GitHub repository rather than uploading the extracted directory directly with histories from others. `.gitignore` excludes environment and private sessions. See `docs/GITHUB_RELEASE_GUIDE.md`.
