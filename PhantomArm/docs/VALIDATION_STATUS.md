# Validation status on October 30, 2026

The project source was prepared on a Linux-based authoring container, not on the user's Windows computer.

**Checks completed locally:**
- Python syntax compilation (`compileall`) succeeded after fixes.
- `python -m pytest -q` reported **37 passed, 1 skipped**.
- Test coverage includes FK/IK numerical roundtrips, controller mapping and interlocks, telemetry validation, slew limiting, and recording/replay.
- The MJCF source XML and the SVG architectural/wiring drawings were checked for valid XML structure.
- 1,125 workspace target points were checked by the analytic IK solver; 0 rejected in the supported roll/pitch/height envelope.

**Checks NOT completed in the authoring environment:**
- MuJoCo was unavailable for installation due to disabled network package access. Its MJCF **compiler**, the running 3D viewer, and simulator dynamics have therefore **not yet been verified**. The included `phantomarm.verify_model` script checks these on your Windows laptop.
- Arduino Pico 2 W firmware compile was not performed (Arduino IDE board package unavailable in authoring environment).
- Physical MPU6050, USB serial port, optional pushbutton/LEDs and real hardware were not connected or tested.

**Please do not claim the full project is already validated.** Run `scripts/setup_windows.bat`, then `scripts/test_all.bat` and hardware steps in `docs/WINDOWS_SETUP.md`, report any errors, and tick the real hardware checklist before final publication.
