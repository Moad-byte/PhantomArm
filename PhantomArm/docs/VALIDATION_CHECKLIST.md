# Evidence and release-gate checklist

Do **not** mark a test complete unless you actually ran it. Check results before claiming any number of passing tests on GitHub or LinkedIn.

## A — Software setup
- [ ] Supported Python (3.13 on current Windows setup) active inside `.venv`
- [ ] `pip check` reports no broken requirements
- [ ] `pytest -v tests` passes all tests (record real total)
- [ ] `python -m phantomarm.verify_model` checks four FK/IK vs MuJoCo positions
- [ ] `--mode demo --headless --duration 5` completes without exception

## B — 3D simulation
- [ ] 3D window opens with floor, blue/teal 4-DOF arm, orange finger tips
- [ ] Stationary demo starts without automatic X/Y motion; arrow keys control Z and Y; 7/8 control X; sweep demo separately moves smoothly toward orange target marker
- [ ] Press 9 freezes current joint setpoints
- [ ] Press Space resets stop, Home parks, End resumes; Enter logs calibration confirmation
- [ ] Up/Down changes height; 6 toggles finger aperture
- [ ] Closed gripper and open gripper remain physically plausible

## C — Sensor and hardware
- [ ] Physically photographed GP4 SDA, GP5 SCL, 3V3, GND
- [ ] Firmware compiles for **Raspberry Pi Pico 2 W**
- [ ] Serial Monitor displays parseable PA1 messages at approximately 50 Hz
- [ ] Pitch/roll values respond to real sensor tilt
- [ ] Button press reports 1 while pressed, then 0 after release
- [ ] Disconnect or invalid sensor state produces host HOLD
- [ ] Real hardware controls simulated robot and gripper
- [ ] COM port closes cleanly, no use-after-close crashes

## D — Regression and recordings
- [ ] Record physical session to `recordings/hardware_session.jsonl`
- [ ] Replay session using `--mode replay`
- [ ] Log `logs/last_session.csv` has nonempty position and error fields
- [ ] Export test transcript (date, number passed, system info)

## E — Portfolio publication
- [ ] Repository README explains problem, implementation and limitations
- [ ] Original source and MJCF model committed
- [ ] Firmware source and required library dependencies documented
- [ ] Wiring table/diagram visible in repo
- [ ] Screenshots/GIF/MP4 of **real** simulator and hardware in `media/`
- [ ] Proper upstream attribution (MuJoCo, Adafruit)
- [ ] GitHub release tag created only after software + hardware checks
- [ ] LinkedIn project and post contain verified statements only

## Acceptance criteria

A complete **portfolio** release has a working Windows simulator, a working physical IMU controller, deterministic tests, live arm movement and gripper toggle, fault/ESTOP demonstrations, recorded/replayed session, and readable documentation. It does not require a manufactured robot, ROS 2, 3D printer, motor drivers, a physical gripper, or Ubuntu.
