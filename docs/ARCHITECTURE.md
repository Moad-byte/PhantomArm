# Architecture and robotics decisions

```text
MPU6050 (3-axis acceleration + gyroscope)
    │ I²C SDA GP4, SCL GP5
    ▼
Raspberry Pi Pico 2 W (Arduino C++)
  complementary tilt estimate • validity checks • button debounce
    │ USB CDC serial: PA1,ms,seq,roll,pitch,button,valid
    ▼
Python telemetry receiver / recorder
  parser • monotonic sequence guard • 400 ms watchdog
    │
    ▼
Motion controller
  neutral calibration • EMA • tilt clipping ±35°
  roll -> lateral Y, pitch -> forward X
  W/S keys -> height Z, button -> gripper
    │
    ▼
Analytic inverse kinematics / joint rate limits
  base yaw + shoulder + elbow + wrist
  solver enforces 4 joint-limit ranges
    │
    ▼
Original MuJoCo 3D MJCF robot + position actuators
  4 arm joints • 2 symmetrical sliding gripper fingers
    │
    ▼
Logs, CSV metrics, motion replay, verification tests
```

## Robot geometry

- Shoulder joint world reference Z = **0.22 m**.
- Upper arm length **0.30 m**, forearm **0.26 m**, wrist tool **0.10 m**.
- Position `(x,y,z)` represents `tool_tip` in base world coordinates.
- The wrist is controlled to maintain a horizontal tool orientation in the yaw plane.
- Base yaw `atan2(y,x)`; use classic planar 2-link IK after subtracting 0.1 m of wrist reach.
- Model comprises 4 independent revolute joints and 2 prismatic gripper finger joints (6 actuators total).
- Handheld tilt controls **two target axes**, not all robot degrees of freedom. Z is adjusted with keyboard W/S.

## Safety and limits

- Tilt clipped to ±35° and filtered using host EMA; workspace target `x=0.38 ±0.11 m`, `y=0 ±0.17 m`, `z=0.30…0.52 m`.
- IK refuses geometrically unreachable positions or joint limit violations.
- Joint targets are slew rate limited.
- Serial telemetry lacking valid samples for >0.40 s causes HOLD (freeze current simulated arm state).
- Software ESTOP `E` prevents control until reset `R`.
- Unexpected USB errors close the run and report failures.
- A reboot/resend with non-increasing sequence numbers is rejected; unplugged/restarted hardware should be reconnected and recalibrated.
- Not designed for industrial motion safety or physical motor control.

## Testing strategy

- Pure Python regression suite for IK/FK roundtrips, telemetry protocol, orientation mapping, clamps, gripper, watchdog and interlock.
- Optional MJCF compiler test using actual MuJoCo installed by Windows setup.
- `python -m phantomarm.verify_model` compares four IK target positions to the MuJoCo simulation's `tool_tip` sensor coordinates: geometry mismatch is a release blocker.
- Headless `--mode demo` validates the full loop without needing an OpenGL window.
- Live Pico 2 W demonstration requires actual hardware and is not covered by offline software tests.

## Why original MJCF instead of cloned repository or CAD?

The MJCF XML is an original lightweight robot model with geometry built from primitive boxes, cylinders, capsules, and spheres. It is a real 3D physics scene, editable directly in a text editor, and avoids unnecessary CAD dependencies. MuJoCo engine remains upstream third-party software.
