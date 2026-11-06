<p align="center">
  <img src="media/phantomarm_banner.gif" alt="PhantomArm — animated hardware-to-robot teleoperation banner" width="100%" />
</p>

# PhantomArm

**IMU-Controlled Robotic Arm Teleoperation · Physical Controller + MuJoCo Simulation**

`Raspberry Pi Pico 2 W` · `MPU6050` · `Arduino C++` · `USB Serial` · `Python` · `MuJoCo`

*Build a physical motion controller, move a simulated robot in real time, and perform genuine contact-based pick-and-place.*

**v1.0.0 · Windows-first · 4-DOF arm + 2-finger gripper · PA2 hardware controller**

---

## See it working

[![PhantomArm real hardware and simulated robotic arm — click to watch the complete demonstration](media/demo_preview.jpg)](media/phantomarm_hardware_demo.mp4)

**▶ [Watch the real-time hardware-controlled demonstration (MP4)](media/phantomarm_hardware_demo.mp4)** — two simultaneous views: the **MuJoCo robot** and the **physical Pico 2 W + MPU6050 controller**. The operator approaches, grasps, lifts, transports, releases, and resets the cube. **The physical hardware controls a simulated arm; no physical industrial arm is claimed.**

<table>
<tr><th>Physical controller used in the demo</th><th>What the project demonstrates</th></tr>
<tr>
<td width="45%"><img src="media/phantomarm_hardware_setup.jpg" alt="Actual Pico 2 W, MPU6050 and four-button breadboard" width="100%"></td>
<td valign="top"><ul>
<li><strong>Human-in-the-loop teleoperation:</strong> tilt the MPU6050 for X/Y positioning.</li>
<li><strong>Four tactile buttons:</strong> hold to raise, lower, close or open the gripper.</li>
<li><strong>Embedded-to-PC communication:</strong> structured PA2 messages over USB serial.</li>
<li><strong>Robotics math:</strong> calibrated, filtered target poses; analytic inverse kinematics; slew limiting.</li>
<li><strong>Physics-based manipulation:</strong> friction/contact-driven gripping with a free-moving cube.</li>
<li><strong>Recoverability:</strong> cube-only reset without resetting the arm or its controller.</li>
</ul></td>
</tr>
</table>

> **Scope:** This is a working **physical-controller → simulated-robot** prototype. The industrial concept image included in the media folder illustrates possible future applications; it is not a photograph of an installed industrial system.

## On this page

[Quick start](#quick-start-windows) · [Architecture](#system-architecture) · [Hardware and wiring](#hardware-and-wiring) · [Full control map](#control-map) · [Serial protocol](#pa2-serial-telemetry) · [Engineering calculations](#engineering-design-and-calculations) · [Simulation and grasp](#mujoco-robot-and-grasp-physics) · [Validation](#tests-and-validation) · [Troubleshooting](#troubleshooting)

## Project at a glance

| Area | Implementation |
|:--|:--|
| **Manipulator** | Original 4-revolute-joint arm: base yaw, shoulder pitch, elbow pitch, wrist pitch; 2 independently actuated finger slides |
| **Physical interface** | Raspberry Pi Pico 2 W (RP2350), MPU6050 IMU, four momentary pushbuttons |
| **Movement** | Sensor tilt → X/Y target; held buttons → continuous Z and gripper adjustments |
| **Transport** | I²C from MPU6050 to Pico; PA2 text telemetry over USB serial at **115200 baud** |
| **Host control** | Python calibration, exponential smoothing, workspace constraints, analytic IK and motion slew |
| **Simulation** | MuJoCo with six position actuators, rigid-body contacts, frictional grasp and movable cube |
| **Completed demo** | Hardware-controlled pickup → lift → sideways transfer → release → cube-only reset |
| **Operating system** | Windows for the tested GUI/hotkey workflow; core mathematics can run independently |

## Quick start (Windows)

### 1. Install the software

- **Python 3.11** is targeted by the included setup batch file (`py -3.11`).
- The project pins **MuJoCo 3.14.0** and uses `numpy`, `pyserial` and `pytest` (see `requirements.txt`).
- For hardware, install Arduino IDE and the **Raspberry Pi Pico/RP2040/RP2350** board package; select **Raspberry Pi Pico 2 W**.
- Flash the **final four-button PA2 firmware** from `sketch/sketch.ino` (not the older prototype under `firmware/`).

Open **PowerShell inside the repository root** (where `README.md` and `scripts/` are visible):

```powershell
# One-time Python environment setup
.\scripts\setup_windows.bat

# Verify available serial ports (close Arduino Serial Monitor first)
.\.venv\Scripts\python.exe -m serial.tools.list_ports -v
```

### 2. Run the hardware-controlled robot

```powershell
# Replace COM3 with the port listed on YOUR computer
.\.venv\Scripts\python.exe -m phantomarm.app --mode serial --port COM3 --scene grasp
```

Keep the MPU6050 steady for the initial calibration. Click the MuJoCo viewer to use keyboard fallback controls. The four breadboard buttons work through the serial connection.

**No Pico connected?** Use simulation-only mode:

```powershell
.\scripts\run_grasp_demo.bat
```

**Want to inspect the physics without using the GUI?**

```powershell
.\scripts\test_all.bat
.\scripts\run_grasp_diagnostic.bat
```

## System architecture

```mermaid
flowchart LR
    IMU["MPU6050<br/>Roll + pitch"] -->|"I²C · GP4 / GP5"| PICO["Pico 2 W<br/>Arduino C++"]
    BTN["4 buttons<br/>Z up/down · grip open/close"] --> PICO
    PICO -->|"USB · PA2 · 115200"| PARSER["Python parser<br/>sequence + validity"]
    PARSER --> FILTER["Neutral calibration<br/>EMA smoothing"]
    FILTER --> TARGET["X/Y/Z target<br/>workspace guard"]
    TARGET --> IK["Analytic IK<br/>joint slew"]
    IK --> MJ["MuJoCo<br/>6 actuators"]
    MJ --> CUBE["Contact-based<br/>pick and place"]
    KB["Keyboard fallback<br/>Insert cube reset"] -.-> TARGET
    KB -.-> MJ
```

**End-to-end sequence:** (1) the MPU6050 produces gravity-based tilt readings; (2) the Pico reads them and four buttons; (3) Python validates PA2 packets and captures a neutral pose; (4) filtered roll/pitch become Cartesian targets; (5) analytic IK finds a joint configuration; (6) rate-limited actuator commands drive the MuJoCo arm; (7) contact forces—not a scripted attachment—determine whether the cube is held.

## Hardware and wiring

The complete build needs **one Pico 2 W, one MPU6050 breakout, four normally-open four-legged tactile switches, one breadboard, jumper wires and a USB data cable**. For the four-legged switches, use one terminal from **each of the two separate contact pairs**: terminals on the same internally connected side are not a working switch connection.

| Signal / component | Pico GPIO | Physical Pico pin | Connection / behavior |
|:--|:--:|:--:|:--|
| MPU6050 **VCC** | 3V3(OUT) | **36** | Supply sensor breakout from 3.3 V; check module voltage compatibility |
| MPU6050 **GND** | GND | **38** | Common ground |
| MPU6050 **SDA** | GP4 | **6** | I²C data |
| MPU6050 **SCL** | GP5 | **7** | I²C clock |
| **Z Up** switch | GP16 | **21** | Switch between GPIO and GND; **hold** to raise |
| **Z Down** switch | GP17 | **22** | Switch between GPIO and GND; **hold** to lower |
| **Grip Close** switch | GP18 | **24** | Switch between GPIO and GND; **hold** to close |
| **Grip Open** switch | GP19 | **25** | Switch between GPIO and GND; **hold** to open |

**Firmware wiring settings:** `Wire.setSDA(4)`, `Wire.setSCL(5)`, `Wire.setClock(100000)` (**100 kHz I²C**). The sensor is at **I²C address `0x68`** in the demonstrated build. Buttons use `INPUT_PULLUP`: **LOW = pressed**, **HIGH = released**; no extra external pull-up is required for this configuration. The code debounces inputs for **25 ms**.

**Safety when wiring:** disconnect USB power, verify the 3.3 V and ground connections, and ensure that a button never directly shorts 3.3 V to ground. All four button ground connections may share the same rail.

## Control map

### Physical controller (used in the demo)

| Input | Action | Behavior when released |
|:--|:--|:--|
| Tilt MPU6050 **forward / backward** | X-axis target movement | Target follows measured tilt relative to startup neutral |
| Tilt MPU6050 **left / right** | Y-axis target movement | Target follows measured tilt relative to startup neutral |
| **Hold GP16** | Raise arm, **Z+** | Stop changing height |
| **Hold GP17** | Lower arm, **Z−** | Stop changing height |
| **Hold GP18** | Gradually **close** gripper | Retain current commanded finger position |
| **Hold GP19** | Gradually **open** gripper | Retain current commanded finger position |

When both height buttons are held, neither direction is sent. When both gripper buttons are held, neither gripper direction is sent. Releasing both fingers does **not** make the cube magically stick: real MuJoCo contact and friction must maintain the hold.

### Keyboard fallback and maintenance

| Key | Action |
|:--|:--|
| **↑ / ↓** | Raise / lower the robot end-effector target |
| **← / →** | Move sideways (Y+ / Y−) |
| **7 / 8** | Forward / backward (X+ / X−) |
| **Hold 6** | Close the gripper gradually |
| **Hold Enter** | Open the gripper gradually |
| **Insert** | **Cube-only respawn**; does not home or reset the arm (Windows) |
| **Type `cube` + Enter in running terminal** | Backup cube-only respawn |
| **F11** | Re-zero current IMU orientation |
| **9** | Simulation emergency stop |
| **Space** | Release emergency stop / begin recalibration |
| **Home / End** | Park arm / resume live control |
| **F12** | Quit simulation |

**Do not use Delete or Backspace for cube reset.** Those keys conflicted with MuJoCo's own viewer behavior in earlier iterations. The confirmed operational shortcut is **Insert**.

## PA2 serial telemetry

### Message layout

The final Arduino firmware emits a CSV-like ASCII message every **50 ms (~20 Hz)**:

```text
PA2,milliseconds,sequence,roll_deg,pitch_deg,z_up,z_down,grip_close,grip_open,valid
```

Illustrative PA2 frame:

```text
PA2,25059,474,-0.14,11.78,0,0,1,0,1
```

| Field | Meaning |
|:--|:--|
| `PA2` | Protocol version for the four-button controller |
| `milliseconds` | Pico uptime in milliseconds (`millis()`) |
| `sequence` | Increasing frame counter; repeated / older frames are rejected |
| `roll_deg`, `pitch_deg` | Accelerometer-derived tilt in degrees (not absolute spatial position) |
| `z_up`, `z_down` | Independent continuous-height requests (`0` or `1`) |
| `grip_close`, `grip_open` | Independent continuous finger-control requests (`0` or `1`) |
| `valid` | Whether the sensor read succeeded (`1`) or failed (`0`) |

The host checks field count, numeric values, finite angles, binary flags and ordered sequence numbers; invalid frames are not used for motion. The earlier **PA1** format is still supported for compatibility with demos/replays, but **PA2** is required for the four physical buttons. The current firmware does **not** command gripper state by physically positioning the user's hand—it uses the two gripper buttons.

## Engineering design and calculations

These are the **actual equations and parameters implemented in this version**, not generic examples of how a teleoperation project might work.

### 1. Tilt estimation and neutral calibration

The firmware reads 3-axis accelerometer registers beginning at `0x3B` and scales the configured **±2g** values by **16,384 counts/g**:

$$a_x = \frac{A_x}{16384},\quad a_y = \frac{A_y}{16384},\quad a_z = \frac{A_z}{16384}$$

The current firmware's tilt estimates are:

$$\mathrm{roll} = \operatorname{atan2}(a_y,a_z)\,\frac{180}{\pi}$$

$$\mathrm{pitch} = \operatorname{atan2}\!\left(-a_x,\sqrt{a_x^2+a_z^2}\right)\frac{180}{\pi}$$

These are **accelerometer-based estimates**, not a gyroscope/accelerometer fusion filter. They are most reliable when the controller is being tilted without significant linear acceleration. The Python controller stores the **first valid roll and pitch** as neutral (e.g., a +11.6° initial pitch is treated as zero relative motion), and computes:

$$r=\operatorname{clip}(\mathrm{roll}-\mathrm{roll}_0,-35^\circ,35^\circ)$$
$$p=\operatorname{clip}(\mathrm{pitch}-\mathrm{pitch}_0,-35^\circ,35^\circ)$$

An exponential moving average (EMA) smooths both axes, with **α = 0.25**:

$$\bar r_k = 0.25r_k + 0.75\bar r_{k-1},\qquad \bar p_k = 0.25p_k + 0.75\bar p_{k-1}$$

### 2. Mapping tilt to robot X/Y targets

The filtered angles set the **desired tool position**, not the user's physical hand coordinates:

$$x^* = \operatorname{clip}\!\left(0.38-\frac{0.11\bar p}{35}+d_x,\ 0.27,\ 0.49\right)\ \mathrm{m}$$

$$y^* = \operatorname{clip}\!\left(\frac{0.17\bar r}{35}+d_y,\ -0.17,\ 0.17\right)\ \mathrm{m}$$

Here `d_x` and `d_y` are optional keyboard jog offsets. With zero offsets:

| Input | Target response |
|:--|:--|
| Neutral roll/pitch | `(x, y) ≈ (0.38, 0.00)` m |
| `+35°` pitch | X approaches **0.27 m** |
| `−35°` pitch | X approaches **0.49 m** |
| `+35°` roll | Y approaches **+0.17 m** |
| `−35°` roll | Y approaches **−0.17 m** |

**Sensitivity:** a 1° filtered pitch change requests approximately **3.14 mm** in X, while 1° of roll requests **4.86 mm** in Y, before workspace and reachability clamping.

### 3. Z motion and finger motion

The button-driven target-height rate is **0.035 m/s = 3.5 cm/s** while held. Height is constrained to **0.30–0.52 m**, and the grasp-scene keep-out raises the minimum tool target to **0.35 m above either pedestal**. Unreachable IK targets are rejected.

| Command | Implemented speed | Movement range | Approximate full-stroke time |
|:--|:--:|:--:|:--:|
| **Z raise/lower** | **35 mm/s** | Requested height: 0.30–0.52 m (subject to keep-out) | Depends on distance |
| **Close finger slide** | **14 mm/s** | 0–25 mm per finger | **1.79 s** |
| **Open finger slide** | **9 mm/s** | 0–25 mm per finger | **2.78 s** |

The times are **commanded-target travel estimates**, not guarantees of achieved physical contact positions. Finger actuator dynamics and contact loads can change the actual motion.

### 4. Four-joint arm kinematics (analytic IK)

The arm uses **base yaw + shoulder + elbow + wrist pitch**, with two independently driven finger slides:

| Kinematic parameter | Value |
|:--|--:|
| Shoulder reference height `h` | **0.22 m** |
| Upper-arm length `L₁` | **0.30 m** |
| Forearm length `L₂` | **0.26 m** |
| Wrist tool offset `Lw` | **0.10 m** |

For a desired target $(x,y,z)$, calculate radial reach and base yaw:

$$r=\sqrt{x^2+y^2},\qquad q_{yaw}=\operatorname{atan2}(y,x)$$

Remove the horizontal wrist offset and shoulder height:

$$u=r-L_w,\qquad v=z-h$$

Use the planar two-link cosine rule:

$$D=\frac{u^2+v^2-L_1^2-L_2^2}{2L_1L_2}$$

A target is geometrically unreachable when **`D` is outside `[-1, 1]`**. Otherwise:

$$\beta=\cos^{-1}(D),\qquad \alpha=\operatorname{atan2}(v,u)-\operatorname{atan2}(L_2\sin\beta,L_1+L_2\cos\beta)$$

The MuJoCo model's positive local Y rotation convention gives:

$$\boxed{(q_{yaw},q_{shoulder},q_{elbow},q_{wrist})=(\operatorname{atan2}(y,x),-\alpha,-\beta,\alpha+\beta)}$$

This wrist choice keeps the tool approximately horizontal in the model. The solver then checks all joint bounds:

| Joint | Minimum | Maximum |
|:--|--:|--:|
| Base yaw | −1.55 rad | +1.55 rad |
| Shoulder pitch | −1.65 rad | +1.65 rad |
| Elbow pitch | −2.50 rad | +2.50 rad |
| Wrist pitch | −2.80 rad | +2.80 rad |

**Worked example — initial grasp-scene hover target:** $(x,y,z)=(0.38,0,0.43)\,\mathrm{m}$. Then $r=0.38$, $u=0.28$, $v=0.21$ and **`D = −0.225`**. The computed joint command is approximately:

```text
[yaw, shoulder, elbow, wrist] = [0.000, 0.166, -1.798, 1.632] rad
                               = [0.0°, 9.5°, -103.0°, 93.5°]
```

**Forward-kinematics verification:**

$$r=L_1\cos(q_s)+L_2\cos(q_s+q_e)+L_w\cos(q_s+q_e+q_w)$$
$$z=h-L_1\sin(q_s)-L_2\sin(q_s+q_e)-L_w\sin(q_s+q_e+q_w)$$
$$x=r\cos(q_{yaw}),\qquad y=r\sin(q_{yaw})$$

The model-verification script compares predicted tool positions against MuJoCo site coordinates for **four target poses**, requiring position error below **`1e−4 m` (0.1 mm)** for the static model checks—not a claim of 0.1-mm tracking accuracy during operation.

### 5. Rate limiting, controller states and timeout

Joint targets pass through the slew limiter:

$$q_{k+1,i}=q_{k,i}+\operatorname{clip}(q^*_{i}-q_{k,i},-v_i\Delta,v_i\Delta)$$

with nominal slew parameters **`(1.6, 1.4, 1.6, 2.0)`** for yaw, shoulder, elbow and wrist; the grasp application supplies a conservative step argument of **`0.009`** to avoid abrupt grasp movements. MuJoCo separately integrates physical actuator and contact dynamics.

| Control state / guard | Behavior |
|:--|:--|
| **READY** | Valid, recent telemetry may command motion |
| **HOLD** | On stale/invalid input, hold last commanded joints; pause physics in this simulation implementation |
| **ESTOP** | Latch software stop until manually released |
| **Telemetry timeout** | No fresh valid sample for more than **0.40 s** → HOLD |
| **Out-of-order frame** | Reject sequence numbers not increasing; recalibration required after device reboot |
| **Conflicting physical buttons** | Opposite Z/gripper requests cancel instead of fighting each other |
| **Pedestal keep-out** | Tool target Z is clamped to ≥ **0.35 m** near pickup/drop surfaces |

**Important:** This is a **software simulation stop**, not a certified hardware emergency-stop mechanism or comprehensive collision-free motion planner.

## MuJoCo robot and grasp physics

The robot and grasp challenge are defined in a readable **MJCF XML** model, `assets/phantom_arm_grasp.xml`.

| Simulated property | Model value |
|:--|:--|
| Arm joint actuators | **4** position actuators |
| Gripper actuators | **2** slide-position actuators (one per finger) |
| Cube | **46 × 46 × 46 mm**, **10 g** (`0.010 kg`) |
| Cube spawn center | **(0.43, 0.00, 0.350) m** |
| Pickup pedestal center | **(0.43, 0.00) m** in XY |
| Drop pedestal center | **(0.43, 0.12) m** in XY — **12 cm** sideways |
| Both pedestal tops | **Z = 0.327 m** |
| Initial grasp-scene tool hover target | **(0.38, 0.00, 0.43) m** |
| Gripper pad friction / cube friction | Sliding-friction coefficients **8.0 / 5.0** in their respective MJCF contact settings |
| MuJoCo integrator | `implicitfast`, timestep **0.003 s**, `iterations=100`, `noslip_iterations=5` |
| Cube physics | **Free joint**: 7 position coordinates (XYZ + quaternion) and 6 velocities |

Each position actuator uses stiffness/damping (`kp` / `kv`) instead of instant joint teleportation:

| Actuator | `kp` | `kv` |
|:--|--:|--:|
| Yaw | 170 | 22 |
| Shoulder | 200 | 24 |
| Elbow | 160 | 18 |
| Wrist | 70 | 10 |
| Left / right finger (each) | 1500 | 45 |

**How grasping works:** Fingers close gradually around a dynamic rigid-body cube. The simulator resolves contact between both finger pads and the cube. The cube is lifted or dropped as a consequence of simulated forces and friction; the code does **not** teleport, weld, or artificially attach it to the fingers. The goal marker represents the desired target, not a guaranteed measured tip position.

**Cube-only reset:** Press **Insert** or type `cube` in the running terminal. Before respawning, the application checks whether the robot occupies the cube's starting volume. If clear, only the cube free joint's **7 position coordinates and 6 velocity coordinates** are restored; the arm's joint state, actuator commands, gripper target and controller target remain in place. If the spawn is obstructed, reset is rejected—move the arm aside and retry.

## Tests and validation

### How to reproduce

```powershell
# Python unit tests + compiled-MJCF / IK-FK verification
.\scripts\test_all.bat

# Three complete headless physics trials of the contact-based grasp
.\scripts\run_grasp_diagnostic.bat
```

### Reported and independently checked outcomes

| Evidence | Result | What it supports |
|:--|:--|:--|
| **Automated tests in this review environment** | **76 passed, 9 skipped** | Python/structural tests passed; MuJoCo-dependent tests skipped because MuJoCo was not installed here |
| **Windows MuJoCo physics diagnostic** | **3/3 full trials previously reported passing** | Earlier validated runs reported successful grasp, hold, transport, release and reset; **not rerun in this review environment** |
| **Real controller demonstration** | **Completed by project creator; video included above** | User-operated X/Y tilt, button-driven Z/grip, cube transfer and reset |
| **Static model check specification** | **4 IK/FK target poses; error < 0.1 mm** | Expected analytical-model consistency criterion; run on a MuJoCo-enabled machine to reproduce |

### What the three-trial physics diagnostic checks

| Stage | Pass criterion |
|:--|:--|
| **1. Stable startup** | Cube remains on original platform during **≥3 s** hover; test checks vertical settling ≤ **8 mm** |
| **2. Physical grip** | Both left and right finger pads contact the free cube (**2 distinct finger contacts**) |
| **3. Lift** | Cube rises at least **30 mm** above original position, retaining both pad contacts; previously reported runs reached ~**58 mm** |
| **4. Hold** | Maintain two-pad grip for **≥3 s** |
| **5. Transport** | Move from pickup toward target at **Y = 0.12 m**, maintaining contact |
| **6. Release** | Cube contacts destination pedestal after fingers open and arm retracts |
| **7. Reset guard** | Block cube respawn if robot obstructs the original volume |
| **8. Cube-only reset** | Restore cube position/velocity while preserving all six robot joint positions, velocities and actuator controls; repeat sequence **3 times** |

The local automated tests also cover telemetry parsing and invalid inputs, neutral capture, smoothing, timeout handling, joint-limit reachability, keyboard jogging, button ramping, collision flags, reset invariance and scene structure. **A passing unit suite alone is not presented as proof of physical grasping**—the contact-physics diagnostic and the actual hardware video serve that separate purpose.

## Troubleshooting

| Symptom | What to check |
|:--|:--|
| `\.venv\Scripts\python.exe` not found | You are likely outside the project root or haven't run `scripts\setup_windows.bat`. Check the VS Code terminal working directory. |
| `COM3` unavailable or permission error | Arduino Serial Monitor/miniterm may still be holding the port. Close it; list ports and use the correct COM number. |
| Arm does not move in serial mode | Verify PA2 lines, `valid=1`, increasing sequence numbers, board calibration and that the app was started with `--mode serial --port COMx --scene grasp`. |
| Buttons do not respond | Use **GP16/17/18/19** to GND, not 3.3 V; check opposite sides of each four-pin switch and `INPUT_PULLUP` behavior. |
| Arm moves unexpectedly after startup | Hold the sensor steady and press **F11** to recalibrate the neutral orientation. |
| Cube will not reset | Use **Insert**, not Delete/Backspace. Clear the pickup spawn position first; `cube` + Enter in the simulation terminal is the fallback. |
| Cube is wedged at a corner | Hold **open gripper**, retract or raise gently, then use cube-only reset after clearing the spawn; imperfect off-angle grasps are a known limitation. |
| Cannot descend onto pedestal | A conservative **0.35 m** tool-height keep-out is intentionally active near both platforms. |
| Script says test failed | Run both the unit suite and headless physics diagnostic; check the full output rather than relying on historical pass counts. |

## Repository structure

```text
PhantomArm/
├── README.md                     # THIS complete overview
├── .gitignore / LICENSE / requirements.txt
├── phantomarm/                   # Parser, controller, IK, safety, grasp, application
├── sketch/sketch.ino             # FINAL four-button PA2 firmware
├── assets/                       # Standard and grasp-scene MJCF models
├── scripts/                      # Windows setup, demos, tests, physics diagnostic
├── tests/                        # Unit and MuJoCo-dependent regression tests
├── media/                        # Animated banner, hardware photo, real demo, preview
├── recordings/                   # Example telemetry for replay
├── tools/                        # Session log plotting
├── firmware/                     # Historical first-generation firmware (not PA2)
└── docs/                         # Optional background and iteration history
```

Everything needed to **understand, build, operate, calculate, and validate** the current system is described **on this page**. Other project files are provided for reproducibility and source inspection—not as required reading to understand the project.

## Limitations & future directions

- **Virtual manipulator, physical controller:** no fabricated industrial arm or real motor load is connected.
- **Accelerometer tilt rather than full 3D tracking:** the controller estimates orientation under gravity; translational acceleration and fast motion may disturb readings.
- **No production safety certification:** software stop, simple pedestal keep-out and contact simulation are not industrial safety systems.
- **Corner grasps:** off-center cube orientations can wedge between high-friction pads and the wrist; cube-only recovery is available.
- **Next engineering extensions:** filtered gyro-assisted attitude estimation, improved grasp geometry, measured response latency, physical closed-loop actuator integration and more complex manipulation tasks.

## References & license

Project equations, thresholds, circuit mapping, test criteria and engineering values above were derived from the **checked-in final source** rather than generic example code. Useful upstream documentation:

- [Raspberry Pi — Pico 2 / Pico 2 W documentation](https://www.raspberrypi.com/documentation/microcontrollers/pico-series.html)
- [Arduino-Pico — I²C (`Wire`) reference](https://arduino-pico.readthedocs.io/en/latest/wire.html)
- [TDK InvenSense — MPU-6000/6050 register map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map.pdf)
- [MuJoCo documentation](https://mujoco.readthedocs.io/en/stable/)
- [pySerial documentation](https://pyserial.readthedocs.io/en/latest/)

**License:** [MIT](LICENSE). Images of potential industrial applications are **concept illustrations**, not evidence of production deployment.
