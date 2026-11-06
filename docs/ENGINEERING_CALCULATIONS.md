# PhantomArm — Engineering Calculations (Optional Technical Appendix)

This appendix preserves the mathematical detail from the earlier README. The main [README](../README.md) contains all required wiring, controls, design parameters, physics, setup, and tests. This page is for readers interested in derivations. Equations are shown as plain-text pseudocode to avoid unsupported rendering macros in some Markdown viewers.

## 1. Orientation estimation and calibration

Accelerometer registers begin at MPU6050 address `0x3B`. With ±2 g settings, divide raw accelerometer counts by 16,384:

```text
ax = Ax / 16384
ay = Ay / 16384
az = Az / 16384

roll_deg  = atan2(ay, az) * 180 / pi
pitch_deg = atan2(-ax, sqrt(ax*ax + az*az)) * 180 / pi
```

`roll_deg` and `pitch_deg` estimate orientation from gravity. The current implementation is **not** gyro fusion and can be affected by linear acceleration.

At initialization, the controller records `roll0` and `pitch0` from the first valid reading, then subtracts them from subsequent readings:

```text
r = clamp(roll_deg - roll0, -35, 35)
p = clamp(pitch_deg - pitch0, -35, 35)

filtered_r[k] = 0.25 * r[k] + 0.75 * filtered_r[k-1]
filtered_p[k] = 0.25 * p[k] + 0.75 * filtered_p[k-1]
```

## 2. Tilt to Cartesian workspace

```text
x_target = clamp(0.38 - 0.11 * filtered_p / 35 + dx, 0.27, 0.49)
y_target = clamp(0.17 * filtered_r / 35 + dy, -0.17, 0.17)
z_target = controller height, subject to safety and reachability checks
```

At zero keyboard offset: 1° pitch corresponds to about 0.11/35 m = **3.14 mm** requested X movement; 1° roll corresponds to 0.17/35 m = **4.86 mm** requested Y movement. These are commanded position sensitivities, before clamping.

## 3. Analytical inverse kinematics

The arm has shoulder reference height `h=0.22 m`, upper-arm length `L1=0.30 m`, forearm length `L2=0.26 m`, and tool offset `Lw=0.10 m`.

```text
r     = sqrt(x*x + y*y)
yaw   = atan2(y, x)
u     = r - Lw
v     = z - h
D     = (u*u + v*v - L1*L1 - L2*L2) / (2*L1*L2)

if D < -1 or D > 1: target is unreachable

beta  = acos(D)
alpha = atan2(v, u) - atan2(L2*sin(beta), L1 + L2*cos(beta))

q_yaw      = yaw
q_shoulder = -alpha
q_elbow    = -beta
q_wrist    = alpha + beta
```

The wrist is chosen to keep the model's tool approximately horizontal. Each joint must also satisfy its limits:

| Joint | Allowed interval (radians) |
|:--|:--|
| Base yaw | −1.55 to +1.55 |
| Shoulder | −1.65 to +1.65 |
| Elbow | −2.50 to +2.50 |
| Wrist | −2.80 to +2.80 |

### Worked initial hover target

For `x=0.38 m`, `y=0`, `z=0.43 m`, the calculation yields `r=0.38`, `u=0.28`, `v=0.21`, `D≈−0.225`, and joint targets:

```text
[yaw, shoulder, elbow, wrist] ≈ [0.000, 0.166, -1.798, 1.632] radians
                                ≈ [0.0, 9.5, -103.0, 93.5] degrees
```

### Forward kinematics (static model verification)

```text
r = L1*cos(qs) + L2*cos(qs+qe) + Lw*cos(qs+qe+qw)
z = h - L1*sin(qs) - L2*sin(qs+qe) - Lw*sin(qs+qe+qw)
x = r*cos(yaw)
y = r*sin(yaw)
```

The source's static model tests check predicted vs MuJoCo tool-site coordinates with a **0.1 mm model-consistency** tolerance. This is **not** the achieved motion-control accuracy.

## 4. Joint command slew and simulation behavior

```text
q_cmd_next[i] = q_cmd[i] + clamp(q_target[i] - q_cmd[i], -rate[i]*step, rate[i]*step)
```

The implementation includes nominal per-joint slew factors of `(1.6, 1.4, 1.6, 2.0)` and uses a conservative `0.009` grasp-mode step argument. The physics engine then integrates actuator and contact dynamics separately. Requested height changes at **35 mm/s**; gripper closes at **14 mm/s** and opens at **9 mm/s** (targets, not guaranteed achieved positions).

The cube itself is a 46 mm sided, 10 g free body, with 7 joint position coordinates and 6 joint velocity coordinates. Its hold is contact-based, not scripted attachment.

## Sources and scope

The equations and thresholds here reflect the project source files (`phantomarm/controller.py`, `phantomarm/kinematics.py`, `phantomarm/motion.py`, `sketch/sketch.ino`, and MJCF assets). For upstream hardware and simulator specifications, see the [References & license](../README.md#references--license) section in the main README. This appendix is optional and does not change source-code behavior.
