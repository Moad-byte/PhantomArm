# From A to Z — understanding the engineering

## 1. The problem

Physical robotic arms are expensive, require motor safety safeguards and can damage components if commanded outside limits. PhantomArm is a low-cost *teleoperation learning system*: a real sensor-operated control interface commands a virtual robot in a physics environment. The project demonstrates the interface but **does not claim closed-loop control of a real robot**.

## 2. What the sensor actually measures

The MPU6050 provides three-axis specific force and angular velocity, not absolute XYZ hand position. Acceleration-based gravity tilt estimates are noisy during fast translational motion. Integration of gyro rates drifts over time. A complementary filter uses gyroscope for short-term responsiveness and accelerometer tilt to correct longer-term bias.

`roll = 0.98*(previous_roll + gyro_x_deg_s * dt) + 0.02*roll_accel`

`pitch = 0.98*(previous_pitch + gyro_y_deg_s * dt) + 0.02*pitch_accel`

Angles are referenced to a neutral pose on the laptop, then smoothed again with an EMA.

## 3. From tilt to end-effector target

- Pitch controls forward X around `0.38m ±0.11m`.
- Roll controls left-right Y around `0 ±0.17m`.
- W and S keyboard keys control height Z from `0.30m` to `0.52m`.
- GP15 button toggles finger opening.
- Tilt clipped at ±35°, not extrapolated into unsafe workspace.

These are **position commands**, not reconstructed controller hand coordinates.

## 4. Analytic inverse kinematics (IK)

For horizontal radius `r=sqrt(x²+y²)`, compute base yaw `atan2(y,x)`. Subtract wrist length 0.1 m and shoulder height 0.22 m:

`u = r - 0.10; v = z - 0.22`

For upper arm `L1=0.30`, forearm `L2=0.26`:

`D=(u²+v²-L1²-L2²)/(2 L1 L2)`.

If D is outside [-1,1], reject target as unreachable. Otherwise `beta=acos(D)`; `alpha=atan2(v,u)-atan2(L2 sin(beta),L1+L2 cos(beta))`. MuJoCo joints have +Y rotation, so shoulder `-alpha`, elbow `-beta`, wrist `alpha+beta`, keeping tool horizontal.

## 5. Motor control in simulator

MJCF position actuators convert joint position targets into torques/forces. A slew-rate filter constrains the maximum requested angular change each simulation tick. MuJoCo steps the physics (timestep 0.005 seconds) in four substeps per nominal 20 ms software cycle.

## 6. Why testing matters

Our solver is pure Python, so mathematical validation does not need the simulation window. `verify_model` separately compares our forward kinematics with **actual MuJoCo geometry** across four target poses. That catches misconfigured joint axes or link lengths which conventional Python unit tests would not catch.

## 7. Threats to reliable behavior

- Sensor missing or invalid: firmware transmits `valid=0`, host HOLD.
- Duplicate/older sequence: reject sample; neutral reset after reconnection.
- No valid telemetry >400 ms: host freezes at current simulated joint positions.
- Keyboard E: latches ESTOP until user presses R.
- Out-of-reach target: reject; hold previous safe command.
- Physical board movement: no actual motor force is produced; virtual robot only.

## 8. What makes it different from StabilityGuard?

StabilityGuard detected motion hazards with embedded state logic; PhantomArm adds *human motion teleoperation, analytical kinematics, serial protocol design, physics-based 3D actuation, gripper operation, data replay and joint-space safeguards*. They can both appear on a CV because they demonstrate different competencies.
