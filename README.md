# Four-Wheeled Inverted Pendulum — MATLAB & Simulink

A MATLAB/Simulink control-system project for modeling, stabilization, disturbance rejection, nonlinear validation, and 3D visualization of a four-wheeled inverted pendulum.

> **Note:** This is a new portfolio implementation inspired by the project concept, not an exact reconstruction of the original bachelor's thesis results.

## Project Workflow

**Physical system → Nonlinear model → Linearization → State-space → Controllability → LQR → MATLAB simulation → Disturbance rejection → Simulink → Nonlinear validation → 3D visualization → Performance analysis**

## Objectives

- Develop a mathematical model of the inverted pendulum.
- Linearize the nonlinear dynamics around the upright equilibrium.
- Build and analyze a state-space model.
- Verify controllability.
- Design an LQR state-feedback controller.
- Stabilize the system from an initial 5° tilt.
- Test rejection of a 2 N external disturbance.
- Implement the controller in Simulink.
- Validate the controller against the nonlinear model.
- Quantitatively evaluate performance.
- Visualize the system in 3D.

## System Model

The physical visualization represents a four-wheeled mobile base with an inverted pendulum. The dynamics use an equivalent rigid mobile-base/cart-pole abstraction rather than individual wheel, tire, suspension, and motor dynamics.

### Parameters

| Parameter | Value |
|---|---:|
| Base mass, M | 2.500 kg |
| Pendulum mass, m | 0.500 kg |
| Pivot-to-CoM distance, l | 0.375 m |
| Pendulum inertia, I | 0.0150 kg·m² |
| Gravity, g | 9.81 m/s² |
| Initial angle | 5° |
| Disturbance | 2 N |
| Disturbance interval | 2.0–2.2 s |

The visualization uses a physical pendulum length of 0.50 m, while the dynamics use a 0.375 m pivot-to-CoM distance.

## Nonlinear Dynamics

M x_ddot + m l theta_ddot cos(theta) - m l theta_dot² sin(theta) = F

(I + m l²) theta_ddot + m l x_ddot cos(theta) - m g l sin(theta) = 0

## Linearization and State-Space Model

The model is linearized around:

theta = 0, theta_dot = 0, x_dot = 0

using:

sin(theta) ≈ theta,   cos(theta) ≈ 1

State vector:

X = [x, x_dot, theta, theta_dot]^T

State-space form:

X_dot = A X + B F

A =

[ 0   1        0        0
  0   0     -1.9362     0
  0   0        0        1
  0   0     25.8158     0 ]

B =

[ 0
  0.4789
  0
 -1.0526 ]

The open-loop system contains an unstable eigenvalue of approximately +5.0809.

## Controllability

The controllability matrix is:

C = [B  AB  A²B  A³B]

with:

rank(C) = 4

Therefore, all four modeled states are controllable through the selected input.

## LQR Controller

State feedback:

F = -KX

with:

Q = diag([1, 1, 100, 10])
R = 1

LQR gain:

K = [-1.0000, -2.9233, -61.5377, -12.5668]

The closed-loop eigenvalues have negative real parts, indicating stable linearized closed-loop behavior.

## MATLAB Simulation Results

| Metric | Result |
|---|---:|
| Initial angle | 5.00° |
| Maximum |angle| | 5.00° |
| Final angle | 0.211° |
| Maximum |LQR force| | 5.370 N |
| Maximum |chassis displacement| | 0.293 m |
| Maximum |chassis velocity| | 0.279 m/s |
| ±0.5° settling time | 2.655 s |

## Disturbance Rejection

A 2 N external disturbance was applied from 2.0 to 2.2 s.

Key results:

- Maximum angle deviation during disturbance: approximately 1.64°.
- Final angle: approximately 0.21°.
- Maximum LQR force: approximately 5.37 N.
- Recovery to ±0.5°: approximately 0.45 s after disturbance removal.

The tighter ±0.2° criterion was not continuously reached before the end of the 5 s simulation.

## Simulink Implementation

The LQR controller and plant were implemented in Simulink and compared with the MATLAB simulation.

The state trajectories and LQR control signal agree very closely. The small disturbance-signal discrepancy occurs at the edges of the discontinuous step during interpolation in post-processing.

## Nonlinear Validation

After designing the controller using the linearized model, its behavior was validated against the original nonlinear dynamics.

Results:

- Initial angle: 5.000°
- Minimum angle: approximately −1.845°
- Final angle: approximately 0.212°
- Maximum control force: approximately 5.37 N
- Maximum chassis position: approximately 0.294 m
- Maximum chassis velocity: approximately 0.280 m/s
- No NaN/Inf values

> **Interview explanation:** “After designing the controller using the linearized model, I validated its behavior against the original nonlinear dynamics to verify that the controller remains effective beyond the linear approximation.”

## 3D Visualization

The project includes a 3D animation showing:

- Four-wheel chassis
- Pendulum motion
- Chassis movement
- LQR stabilization
- Disturbance response
- Recovery behavior

> **Portfolio explanation:** “The 3D visualization provides an intuitive representation of the controller’s dynamic response and complements the quantitative MATLAB and Simulink results.”

## Project Structure

```text
four-wheeled-inverted-pendulum/
├── data/
├── docs/
├── matlab/
├── results/
├── simulink/
└── .gitignore
```

## Tools

- MATLAB
- Simulink
- LQR / Control System Toolbox
- MATLAB ODE solvers
- State-space modeling
- Numerical simulation
- 3D MATLAB visualization
- Git / GitHub

## Key Engineering Takeaways

1. The inverted pendulum is open-loop unstable and requires feedback control.
2. Linearization provides a practical model for controller design around the operating point.
3. Controllability verifies that the modeled states can be influenced by the input.
4. LQR provides a systematic balance between state regulation and control effort.
5. Disturbance testing evaluates controller recovery behavior.
6. Simulink provides an independent block-diagram implementation.
7. Nonlinear validation checks behavior beyond the linear approximation.
8. Quantitative metrics make controller performance measurable.

## Limitations

This is a simulation and control-design study. It does not model individual wheel motors, tire-ground contact, wheel slip, detailed suspension dynamics, motor electrical dynamics, sensor noise/delay, or full real-world friction.

## Future Extensions

- Individual wheel/motor dynamics
- ROS 2 integration
- Gazebo simulation
- IMU/encoder sensor modeling
- State estimation and sensor fusion
- Computer vision
- Model Predictive Control
- Hardware implementation

## Author

**Naresh Tuppathi**  
M.Sc. Mechatronics and Cyber-Physical Systems  
Technische Hochschule Deggendorf, Germany

## Portfolio Summary

> Designed and simulated a four-wheeled inverted pendulum control system using MATLAB and Simulink. Developed nonlinear and linearized state-space models, verified controllability, designed an LQR state-feedback controller, evaluated disturbance rejection, validated the controller against nonlinear dynamics, and created a 3D visualization of the system response.
