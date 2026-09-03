# 6-DOF Flight Dynamics Simulator

A C++ rigid-body flight dynamics simulator developed to explore numerical simulation, vehicle dynamics, and six-degree-of-freedom (6-DOF) motion.

The simulator numerically integrates the translational and rotational equations of motion of a simplified flight vehicle using a fourth-order Runge-Kutta (RK4) method.

## Current Features

- 3D position and velocity dynamics
- Quaternion-based vehicle orientation
- Body-frame angular velocity
- 6-DOF rigid-body equations of motion
- Gravity
- Thrust transformed from body to world coordinates
- Velocity-dependent aerodynamic drag
- Full inertia tensor representation
- Applied body-frame torque
- Torque-free rotational coupling
- RK4 time integration
- Quaternion normalization
- Eigen-based vector, matrix, and quaternion operations

## Numerical Validation

The current implementation has been tested against several analytical and physical consistency checks:

- Constant-gravity trajectory compared with the analytical solution
- Forward Euler convergence with decreasing timestep
- RK4 constant-acceleration trajectory
- Quaternion rotation compared with analytical angular motion
- Constant-torque angular acceleration compared with the analytical solution
- Conservation of rotational kinetic energy during torque-free motion
- Conservation of angular momentum magnitude during torque-free motion

## Dependencies

- C++17
- Eigen 3

## Build

Example using Clang and a Homebrew installation of Eigen:

```bash
clang++ -std=c++17 -I/opt/homebrew/opt/eigen/include/eigen3 src/main.cpp -o flight_sim
