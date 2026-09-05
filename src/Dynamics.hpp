#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

#include "State.hpp"
#include "StateDerivative.hpp"
#include "Vehicle.hpp"
#include "Environment.hpp"

StateDerivative derivatives(const State& state, const Vehicle& vehicle, const Environment& environment)
{
    // Retrieve environmental parameters
    const double gravity = environment.gravity;
    const double airDensity = environment.airDensity;

    // Calculate air-relative velocity
    Eigen::Vector3d airRelativeVelocity = state.velocity - environment.windVelocity;
    Eigen::Vector3d airRelativeVelocityBody = state.orientation.inverse() * airRelativeVelocity;

    // Calculate axial velocity and drag
    double axialVelocity = airRelativeVelocityBody.z();
    double axialDrag = -0.5 * airDensity * vehicle.axialDragCoefficient * vehicle.axialArea * axialVelocity * std::abs(axialVelocity);
    Eigen::Vector3d axialDragBody(0.0, 0.0, axialDrag);

    // Calculate lateral velocity and drag
    Eigen::Vector3d lateralVelocity(airRelativeVelocityBody.x(), airRelativeVelocityBody.y(), 0.0);
    double lateralSpeed = lateralVelocity.norm();
    Eigen::Vector3d lateralDragBody = -0.5 * airDensity * vehicle.lateralDragCoefficient * vehicle.lateralArea * lateralSpeed * lateralVelocity;

    // Calculate the quaternion derivative
    Eigen::Quaterniond omegaQuat(0.0, state.angularVelocity.x(), state.angularVelocity.y(), state.angularVelocity.z());
    Eigen::Quaterniond qDot = state.orientation * omegaQuat;
    qDot.coeffs() *= 0.5;

    // Calculate angular momentum and angular acceleration
    Eigen::Vector3d angularMomentum = vehicle.inertia * state.angularVelocity;
    Eigen::Vector3d angularAcceleration = vehicle.inertia.inverse() * (vehicle.torque - state.angularVelocity.cross(angularMomentum));

    // Calculate forces
    Eigen::Vector3d gravityForce(0, 0, -vehicle.mass * gravity);
    Eigen::Vector3d thrustBody(0, 0, vehicle.thrust);
    Eigen::Vector3d thrustForce = state.orientation * thrustBody;
    Eigen::Vector3d aerodynamicForceBody = axialDragBody + lateralDragBody;
    Eigen::Vector3d aerodynamicForce = state.orientation * aerodynamicForceBody;

    // Calculate net force
    Eigen::Vector3d netForce = thrustForce + gravityForce + aerodynamicForce;

    // Populate the StateDerivative structure
    StateDerivative dxdt;

    dxdt.position = state.velocity;
    dxdt.velocity = netForce / vehicle.mass;
    dxdt.orientation = qDot;
    dxdt.angularVelocity = angularAcceleration;

    return dxdt;
}