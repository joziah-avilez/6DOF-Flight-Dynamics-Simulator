#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

#include "State.hpp"
#include "StateDerivative.hpp"
#include "Vehicle.hpp"
#include "Environment.hpp"

StateDerivative derivatives(const State& state, const Vehicle& vehicle, const Environment& environment, double time)
{
    // Retrieve environmental parameters
    const double gravity = environment.gravity;
    const double altitude = state.position.z();
    const double airDensity = environment.airDensity(altitude);

    // Calculate air-relative velocity
    Eigen::Vector3d airRelativeVelocity = state.velocity - environment.windVelocity;
    Eigen::Vector3d airRelativeVelocityBody = state.orientation.inverse() * airRelativeVelocity;

    // Calculate dynamic pressure
    double airSpeed = airRelativeVelocity.norm();
    double dynamicPressure = 0.5 * airDensity * airSpeed * airSpeed;

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

    // Calculate forces
    Eigen::Vector3d gravityForce(0, 0, -vehicle.mass * gravity);
    double currentThrust = (time < vehicle.burnTime) ? vehicle.thrust : 0.0;
    Eigen::Vector3d thrustBody(0, 0, currentThrust);
    Eigen::Vector3d thrustForce = state.orientation * thrustBody;
    Eigen::Vector3d aerodynamicForceBody = axialDragBody + lateralDragBody;
    Eigen::Vector3d aerodynamicForce = state.orientation * aerodynamicForceBody;

    // Calculate torques
    Eigen::Vector3d aerodynamicTorque = vehicle.centerOfPressure.cross(aerodynamicForceBody);
    Eigen::Vector3d dampingTorque = Eigen::Vector3d::Zero();
    // Apply damping torque based on angular velocity and dynamic pressure
    if (airSpeed > 1e-6)
    {
        double pitchYawFactor = dynamicPressure * vehicle.lateralArea * vehicle.referenceLength * vehicle.referenceLength / (2.0 * airSpeed);
        dampingTorque.x() = -pitchYawFactor * vehicle.pitchYawDampingCoefficient * state.angularVelocity.x();
        dampingTorque.y() = -pitchYawFactor * vehicle.pitchYawDampingCoefficient * state.angularVelocity.y();

        double rollFactor = dynamicPressure * vehicle.axialArea * vehicle.referenceDiameter * vehicle.referenceDiameter / (2.0 * airSpeed);
        dampingTorque.z() = -rollFactor * vehicle.rollDampingCoefficient * state.angularVelocity.z();
    }

    // Calculate angular momentum and angular acceleration
    Eigen::Vector3d totalTorque = vehicle.torque + aerodynamicTorque + dampingTorque;
    Eigen::Vector3d angularMomentum = vehicle.inertia * state.angularVelocity;
    Eigen::Vector3d angularAcceleration = vehicle.inertia.inverse() * (totalTorque - state.angularVelocity.cross(angularMomentum));

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