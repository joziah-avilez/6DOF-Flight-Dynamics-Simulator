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
    const double gravity = environment.gravity;
    const double airDensity = environment.airDensity;

    Eigen::Quaterniond omegaQuat(0.0, state.angularVelocity.x(), state.angularVelocity.y(), state.angularVelocity.z());
    Eigen::Quaterniond qDot = state.orientation * omegaQuat;
    qDot.coeffs() *= 0.5;

    Eigen::Vector3d angularMomentum = vehicle.inertia * state.angularVelocity;
    Eigen::Vector3d angularAcceleration = vehicle.inertia.inverse() * (vehicle.torque - state.angularVelocity.cross(angularMomentum));

    Eigen::Vector3d gravityForce(0, 0, -vehicle.mass * gravity);
    Eigen::Vector3d thrustBody(0, 0, vehicle.thrust);
    Eigen::Vector3d thrustForce = state.orientation * thrustBody;
    Eigen::Vector3d dragForce = -0.5 * airDensity * vehicle.dragCoefficient * vehicle.referenceArea * state.velocity.norm() * state.velocity;

    Eigen::Vector3d netForce = thrustForce + gravityForce + dragForce;

    StateDerivative dxdt;

    dxdt.position = state.velocity;
    dxdt.velocity = netForce / vehicle.mass;
    dxdt.orientation = qDot;
    dxdt.angularVelocity = angularAcceleration;

    return dxdt;
}