#pragma once

#include "State.hpp"
#include "StateDerivative.hpp"
#include "Dynamics.hpp"
#include "Vehicle.hpp"
#include "Environment.hpp"

State eulerStep(const State& state, const Vehicle& vehicle, const Environment& environment, double dt)
{
    StateDerivative dxdt = derivatives(state, vehicle, environment);

    State nextState;

    nextState.position = state.position + dt * dxdt.position;
    nextState.velocity = state.velocity + dt * dxdt.velocity;
    nextState.orientation = state.orientation;

    return nextState;
}

State rk4Step(const State& state, const Vehicle& vehicle, const Environment& environment, double dt)
{
    StateDerivative k1 = derivatives(state, vehicle, environment);

    State state2;
    state2.position = state.position + 0.5 * dt * k1.position;
    state2.velocity = state.velocity + 0.5 * dt * k1.velocity;
    state2.orientation.coeffs() = state.orientation.coeffs() + 0.5 * dt * k1.orientation.coeffs();
    state2.orientation.normalize();
    state2.angularVelocity = state.angularVelocity + 0.5 * dt * k1.angularVelocity;
    StateDerivative k2 = derivatives(state2, vehicle, environment);

    State state3;
    state3.position = state.position + 0.5 * dt * k2.position;
    state3.velocity = state.velocity + 0.5 * dt * k2.velocity;
    state3.orientation.coeffs() = state.orientation.coeffs() + 0.5 * dt * k2.orientation.coeffs();
    state3.orientation.normalize();
    state3.angularVelocity = state.angularVelocity + 0.5 * dt * k2.angularVelocity;
    StateDerivative k3 = derivatives(state3, vehicle, environment);

    State state4;
    state4.position = state.position + dt * k3.position;
    state4.velocity = state.velocity + dt * k3.velocity;
    state4.orientation.coeffs() = state.orientation.coeffs() + dt * k3.orientation.coeffs();
    state4.orientation.normalize();
    state4.angularVelocity = state.angularVelocity + dt * k3.angularVelocity;
    StateDerivative k4 = derivatives(state4, vehicle, environment);

    State nextState;

    nextState.position = state.position + (dt / 6.0) * (k1.position + 2.0 * k2.position + 2.0 * k3.position + k4.position);
    nextState.velocity = state.velocity + (dt / 6.0) * (k1.velocity + 2.0 * k2.velocity + 2.0 * k3.velocity + k4.velocity);
    nextState.angularVelocity = state.angularVelocity + (dt / 6.0) * (k1.angularVelocity + 2.0 * k2.angularVelocity + 2.0 * k3.angularVelocity + k4.angularVelocity);
    nextState.orientation.coeffs() = state.orientation.coeffs() + (dt / 6.0) * (k1.orientation.coeffs() + 2.0 * k2.orientation.coeffs() + 2.0 * k3.orientation.coeffs() + k4.orientation.coeffs());
    nextState.orientation.normalize();

    return nextState;
}