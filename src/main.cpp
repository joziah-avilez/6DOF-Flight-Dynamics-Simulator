#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "State.hpp"
#include "Dynamics.hpp"
#include "Integrator.hpp"
#include "Vehicle.hpp"
#include "Environment.hpp"

int main() 
{
    State state;
    state.position = Eigen::Vector3d(0.0, 0.0, 100.0);
    state.velocity = Eigen::Vector3d(20.0, 0.0, 0.0);
    state.orientation = Eigen::Quaterniond::Identity();
    state.angularVelocity = Eigen::Vector3d::Zero();

    Environment environment;
    environment.gravity = 9.81;
    environment.airDensity = 1.225;
    environment.windVelocity = Eigen::Vector3d::Zero();

    Vehicle vehicle;
    vehicle.mass = 1000.0;
    vehicle.thrust = 0.0;
    vehicle.axialArea = M_PI * 0.5 * 0.5; // Assuming radius of 0.5 m for the axial area
    vehicle.lateralArea = 5.0; // Assuming a length of 5 m and width of 1 m for the lateral area
    vehicle.axialDragCoefficient = 0.5;
    vehicle.lateralDragCoefficient = 1.0;
    vehicle.inertia << 500.0, 0.0, 0.0,
                        0.0, 500.0, 0.0,
                        0.0, 0.0, 100.0;
    vehicle.centerOfPressure = Eigen::Vector3d(0.0, 0.0, -1.0); // Assuming the center of pressure is 1 m behind the center of mass 
    vehicle.angularDampingCoefficient = 100.0; // N*m*s/rad
    vehicle.torque = Eigen::Vector3d::Zero();

    double t = 0.0;
    double tFinal = 2.0;
    double dt = 0.1;

    while (t < tFinal)
    {
        state = rk4Step(state, vehicle, environment, dt);
        Eigen::Vector3d thrustDirection = state.orientation * Eigen::Vector3d::UnitZ();
        double rotationalEnergy = 0.5 * state.angularVelocity.transpose() * vehicle.inertia * state.angularVelocity;
        double angularMomentumMagnitude = (vehicle.inertia * state.angularVelocity).norm();
        t += dt;

        std::cout << "\nt = " << t << " s\n"
                  << "  Position [m]          : " << state.position.transpose() << "\n"
                  << "  Velocity [m/s]        : " << state.velocity.transpose() << "\n"
                  << "  Thrust direction      : " << thrustDirection.transpose() << "\n"
                  << "  Angular velocity [rad/s]: " << state.angularVelocity.transpose() << "\n"
                  << "  Rotational energy [J]: " << rotationalEnergy << "\n"
                  << "  Angular momentum [kg*m^2/s]: " << angularMomentumMagnitude << "\n";
    }


    return 0;
}