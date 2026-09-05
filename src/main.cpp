#include <iostream>
#include <cmath>
#include <fstream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "State.hpp"
#include "Dynamics.hpp"
#include "Integrator.hpp"
#include "Vehicle.hpp"

#include "Environment.hpp"

void writeState(std::ofstream& outputFile, double t, const State& state)
{
    outputFile << t << ","
               << state.position.x() << ","
               << state.position.y() << ","
               << state.position.z() << ","
               << state.velocity.x() << ","
               << state.velocity.y() << ","
               << state.velocity.z() << ","
               << state.orientation.w() << ","
               << state.orientation.x() << ","
               << state.orientation.y() << ","
               << state.orientation.z() << ","
               << state.angularVelocity.x() << ","
               << state.angularVelocity.y() << ","
               << state.angularVelocity.z() << "\n";
}

int main() 
{
    State state;
    state.position = Eigen::Vector3d(0.0, 0.0, 100.0);
    state.velocity = Eigen::Vector3d(0.0, 0.0, 20.0);
    state.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(5.0 * M_PI / 180.0, Eigen::Vector3d::UnitY()));
    state.angularVelocity = Eigen::Vector3d(0.0, 0.0, 0.0);

    Environment environment;
    environment.gravity = 9.81;
    environment.airDensity = 1.225;
    environment.windVelocity = Eigen::Vector3d::Zero();

    Vehicle vehicle;
    vehicle.mass = 1000.0; // kg
    vehicle.thrust = 12000.0; // N
    vehicle.referenceLength = 5.0; // m
    vehicle.referenceDiameter = 1.0; // m
    vehicle.axialArea = M_PI * (0.5 * vehicle.referenceDiameter) * (0.5 * vehicle.referenceDiameter); // m^2
    vehicle.lateralArea = vehicle.referenceLength * vehicle.referenceDiameter; // m^2
    vehicle.axialDragCoefficient = 0.5;
    vehicle.lateralDragCoefficient = 1.0;
    vehicle.inertia << 500.0, 0.0, 0.0,
                        0.0, 500.0, 0.0,
                        0.0, 0.0, 100.0;
    vehicle.centerOfPressure = Eigen::Vector3d(0.0, 0.0, -1.0); // Assuming the center of pressure is 1 m behind the center of mass 
    vehicle.pitchYawDampingCoefficient = 0.1;
    vehicle.rollDampingCoefficient = 0.05;
    vehicle.torque = Eigen::Vector3d::Zero();

    double t = 0.0;
    double tFinal = 2.0;
    double dt = 0.1;

    std::ofstream outputFile("output/simulation.csv");
    if (!outputFile.is_open())
    {
        std::cerr << "Error: could not open output/simulation.csv. Run the simulator from the project root directory.\n";
        return 1;
    }
    outputFile << "time,"
               << "x,y,z,"
               << "vx,vy,vz,"
               << "qw,qx,qy,qz,"
               << "wx,wy,wz\n";

    writeState(outputFile, t, state);

    while (t < tFinal)
    {
        state = rk4Step(state, vehicle, environment, dt);
        t += dt;

        writeState(outputFile, t, state);

    }

    outputFile.close();

    std::cout << "Simulation complete.\n"
              << "Duration: " << tFinal << " s\n"
              << "Time step: " << dt << " s\n"
              << "Results written to output/simulation.csv\n";

    return 0;
}