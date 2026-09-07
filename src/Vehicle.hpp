#pragma once
#include <Eigen/Dense>

struct Vehicle
{
    double mass;
    double thrust;
    double burnTime;
    double axialArea;
    double lateralArea;
    double axialDragCoefficient;
    double lateralDragCoefficient;
    double pitchYawDampingCoefficient;
    double rollDampingCoefficient;
    double referenceLength;
    double referenceDiameter;
    Eigen::Matrix3d inertia;
    Eigen::Vector3d torque;
    Eigen::Vector3d centerOfPressure;
};