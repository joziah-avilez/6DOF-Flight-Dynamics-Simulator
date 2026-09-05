#pragma once
#include <Eigen/Dense>

struct Vehicle
{
    double mass;
    double thrust;
    double axialArea;
    double lateralArea;
    double axialDragCoefficient;
    double lateralDragCoefficient;
    Eigen::Matrix3d inertia;
    Eigen::Vector3d torque;
};