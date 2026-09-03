#pragma once
#include <Eigen/Dense>

struct Vehicle
{
    double mass;
    double thrust;
    double dragCoefficient;
    double referenceArea;
    Eigen::Matrix3d inertia;
    Eigen::Vector3d torque;
};