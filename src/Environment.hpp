#pragma once
#include <Eigen/Dense>

struct Environment
{
    double gravity;
    double airDensity;
    Eigen::Vector3d windVelocity;
};