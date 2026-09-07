#pragma once
#include <cmath>
#include <Eigen/Dense>

struct Environment
{
    double gravity;
    double seaLevelAirDensity;
    double scaleHeight;
    Eigen::Vector3d windVelocity;

    double airDensity(double altitude) const
    {
        return seaLevelAirDensity * std::exp(-altitude / scaleHeight);
    }
};