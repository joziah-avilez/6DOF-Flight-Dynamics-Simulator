#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

struct StateDerivative
{
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d angularVelocity;
};