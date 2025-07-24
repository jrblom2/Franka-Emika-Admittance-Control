#pragma once
#include <eigen3/Eigen/Dense>
#include <array>

Eigen::Matrix<double, 6, 1> butterworth_filter(const Eigen::Matrix<double, 6, 1>& input);