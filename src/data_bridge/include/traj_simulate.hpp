#pragma once

#include <vector>
#include <eigen3/Eigen/Dense>
#include <iostream>

struct trajectory {
    std::vector<Eigen::Vector3d> position;
    std::vector<Eigen::Vector3d> velocity;
    std::vector<Eigen::Vector3d> acceleration;
};

struct trajectory_6d {
    std::vector<Eigen::Matrix<double, 6, 1>> position;
    std::vector<Eigen::Matrix<double, 6, 1>> velocity;
    std::vector<Eigen::Matrix<double, 6, 1>> acceleration;
};
// Simulates a mass-spring-damper system using RK4 integration.
// 
// Parameters:
// - x0_vec: Initial position vector
// - k: Spring constant
// - c: Damping coefficient
// - m: Mass vector (one for each dimension)
//
// Returns:
// - A set of vectors containing position, velocity, and acceleration over time
trajectory spring_simulate(
    const Eigen::Vector3d& x0_vec,
    double k,
    double c,
    const Eigen::Matrix3d& m,
    const Eigen::Vector3d& f_ext,
    const std::function<Eigen::Vector3d(double)>& set_point_func);

trajectory_6d spring_simulate_6d(
    const Eigen::Matrix<double, 6, 1>& x0_vec,         // x y z roll pitch yaw
    const Eigen::Matrix<double, 6, 6>& stiffness, 
    const Eigen::Matrix<double, 6, 6>& damping, 
    const Eigen::Matrix<double, 6, 6>& m,               // 6x6 mass/inertia matrix
    const Eigen::Matrix<double, 6, 1>& f_ext,           // external force + torque
    const std::function<Eigen::Matrix<double, 6, 1>(double)>& set_point_func);

trajectory sin_simulate(const Eigen::Vector3d& x0_vec, const Eigen::Vector3d& v0_vec);