#pragma once

#include <vector>
#include <eigen3/Eigen/Dense>

struct trajectory {
    std::vector<Eigen::Vector3d> position;
    std::vector<Eigen::Vector3d> velocity;
    std::vector<Eigen::Vector3d> acceleration;
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
trajectory spring_simulate(const Eigen::Vector3d& x0_vec, double k, double c, const Eigen::Vector3d& m, const Eigen::Vector3d& f_ext);
