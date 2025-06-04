#pragma once

#include <vector>
#include <eigen3/Eigen/Dense>

// Simulates a mass-spring-damper system using RK4 integration.
// 
// Parameters:
// - x0_vec: Initial position vector
// - k: Spring constant
// - c: Damping coefficient
// - m: Mass vector (one for each dimension)
//
// Returns:
// - A vector of position vectors over time
std::vector<Eigen::Vector3d> simulate(const Eigen::Vector3d& x0_vec, double k, double c, const Eigen::Vector3d& m);
