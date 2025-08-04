#pragma once
#include <fstream>
#include <iomanip>
#include <filesystem>
#include <ctime>
#include <vector>
#include <eigen3/Eigen/Dense>

#include "SafeQueue.hpp"

void robot_dump(const std::vector<queue_package>&, bool, int, int);