#include "butterworth.hpp"

// Filter order
constexpr int order = 2;

// Filter coefficients from Python, 2nd order with 1000 sampling rate and 10hz cutoff
const std::array<double, 3> b = {0.00094469, 0.00188938, 0.00094469};
const std::array<double, 3> a = {1.0, -1.91119707, 0.91497583};

// Buffers (history)
static std::array<Eigen::Matrix<double, 6, 1>, 3> x_history = {};
static std::array<Eigen::Matrix<double, 6, 1>, 3> y_history = {};

Eigen::Matrix<double, 6, 1> butterworth_filter(const Eigen::Matrix<double, 6, 1>& input) {
    // Shift history
    x_history[2] = x_history[1];
    x_history[1] = x_history[0];
    x_history[0] = input;

    y_history[2] = y_history[1];
    y_history[1] = y_history[0];

    // Apply difference equation (per element)
    Eigen::Matrix<double, 6, 1> output = 
        b[0] * x_history[0] +
        b[1] * x_history[1] +
        b[2] * x_history[2] -
        a[1] * y_history[1] -
        a[2] * y_history[2];

    y_history[0] = output;

    return output;
}