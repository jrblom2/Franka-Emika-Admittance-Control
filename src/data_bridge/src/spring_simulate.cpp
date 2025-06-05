#include "spring_simulate.hpp"

std::vector<Eigen::Vector3d> simulate(const Eigen::Vector3d& x0_vec, double k, double c, const Eigen::Vector3d& m, const Eigen::Vector3d& f_ext) {
    // Parameters
    double v0 = 0.0;

    // Time setup
    double dt = 0.001;
    double T = 10.0;
    int n_steps = static_cast<int>(T / dt);

    // Storage for the result
    std::vector<Eigen::Vector3d> positions(n_steps);

    // Initial state
    Eigen::Vector3d x = x0_vec;
    Eigen::Vector3d v = Eigen::Vector3d::Constant(v0);
    positions[0] = x;

    // end effector speed is just v
    auto dx_dt = [](const Eigen::Vector3d& v) {
        return v;
    };

    // end effector acceleration is same equation as in admittance control
    auto dv_dt = [&](const Eigen::Vector3d& x, const Eigen::Vector3d& v) {
        Eigen::Vector3d f_adjusted = f_ext;
    
        // Invert external force component-wise where position > 0
        for (int i = 0; i < 3; ++i) {
            if (x[i] > 0) {
                f_adjusted[i] *= -1.0;
            }
        }
    
        Eigen::Vector3d dv = (-c * v - k * x + f_adjusted).array() / m.array();
        return dv;
    };

    // RK4 Integration
    for (int i = 1; i < n_steps; ++i) {
        Eigen::Vector3d k1_x = dx_dt(v);
        Eigen::Vector3d k1_v = dv_dt(x, v);

        Eigen::Vector3d k2_x = dx_dt(v + 0.5 * dt * k1_v);
        Eigen::Vector3d k2_v = dv_dt(x + 0.5 * dt * k1_x, v + 0.5 * dt * k1_v);

        Eigen::Vector3d k3_x = dx_dt(v + 0.5 * dt * k2_v);
        Eigen::Vector3d k3_v = dv_dt(x + 0.5 * dt * k2_x, v + 0.5 * dt * k2_v);

        Eigen::Vector3d k4_x = dx_dt(v + dt * k3_v);
        Eigen::Vector3d k4_v = dv_dt(x + dt * k3_x, v + dt * k3_v);

        x += (dt / 6.0) * (k1_x + 2 * k2_x + 2 * k3_x + k4_x);
        v += (dt / 6.0) * (k1_v + 2 * k2_v + 2 * k3_v + k4_v);

        positions[i] = x;
    }

    return positions;
}
