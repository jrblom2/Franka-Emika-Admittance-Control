#include "traj_simulate.hpp"

trajectory spring_simulate(
    const Eigen::Vector3d& x0_vec,
    double k,
    double c,
    const Eigen::Matrix3d& m,
    const Eigen::Vector3d& f_ext,
    const std::function<Eigen::Vector3d(double)>& set_point_func)
{
    double dt = 0.001;
    double T = 20.0;
    int n_steps = static_cast<int>(T / dt);

    std::vector<Eigen::Vector3d> positions(n_steps);
    std::vector<Eigen::Vector3d> velocities(n_steps);
    std::vector<Eigen::Vector3d> accelerations(n_steps);

    Eigen::Vector3d x = x0_vec;
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    Eigen::Vector3d a = Eigen::Vector3d::Zero();

    positions[0] = x;
    velocities[0] = v;
    accelerations[0] = a;

    for (int i = 1; i < n_steps; ++i) {
        double t = i * dt;

        // Compute acceleration at current state
        Eigen::Vector3d set_point = set_point_func(t);
        Eigen::Vector3d a = m.inverse() * (f_ext - (c * v) - (k * (x - set_point)));

        // Euler integration step
        v += dt * a;
        x += dt * v;

        // Store results
        positions[i] = x;
        velocities[i] = v;
        accelerations[i] = a;
    }

    return {positions, velocities, accelerations};
}

trajectory_6d spring_simulate_6d(
    const Eigen::Matrix<double, 6, 1>& x0_vec,
    const Eigen::Matrix<double, 6, 6>& stiffness,
    const Eigen::Matrix<double, 6, 6>& damping,
    const Eigen::Matrix<double, 6, 6>& m,
    const std::function<Eigen::Matrix<double, 6, 1>(double)>& f_ext,
    const std::function<Eigen::Matrix<double, 6, 1>(double)>& set_point)
{
    double dt = 0.001;
    double T = 20.0;
    int n_steps = static_cast<int>(T / dt);

    std::vector<Eigen::Matrix<double, 6, 1>> positions(n_steps);
    std::vector<Eigen::Matrix<double, 6, 1>> velocities(n_steps);
    std::vector<Eigen::Matrix<double, 6, 1>> accelerations(n_steps);

    Eigen::Matrix<double, 6, 1> x = x0_vec;
    Eigen::Matrix<double, 6, 1> v = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> a = Eigen::Matrix<double, 6, 1>::Zero();

    positions[0] = x;
    velocities[0] = v;
    accelerations[0] = a;

    for (int i = 1; i < n_steps; ++i) {
        double t = i * dt;

        // Compute acceleration at current state
        a = m.inverse() * (f_ext(t) - (damping * v) - (stiffness * (x - set_point(t))));
        // Euler integration step
        v += dt * a;
        x += dt * v;

        // Store results
        positions[i] = x;
        velocities[i] = v;
        accelerations[i] = a;
    }

    return {positions, velocities, accelerations};
}

trajectory sin_simulate(const Eigen::Vector3d& x0_vec, const Eigen::Vector3d& v0_vec) {
    double dt = 0.001;
    double T = 20.0;
    int n_steps = static_cast<int>(T / dt);

    std::vector<Eigen::Vector3d> positions(n_steps);
    std::vector<Eigen::Vector3d> velocities(n_steps);
    std::vector<Eigen::Vector3d> accelerations(n_steps);

    Eigen::Vector3d x = x0_vec;
    Eigen::Vector3d v = v0_vec;
    Eigen::Vector3d a = Eigen::Vector3d::Zero();

    positions[0] = x;
    velocities[0] = v;
    accelerations[0] = a;

    auto dv_dt = [](int fullCount) {
        Eigen::Vector3d acc = Eigen::Vector3d::Zero();
        acc(1) = 0.25 * std::cos(fullCount * 2.0 * M_PI / 4000.0); // only y-direction
        return acc;
    };

    for (int i = 1; i < n_steps; ++i) {
        int fullCount = i;

        // RK4 integration for x and v (with known a as function of time/index)
        Eigen::Vector3d k1_v = dv_dt(fullCount);
        Eigen::Vector3d k1_x = v;

        Eigen::Vector3d k2_v = dv_dt(fullCount + 0.5);
        Eigen::Vector3d k2_x = v + 0.5 * dt * k1_v;

        Eigen::Vector3d k3_v = dv_dt(fullCount + 0.5);
        Eigen::Vector3d k3_x = v + 0.5 * dt * k2_v;

        Eigen::Vector3d k4_v = dv_dt(fullCount + 1);
        Eigen::Vector3d k4_x = v + dt * k3_v;

        x += (dt / 6.0) * (k1_x + 2.0 * k2_x + 2.0 * k3_x + k4_x);
        v += (dt / 6.0) * (k1_v + 2.0 * k2_v + 2.0 * k3_v + k4_v);

        Eigen::Vector3d a_current = dv_dt(fullCount);

        positions[i] = x;
        velocities[i] = v;
        accelerations[i] = a_current;
    }

    return {positions, velocities, accelerations};
}

