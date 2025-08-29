#include "data_dumper.hpp"

void robot_dump(const std::vector<queue_package> & data, bool full, int MAX_BUFFER_SIZE, int dump_index) {
    //setup output directory
    time_t now = time(NULL);
    struct tm *timenow = localtime(&now);
    char buffer[80];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d_%H-%M-%S", timenow);
    std::string output = "data/data_" + std::string(buffer);
    std::filesystem::path output_dir = output;
    std::filesystem::create_directory(output_dir);

    // Open file streams for each member
    std::ofstream desired_accel_file(output_dir / "desired_accel.csv");
    std::ofstream actual_wrench_file(output_dir / "actual_wrench.csv");
    std::ofstream ergodic_accel_file(output_dir / "ergodic_accel.csv");
    std::ofstream orientation_error_file(output_dir / "orientation_error.csv");
    std::ofstream translation_file(output_dir / "translation.csv");
    std::ofstream translation_d_file(output_dir / "translation_d.csv");
    std::ofstream velocity_file(output_dir / "velocity.csv");
    std::ofstream accel_file(output_dir / "accel.csv");
    std::ofstream torques_d_file(output_dir / "torques_d.csv");
    std::ofstream torques_o_file(output_dir / "torques_o.csv");
    std::ofstream torques_c_file(output_dir / "torques_c.csv");
    std::ofstream torques_g_file(output_dir / "torques_g.csv");
    std::ofstream torques_f_file(output_dir / "torques_f.csv");
    std::ofstream desired_joint_accel_file(output_dir / "joints_accel_d.csv");
    std::ofstream joint_vel_file(output_dir / "joints_vel.csv");

    // Set precision for floats
    desired_accel_file << std::fixed << std::setprecision(4);
    actual_wrench_file << std::fixed << std::setprecision(4);
    ergodic_accel_file << std::fixed << std::setprecision(4);
    orientation_error_file << std::fixed << std::setprecision(4);
    translation_file << std::fixed << std::setprecision(4);
    translation_d_file << std::fixed << std::setprecision(4);
    velocity_file << std::fixed << std::setprecision(4);
    accel_file << std::fixed << std::setprecision(4);
    torques_d_file << std::fixed << std::setprecision(4);
    torques_o_file << std::fixed << std::setprecision(4);
    torques_c_file << std::fixed << std::setprecision(4);
    torques_g_file << std::fixed << std::setprecision(4);
    torques_f_file << std::fixed << std::setprecision(4);
    desired_joint_accel_file << std::fixed << std::setprecision(4);
    joint_vel_file << std::fixed << std::setprecision(4);

    // Iterate through the data
    size_t count = full ? MAX_BUFFER_SIZE : dump_index;
    for (size_t i = 0; i < count; ++i) {
        size_t index = full ? (dump_index + i) % MAX_BUFFER_SIZE : i;
        const queue_package& moment = data[index];

        for (int i = 0; i < moment.desired_accel.size(); ++i)
            desired_accel_file << moment.desired_accel(i) << (i < moment.desired_accel.size() - 1 ? "," : "\n");

        for (int i = 0; i < moment.actual_wrench.size(); ++i)
            actual_wrench_file << moment.actual_wrench(i) << (i < moment.actual_wrench.size() - 1 ? "," : "\n");

        for (int i = 0; i < moment.ergodic_accel.size(); ++i)
            ergodic_accel_file << moment.ergodic_accel(i) << (i < moment.ergodic_accel.size() - 1 ? "," : "\n");

        for (int i = 0; i < moment.orientation_error.size(); ++i)
            orientation_error_file << moment.orientation_error(i) << (i < moment.orientation_error.size() - 1 ? "," : "\n");

        for (int i = 0; i < moment.translation.size(); ++i)
            translation_file << moment.translation(i) << (i < moment.translation.size() - 1 ? "," : "\n");

        for (int i = 0; i < moment.translation_d.size(); ++i)
            translation_d_file << moment.translation_d(i) << (i < moment.translation_d.size() - 1 ? "," : "\n");

        for (int i = 0; i < moment.velocity.size(); ++i)
            velocity_file << moment.velocity(i) << (i < moment.velocity.size() - 1 ? "," : "\n");
        
        for (int i = 0; i < moment.accel.size(); ++i)
            accel_file << moment.accel(i) << (i < moment.accel.size() - 1 ? "," : "\n");

        for (int i = 0; i < moment.torques_d.size(); ++i)
            torques_d_file << moment.torques_d(i) << (i < moment.torques_d.size() - 1 ? "," : "\n");

        for (int i = 0; i < moment.torques_o.size(); ++i)
            torques_o_file << moment.torques_o(i) << (i < moment.torques_o.size() - 1 ? "," : "\n");

        for (int i = 0; i < moment.torques_c.size(); ++i)
            torques_c_file << moment.torques_c(i) << (i < moment.torques_c.size() - 1 ? "," : "\n");

        for (int i = 0; i < moment.torques_g.size(); ++i)
            torques_g_file << moment.torques_g(i) << (i < moment.torques_g.size() - 1 ? "," : "\n");
        
        for (int i = 0; i < moment.torques_f.size(); ++i)
            torques_f_file << moment.torques_f(i) << (i < moment.torques_f.size() - 1 ? "," : "\n");
        
        for (int i = 0; i < moment.ddq_d.size(); ++i)
            desired_joint_accel_file << moment.ddq_d(i) << (i < moment.ddq_d.size() - 1 ? "," : "\n");
        
        for (int i = 0; i < moment.dq.size(); ++i)
            joint_vel_file << moment.dq(i) << (i < moment.dq.size() - 1 ? "," : "\n");
    }
}
