#include "minimal_publisher.hpp"

MinimalPublisher::MinimalPublisher(SafeQueue<queue_package> & squeue_transfer)
: Node("minimal_publisher"), squeue_transfer_(squeue_transfer)
{
  publisher_ = this->create_publisher<data_interfaces::msg::Robot>("robot_data", 10);

  auto timer_callback = [this]() -> void {
    queue_package data;
    auto message = data_interfaces::msg::Robot();

    while (squeue_transfer_.Consume(data)) {
      message.wrench.force.x = data.desired_wrench(0, 0);
      message.wrench.force.y = data.desired_wrench(1, 0);
      message.wrench.force.z = data.desired_wrench(2, 0);
      message.wrench.torque.x = data.desired_wrench(3, 0);
      message.wrench.torque.y = data.desired_wrench(4, 0);
      message.wrench.torque.z = data.desired_wrench(5, 0);

      message.position.position.x = data.translation[0];
      message.position.position.y = data.translation[1];
      message.position.position.z = data.translation[2];

      message.position.orientation.x = data.orientation_error(0, 0);
      message.position.orientation.y = data.orientation_error(1, 0);
      message.position.orientation.z = data.orientation_error(2, 0);
      message.position.orientation.w = 1.0;

      message.position_d.position.x = data.translation_d[0];
      message.position_d.position.y = data.translation_d[1];
      message.position_d.position.z = data.translation_d[2];
      
      message.torques_desired.resize(7);
      for (size_t i = 0; i < 7; ++i) {
          message.torques_desired[i] = data.torques_d(i);
      }

      message.torques_observed.resize(7);
      for (size_t i = 0; i < 7; ++i) {
          message.torques_observed[i] = data.torques_o(i);
      }

      message.torques_coriolis.resize(7);
      for (size_t i = 0; i < 7; ++i) {
          message.torques_coriolis[i] = data.torques_c(i);
      }

      message.torques_gravity.resize(7);
      for (size_t i = 0; i < 7; ++i) {
          message.torques_gravity[i] = data.torques_g(i);
      }

      publisher_->publish(message);
    }
  };

  timer_ = this->create_wall_timer(std::chrono::milliseconds(10), timer_callback);
}
