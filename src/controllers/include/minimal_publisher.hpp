#pragma once

#include <memory>
#include <chrono>
#include "SafeQueue.hpp"
#include "rclcpp/rclcpp.hpp"
#include "data_interfaces/msg/robot.hpp"

class MinimalPublisher : public rclcpp::Node
{
public:
  explicit MinimalPublisher(SafeQueue<queue_package> & squeue_transfer);

private:
  void timer_callback();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<data_interfaces::msg::Robot>::SharedPtr publisher_;
  SafeQueue<queue_package> & squeue_transfer_;
};