#pragma once

#include <memory>
#include <chrono>
#include "SafeQueue.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "data_interfaces/msg/robot.hpp"

class MinimalPublisher : public rclcpp::Node
{
public:
  explicit MinimalPublisher(SafeQueue<queue_package> & squeue_transfer, Eigen::Vector3d & goal,
    std::mutex & goal_mutex);
  void init();

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<data_interfaces::msg::Robot>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goal_subscription_;
  SafeQueue<queue_package> & squeue_transfer_;
  Eigen::Vector3d & goal_;
  std::mutex & goal_mutex_;
};