#pragma once

#include <memory>
#include <chrono>
#include "SafeQueue.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "data_interfaces/msg/robot.hpp"

class MinimalPublisher : public rclcpp::Node
{
public:
  explicit MinimalPublisher(SafeQueue<queue_package> & squeue_transfer, Eigen::Vector3d & goal,
    std::mutex & goal_mutex, bool & use_goal, std::mutex & use_goal_mutex);
  void init();

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<data_interfaces::msg::Robot>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goal_subscription_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr toggle_goal_service_;
  SafeQueue<queue_package> & squeue_transfer_;
  Eigen::Vector3d & goal_;
  std::mutex & goal_mutex_;
  bool & use_goal_point_;
  std::mutex & use_goal_mutex_;
};