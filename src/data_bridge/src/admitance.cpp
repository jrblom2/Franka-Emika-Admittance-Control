// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <string>

#include <eigen3/Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "examples_common.h"
#include "net_ft/hardware_interface.hpp"
#include "SafeQueue.hpp"

#include "geometry_msgs/msg/wrench.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher(SafeQueue<Eigen::Matrix<double, 6, 1>> & squeue)
  : Node("minimal_publisher"), squeue_(squeue)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>("data_bridge", 10);
    auto timer_callback =
      [this]() -> void {
        Eigen::Matrix<double, 6, 1> data;
        auto message = geometry_msgs::msg::Wrench();

        // 1000 messages a second is crazy clear queue 100 times a second and just send last one.
        while(squeue_.Consume(data)) {
        }
        message.force.x = data(0,0);
        message.force.y = data(1,0);
        message.force.z = data(2,0);
        message.torque.x = data(3,0);
        message.torque.y = data(4,0);
        message.torque.z = data(5,0);
        this->publisher_->publish(message);
      };
    timer_ = this->create_wall_timer(std::chrono::milliseconds(20), timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr publisher_;
  SafeQueue<Eigen::Matrix<double, 6, 1>> & squeue_;
};

/**
 * @example cartesian_impedance_control.cpp
 * An example showing a simple cartesian impedance controller without inertia shaping
 * that renders a spring damper system where the equilibrium is the initial configuration.
 * After starting the controller try to push the robot around and try different stiffness levels.
 *
 * @warning collision thresholds are set to high values. Make sure you have the user stop at hand!
 */

int main(int argc, char** argv) {
  // Check whether the required arguments were passed
  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" <<  "<control-calc>" << std::endl;
    return -1;
  }

  // RCL Init
  rclcpp::init(argc, argv);

  std::string calc_mode{argv[2]};

  // Compliance parameters
  const double translational_stiffness{150.0};
  const double rotational_stiffness{5.0};
  Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
  stiffness.setZero();
  stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  damping.setZero();
  damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
                                     Eigen::MatrixXd::Identity(3, 3);
  damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
                                         Eigen::MatrixXd::Identity(3, 3);

  //connect to sensor
  net_ft_driver::ft_info input;
  input.ip_address = "192.168.18.12";
  input.sensor_type = "ati_axia";
  input.rdt_sampling_rate = 1000;
  input.use_biasing = "true";
  input.internal_filter_rate = 0;

  Eigen::Matrix<double, 3, 3> sensor_rotation;
  //45 degrees clockwise
  sensor_rotation << std::cos(M_PI_4), -std::sin(M_PI_4), 0,
                      std::sin(M_PI_4), std::cos(M_PI_4), 0,
                      0,                0,                1;
  
  Eigen::MatrixXd sensor_adjoint(6, 6);
  sensor_adjoint.setZero();
  sensor_adjoint.topLeftCorner(3, 3) << sensor_rotation;
  sensor_adjoint.bottomRightCorner(3,3) << sensor_rotation;

  net_ft_driver::NetFtHardwareInterface sensor = net_ft_driver::NetFtHardwareInterface(input);
  std::cout << "Sensor started." << std::endl;

  // thread-safe queue to transfer robot data to ROS
  SafeQueue<Eigen::Matrix<double, 6, 1>> transfer;

  try {
    // connect to robot
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);

    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    // load the kinematics and dynamics model
    franka::Model model = robot.loadModel();

    franka::RobotState initial_state = robot.readOnce();

    // equilibrium point is the initial position
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d position_d(initial_transform.translation());
    Eigen::Quaterniond orientation_d(initial_transform.rotation());

    // set collision behavior
    robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

    // define callback for the torque control loop
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        impedance_control_callback = [&](const franka::RobotState& robot_state,
                                         franka::Duration duration) -> franka::Torques {
      // get state variables
      std::array<double, 7> coriolis_array = model.coriolis(robot_state);
      std::array<double, 42> jacobian_array =
          model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
      std::array<double, 49> mass_array = model.mass(robot_state);
                      
      //update sensor data
      sensor.read();
      std::array<double, 6> ft_reading = sensor.ft_sensor_measurements_;
      //swap sign for gravity
      ft_reading[2] = -ft_reading[2];

      // convert to Eigen
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 7>> mass(mass_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
      Eigen::Map<Eigen::Matrix<double, 6, 1>> fext(ft_reading.data());
      Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      Eigen::Vector3d position(transform.translation());
      Eigen::Quaterniond orientation(transform.rotation());
      
      transfer.Produce(Eigen::Matrix<double, 6, 1>(fext));
      //translate wrench from FT sensor as wrench in EE frame. MR 3.98
      fext = sensor_adjoint.transpose() * fext;
      
      // static, set initial to current jacobian. Double check this. Is duration.toSec right?
      static Eigen::Matrix<double, 6, 7> old_jacobian = jacobian;
      
      Eigen::Matrix<double, 6, 7> djacobian;
      // arbitrary cutoff for no duration, expected duration is 0.001
      if (duration.toSec() < 0.00000001) {
        djacobian.setZero();
      } else {
        djacobian = (jacobian - old_jacobian)/duration.toSec();
      }

      // non static update
      old_jacobian = jacobian;
      
      // mass matrix in cartesian space, Alpha. Use this in place of any operation using M but needs 6x6
      Eigen::Matrix<double, 6, 6> alpha;
      alpha << (jacobian * mass.inverse() * jacobian.transpose()).inverse();

      // compute error to desired equilibrium pose
      // position error
      Eigen::Matrix<double, 6, 1> error;
      // TODO what is X in our control scheme? Or do we not use x?
      error.head(3) << position - position_d;
      
      // orientation error
      // "difference" quaternion
      if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
        orientation.coeffs() << -orientation.coeffs();
      }

      // "difference" quaternion
      Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
      error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
      // Transform to base frame
      error.tail(3) << -transform.rotation() * error.tail(3);

      //MR 11.66
      Eigen::VectorXd ddx_d(6);
      ddx_d << alpha.inverse() * (fext - (damping * (jacobian * dq)) - (stiffness * error));
      // std::cout << "ddx: " << ddx_d << std::endl;

      // compute control
      Eigen::VectorXd tau_task(7), tau_d(7);

      //different methods for calcuating torque from force input, often results in same calculation but not always?
      if (calc_mode == "JOINTACCEL") {
        //MR 11.66
        Eigen::VectorXd ddq_d(7);
        ddq_d << jacobian.completeOrthogonalDecomposition().pseudoInverse() * (ddx_d - (djacobian * dq));
        //MR 8.1
        tau_task << mass * ddq_d;
      } else  if (calc_mode == "PLUGBACK") {
        //MR 11.65
        tau_task << jacobian.transpose() * (-(alpha * ddx_d) - (damping * (jacobian * dq)) - (stiffness * error));
      } else if (calc_mode == "SIMPLE") {
        tau_task << jacobian.transpose() * fext;
      } else {
        std::cout << "Invalid control" << std::endl;
      }

      // std::cout << "task: " << tau_task << std::endl;
      tau_d << tau_task + coriolis;

      std::array<double, 7> tau_d_array{};
      Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;
      return tau_d_array;
    };

    // start real-time control loop
    std::cout << "WARNING: Collision thresholds are set to high values. "
              << "Make sure you have the user stop at hand!" << std::endl
              << "After starting try to push the robot and see how it reacts." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    // data bridge through ROS2 setup
    auto node = std::make_shared<MinimalPublisher>(transfer);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor](){ executor.spin(); });

    // std::thread transfer_thread(transfer_callback);
    robot.control(impedance_control_callback);

    rclcpp::shutdown();
    spin_thread.join();

  } catch (const franka::Exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }

  return 0;
}
