// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <array>
#include <vector>
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
#include "spring_simulate.hpp"
#include "minimal_publisher.hpp"

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
  const double translational_stiffness{10.0};
  const double rotational_stiffness{50.0};
  const double translational_damping_factor{0.0};
  const double rotational_damping_factor{2.0};
  const double virtual_mass_scaling{1.0};
  Eigen::MatrixXd stiffness(6, 6), damping(6, 6), virtual_mass(6, 6);
  stiffness.setZero();
  stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  damping.setZero();
  damping.topLeftCorner(3, 3) << translational_damping_factor * sqrt(translational_stiffness) *
                                     Eigen::MatrixXd::Identity(3, 3);
  damping.bottomRightCorner(3, 3) << rotational_damping_factor * sqrt(rotational_stiffness) *
                                         Eigen::MatrixXd::Identity(3, 3);
  
  //mass matrix of robot is about as follows:
  virtual_mass.setZero();
  virtual_mass(0,0) = 11;
  virtual_mass(1,1) = 4;
  virtual_mass(2,2) = 5;
  virtual_mass(3,3) = 1;
  virtual_mass(4,4) = 1;
  virtual_mass(5,5) = 1;
  virtual_mass = virtual_mass * virtual_mass_scaling;

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
  SafeQueue<queue_package> transfer_package;

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

    std::vector<Eigen::Vector3d> expected;
    if (calc_mode == "SPRINGDEMO") {
      std::array<double, 7> springY_goal = {{0.109, -0.414, 0.579, -2.011, 0.223, 1.667, 1.414}};
      MotionGenerator spring_motion_generator(0.5, springY_goal);

      robot.control(spring_motion_generator);
      std::cout << "Finished moving to spring offset configuration." << std::endl;

      franka::RobotState spring_state = robot.readOnce();

      // spring point is about 0.3 in the y direction
      Eigen::Affine3d spring_transform(Eigen::Matrix4d::Map(spring_state.O_T_EE.data()));
      Eigen::Vector3d position_spring(spring_transform.translation());
      expected = simulate(
        position_spring - position_d,
        translational_stiffness, 
        translational_damping_factor * sqrt(translational_stiffness),
        virtual_mass.diagonal().head<3>()
      );
    }

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
                      
      // update sensor data
      sensor.read();
      std::array<double, 6> ft_reading = sensor.ft_sensor_measurements_;

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

      // TODO wrench translations should all be part of the adjoint
      // translate wrench from FT sensor as wrench in EE frame. MR 3.98
      fext = sensor_adjoint.transpose() * fext;
      // swap sign for gravity
      fext(2) = -fext(2);
      // swap sign for x-axis
      fext(0) = -fext(0);
      // torque in Z and X already resist user, invert Y to also resist user
      fext(4) = -fext(4);
      
      // static, set initial to current jacobian. Double check this.
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
      // diagnol of alpha is about 11,4,5,1,1,1
      Eigen::Matrix<double, 6, 6> alpha;
      alpha << (jacobian * mass.inverse() * jacobian.transpose()).inverse();

      // compute error to desired equilibrium pose
      // position error
      Eigen::Matrix<double, 6, 1> error;
      if (calc_mode == "TRACK") {
        error.head(3).setZero();
      } else if (calc_mode == "SPRINGY") {
        error.segment<3>(0) << 0.0, position(1) - position_d(1), 0.0;
      } else if (calc_mode == "SPRING" || calc_mode == "SPRINGDEMO") {
        error.head(3) << position - position_d;
      }
      
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

      // MR 11.66, using mass matrix of robot as virtual mass. Have not found a better alternative after testing.
      Eigen::VectorXd ddx_d(6);
      ddx_d << virtual_mass.inverse() * (fext - (damping * (jacobian * dq)) - (stiffness * error));

      // compute control
      Eigen::VectorXd tau_task(7), tau_error(7), tau_d(7);

      // MR 11.66
      Eigen::VectorXd ddq_d(7);
      ddq_d << jacobian.completeOrthogonalDecomposition().pseudoInverse() * (ddx_d - (djacobian * dq));
      // MR 8.1
      tau_task << mass * ddq_d;
      
      // // PI control to account for sticky slow movements
      // double P_gain = 0.07;
      // static Eigen::Matrix<double, 7, 1> old_dq = dq;

      // // observed joint acceleration, diff in velocities over time.
      // Eigen::Matrix<double, 7, 1> observed_ddq;
      // if (duration.toSec() < 0.00000001) {
      //   observed_ddq = ddq_d;
      // } else {
      //   observed_ddq = (dq - old_dq) / duration.toSec();
      // }
      // // error equals current desired acceleration - what we just observed?
      // Eigen::Matrix<double, 7, 1> ddq_error = ddq_d - observed_ddq;
      // Eigen::Matrix<double, 7, 1> PI_ddq = P_gain * ddq_error;
      // tau_error << mass * PI_ddq;
      // old_dq = dq;

      // std::cout << "tau task: " << tau_task << std::endl;
      // std::cout << "tau_error: " << tau_error << std::endl;

      // add all control elements together
      tau_d << tau_task + coriolis;
      std::array<double, 7> tau_d_array{};
      Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

      static Eigen::Vector3d predicted = position;

      // publish results
      static int count = 0;
      static int fullCount = 0;
      count++;

      if (expected.size() > 0 && fullCount < 10000) {
        predicted = expected[fullCount] + position_d;
      }
      fullCount++;

      if (count == 10) {
        queue_package new_package;
        new_package.desired_wrench = Eigen::Matrix<double, 6, 1>(ddx_d);
        new_package.orientation_error = Eigen::Matrix<double, 3, 1>(error.tail(3));
        new_package.translation = Eigen::Vector3d(position);
        new_package.translation_d = Eigen::Vector3d(predicted);
        transfer_package.Produce(std::move(new_package));
        count = 0;
      }

      //predicted position based on acceleration control. Update after message publish so we pair last rounds predicted with the actual change.
      if (duration.toSec() < 0.00000001) {
        predicted = position;
      } else {
        predicted = position + ((jacobian * dq).head(3) * duration.toSec()) + (0.5*ddx_d.head(3)*pow(duration.toSec(), 2.0));
      }

      return tau_d_array;
    };

    // start real-time control loop
    std::cout << "WARNING: Collision thresholds are set to high values. "
              << "Make sure you have the user stop at hand!" << std::endl
              << "After starting try to push the robot and see how it reacts." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    // data bridge through ROS2 setup
    auto node = std::make_shared<MinimalPublisher>(transfer_package);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor](){ executor.spin(); });

    robot.control(impedance_control_callback);

    rclcpp::shutdown();
    spin_thread.join();

  } catch (const franka::Exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }

  return 0;
}
