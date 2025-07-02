// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <array>
#include <vector>
#include <cmath>
#include <functional>
#include <iostream>
#include <string>
#include <chrono>
#include <csignal>

#include <eigen3/Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "examples_common.h"
#include "net_ft/hardware_interface.hpp"
#include "SafeQueue.hpp"
#include "traj_simulate.hpp"
#include "minimal_publisher.hpp"
#include "data_dumper.hpp"

volatile bool robot_stop = false; // Global flag

void signal_handler(int signal) {
    if (signal == SIGINT) {
        std::cerr << "\nCtrl+C detected. Initiating graceful shutdown..." << std::endl;
        robot_stop = true;
    }
}

/**
 * @example cartesian_impedance_control.cpp
 * An example showing a simple cartesian impedance controller without inertia shaping
 * that renders a spring damper system where the equilibrium is the initial configuration.
 * After starting the controller try to push the robot around and try different stiffness levels.
 *
 * @warning collision thresholds are set to high values. Make sure you have the user stop at hand!
 */
int main(int argc, char** argv) {
  std::signal(SIGINT, signal_handler);
  // Check whether the required arguments were passed
  if (argc != 4) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" <<  " <distance>" << " <publish?>" << std::endl;
    return -1;
  }

  std::string start_distance{argv[2]};
  std::string ros2_publish{argv[3]};

  // Compliance parameters
  const double translational_stiffness{50.0};
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
  virtual_mass(0,0) = 1;
  virtual_mass(1,1) = 1;
  virtual_mass(2,2) = 1;
  virtual_mass(3,3) = 1;
  virtual_mass(4,4) = 1;
  virtual_mass(5,5) = 1;
  virtual_mass = virtual_mass * virtual_mass_scaling;

  // thread-safe queue to transfer robot data to ROS
  std::thread spin_thread;
  SafeQueue<queue_package> transfer_package;
  std::vector<queue_package> dump_vector;

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

    // if this is a demo, move to offset position and simulate expected movement
    std::vector<Eigen::Vector3d> expected_pos;
    std::vector<Eigen::Vector3d> expected_vel;
    std::vector<Eigen::Vector3d> expected_accel;
    trajectory sim_traj = sin_simulate(
      position_d,
      Eigen::Vector3d::Zero()
    );
    expected_pos = sim_traj.position;
    expected_vel = sim_traj.velocity;
    expected_accel = sim_traj.acceleration;

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
      std::array<double, 7> gravity_array = model.gravity(robot_state);
      std::array<double, 42> jacobian_array =
          model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
      std::array<double, 49> mass_array = model.mass(robot_state);

      // convert to Eigen
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 7>> mass(mass_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_J(robot_state.tau_J.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());
      Eigen::Matrix<double, 6, 1> fext;
      Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      Eigen::Vector3d position(transform.translation());
      Eigen::Quaterniond orientation(transform.rotation());
      
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

      // compute error to desired equilibrium pose
      // position error
      Eigen::Matrix<double, 6, 1> error;
      static int fullCount = 0;
      Eigen::Vector3d track = position_d;
      // track(1) = track(1) +  0.1 * (1 - cos(fullCount * 2 * M_PI / 4000.0));
      error.head(3) << position - track;
      
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

      // MR 11.66
      Eigen::VectorXd ddx_d(6);
      ddx_d << virtual_mass.inverse() * (fext - (damping * (jacobian * dq)) - (stiffness * error));
      // ddx_d(1) = 1.0 * cos(fullCount * 2 * M_PI / 4000.0);
      ddx_d.tail(3).setZero();
      
      // compute control
      Eigen::VectorXd tau_task(7), tau_d(7);

      static Eigen::VectorXd last_task = Eigen::VectorXd::Zero(7);

      // MR 6.7 weighted pseudoinverse
      Eigen::VectorXd joint_weights = Eigen::VectorXd::Ones(7);
      // joint_weights(0) = 0.1;
      Eigen::MatrixXd W_inv = joint_weights.asDiagonal().inverse();
      Eigen::MatrixXd weighted_pseudo_inverse = W_inv.topLeftCorner(4,4) * jacobian.topRows(3).leftCols(4).transpose() * (jacobian.topRows(3).leftCols(4) * W_inv.topLeftCorner(4,4) * jacobian.topRows(3).leftCols(4).transpose()).inverse();

      // MR 11.66
      Eigen::VectorXd ddq_d(7);
      ddq_d.setZero();
      ddq_d.head(4) << weighted_pseudo_inverse * (ddx_d.head(3) - (djacobian.topRows(3).leftCols(4) * dq.head(4)));
      // MR 8.1
      tau_task.head(4) << mass.topLeftCorner(4,4) * ddq_d.head(4);

      // control orientation with just last three joints.
      tau_task.tail(3) << jacobian.rightCols(3).bottomRows(3).transpose() * (-stiffness.bottomRightCorner(3, 3) * error.tail(3) - damping.bottomRightCorner(3, 3) * (jacobian.rightCols(3).bottomRows(3) * dq.tail(3)));

      // add all control elements together
      tau_d << tau_task + coriolis;
      double max_torque_accel = 80.0 / 1000;
      // if torque acceleration exceeds 80/s^2, throttle to 80.
      for (int i = 0; i < tau_d.size(); ++i) {
        tau_d(i) = std::clamp(tau_d(i), last_task(i) - max_torque_accel, last_task(i) + max_torque_accel);
      }
      last_task = tau_d;

      // output format
      std::array<double, 7> tau_d_array;
      Eigen::Map<Eigen::Matrix<double, 7, 1>>(tau_d_array.data()) = tau_d;
      franka::Torques torques = tau_d_array;

      // publish results
      static int count = 0;
      count++;
      static Eigen::Vector3d predicted = position;

      if (expected_pos.size() > 0 && fullCount < (int)expected_pos.size()) {
        predicted = expected_pos[fullCount] + position_d;
      }
      fullCount++;

      if (count == 10) {
        queue_package new_package;
        new_package.desired_accel = Eigen::Matrix<double, 6, 1>(ddx_d);
        new_package.actual_wrench = Eigen::Matrix<double, 6, 1>(fext);
        new_package.orientation_error = Eigen::Matrix<double, 3, 1>(error.tail(3));
        new_package.translation = Eigen::Vector3d(position);
        new_package.translation_d = Eigen::Vector3d::Zero();
        new_package.velocity = (jacobian * dq).head(3).reshaped();
        new_package.torques_d = tau_task;
        new_package.torques_o = tau_J_d.reshaped();
        new_package.torques_c = coriolis.reshaped();
        new_package.torques_g = tau_J.reshaped() - gravity.reshaped();
        new_package.ddq_d = ddq_d;
        new_package.dq = dq;
        if (ros2_publish == "TRUE") {
          transfer_package.Produce(std::move(new_package));
        }
        dump_vector.push_back(std::move(new_package));
        count = 0;
      }

      // if ctrl-c is pressed, robot should stop
      if (robot_stop) {
        return franka::MotionFinished(torques);
      }
      return torques;
    };

    // start real-time control loop
    std::cout << "WARNING: Collision thresholds are set to high values. "
              << "Make sure you have the user stop at hand!" << std::endl
              << "After starting try to push the robot and see how it reacts." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    // data bridge through ROS2 setup
    if (ros2_publish == "TRUE") {
      rclcpp::init(argc, argv);
      auto node = std::make_shared<MinimalPublisher>(transfer_package);
      rclcpp::executors::MultiThreadedExecutor executor;
      executor.add_node(node);
      spin_thread = std::thread([&executor]() { executor.spin(); });
    }

    robot.control(impedance_control_callback);
  } catch (const franka::Exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  } catch (const std::exception& ex) {
    std::cerr << ex.what() << std::endl;
  } catch (...) {
      std::cerr << "Unknown exception caught." << std::endl;
  }

  if (ros2_publish == "TRUE") {
    rclcpp::shutdown();
    spin_thread.join();
  }

  robot_dump(dump_vector);

  return 0;
}
