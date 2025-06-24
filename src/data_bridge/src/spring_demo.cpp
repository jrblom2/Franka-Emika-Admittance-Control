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
#include "spring_simulate.hpp"
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
  const double translational_stiffness{15.0};
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

  // phantom force for demo testing
  Eigen::Matrix<double, 6, 1> phantom_fext = {0.0, 1.0, 0.0, 0.0, 0.0, 0.0};

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
    std::array<double, 7> spring_goal;
    if (start_distance == "FAR") {
      spring_goal = {{0.109, -0.414, 0.579, -2.011, 0.223, 1.667, 1.414}};
    } else {
      spring_goal = {{0.072, -0.733, 0.201, -2.310, 0.137, 1.587, 1.009}};
    }
    MotionGenerator spring_motion_generator(0.5, spring_goal);

    robot.control(spring_motion_generator);
    std::cout << "Finished moving to spring offset configuration." << std::endl;

    franka::RobotState spring_state = robot.readOnce();

    // spring point is about 0.3 in the y direction
    Eigen::Affine3d spring_transform(Eigen::Matrix4d::Map(spring_state.O_T_EE.data()));
    Eigen::Vector3d position_spring(spring_transform.translation());
    trajectory sim_traj = simulate(
      position_spring - position_d,
      translational_stiffness, 
      translational_damping_factor * sqrt(translational_stiffness),
      virtual_mass.diagonal().head<3>(),
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
      
      // mass matrix in cartesian space, Alpha. Use this in place of any operation using M but needs 6x6
      // diagnol of alpha is about 11,4,5,1,1,1
      // Eigen::Matrix<double, 6, 6> alpha;
      // alpha << (jacobian * mass.inverse() * jacobian.transpose()).inverse();

      // compute error to desired equilibrium pose
      // position error
      Eigen::Matrix<double, 6, 1> error;
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

      // MR 11.66
      Eigen::VectorXd ddx_d(6);

      // fext = phantom_fext * (jacobian * dq)[1];
      fext.setZero();
      static int fullCount = 0;

      //pull for two sec, go to 10 N over two sec
      // if (fullCount < 2000) {
      //   fext[1] = fullCount / 200.0;
      // } 
      ddx_d << virtual_mass.inverse() * (fext - (damping * (jacobian * dq)) - (stiffness * error));

      // feed forward control instead for demo
      // if (expected_accel.size() > 0 && fullCount < (int)expected_accel.size()) {
      //   ddx_d.head<3>() = expected_accel[fullCount];
      //   ddx_d.tail(3).setZero();
      // }
      
      // compute control
      Eigen::VectorXd tau_task(7), tau_error(7), tau_d(7);

      // MR 11.66
      Eigen::VectorXd ddq_d(7);
      ddq_d << jacobian.completeOrthogonalDecomposition().pseudoInverse() * (ddx_d - (djacobian * dq));
      // MR 8.1
      tau_task << mass * ddq_d;

      // add all control elements together
      tau_d << tau_task + coriolis;

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

      if (count == 20) {
        queue_package new_package;
        new_package.desired_accel = Eigen::Matrix<double, 6, 1>(ddx_d);
        new_package.actual_wrench = Eigen::Matrix<double, 6, 1>(fext);
        new_package.orientation_error = Eigen::Matrix<double, 3, 1>(error.tail(3));
        new_package.translation = Eigen::Vector3d(position);
        new_package.translation_d = Eigen::Vector3d(predicted);
        new_package.velocity = (jacobian * dq).head(3).reshaped();
        new_package.torques_d = tau_d;
        new_package.torques_o = tau_J_d.reshaped();
        new_package.torques_c = coriolis.reshaped();
        new_package.torques_g = tau_J.reshaped() - gravity.reshaped();
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
