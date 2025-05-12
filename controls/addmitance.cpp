// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <array>
#include <cmath>
#include <functional>
#include <iostream>

#include <eigen3/Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "examples_common.h"
#include "net_ft/hardware_interface.hpp"

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
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  // Compliance parameters
  const double translational_stiffness{0.0};
  const double rotational_stiffness{0.0};
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
  input.rdt_sampling_rate = 500;
  input.use_biasing = "false";
  input.internal_filter_rate = 0;

  net_ft_driver::NetFtHardwareInterface sensor = net_ft_driver::NetFtHardwareInterface(input);

  try {
    // connect to robot
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);
    // load the kinematics and dynamics model
    franka::Model model = robot.loadModel();

    franka::RobotState initial_state = robot.readOnce();

    //update sensor data
    sensor.read();

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

      std::array<double, 6> ft_reading = sensor.ft_sensor_measurements_;

      // convert to Eigen
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 7>> mass(mass_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
      Eigen::Map<const Eigen::Matrix<double, 6, 1>> fext(ft_reading.data());
      Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      Eigen::Vector3d position(transform.translation());
      Eigen::Quaterniond orientation(transform.rotation());
      
      // static, set initial to current jacobian. Double check this. Is duration.toSec right?
      static Eigen::Matrix<double, 6, 7> old_jacobian = jacobian;
      Eigen::Matrix<double, 6, 7> djacobian = (jacobian - old_jacobian)/duration.toSec();

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

      // ddx_d = M^-1(force - (damping*dx) - (stiffness*x)). Compute desired end effector acceleration
      Eigen::VectorXd ddx_d(6);
      ddx_d << alpha.inverse() * (fext - (damping * (jacobian * dq)) - (stiffness * error));
      std::cout << "ddx: " << ddx_d << std::endl;

      // compute control
      Eigen::VectorXd tau_task(7), tau_d(7);

      // dynamics or term replacement. Two different computions that should arrive at the same output. //TODO get either working.
      bool use_dynamics = true;

      if (use_dynamics) {
        // ddq_d = VampireJacobian * (ddx_d - (dJacobian * dq))
        Eigen::VectorXd ddq_d(7);
        ddq_d << jacobian.completeOrthogonalDecomposition().pseudoInverse() * (ddx_d - (djacobian * dq));

        // Inverse dynamics: tau = M * ddq + compensation term (coriolis)
        tau_task << mass * ddq_d;
      } else {
        // with completed computation for external force in EE space, plug that back in for joint torques?
        tau_task << jacobian.transpose() * (-(alpha * ddx_d) - (damping * (jacobian * dq)) - (stiffness * error));
      }

      std::cout << "task: " << tau_task << std::endl;
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
    robot.control(impedance_control_callback);

  } catch (const franka::Exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }

  return 0;
}
