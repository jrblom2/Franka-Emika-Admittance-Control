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
#include <fstream>

#include <eigen3/Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "examples_common.h"
#include "net_ft/hardware_interface.hpp"
#include "SafeQueue.hpp"
#include "json.hpp"
#include "traj_simulate.hpp"
#include "minimal_publisher.hpp"
#include "data_dumper.hpp"

using json = nlohmann::json;

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
  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " <config-name>" << " <publish?>"<< std::endl;
    return -1;
  }
  std::string config_name{argv[1]};
  std::string ros2_publish{argv[2]};

  std::string package_share_dir = ament_index_cpp::get_package_share_directory("data_bridge");
  std::string config_path = package_share_dir + "/config/config.json";
  std::ifstream f(config_path);
  
  json config = json::parse(f);

  //stiffness
  std::vector<double> stiffness_values = config[config_name]["stiffness"];
  Eigen::VectorXd stiffness_vec = Eigen::Map<Eigen::VectorXd>(stiffness_values.data(), stiffness_values.size());
  Eigen::MatrixXd stiffness = stiffness_vec.asDiagonal();

  //damping
  std::vector<double> damping_values = config[config_name]["damping"];
  Eigen::VectorXd damping_vec = Eigen::Map<Eigen::VectorXd>(damping_values.data(), damping_values.size());
  Eigen::MatrixXd damping = damping_vec.asDiagonal();

  //mass matrix
  std::vector<double> mass_values = config[config_name]["mass"];
  Eigen::VectorXd mass_vec = Eigen::Map<Eigen::VectorXd>(mass_values.data(), mass_values.size());
  Eigen::MatrixXd virtual_mass = mass_vec.asDiagonal();

  //joint weights
  std::vector<double> weight_values = config[config_name]["joint_weight"];
  Eigen::VectorXd joint_weights = Eigen::Map<Eigen::VectorXd>(weight_values.data(), weight_values.size());
  Eigen::MatrixXd W_inv = joint_weights.asDiagonal().inverse();

  //friction comp
  std::vector<double> coulomb_values = config[config_name]["friction_coulomb"];
  Eigen::VectorXd coulomb_frictions = Eigen::Map<Eigen::VectorXd>(coulomb_values.data(), coulomb_values.size());
  std::vector<double> viscous_values = config[config_name]["friction_viscous"];
  Eigen::VectorXd viscous_frictions = Eigen::Map<Eigen::VectorXd>(viscous_values.data(), viscous_values.size());

  //connect to sensor
  net_ft_driver::ft_info input;
  input.ip_address = "192.168.18.12";
  input.sensor_type = "ati_axia";
  input.rdt_sampling_rate = 4000;
  input.use_biasing = "true";
  input.internal_filter_rate = 0;
  net_ft_driver::NetFtHardwareInterface sensor = net_ft_driver::NetFtHardwareInterface(input);

  // setup sensor transform
  Eigen::Matrix<double, 3, 3> sensor_rotation;
  //rotated to align with sensor frame, 90 degrees counter clockwise
  sensor_rotation <<  std::cos(-M_PI_2), -std::sin(-M_PI_2), 0,
                      std::sin(-M_PI_2), std::cos(-M_PI_2), 0,
                      0,                0,                1;
  
  // shifted down in sensor frame (up to the user)
  Eigen::Vector3d sensor_translation {0.0, 0.0, -0.0424};
  Eigen::Matrix3d sensor_translation_skew;
  sensor_translation_skew <<     0,                          -sensor_translation.z(),  sensor_translation.y(),
                                 sensor_translation.z(),     0,                        -sensor_translation.x(),
                                 -sensor_translation.y(),    sensor_translation.x(),   0;
  
  Eigen::MatrixXd sensor_ee_adjoint(6, 6);
  sensor_ee_adjoint.setZero();
  sensor_ee_adjoint.topLeftCorner(3, 3) << sensor_rotation;
  sensor_ee_adjoint.bottomRightCorner(3,3) << sensor_rotation;
  sensor_ee_adjoint.bottomLeftCorner(3,3) << sensor_translation_skew * sensor_rotation;

  double gravity_comp = 2.75;
  
  // thread-safe queue to transfer robot data to ROS
  std::thread spin_thread;
  SafeQueue<queue_package> transfer_package;
  std::vector<queue_package> dump_vector;

  try {
    // connect to robot
    franka::Robot robot(config["robot_ip"]);
    setDefaultBehavior(robot, 0.80);

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
    
    auto set_point_func_sim = [&](double) -> Eigen::Matrix<double, 6, 1> {
      Eigen::Matrix<double, 6, 1> set {position_d(0), position_d(1), position_d(2), 0.0, 0.0, 0.0};
      return set;
    };

    auto fext_func = [&](double t) -> Eigen::Matrix<double, 6, 1> {
        Eigen::Matrix<double, 6, 1> fext_dummy;
        fext_dummy << 0.0,
              (std::sin(t  * 2 * M_PI / 4.0)),
              0.0,
              0.0,
              0.0,
              0.0;
        return fext_dummy;

    };
    Eigen::Matrix<double, 6, 1> x0_vec;
    x0_vec << position_d, 0.0, 0.0, 0.0;

    Eigen::MatrixXd damping_sim = damping;
    // damping_sim(1,1) = 16.0;
    std::vector<Eigen::Matrix<double, 6, 1>> expected_pos;
    std::vector<Eigen::Matrix<double, 6, 1>> expected_vel;
    std::vector<Eigen::Matrix<double, 6, 1>> expected_accel;
    trajectory_6d sim_traj = spring_simulate_6d(
      x0_vec,
      stiffness,
      damping_sim,
      virtual_mass,
      fext_func,
      set_point_func_sim);

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
                      
      // update sensor data
      sensor.read();
      std::array<double, 6> ft_reading = sensor.ft_sensor_measurements_;

      static int fullCount = 0;

      // convert to Eigen
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 7>> mass(mass_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_J(robot_state.tau_J.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());
      Eigen::Map<Eigen::Matrix<double, 6, 1>> sensor_fext(ft_reading.data());
      Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      Eigen::Vector3d position(transform.translation());
      Eigen::Quaterniond orientation(transform.rotation());
      
      static Eigen::Matrix<double, 6, 7> old_jacobian = jacobian;
      static Eigen::Vector3d old_velocity = Eigen::Vector3d::Zero();
      static Eigen::Vector3d old_position = position;
      
      Eigen::Matrix<double, 6, 7> djacobian;
      Eigen::Vector3d velocity;
      Eigen::Vector3d accel;
      // arbitrary cutoff for no duration, expected duration is 0.001
      if (duration.toSec() < 0.00000001) {
        djacobian.setZero();
        velocity.setZero();
        accel.setZero();
      } else {
        djacobian = (jacobian - old_jacobian)/duration.toSec();
        velocity = (position - old_position)/duration.toSec();
        accel = (velocity - old_velocity)/duration.toSec();
      }

      // non static update
      old_jacobian = jacobian;
      old_velocity = velocity;
      old_position = position;

      // translate wrench from FT sensor as wrench in EE frame. MR 3.98
      Eigen::Matrix<double, 6, 1> ee_fext = sensor_ee_adjoint.transpose() * sensor_fext;

      // translate gravity wrench into EE frame
      Eigen::Matrix<double, 6, 1> gravity_wrench {0.0, 0.0, -gravity_comp, 0.0, 0.0, 0.0};
      Eigen::MatrixXd base_ee_adjoint(6, 6);
      base_ee_adjoint.setZero();
      base_ee_adjoint.topLeftCorner(3, 3) << transform.rotation();
      base_ee_adjoint.bottomRightCorner(3,3) << transform.rotation();
      Eigen::Matrix<double, 6, 1> ee_gravity = base_ee_adjoint.transpose() * gravity_wrench;
      ee_fext(0) = ee_fext(0) - ee_gravity(0);
      ee_fext(1) = ee_fext(1) - ee_gravity(1);
      // add gravity comp back to account for sensor bias
      ee_fext(2) = ee_fext(2) - ee_gravity(2) + gravity_comp;

      // translate gravity compensated wrench at EE to base frame to express acceleration in cartesian space.
      Eigen::MatrixXd ee_base_adjoint(6, 6);
      ee_base_adjoint.setZero();
      ee_base_adjoint.topLeftCorner(3, 3) << transform.rotation().transpose();
      ee_base_adjoint.bottomRightCorner(3,3) << transform.rotation().transpose();
      Eigen::Matrix<double, 6, 1> base_fext = ee_base_adjoint.transpose() * ee_fext;

      if (config[config_name]["swap_torque"]) {
        base_fext(3) = -base_fext(3);
        base_fext(4) = -base_fext(4);
        base_fext(5) = -base_fext(5);
      }
      if (config[config_name]["use_dummy_force"]) {
        base_fext = fext_func(fullCount/1000.0);
      }

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

      ddx_d << virtual_mass.inverse() * (base_fext - (damping * (jacobian * dq)) - (stiffness * error));

      // compute control
      Eigen::VectorXd tau_task(7), tau_d(7), tau_friction(7);
      static Eigen::VectorXd last_task = Eigen::VectorXd::Zero(7);

      // MR 6.7 weighted pseudoinverse
      Eigen::MatrixXd weighted_pseudo_inverse = W_inv * jacobian.transpose() * (jacobian * W_inv * jacobian.transpose()).inverse();
      
      // MR 11.66
      Eigen::VectorXd ddq_d(7);
      ddq_d << weighted_pseudo_inverse * (ddx_d - (djacobian * dq));
      
      // MR 8.1
      tau_task << mass * ddq_d;

      // coloumb friction
      double epsilon = config[config_name]["friction_sign_epsilon"];
      Eigen::VectorXd dq_smooth_sign = dq.array() / (dq.array().square() + epsilon * epsilon).sqrt();

      // total friction comp
      tau_friction =  coulomb_frictions.cwiseProduct(dq_smooth_sign) + viscous_frictions.cwiseProduct(dq);

      // add all control elements together
      tau_d << tau_task + coriolis + tau_friction;

      //Spec sheet lists 1000/sec as maximum but in practice should be much lower for smooth human use.
      double max_torque_accel = 10.0 / 1000;
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
      static Eigen::Matrix<double, 6, 1> predicted;
      predicted << position, error.tail(3); 

      if (expected_pos.size() > 0 && fullCount < (int)expected_pos.size() && config[config_name]["use_dummy_force"]) {
        predicted = expected_pos[fullCount];
      }
      fullCount++;

      if (count == 10) {
        queue_package new_package;
        new_package.desired_accel = Eigen::Matrix<double, 6, 1>(ddx_d);
        new_package.actual_wrench = Eigen::Matrix<double, 6, 1>(base_fext);
        new_package.orientation_error = Eigen::Matrix<double, 3, 1>(error.tail(3));
        new_package.translation = Eigen::Vector3d(position);
        new_package.translation_d = Eigen::Vector3d(predicted.head(3));
        new_package.velocity = velocity;
        new_package.accel = accel;
        new_package.torques_d = tau_d;
        new_package.torques_o = tau_J_d.reshaped();
        new_package.torques_c = coriolis.reshaped();
        new_package.torques_g = tau_J.reshaped() - gravity.reshaped();
        new_package.torques_f = tau_friction;
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
    sensor.re_bias();

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
  sensor.on_deactivate();
  return 0;
}
