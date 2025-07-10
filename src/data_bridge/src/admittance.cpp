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
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << " <publish?>"<< std::endl;
    return -1;
  }
  std::string ros2_publish{argv[2]};

  std::string package_share_dir = ament_index_cpp::get_package_share_directory("data_bridge");
  std::string config_path = package_share_dir + "/config/config.json";
  std::ifstream f(config_path);
  
  json config = json::parse(f);

  // Compliance parameters
  const double translational_stiffness{config["admittance"]["translation_stiffness"]};
  const double rotational_stiffness{config["admittance"]["rotation_stiffness"]};
  const double translational_damping_factor{config["admittance"]["translation_damping"]};
  const double rotational_damping_factor{config["admittance"]["rotation_damping"]};
  Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
  stiffness.setZero();
  stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  damping.setZero();
  damping.topLeftCorner(3, 3) << translational_damping_factor * Eigen::MatrixXd::Identity(3, 3);
  damping.bottomRightCorner(3, 3) << rotational_damping_factor * Eigen::MatrixXd::Identity(3, 3);
  
  //mass matrix
  std::vector<double> mass_values = config["admittance"]["mass"];
  Eigen::VectorXd mass_vec = Eigen::Map<Eigen::VectorXd>(mass_values.data(), mass_values.size());
  Eigen::MatrixXd virtual_mass = mass_vec.asDiagonal();

  //joint weights
  std::vector<double> weight_values = config["admittance"]["joint_weight"];
  Eigen::VectorXd joint_weights = Eigen::Map<Eigen::VectorXd>(weight_values.data(), weight_values.size());
  Eigen::MatrixXd W_inv = joint_weights.asDiagonal().inverse();

  // phantom force for demo testing
  Eigen::Matrix<double, 6, 1> phantom_fext = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

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
  std::thread spin_thread;
  SafeQueue<queue_package> transfer_package;
  std::vector<queue_package> dump_vector;

  try {
    // connect to robot
    franka::Robot robot(argv[1]);
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

    auto set_point_func = [&](double) -> Eigen::Matrix<double, 6, 1> {
      return Eigen::Matrix<double, 6, 1>::Zero();
    };
    auto fext_func = [&](double t) -> Eigen::Matrix<double, 6, 1> {
      Eigen::Matrix<double, 6, 1> fext;
      fext << 0.0,
              (1.0 - std::cos(t  * 2 * M_PI / 4.0)),
              0.0,
              0.0,
              0.0,
              0.0;
      return fext;
    };
    Eigen::Matrix<double, 6, 1> x0_vec;
    x0_vec << position_d, 0.0, 0.0, 0.0;

    std::vector<Eigen::Matrix<double, 6, 1>> expected_pos;
    std::vector<Eigen::Matrix<double, 6, 1>> expected_vel;
    std::vector<Eigen::Matrix<double, 6, 1>> expected_accel;
    trajectory_6d sim_traj = spring_simulate_6d(
      x0_vec,
      Eigen::Matrix<double, 6, 6>::Zero(),
      damping,
      virtual_mass,
      fext_func,
      set_point_func);

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

      // convert to Eigen
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 7>> mass(mass_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_J(robot_state.tau_J.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());
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
      // invert Z and X torque from sensor frame to EE frame
      fext(5) = -fext(5);
      fext(3) = -fext(3);
      
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

      static int fullCount = 0;

      // MR 11.66
      Eigen::VectorXd ddx_d(6);
      fext = fext_func(fullCount/1000.0);
      ddx_d << virtual_mass.inverse() * (fext - (damping * (jacobian * dq)) - (stiffness * error));
      
      // compute control
      Eigen::VectorXd tau_task(7), tau_error(7), tau_d(7);
      static Eigen::VectorXd last_task = Eigen::VectorXd::Zero(7);

      // MR 6.7 weighted pseudoinverse
      Eigen::MatrixXd weighted_pseudo_inverse = W_inv * jacobian.transpose() * (jacobian * W_inv * jacobian.transpose()).inverse();
      
      // MR 11.66
      Eigen::VectorXd ddq_d(7);
      ddq_d << weighted_pseudo_inverse * (ddx_d - (djacobian * dq));
      
      // MR 8.1
      tau_task << mass * ddq_d;

      // add all control elements together
      tau_d << tau_task + coriolis;

      //Spec sheet lists 1000/sec as maximum but in practice should be much lower for smooth human use.
      double max_torque_accel = 100.0 / 1000;
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

      if (expected_pos.size() > 0 && fullCount < (int)expected_pos.size()) {
        predicted = expected_pos[fullCount];
      }
      fullCount++;

      if (count == 10) {
        queue_package new_package;
        new_package.desired_accel = Eigen::Matrix<double, 6, 1>(ddx_d);
        new_package.actual_wrench = Eigen::Matrix<double, 6, 1>(fext);
        new_package.orientation_error = Eigen::Matrix<double, 3, 1>(error.tail(3) - predicted.tail(3));
        new_package.translation = Eigen::Vector3d(position);
        new_package.translation_d = Eigen::Vector3d(predicted.head(3));
        new_package.velocity = (jacobian * dq).head(3).reshaped();
        new_package.torques_d = tau_d;
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
