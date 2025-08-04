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
#include "butterworth.hpp"

using json = nlohmann::json;

volatile bool robot_stop = false; // Global flag

void signal_handler(int signal) {
    if (signal == SIGINT) {
        std::cerr << "\nCtrl+C detected. Initiating graceful shutdown..." << std::endl;
        robot_stop = true;
    }
}

/**
 * An admittance controller designed to interface with a Axia M8 F/T sensor at the wrist. Bounding boxes,
 * velocity limits, and control parameters can be configured in the configuration named when the contoller
 * is called.
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

  std::string package_share_dir = ament_index_cpp::get_package_share_directory("controllers");
  std::string config_path = package_share_dir + "/config/config.json";
  std::ifstream f(config_path);
  
  json config = json::parse(f);

  //how fast the torque is allowed to change
  double torque_smoothing = config[config_name]["torque_smoothing"];

  double force_limit = config[config_name]["force_limit"];

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
  bool use_friction_comp = config[config_name]["use_friction_comp"];
  double coulomb_epsilon = config[config_name]["friction_comp"]["friction_sign_epsilon"];
  std::vector<double> coulomb_values = config[config_name]["friction_comp"]["friction_coulomb"];
  Eigen::VectorXd coulomb_frictions = Eigen::Map<Eigen::VectorXd>(coulomb_values.data(), coulomb_values.size());
  std::vector<double> viscous_values = config[config_name]["friction_comp"]["friction_viscous"];
  Eigen::VectorXd viscous_frictions = Eigen::Map<Eigen::VectorXd>(viscous_values.data(), viscous_values.size());

  //boundry conditions
  bool use_boundry = config[config_name]["use_boundry"];

  std::vector<double> boundry_min_values = config[config_name]["boundry"]["min"];
  Eigen::VectorXd boundry_min = Eigen::Map<Eigen::VectorXd>(boundry_min_values.data(), boundry_min_values.size());

  std::vector<double> boundry_max_values = config[config_name]["boundry"]["max"];
  Eigen::VectorXd boundry_max = Eigen::Map<Eigen::VectorXd>(boundry_max_values.data(), boundry_max_values.size());

  double boundry_trans_stiffness = config[config_name]["boundry"]["trans_stiffness"];
  double boundry_rot_stiffness = config[config_name]["boundry"]["rot_stiffness"];
  double boundry_trans_damping = config[config_name]["boundry"]["trans_damping"];
  double boundry_rot_damping = config[config_name]["boundry"]["rot_damping"];

  //velocity limits
  bool use_velocity_max = config[config_name]["use_velocity_max"];
  std::vector<double> velocity_max_values = config[config_name]["velocity_max"]["max_velocity"];
  Eigen::VectorXd velocity_max = Eigen::Map<Eigen::VectorXd>(velocity_max_values.data(), velocity_max_values.size());

  std::vector<double> velocity_max_damping_values = config[config_name]["velocity_max"]["damping"];
  Eigen::VectorXd velocity_max_damping = Eigen::Map<Eigen::VectorXd>(velocity_max_damping_values.data(), velocity_max_damping_values.size());

  //connect to sensor, see data sheet for filter selection based on sampling rate.
  net_ft_driver::ft_info input;
  input.ip_address = "192.168.18.12";
  input.sensor_type = "ati_axia";
  input.rdt_sampling_rate = 2000;
  input.use_biasing = "true";
  input.internal_filter_rate = 5;
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

  const int MAX_BUFFER_SIZE = 60000;
  std::vector<queue_package> dump_vector(MAX_BUFFER_SIZE);
  int dump_index = 0;
  bool buffer_full = false;

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

    // auto fext_func = [&](double t) -> Eigen::Matrix<double, 6, 1> {
    //     Eigen::Matrix<double, 6, 1> fext_dummy;
    //     fext_dummy << (std::sin(t  * 2 * M_PI / 4.0)),
    //           (std::sin(t  * 2 * M_PI / 4.0)),
    //           (std::sin(t  * 2 * M_PI / 4.0)),
    //           0.0,
    //           0.0,
    //           0.0;
    //     return fext_dummy;

    // };
    auto fext_func = [&](double t) -> Eigen::Matrix<double, 6, 1> {
      Eigen::Matrix<double, 6, 1> fext_dummy;

      // Chirp parameters
      double f0 = 0.1;      // Start frequency (Hz)
      double f1 = 100.0;      // End frequency (Hz)
      double T = 30.0;      // Chirp duration (seconds)
      double k = (f1 - f0) / T;

      // Compute chirp phase: φ(t) = 2π(f0 * t + 0.5 * k * t^2)
      double phi = 2 * M_PI * (f0 * t + 0.5 * k * t * t);

      // Chirp amplitude (optional scaling)
      double amplitude = 1.0;

      fext_dummy << 0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    amplitude * std::sin(phi);

      return fext_dummy;
    };

    Eigen::Matrix<double, 6, 1> x0_vec;
    x0_vec << position_d, 0.0, 0.0, 0.0;

    std::vector<Eigen::Matrix<double, 6, 1>> expected_pos;
    std::vector<Eigen::Matrix<double, 6, 1>> expected_vel;
    std::vector<Eigen::Matrix<double, 6, 1>> expected_accel;
    trajectory_6d sim_traj = spring_simulate_6d(
      x0_vec,
      stiffness,
      damping,
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
      Eigen::Map<Eigen::Matrix<double, 6, 1>> sensor_fext_raw(ft_reading.data());
      Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      Eigen::Vector3d position(transform.translation());
      Eigen::Quaterniond orientation(transform.rotation());

      // compute error to desired equilibrium pose
      // position error
      Eigen::Matrix<double, 6, 1> error;
      error.head(3) << position - position_d;
      
      // orientation error
      // "difference" quaternion
      if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
        orientation.coeffs() << -orientation.coeffs();
      }

      // "difference" quaternion for use in control
      Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
      error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
      // Transform to base frame
      error.tail(3) << -transform.rotation() * error.tail(3);

      // axis angle representation for use in boundaries and logging (in base frame)
      Eigen::AngleAxisd angle_axis(error_quaternion);
      Eigen::Vector3d orientation_error_axis_angle = -transform.rotation() * (angle_axis.angle() * angle_axis.axis());
      
      Eigen::VectorXd position_6d(6);
      position_6d << position, orientation_error_axis_angle;
      static Eigen::Matrix<double, 6, 7> old_jacobian = jacobian;
      static Eigen::VectorXd old_velocity = Eigen::VectorXd::Zero(6);
      static Eigen::VectorXd old_position = position_6d;
      static const double velocity_alpha = 0.1;
      
      Eigen::Matrix<double, 6, 7> djacobian;
      Eigen::VectorXd velocity;
      Eigen::VectorXd velocity_raw;
      Eigen::VectorXd accel;
      Eigen::VectorXd accel_raw;
      // arbitrary cutoff for no duration, expected duration is 0.001
      if (duration.toSec() < 0.00000001) {
        djacobian.setZero();
        velocity.setZero(6);
        accel.setZero(6);
      } else {
        djacobian = (jacobian - old_jacobian)/duration.toSec();
        velocity_raw = (position_6d - old_position)/duration.toSec();
        velocity = velocity_alpha * velocity_raw + (1.0 - velocity_alpha) * old_velocity;
        accel = (velocity - old_velocity)/duration.toSec();
      }

      // non static update
      old_jacobian = jacobian;
      old_velocity = velocity;
      old_position = position_6d;

      // ask Mr. Stephen Butterworth to filter our data for us.
      Eigen::Matrix<double, 6, 1> sensor_fext = sensor_fext_raw;
      // Eigen::Matrix<double, 6, 1> sensor_fext = butterworth_filter(sensor_fext_raw);

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

      // Clamp fext to help prevent off-phase run away
      base_fext = base_fext.unaryExpr([&](double x) {
          return std::clamp(x, -force_limit, force_limit);
      });

      if (config[config_name]["swap_torque"]) {
        base_fext(3) = -base_fext(3);
        base_fext(4) = -base_fext(4);
        base_fext(5) = -base_fext(5);
      }
      if (config[config_name]["use_dummy_force"]) {
        base_fext = fext_func(fullCount/1000.0);
        std::cout << "Time: " << fullCount/1000.0 << std::endl;
      }

      //precompute velocity from jacobian for reuse
      Eigen::Matrix<double, 6, 1> jac_vel = jacobian * dq;

      // compute control MR 11.66
      Eigen::VectorXd ddx_d(6);
      ddx_d << virtual_mass.inverse() * (base_fext - (damping * jac_vel) - (stiffness * error));

      // compute boundry acceleration to keep EE in bounds
      if (use_boundry) {
        Eigen::VectorXd correction = 
            (position_6d - boundry_max).cwiseMax(0.0) +
            (position_6d - boundry_min).cwiseMin(0.0);

        Eigen::VectorXd ddx_b(6);
        ddx_b.setZero();
        // if out of bounds anywhere, apply corrective force and damp user movement
        if ((correction.head(3).array().abs() > 0.001).any()) {
            ddx_b.head(3) = -correction.head(3) * boundry_trans_stiffness - boundry_trans_damping * jac_vel.head(3);
        }
        if ((correction.tail(3).array().abs() > 0.001).any()) {
            ddx_b.tail(3) = -correction.tail(3) * boundry_rot_stiffness - boundry_rot_damping * jac_vel.tail(3);
        }

        ddx_d += ddx_b;
      }

      // apply damping above maximum velocity if we are too fast
      if (use_velocity_max) {
        Eigen::VectorXd ddx_v(6);
        ddx_v.setZero();
        Eigen::Array<bool, Eigen::Dynamic, 1> vel_checks = velocity.array().abs() > velocity_max.array();
        for (int i = 0; i < vel_checks.size(); ++i) {
          if (vel_checks[i]) {
              double excess = std::abs(velocity(i)) - velocity_max(i);
              double sign = (velocity(i) > 0) ? 1.0 : -1.0;
              ddx_v(i) = -sign * velocity_max_damping(i) * excess;
          }
        }
        ddx_d += ddx_v;
      }

      Eigen::VectorXd tau_task(7), tau_d(7);
      static Eigen::VectorXd last_task = Eigen::VectorXd::Zero(7);

      // MR 6.7 weighted pseudoinverse
      Eigen::MatrixXd weighted_pseudo_inverse = W_inv * jacobian.transpose() * (jacobian * W_inv * jacobian.transpose()).inverse();
      
      // translate EE accel to joint accel MR 11.66
      Eigen::VectorXd ddq_d(7);
      ddq_d << weighted_pseudo_inverse * (ddx_d - (djacobian * dq));
      
      // MR 8.1
      tau_task << mass * ddq_d;

      // inverse dynamics, add all control elements together
      tau_d << tau_task + coriolis;

      Eigen::VectorXd tau_friction(7);
      tau_friction.setZero();
      if (use_friction_comp) {
        Eigen::VectorXd dq_smooth_sign = dq.array() / (dq.array().square() + coulomb_epsilon * coulomb_epsilon).sqrt();

        // total friction comp
        tau_friction =  coulomb_frictions.cwiseProduct(dq_smooth_sign) + viscous_frictions.cwiseProduct(dq);
        tau_d += tau_friction;
      }

      //Spec sheet lists 1000/sec as maximum but in practice should be much lower for smooth human use.
      double max_torque_accel = torque_smoothing / 1000;
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
      predicted << position_6d; 

      if (expected_pos.size() > 0 && fullCount < (int)expected_pos.size() && config[config_name]["use_dummy_force"]) {
        predicted = expected_pos[fullCount];
      }
      fullCount++;

      if (count == 1) {
        queue_package new_package;
        new_package.desired_accel = ddx_d;
        new_package.actual_wrench = base_fext;
        new_package.orientation_error = orientation_error_axis_angle;
        new_package.translation = position;
        new_package.translation_d = predicted.head(3);
        new_package.velocity = velocity;
        new_package.accel = accel;
        new_package.torques_d = tau_d;
        new_package.torques_o = tau_J_d;
        new_package.torques_c = coriolis;
        new_package.torques_g = tau_J - gravity;
        new_package.torques_f = tau_friction;
        new_package.ddq_d = ddq_d;
        new_package.dq = dq;
        if (ros2_publish == "TRUE") {
          transfer_package.Produce(queue_package(new_package));
        }

        dump_vector[dump_index] = new_package;
        dump_index = (dump_index + 1) % MAX_BUFFER_SIZE;
        if (dump_index == 0) buffer_full = true;

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
  robot_dump(dump_vector, buffer_full, MAX_BUFFER_SIZE, dump_index);
  sensor.on_deactivate();
  return 0;
}
