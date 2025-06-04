#pragma once
#include <eigen3/Eigen/Dense>
#include "net_ft/hardware_interface.hpp"

class ftWrapper
{
public:
  ftWrapper();
  Eigen::Matrix<double, 6, 1> ft_read();

private:
  net_ft_driver::NetFtHardwareInterface sensor;
  Eigen::MatrixXd sensor_adjoint;
};