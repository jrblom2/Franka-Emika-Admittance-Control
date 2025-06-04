#include "ft_sensor.hpp"

ftWrapper::ftWrapper() {
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
    this->sensor_adjoint = sensor_adjoint;
    this->sensor = net_ft_driver::NetFtHardwareInterface(input);
    std::cout << "Sensor started." << std::endl;
}

Eigen::Matrix<double, 6, 1> ftWrapper::ft_read() {
    this->sensor.read();
    std::array<double, 6> ft_reading = this->sensor.ft_sensor_measurements_;
    Eigen::Map<Eigen::Matrix<double, 6, 1>> fext(ft_reading.data());
    // TODO wrench translations should all be part of the adjoint
    // translate wrench from FT sensor as wrench in EE frame. MR 3.98
    fext = sensor_adjoint.transpose() * fext;
    // swap sign for gravity
    fext(2) = -fext(2);
    // swap sign for x-axis
    fext(0) = -fext(0);
    // torque in Z and X already resist user, invert Y to also resist user
    fext(4) = -fext(4);
    return fext;
}