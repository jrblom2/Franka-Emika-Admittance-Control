#include "./hardware_interface.hpp"

int main() {

    net_ft_driver::ft_info input;
    input.ip_address = "192.168.18.81";
    input.sensor_type = "ati_axia";
    input.rdt_sampling_rate = 1000;
    input.use_biasing = "true";
    input.internal_filter_rate = 0;

    net_ft_driver::NetFtHardwareInterface sensor = net_ft_driver::NetFtHardwareInterface(input);
    double Fx = 0.0;
    double Fy = 0.0;
    double Fz = 0.0;
    double Tx = 0.0;
    double Ty = 0.0;
    double Tz = 0.0;
    while (true) {
        sensor.read();
        Fx = sensor.ft_sensor_measurements_[0];
        Fy = sensor.ft_sensor_measurements_[1];
        Fz = sensor.ft_sensor_measurements_[2];
        Tx = sensor.ft_sensor_measurements_[3];
        Ty = sensor.ft_sensor_measurements_[4];
        Tz = sensor.ft_sensor_measurements_[5];
        std::cout << "Fx: " << Fx
                << " Fy: " << Fy
                << " Fz: " << Fz
                << " Tx: " << Tx
                << " Ty: " << Ty
                << " Tz: " << Tz
        << std::endl;
    }
    return 0;
}