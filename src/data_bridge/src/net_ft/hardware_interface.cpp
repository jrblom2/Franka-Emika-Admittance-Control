// Copyright (c) 2022, Grzegorz Bartyzel
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "./hardware_interface.hpp"

#include <memory>
#include <string>
#include <vector>

#include "./net_ft_interface.hpp"

namespace net_ft_driver
{
NetFtHardwareInterface::NetFtHardwareInterface(const ft_info& info)
{
  // if (hardware_interface::SensorInterface::on_init(info) != CallbackReturn::SUCCESS) {
  //   return 1;
  // }

  ft_sensor_measurements_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  lost_packets_ = 0;
  packet_count_ = 0;
  out_of_order_count_ = 0;
  status_ = 0;

  ip_address_ = info.ip_address;
  sensor_type_ = info.sensor_type;
  use_hardware_biasing_ = info.use_biasing;
  int rdt_rate = info.rdt_sampling_rate;
  int internal_filter_rate = info.internal_filter_rate;

  driver_ = NetFTInterface::create(sensor_type_, ip_address_);
  if (!driver_->set_sampling_rate(rdt_rate)) {
    std::cerr << "Couldn't set RDT sampling rate of the F/T Sensor!" << std::endl;
    throw std::runtime_error("");
  }

  if (!driver_->set_internal_filter(internal_filter_rate)) {
    std::cerr << "Couldn't set internal low pass filter!" << std::endl;
    throw std::runtime_error("");
  }

  std::cout << "Initialize connection with F/T Sensor" << std::endl;
  int activate_error = on_activate();
  if (activate_error) {
    throw std::runtime_error("");
  }
}
// std::vector<hardware_interface::StateInterface> NetFtHardwareInterface::export_state_interfaces()
// {
//   std::vector<hardware_interface::StateInterface> state_interfaces;

//   for (auto& sensor : info_.sensors) {
//     for (size_t j = 0; j < sensor.state_interfaces.size(); ++j) {
//       state_interfaces.emplace_back(hardware_interface::StateInterface(sensor.name, sensor.state_interfaces[j].name,
//                                                                        &ft_sensor_measurements_[j]));
//     }
//   }

//   state_interfaces.emplace_back(hardware_interface::StateInterface("diagnostic", "packet_count", &packet_count_));
//   state_interfaces.emplace_back(hardware_interface::StateInterface("diagnostic", "lost_packets", &lost_packets_));
//   state_interfaces.emplace_back(
//       hardware_interface::StateInterface("diagnostic", "out_of_order_count", &out_of_order_count_));
//   state_interfaces.emplace_back(hardware_interface::StateInterface("diagnostic", "status", &status_));
//   return state_interfaces;
// }

int NetFtHardwareInterface::on_activate()
{
  if (driver_->start_streaming()) {
    if (use_hardware_biasing_ == "True" || use_hardware_biasing_ == "true") {
      if (!driver_->set_bias()) {
        std::cerr << "Couldn't zero sensor with software bias!" << std::endl;
        return 1;
      }
    } else {
      if (!driver_->clear_bias()) {
        std::cerr << "Couldn't clear sensor software bias!" << std::endl;
        return 1;
      }
    }
    std::unique_ptr<SensorData> data = driver_->receive_data();
    if (data) {
      ft_sensor_measurements_ = data->ft_values;
      std::cout << "Successfully started data streaming!" << std::endl;
      return 0;
    }
  }
  std::cerr << "The data stream could not be started!" << std::endl;
  return 1;
}

int NetFtHardwareInterface::re_bias()
{
  if (!driver_->set_bias()) {
        std::cerr << "Couldn't zero sensor with software bias!" << std::endl;
        return 1;
  }
  std::cout << "Zeroed sensor." << std::endl;
  return 0;
}

int NetFtHardwareInterface::on_deactivate()
{
  if (driver_->stop_streaming()) {
    std::cout << "Successfully stoped data streaming!" << std::endl;
    return 0;
  }
  std::cerr << "The data stream could not be stopped!" << std::endl;
  return 1;
}

int NetFtHardwareInterface::read()
{
  auto data = driver_->receive_data();
  if (data) {
    ft_sensor_measurements_ = data->ft_values;
    lost_packets_ = static_cast<double>(data->lost_packets);
    packet_count_ = static_cast<double>(data->packet_count);
    out_of_order_count_ = static_cast<double>(data->out_of_order_count);
    status_ = static_cast<double>(data->status);
  }
  return 1;
}
}  // namespace net_ft_driver
