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

#ifndef NET_FT_DRIVER__HARDWARE_INTERFACE_HPP_
#define NET_FT_DRIVER__HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "./net_ft_interface.hpp"
#include "./visibility_control.h"

namespace net_ft_driver
{
struct ft_info {
  std::string ip_address;
  std::string sensor_type;
  std::string use_biasing;
  int rdt_sampling_rate;
  int internal_filter_rate;
};
class NetFtHardwareInterface
{
public:

  NET_FT_DRIVER_PUBLIC
  NetFtHardwareInterface(const ft_info& info);

  // NET_FT_DRIVER_PUBLIC
  // std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  NET_FT_DRIVER_PUBLIC
  int on_activate();

  NET_FT_DRIVER_PUBLIC
  int on_deactivate();

  NET_FT_DRIVER_PUBLIC
  int read();

  Vector6D ft_sensor_measurements_;
private:
  std::unique_ptr<NetFTInterface> driver_;

  std::string ip_address_;
  std::string sensor_type_;
  std::string use_hardware_biasing_;

  Vector6D offset_ft_values_;

  double packet_count_;
  double lost_packets_;
  double out_of_order_count_;
  double status_;
};
}  // namespace net_ft_driver

#endif  // NET_FT_DRIVER__HARDWARE_INTERFACE_HPP_
