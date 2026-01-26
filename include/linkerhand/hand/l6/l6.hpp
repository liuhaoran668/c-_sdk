#pragma once

#include <cstdint>
#include <string>

#include "linkerhand/can_dispatcher.hpp"
#include "linkerhand/exceptions.hpp"
#include "linkerhand/hand/l6/angle_manager.hpp"
#include "linkerhand/hand/l6/current_manager.hpp"
#include "linkerhand/hand/l6/fault_manager.hpp"
#include "linkerhand/hand/l6/force_sensor_manager.hpp"
#include "linkerhand/hand/l6/speed_manager.hpp"
#include "linkerhand/hand/l6/temperature_manager.hpp"
#include "linkerhand/hand/l6/torque_manager.hpp"

namespace linkerhand::hand::l6 {

class L6 {
 private:
  void ensure_open() const;

  CANMessageDispatcher dispatcher_;
  std::uint32_t arbitration_id_ = 0x28;
  bool closed_ = false;

 public:
  L6(const std::string& side, const std::string& interface_name, const std::string& interface_type = "socketcan");
  ~L6();

  L6(const L6&) = delete;
  L6& operator=(const L6&) = delete;

  AngleManager angle;
  ForceSensorManager force_sensor;
  TorqueManager torque;
  SpeedManager speed;
  TemperatureManager temperature;
  CurrentManager current;
  FaultManager fault;

  void close();
  bool is_closed() const;
};

}  // namespace linkerhand::hand::l6
