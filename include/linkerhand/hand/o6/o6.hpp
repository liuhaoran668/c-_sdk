#pragma once

#include <atomic>
#include <cstdint>
#include <string>

#include "linkerhand/can_dispatcher.hpp"
#include "linkerhand/exceptions.hpp"
#include "linkerhand/hand/o6/angle_manager.hpp"
#include "linkerhand/hand/o6/speed_manager.hpp"
#include "linkerhand/hand/o6/temperature_manager.hpp"
#include "linkerhand/hand/o6/torque_manager.hpp"

namespace linkerhand::hand::o6 {

class O6 {
 private:
  void ensure_open() const;

  CANMessageDispatcher dispatcher_;
  std::uint32_t arbitration_id_ = 0x28;
  std::atomic<bool> closed_{false};

 public:
  O6(const std::string& side, const std::string& interface_name, const std::string& interface_type = "socketcan");
  ~O6();

  O6(const O6&) = delete;
  O6& operator=(const O6&) = delete;

  AngleManager angle;
  TorqueManager torque;
  SpeedManager speed;
  TemperatureManager temperature;

  void close();
  bool is_closed() const;
};

}  // namespace linkerhand::hand::o6
