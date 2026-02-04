#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "linkerhand/can_dispatcher.hpp"
#include "linkerhand/exceptions.hpp"
#include "linkerhand/hand/l6/angle_manager.hpp"
#include "linkerhand/hand/l6/force_sensor_manager.hpp"
#include "linkerhand/lifecycle.hpp"

namespace linkerhand::hand::l6 {

class L6 {
 private:
  std::shared_ptr<linkerhand::Lifecycle> lifecycle_;
  CANMessageDispatcher dispatcher_;
  std::uint32_t arbitration_id_ = 0x28;

 public:
  L6(const std::string& side, const std::string& interface_name, const std::string& interface_type = "socketcan");
  ~L6();

  L6(const L6&) = delete;
  L6& operator=(const L6&) = delete;

  AngleManager angle;
  ForceSensorManager force_sensor;

  void close();
  bool is_closed() const;
};

}  // namespace linkerhand::hand::l6
