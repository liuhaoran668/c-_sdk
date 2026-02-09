#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "linkerhand/can_dispatcher.hpp"
#include "linkerhand/exceptions.hpp"
#include "linkerhand/hand/o6/angle_manager.hpp"
#include "linkerhand/hand/o6/force_sensor_manager.hpp"
#include "linkerhand/lifecycle.hpp"

namespace linkerhand::hand::o6 {

class O6 {
 private:
  std::shared_ptr<linkerhand::Lifecycle> lifecycle_;
  CANMessageDispatcher dispatcher_;
  std::uint32_t arbitration_id_ = 0x28;

 public:
  O6(const std::string& side, const std::string& interface_name, const std::string& interface_type = "socketcan");
  ~O6();

  O6(const O6&) = delete;
  O6& operator=(const O6&) = delete;

  AngleManager angle;
  ForceSensorManager force_sensor;

  void close();
  bool is_closed() const;
};

}  // namespace linkerhand::hand::o6
