#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "linkerhand/can_dispatcher.hpp"

namespace linkerhand::hand::l6 {

class SpeedManager {
 public:
  SpeedManager(std::uint32_t arbitration_id, CANMessageDispatcher& dispatcher);

  SpeedManager(const SpeedManager&) = delete;
  SpeedManager& operator=(const SpeedManager&) = delete;

  void set_speeds(const std::array<int, 6>& speeds);
  void set_speeds(const std::vector<int>& speeds);

 private:
  std::uint32_t arbitration_id_;
  CANMessageDispatcher& dispatcher_;
};

}  // namespace linkerhand::hand::l6

