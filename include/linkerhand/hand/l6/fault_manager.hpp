#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

#include "linkerhand/can_dispatcher.hpp"

namespace linkerhand::hand::l6 {

class FaultManager {
 public:
  FaultManager(std::uint32_t arbitration_id, CANMessageDispatcher& dispatcher);

  FaultManager(const FaultManager&) = delete;
  FaultManager& operator=(const FaultManager&) = delete;

  void clear_faults(std::optional<std::array<int, 6>> joints = std::nullopt, bool all = false);
  void clear_faults(const std::vector<int>& joints);

 private:
  std::uint32_t arbitration_id_;
  CANMessageDispatcher& dispatcher_;
};

}  // namespace linkerhand::hand::l6
