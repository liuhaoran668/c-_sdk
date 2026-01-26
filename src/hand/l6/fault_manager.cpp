#include "linkerhand/hand/l6/fault_manager.hpp"

#include <cstddef>

#include "linkerhand/exceptions.hpp"

namespace linkerhand::hand::l6 {
namespace {

constexpr std::uint8_t kClearFaultCmd = 0x83;
constexpr std::size_t kJointCount = 6;

}  // namespace

FaultManager::FaultManager(std::uint32_t arbitration_id, CANMessageDispatcher& dispatcher)
    : arbitration_id_(arbitration_id), dispatcher_(dispatcher) {}

void FaultManager::clear_faults(std::optional<std::array<int, 6>> joints, bool all) {
  if (all) {
    joints = std::array<int, 6>{1, 1, 1, 1, 1, 1};
  } else if (!joints.has_value()) {
    throw ValidationError("Must provide joints or set all=True");
  }

  const auto& flags = *joints;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    if (flags[i] != 0 && flags[i] != 1) {
      throw ValidationError(
          "Joint " + std::to_string(i) + " flag value " + std::to_string(flags[i]) +
          " must be 0 or 1");
    }
  }

  CanMessage msg{};
  msg.arbitration_id = arbitration_id_;
  msg.is_extended_id = false;
  msg.dlc = 1 + kJointCount;
  msg.data[0] = kClearFaultCmd;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    msg.data[1 + i] = static_cast<std::uint8_t>(flags[i]);
  }
  dispatcher_.send(msg);
}

void FaultManager::clear_faults(const std::vector<int>& joints) {
  if (joints.size() != kJointCount) {
    throw ValidationError(
        "Expected " + std::to_string(kJointCount) + " joint flags, got " +
        std::to_string(joints.size()));
  }
  std::array<int, 6> flags{};
  for (std::size_t i = 0; i < kJointCount; ++i) {
    flags[i] = joints[i];
  }
  clear_faults(std::optional<std::array<int, 6>>(flags), /*all=*/false);
}

}  // namespace linkerhand::hand::l6
