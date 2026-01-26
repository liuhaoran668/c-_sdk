#include "linkerhand/hand/l6/speed_manager.hpp"

#include <cstddef>

#include "linkerhand/exceptions.hpp"

#include "../common.hpp"

namespace linkerhand::hand::l6 {
namespace {

constexpr std::uint8_t kControlCmd = 0x05;
constexpr std::size_t kSpeedCount = 6;

}  // namespace

SpeedManager::SpeedManager(std::uint32_t arbitration_id, CANMessageDispatcher& dispatcher)
    : arbitration_id_(arbitration_id), dispatcher_(dispatcher) {}

void SpeedManager::set_speeds(const std::array<int, 6>& speeds) {
  detail::validate_fixed_u8_array(speeds, /*max_inclusive=*/255, "Speed");
  CanMessage msg{};
  msg.arbitration_id = arbitration_id_;
  msg.is_extended_id = false;
  msg.dlc = 1 + kSpeedCount;
  msg.data[0] = kControlCmd;
  const auto bytes = detail::to_u8_array(speeds);
  for (std::size_t i = 0; i < kSpeedCount; ++i) {
    msg.data[1 + i] = bytes[i];
  }
  dispatcher_.send(msg);
}

void SpeedManager::set_speeds(const std::vector<int>& speeds) {
  detail::validate_fixed_u8_vector(
      speeds, kSpeedCount, /*max_inclusive=*/255, "speeds", "Speed");
  CanMessage msg{};
  msg.arbitration_id = arbitration_id_;
  msg.is_extended_id = false;
  msg.dlc = 1 + kSpeedCount;
  msg.data[0] = kControlCmd;
  const auto bytes = detail::to_u8_array_6(speeds);
  for (std::size_t i = 0; i < kSpeedCount; ++i) {
    msg.data[1 + i] = bytes[i];
  }
  dispatcher_.send(msg);
}

}  // namespace linkerhand::hand::l6
