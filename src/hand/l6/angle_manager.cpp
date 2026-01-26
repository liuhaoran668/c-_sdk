#include "linkerhand/hand/l6/angle_manager.hpp"

#include "../common.hpp"

namespace linkerhand::hand::l6 {
namespace {

constexpr std::uint8_t kControlCmd = 0x01;
constexpr std::size_t kAngleCount = 6;

}  // namespace

void AngleManager::set_angles(const std::array<int, 6>& angles) {
  detail::validate_fixed_u8_array(angles, /*max_inclusive=*/255, "Angle");

  CanMessage msg{};
  msg.arbitration_id = arbitration_id_;
  msg.is_extended_id = false;
  msg.dlc = 1 + kAngleCount;
  msg.data[0] = kControlCmd;
  const auto bytes = detail::to_u8_array(angles);
  for (std::size_t i = 0; i < kAngleCount; ++i) {
    msg.data[1 + i] = bytes[i];
  }
  dispatcher_.send(msg);
}

void AngleManager::set_angles(const std::vector<int>& angles) {
  detail::validate_fixed_u8_vector(
      angles, kAngleCount, /*max_inclusive=*/255, "angles", "Angle");

  CanMessage msg{};
  msg.arbitration_id = arbitration_id_;
  msg.is_extended_id = false;
  msg.dlc = 1 + kAngleCount;
  msg.data[0] = kControlCmd;
  const auto bytes = detail::to_u8_array_6(angles);
  for (std::size_t i = 0; i < kAngleCount; ++i) {
    msg.data[1 + i] = bytes[i];
  }
  dispatcher_.send(msg);
}

void AngleManager::do_send_sense_request() {
  CanMessage msg{};
  msg.arbitration_id = arbitration_id_;
  msg.is_extended_id = false;
  msg.dlc = 1;
  msg.data[0] = kControlCmd;
  dispatcher_.send(msg);
}

std::optional<AngleData> AngleManager::do_parse_message(const CanMessage& msg) {
  if (msg.dlc < 2 || msg.data[0] != kControlCmd) {
    return std::nullopt;
  }

  const std::size_t payload_len = msg.dlc - 1;
  if (payload_len != kAngleCount) {
    return std::nullopt;
  }

  AngleData data{};
  for (std::size_t i = 0; i < kAngleCount; ++i) {
    data.angles[i] = static_cast<int>(msg.data[1 + i]);
  }
  data.timestamp = detail::now_unix_seconds();
  return data;
}

}  // namespace linkerhand::hand::l6
