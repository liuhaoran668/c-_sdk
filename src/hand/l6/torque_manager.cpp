#include "linkerhand/hand/l6/torque_manager.hpp"

#include "../common.hpp"

namespace linkerhand::hand::l6 {
namespace {

constexpr std::uint8_t kControlCmd = 0x02;
constexpr std::size_t kTorqueCount = 6;

}  // namespace

void TorqueManager::set_torques(const std::array<int, 6>& torques) {
  detail::validate_fixed_u8_array(torques, /*max_inclusive=*/255, "Torque");

  CanMessage msg{};
  msg.arbitration_id = arbitration_id_;
  msg.is_extended_id = false;
  msg.dlc = 1 + kTorqueCount;
  msg.data[0] = kControlCmd;
  const auto bytes = detail::to_u8_array(torques);
  for (std::size_t i = 0; i < kTorqueCount; ++i) {
    msg.data[1 + i] = bytes[i];
  }
  dispatcher_.send(msg);
}

void TorqueManager::set_torques(const std::vector<int>& torques) {
  detail::validate_fixed_u8_vector(
      torques, kTorqueCount, /*max_inclusive=*/255, "torques", "Torque");

  CanMessage msg{};
  msg.arbitration_id = arbitration_id_;
  msg.is_extended_id = false;
  msg.dlc = 1 + kTorqueCount;
  msg.data[0] = kControlCmd;
  const auto bytes = detail::to_u8_array_6(torques);
  for (std::size_t i = 0; i < kTorqueCount; ++i) {
    msg.data[1 + i] = bytes[i];
  }
  dispatcher_.send(msg);
}

void TorqueManager::do_send_sense_request() {
  CanMessage msg{};
  msg.arbitration_id = arbitration_id_;
  msg.is_extended_id = false;
  msg.dlc = 1;
  msg.data[0] = kControlCmd;
  dispatcher_.send(msg);
}

std::optional<TorqueData> TorqueManager::do_parse_message(const CanMessage& msg) {
  if (msg.dlc < 2 || msg.data[0] != kControlCmd) {
    return std::nullopt;
  }

  const std::size_t payload_len = msg.dlc - 1;
  if (payload_len != kTorqueCount) {
    return std::nullopt;
  }

  TorqueData data{};
  for (std::size_t i = 0; i < kTorqueCount; ++i) {
    data.torques[i] = static_cast<int>(msg.data[1 + i]);
  }
  data.timestamp = detail::now_unix_seconds();
  return data;
}

}  // namespace linkerhand::hand::l6
