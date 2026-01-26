#include "linkerhand/hand/l6/current_manager.hpp"

#include "../common.hpp"

namespace linkerhand::hand::l6 {
namespace {

constexpr std::uint8_t kSenseCmd = 0x36;
constexpr std::size_t kCurrentCount = 6;

}  // namespace

void CurrentManager::do_send_sense_request() {
  CanMessage msg{};
  msg.arbitration_id = arbitration_id_;
  msg.is_extended_id = false;
  msg.dlc = 1;
  msg.data[0] = kSenseCmd;
  dispatcher_.send(msg);
}

std::optional<CurrentData> CurrentManager::do_parse_message(const CanMessage& msg) {
  if (msg.dlc < 2 || msg.data[0] != kSenseCmd) {
    return std::nullopt;
  }

  const std::size_t payload_len = msg.dlc - 1;
  if (payload_len != kCurrentCount) {
    return std::nullopt;
  }

  CurrentData data{};
  for (std::size_t i = 0; i < kCurrentCount; ++i) {
    data.currents[i] = static_cast<int>(msg.data[1 + i]);
  }
  data.timestamp = detail::now_unix_seconds();
  return data;
}

}  // namespace linkerhand::hand::l6
