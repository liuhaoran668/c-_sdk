#include "linkerhand/hand/o6/torque_manager.hpp"

#include <cstddef>

#include "linkerhand/exceptions.hpp"

#include "../common.hpp"

namespace linkerhand::hand::o6 {
namespace {

constexpr std::uint8_t kControlCmd = 0x02;
constexpr std::size_t kTorqueCount = 6;

}  // namespace

TorqueManager::TorqueManager(std::uint32_t arbitration_id, CANMessageDispatcher& dispatcher)
    : arbitration_id_(arbitration_id), dispatcher_(dispatcher) {
  subscription_id_ = dispatcher_.subscribe([this](const CanMessage& msg) { on_message(msg); });
}

TorqueManager::~TorqueManager() { dispatcher_.unsubscribe(subscription_id_); }

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

std::optional<std::pair<std::array<int, 6>, double>> TorqueManager::get_current_torques() const {
  std::lock_guard<std::mutex> lock(latest_mutex_);
  if (!latest_data_.has_value()) {
    return std::nullopt;
  }
  return std::make_pair(latest_data_->torques, latest_data_->timestamp);
}

void TorqueManager::on_message(const CanMessage& msg) {
  if (msg.arbitration_id != arbitration_id_) {
    return;
  }
  if (msg.dlc < 2 || msg.data[0] != kControlCmd) {
    return;
  }

  const std::size_t payload_len = msg.dlc - 1;
  if (payload_len != kTorqueCount) {
    return;
  }

  TorqueData data{};
  for (std::size_t i = 0; i < kTorqueCount; ++i) {
    data.torques[i] = static_cast<int>(msg.data[1 + i]);
  }
  data.timestamp = detail::now_unix_seconds();

  std::lock_guard<std::mutex> lock(latest_mutex_);
  latest_data_ = data;
}

}  // namespace linkerhand::hand::o6
