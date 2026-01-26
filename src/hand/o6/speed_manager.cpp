#include "linkerhand/hand/o6/speed_manager.hpp"

#include <cstddef>

#include "../common.hpp"

namespace linkerhand::hand::o6 {
namespace {

constexpr std::uint8_t kSpeedCmd = 0x05;
constexpr std::uint8_t kAccelerationCmd = 0x07;
constexpr std::size_t kCount = 6;

}  // namespace

SpeedManager::SpeedManager(std::uint32_t arbitration_id, CANMessageDispatcher& dispatcher)
    : arbitration_id_(arbitration_id), dispatcher_(dispatcher) {
  subscription_id_ = dispatcher_.subscribe([this](const CanMessage& msg) { on_message(msg); });
}

SpeedManager::~SpeedManager() { dispatcher_.unsubscribe(subscription_id_); }

void SpeedManager::set_speeds(const std::array<int, 6>& speeds) {
  detail::validate_fixed_u8_array(speeds, /*max_inclusive=*/255, "Speed");
  CanMessage msg{};
  msg.arbitration_id = arbitration_id_;
  msg.is_extended_id = false;
  msg.dlc = 1 + kCount;
  msg.data[0] = kSpeedCmd;
  const auto bytes = detail::to_u8_array(speeds);
  for (std::size_t i = 0; i < kCount; ++i) {
    msg.data[1 + i] = bytes[i];
  }
  dispatcher_.send(msg);
}

void SpeedManager::set_speeds(const std::vector<int>& speeds) {
  detail::validate_fixed_u8_vector(
      speeds, kCount, /*max_inclusive=*/255, "speeds", "Speed");
  CanMessage msg{};
  msg.arbitration_id = arbitration_id_;
  msg.is_extended_id = false;
  msg.dlc = 1 + kCount;
  msg.data[0] = kSpeedCmd;
  const auto bytes = detail::to_u8_array_6(speeds);
  for (std::size_t i = 0; i < kCount; ++i) {
    msg.data[1 + i] = bytes[i];
  }
  dispatcher_.send(msg);
}

void SpeedManager::set_accelerations(const std::array<int, 6>& accelerations) {
  detail::validate_fixed_u8_array(accelerations, /*max_inclusive=*/254, "Acceleration");
  CanMessage msg{};
  msg.arbitration_id = arbitration_id_;
  msg.is_extended_id = false;
  msg.dlc = 1 + kCount;
  msg.data[0] = kAccelerationCmd;
  const auto bytes = detail::to_u8_array(accelerations);
  for (std::size_t i = 0; i < kCount; ++i) {
    msg.data[1 + i] = bytes[i];
  }
  dispatcher_.send(msg);
}

void SpeedManager::set_accelerations(const std::vector<int>& accelerations) {
  detail::validate_fixed_u8_vector(
      accelerations, kCount, /*max_inclusive=*/254, "accelerations", "Acceleration");
  CanMessage msg{};
  msg.arbitration_id = arbitration_id_;
  msg.is_extended_id = false;
  msg.dlc = 1 + kCount;
  msg.data[0] = kAccelerationCmd;
  const auto bytes = detail::to_u8_array_6(accelerations);
  for (std::size_t i = 0; i < kCount; ++i) {
    msg.data[1 + i] = bytes[i];
  }
  dispatcher_.send(msg);
}

std::optional<std::pair<std::array<int, 6>, double>> SpeedManager::get_current_speeds() const {
  std::lock_guard<std::mutex> lock(latest_mutex_);
  if (!latest_speed_data_.has_value()) {
    return std::nullopt;
  }
  return std::make_pair(latest_speed_data_->speeds, latest_speed_data_->timestamp);
}

std::optional<std::pair<std::array<int, 6>, double>> SpeedManager::get_current_accelerations() const {
  std::lock_guard<std::mutex> lock(latest_mutex_);
  if (!latest_acceleration_data_.has_value()) {
    return std::nullopt;
  }
  return std::make_pair(latest_acceleration_data_->accelerations, latest_acceleration_data_->timestamp);
}

void SpeedManager::on_message(const CanMessage& msg) {
  if (msg.arbitration_id != arbitration_id_) {
    return;
  }
  if (msg.dlc < 2) {
    return;
  }

  const std::size_t payload_len = msg.dlc - 1;
  if (payload_len != kCount) {
    return;
  }

  if (msg.data[0] == kSpeedCmd) {
    SpeedData data{};
    for (std::size_t i = 0; i < kCount; ++i) {
      data.speeds[i] = static_cast<int>(msg.data[1 + i]);
    }
    data.timestamp = detail::now_unix_seconds();

    std::lock_guard<std::mutex> lock(latest_mutex_);
    latest_speed_data_ = data;
    return;
  }

  if (msg.data[0] == kAccelerationCmd) {
    AccelerationData data{};
    for (std::size_t i = 0; i < kCount; ++i) {
      data.accelerations[i] = static_cast<int>(msg.data[1 + i]);
    }
    data.timestamp = detail::now_unix_seconds();

    std::lock_guard<std::mutex> lock(latest_mutex_);
    latest_acceleration_data_ = data;
    return;
  }
}

}  // namespace linkerhand::hand::o6
