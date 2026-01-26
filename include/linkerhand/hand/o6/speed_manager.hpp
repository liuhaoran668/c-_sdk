#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <utility>
#include <vector>

#include "linkerhand/can_dispatcher.hpp"

namespace linkerhand::hand::o6 {

struct SpeedData {
  std::array<int, 6> speeds{};
  double timestamp = 0.0;
};

struct AccelerationData {
  std::array<int, 6> accelerations{};
  double timestamp = 0.0;
};

class SpeedManager {
 public:
  SpeedManager(std::uint32_t arbitration_id, CANMessageDispatcher& dispatcher);
  ~SpeedManager();

  SpeedManager(const SpeedManager&) = delete;
  SpeedManager& operator=(const SpeedManager&) = delete;

  void set_speeds(const std::array<int, 6>& speeds);
  void set_speeds(const std::vector<int>& speeds);

  void set_accelerations(const std::array<int, 6>& accelerations);
  void set_accelerations(const std::vector<int>& accelerations);

  std::optional<std::pair<std::array<int, 6>, double>> get_current_speeds() const;
  std::optional<std::pair<std::array<int, 6>, double>> get_current_accelerations() const;

 private:
  void on_message(const CanMessage& msg);

  std::uint32_t arbitration_id_;
  CANMessageDispatcher& dispatcher_;
  std::size_t subscription_id_ = 0;

  mutable std::mutex latest_mutex_;
  std::optional<SpeedData> latest_speed_data_;
  std::optional<AccelerationData> latest_acceleration_data_;
};

}  // namespace linkerhand::hand::o6

