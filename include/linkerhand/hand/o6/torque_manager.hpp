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

struct TorqueData {
  std::array<int, 6> torques{};
  double timestamp = 0.0;
};

class TorqueManager {
 public:
  TorqueManager(std::uint32_t arbitration_id, CANMessageDispatcher& dispatcher);
  ~TorqueManager();

  TorqueManager(const TorqueManager&) = delete;
  TorqueManager& operator=(const TorqueManager&) = delete;

  void set_torques(const std::array<int, 6>& torques);
  void set_torques(const std::vector<int>& torques);

  std::optional<std::pair<std::array<int, 6>, double>> get_current_torques() const;

 private:
  void on_message(const CanMessage& msg);

  std::uint32_t arbitration_id_;
  CANMessageDispatcher& dispatcher_;
  std::size_t subscription_id_ = 0;

  mutable std::mutex latest_mutex_;
  std::optional<TorqueData> latest_data_;
};

}  // namespace linkerhand::hand::o6
