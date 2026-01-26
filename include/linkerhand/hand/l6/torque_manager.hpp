#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "linkerhand/can_dispatcher.hpp"
#include "linkerhand/iterable_queue.hpp"

namespace linkerhand::hand::l6 {

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

  TorqueData get_torques_blocking(double timeout_ms = 100);
  std::optional<TorqueData> get_current_torques() const;

  IterableQueue<TorqueData> stream(double interval_ms = 100, std::size_t maxsize = 100);
  void stop_streaming();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace linkerhand::hand::l6

