#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>

#include "linkerhand/can_dispatcher.hpp"
#include "linkerhand/iterable_queue.hpp"

namespace linkerhand::hand::l6 {

struct TemperatureData {
  std::array<int, 6> temperatures{};
  double timestamp = 0.0;
};

class TemperatureManager {
 public:
  TemperatureManager(std::uint32_t arbitration_id, CANMessageDispatcher& dispatcher);
  ~TemperatureManager();

  TemperatureManager(const TemperatureManager&) = delete;
  TemperatureManager& operator=(const TemperatureManager&) = delete;

  TemperatureData get_temperatures_blocking(double timeout_ms = 100);
  std::optional<TemperatureData> get_current_temperatures() const;

  IterableQueue<TemperatureData> stream(double interval_ms = 100, std::size_t maxsize = 100);
  void stop_streaming();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace linkerhand::hand::l6

