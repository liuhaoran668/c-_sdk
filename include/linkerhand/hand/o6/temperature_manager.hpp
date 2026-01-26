#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>

#include "linkerhand/can_dispatcher.hpp"
#include "linkerhand/iterable_queue.hpp"

namespace linkerhand::hand::o6 {

struct TemperatureData {
  std::array<int, 6> temps{};
  double timestamp = 0.0;
};

class TemperatureManager {
 public:
  TemperatureManager(std::uint32_t arbitration_id, CANMessageDispatcher& dispatcher);
  ~TemperatureManager();

  TemperatureManager(const TemperatureManager&) = delete;
  TemperatureManager& operator=(const TemperatureManager&) = delete;

  std::array<int, 6> get_temperatures_blocking(double timeout_ms = 100);
  std::optional<std::pair<std::array<int, 6>, double>> get_current_temperatures() const;

  IterableQueue<TemperatureData> stream(double interval_ms = 200, std::size_t maxsize = 100);
  void stop_streaming();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace linkerhand::hand::o6

