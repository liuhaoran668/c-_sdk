#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>

#include "linkerhand/can_dispatcher.hpp"
#include "linkerhand/iterable_queue.hpp"

namespace linkerhand::hand::l6 {

struct CurrentData {
  std::array<int, 6> currents{};
  double timestamp = 0.0;
};

class CurrentManager {
 public:
  CurrentManager(std::uint32_t arbitration_id, CANMessageDispatcher& dispatcher);
  ~CurrentManager();

  CurrentManager(const CurrentManager&) = delete;
  CurrentManager& operator=(const CurrentManager&) = delete;

  CurrentData get_currents_blocking(double timeout_ms = 100);
  std::optional<CurrentData> get_current_currents() const;

  IterableQueue<CurrentData> stream(double interval_ms = 100, std::size_t maxsize = 100);
  void stop_streaming();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace linkerhand::hand::l6

