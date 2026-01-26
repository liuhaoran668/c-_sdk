#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "linkerhand/can_dispatcher.hpp"
#include "linkerhand/iterable_queue.hpp"

namespace linkerhand::hand::o6 {

struct AngleData {
  std::array<int, 6> angles{};
  double timestamp = 0.0;
};

class AngleManager {
 public:
  AngleManager(std::uint32_t arbitration_id, CANMessageDispatcher& dispatcher);
  ~AngleManager();

  AngleManager(const AngleManager&) = delete;
  AngleManager& operator=(const AngleManager&) = delete;

  void set_angles(const std::array<int, 6>& angles);
  void set_angles(const std::vector<int>& angles);

  std::array<int, 6> get_angles_blocking(double timeout_ms = 100);
  std::optional<std::pair<std::array<int, 6>, double>> get_current_angles() const;

  IterableQueue<AngleData> stream(double interval_ms = 100, std::size_t maxsize = 100);
  void stop_streaming();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace linkerhand::hand::o6

