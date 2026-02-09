#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>

#include "linkerhand/can_dispatcher.hpp"
#include "linkerhand/iterable_queue.hpp"
#include "linkerhand/lifecycle.hpp"

namespace linkerhand::hand::o6 {

struct ForceSensorData {
  std::array<std::uint8_t, 72> values{};
  double timestamp = 0.0;
};

struct AllFingersData {
  ForceSensorData thumb;
  ForceSensorData index;
  ForceSensorData middle;
  ForceSensorData ring;
  ForceSensorData pinky;
};

class SingleForceSensorManager {
 public:
  SingleForceSensorManager(
      std::uint32_t arbitration_id,
      CANMessageDispatcher& dispatcher,
      std::uint8_t command_prefix);
  SingleForceSensorManager(
      std::uint32_t arbitration_id,
      CANMessageDispatcher& dispatcher,
      std::uint8_t command_prefix,
      std::shared_ptr<linkerhand::Lifecycle> lifecycle);
  ~SingleForceSensorManager();

  SingleForceSensorManager(const SingleForceSensorManager&) = delete;
  SingleForceSensorManager& operator=(const SingleForceSensorManager&) = delete;

  ForceSensorData get_data_blocking(double timeout_ms = 1000);
  std::optional<ForceSensorData> get_latest_data() const;

  IterableQueue<ForceSensorData> stream(double interval_ms = 100, std::size_t maxsize = 100);
  void stop_streaming();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  std::shared_ptr<linkerhand::Lifecycle> lifecycle_;
};

class ForceSensorManager {
 public:
  ForceSensorManager(std::uint32_t arbitration_id, CANMessageDispatcher& dispatcher);
  ForceSensorManager(
      std::uint32_t arbitration_id,
      CANMessageDispatcher& dispatcher,
      std::shared_ptr<linkerhand::Lifecycle> lifecycle);
  ~ForceSensorManager();

  ForceSensorManager(const ForceSensorManager&) = delete;
  ForceSensorManager& operator=(const ForceSensorManager&) = delete;

  AllFingersData get_data_blocking(double timeout_ms = 1000);

  IterableQueue<AllFingersData> stream(double interval_ms = 100, std::size_t maxsize = 100);
  void stop_streaming();

  std::unordered_map<std::string, std::optional<ForceSensorData>> get_latest_data() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  std::shared_ptr<linkerhand::Lifecycle> lifecycle_;
};

}  // namespace linkerhand::hand::o6
