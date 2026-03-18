#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>

#include "linkerhand/can_dispatcher.hpp"
#include "linkerhand/iterable_queue.hpp"
#include "linkerhand/lifecycle.hpp"

namespace linkerhand::hand::l6 {

enum class Finger : std::uint8_t {
  Thumb = 0,
  Index = 1,
  Middle = 2,
  Ring = 3,
  Pinky = 4,
};

inline constexpr std::size_t kFingerCount = 5;

template <typename T>
using FingerArray = std::array<T, kFingerCount>;

struct ForceSensorData {
  std::array<std::uint8_t, 72> values{};
  double timestamp = 0.0;
};

struct AllFingersData {
  FingerArray<ForceSensorData> fingers{};

  ForceSensorData& operator[](Finger f) { return fingers[static_cast<std::size_t>(f)]; }
  const ForceSensorData& operator[](Finger f) const { return fingers[static_cast<std::size_t>(f)]; }
};

class SingleForceSensorManager {
 public:
  SingleForceSensorManager(
      std::uint32_t arbitration_id,
      CANMessageDispatcher& dispatcher,
      std::uint8_t command_prefix,
      std::shared_ptr<linkerhand::Lifecycle> lifecycle);
  ~SingleForceSensorManager();

  SingleForceSensorManager(const SingleForceSensorManager&) = delete;
  SingleForceSensorManager& operator=(const SingleForceSensorManager&) = delete;

  ForceSensorData get_data_blocking(std::chrono::milliseconds timeout = std::chrono::milliseconds{1000});
  std::optional<ForceSensorData> get_latest_data() const;

  IterableQueue<ForceSensorData> stream(std::chrono::milliseconds interval = std::chrono::milliseconds{100},
                                        std::size_t maxsize = 100);
  void stop_streaming();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  std::shared_ptr<linkerhand::Lifecycle> lifecycle_;
};

class ForceSensorManager {
 public:
  ForceSensorManager(
      std::uint32_t arbitration_id,
      CANMessageDispatcher& dispatcher,
      std::shared_ptr<linkerhand::Lifecycle> lifecycle);
  ~ForceSensorManager();

  ForceSensorManager(const ForceSensorManager&) = delete;
  ForceSensorManager& operator=(const ForceSensorManager&) = delete;

  AllFingersData get_data_blocking(std::chrono::milliseconds timeout = std::chrono::milliseconds{1000});

  IterableQueue<AllFingersData> stream(std::chrono::milliseconds interval = std::chrono::milliseconds{100},
                                       std::size_t maxsize = 100);
  void stop_streaming();

  FingerArray<std::optional<ForceSensorData>> get_latest_data() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  std::shared_ptr<linkerhand::Lifecycle> lifecycle_;
};

}  // namespace linkerhand::hand::l6
