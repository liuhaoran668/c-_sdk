#include "linkerhand/hand/o6/angle_manager.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

#include "linkerhand/exceptions.hpp"
#include "linkerhand/iterable_queue.hpp"

#include "../common.hpp"

namespace linkerhand::hand::o6 {
namespace {

constexpr std::uint8_t kControlCmd = 0x01;
constexpr std::size_t kAngleCount = 6;

struct AngleWaiter {
  std::mutex mutex;
  std::condition_variable cv;
  bool ready = false;
  std::optional<std::array<int, 6>> data;
};

}  // namespace

struct AngleManager::Impl {
  Impl(std::uint32_t arbitration_id_, CANMessageDispatcher& dispatcher_)
      : arbitration_id(arbitration_id_), dispatcher(dispatcher_) {
    subscription_id = dispatcher.subscribe([this](const CanMessage& msg) { on_message(msg); });
  }

  ~Impl() {
    try {
      stop_streaming();
    } catch (...) {
    }
    dispatcher.unsubscribe(subscription_id);
  }

  void set_angles(const std::array<int, 6>& angles) {
    detail::validate_fixed_u8_array(angles, /*max_inclusive=*/255, "Angle");
    CanMessage msg{};
    msg.arbitration_id = arbitration_id;
    msg.is_extended_id = false;
    msg.dlc = 1 + kAngleCount;
    msg.data[0] = kControlCmd;
    const auto bytes = detail::to_u8_array(angles);
    for (std::size_t i = 0; i < kAngleCount; ++i) {
      msg.data[1 + i] = bytes[i];
    }
    dispatcher.send(msg);
  }

  void set_angles(const std::vector<int>& angles) {
    detail::validate_fixed_u8_vector(
        angles, kAngleCount, /*max_inclusive=*/255, "angles", "Angle");
    CanMessage msg{};
    msg.arbitration_id = arbitration_id;
    msg.is_extended_id = false;
    msg.dlc = 1 + kAngleCount;
    msg.data[0] = kControlCmd;
    const auto bytes = detail::to_u8_array_6(angles);
    for (std::size_t i = 0; i < kAngleCount; ++i) {
      msg.data[1 + i] = bytes[i];
    }
    dispatcher.send(msg);
  }

  std::array<int, 6> get_angles_blocking(double timeout_ms) {
    if (timeout_ms <= 0) {
      throw ValidationError("timeout_ms must be positive");
    }

    auto waiter = std::make_shared<AngleWaiter>();
    {
      std::lock_guard<std::mutex> lock(waiters_mutex);
      waiters.push_back(waiter);
    }

    bool is_streaming = false;
    {
      std::lock_guard<std::mutex> lock(streaming_mutex);
      is_streaming = streaming_queue.has_value();
    }
    if (!is_streaming) {
      send_sense_request();
    }

    const auto timeout =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double, std::milli>(timeout_ms));

    std::unique_lock<std::mutex> waiter_lock(waiter->mutex);
    const bool ok = waiter->cv.wait_for(waiter_lock, timeout, [&] { return waiter->ready; });
    if (ok && waiter->data.has_value()) {
      return *waiter->data;
    }

    {
      std::lock_guard<std::mutex> lock(waiters_mutex);
      waiters.erase(
          std::remove_if(
              waiters.begin(),
              waiters.end(),
              [&](const auto& w) { return w.get() == waiter.get(); }),
          waiters.end());
    }

    throw TimeoutError("No angle data received within " + std::to_string(timeout_ms) + "ms");
  }

  std::optional<std::pair<std::array<int, 6>, double>> get_current_angles() const {
    std::lock_guard<std::mutex> lock(latest_mutex);
    if (!latest_data.has_value()) {
      return std::nullopt;
    }
    return std::make_pair(latest_data->angles, latest_data->timestamp);
  }

  IterableQueue<AngleData> stream(double interval_ms, std::size_t maxsize) {
    if (interval_ms <= 0) {
      throw ValidationError("interval_ms must be positive");
    }
    if (maxsize == 0) {
      throw ValidationError("maxsize must be positive");
    }

    {
      std::lock_guard<std::mutex> lock(streaming_mutex);
      if (streaming_queue.has_value()) {
        throw StateError("Streaming is already active. Call stop_streaming() first.");
      }
      streaming_queue = IterableQueue<AngleData>(maxsize);
      streaming_interval_ms = interval_ms;
      streaming_running.store(true);
    }

    streaming_thread = std::thread([this] {
      try {
        streaming_loop();
      } catch (...) {
      }
    });

    std::lock_guard<std::mutex> lock(streaming_mutex);
    return *streaming_queue;
  }

  void stop_streaming() {
    std::optional<IterableQueue<AngleData>> queue_to_close;
    {
      std::lock_guard<std::mutex> lock(streaming_mutex);
      if (!streaming_queue.has_value()) {
        return;
      }
      streaming_running.store(false);
      queue_to_close = streaming_queue;
      streaming_queue.reset();
      streaming_interval_ms.reset();
    }

    queue_to_close->close();
    if (streaming_thread.joinable()) {
      streaming_thread.join();
    }
  }

  void send_sense_request() {
    CanMessage msg{};
    msg.arbitration_id = arbitration_id;
    msg.is_extended_id = false;
    msg.dlc = 1;
    msg.data[0] = kControlCmd;
    dispatcher.send(msg);
  }

  void streaming_loop() {
    double interval_ms = 0.0;
    {
      std::lock_guard<std::mutex> lock(streaming_mutex);
      if (!streaming_interval_ms.has_value()) {
        throw StateError("Streaming is not active. Call stream() first.");
      }
      interval_ms = *streaming_interval_ms;
    }

    const auto sleep_duration =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double, std::milli>(interval_ms));

    while (streaming_running.load()) {
      send_sense_request();
      std::this_thread::sleep_for(sleep_duration);
    }
  }

  void on_message(const CanMessage& msg) {
    if (msg.arbitration_id != arbitration_id) {
      return;
    }
    if (msg.dlc < 2 || msg.data[0] != kControlCmd) {
      return;
    }

    const std::size_t payload_len = msg.dlc - 1;
    if (payload_len != kAngleCount) {
      return;
    }

    AngleData data{};
    for (std::size_t i = 0; i < kAngleCount; ++i) {
      data.angles[i] = static_cast<int>(msg.data[1 + i]);
    }
    data.timestamp = detail::now_unix_seconds();

    on_complete_data(data);
  }

  void on_complete_data(const AngleData& data) {
    {
      std::lock_guard<std::mutex> lock(latest_mutex);
      latest_data = data;
    }

    std::vector<std::shared_ptr<AngleWaiter>> waiters_copy;
    {
      std::lock_guard<std::mutex> lock(waiters_mutex);
      waiters_copy = waiters;
      waiters.clear();
    }
    for (const auto& waiter : waiters_copy) {
      std::lock_guard<std::mutex> lock(waiter->mutex);
      waiter->data = data.angles;
      waiter->ready = true;
      waiter->cv.notify_one();
    }

    std::optional<IterableQueue<AngleData>> q;
    {
      std::lock_guard<std::mutex> lock(streaming_mutex);
      q = streaming_queue;
    }
    if (!q.has_value()) {
      return;
    }

    try {
      q->put_nowait(data);
    } catch (const QueueFull&) {
      try {
        q->get_nowait();
        q->put_nowait(data);
      } catch (const QueueEmpty&) {
      }
    }
  }

  std::uint32_t arbitration_id;
  CANMessageDispatcher& dispatcher;
  std::size_t subscription_id = 0;

  mutable std::mutex latest_mutex;
  std::optional<AngleData> latest_data;

  mutable std::mutex waiters_mutex;
  std::vector<std::shared_ptr<AngleWaiter>> waiters;

  mutable std::mutex streaming_mutex;
  std::optional<IterableQueue<AngleData>> streaming_queue;
  std::optional<double> streaming_interval_ms;
  std::atomic<bool> streaming_running{false};
  std::thread streaming_thread;
};

AngleManager::AngleManager(std::uint32_t arbitration_id, CANMessageDispatcher& dispatcher)
    : impl_(std::make_unique<Impl>(arbitration_id, dispatcher)) {}

AngleManager::~AngleManager() = default;

void AngleManager::set_angles(const std::array<int, 6>& angles) { impl_->set_angles(angles); }
void AngleManager::set_angles(const std::vector<int>& angles) { impl_->set_angles(angles); }

std::array<int, 6> AngleManager::get_angles_blocking(double timeout_ms) {
  return impl_->get_angles_blocking(timeout_ms);
}

std::optional<std::pair<std::array<int, 6>, double>> AngleManager::get_current_angles() const {
  return impl_->get_current_angles();
}

IterableQueue<AngleData> AngleManager::stream(double interval_ms, std::size_t maxsize) {
  return impl_->stream(interval_ms, maxsize);
}

void AngleManager::stop_streaming() { impl_->stop_streaming(); }

}  // namespace linkerhand::hand::o6
