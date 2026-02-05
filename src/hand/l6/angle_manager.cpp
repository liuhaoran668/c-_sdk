#include "linkerhand/hand/l6/angle_manager.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

#include "linkerhand/exceptions.hpp"
#include "linkerhand/iterable_queue.hpp"
#include "../common.hpp"

namespace linkerhand::hand::l6 {
namespace {

constexpr std::uint8_t kControlCmd = 0x01;
constexpr std::size_t kAngleCount = 6;

struct AngleWaiter {
  std::mutex mutex;
  std::condition_variable cv;
  bool ready = false;
  std::optional<AngleData> data;
};

}  // namespace

struct AngleManager::Impl {
  Impl(
      std::uint32_t arbitration_id_,
      CANMessageDispatcher& dispatcher_,
      std::shared_ptr<linkerhand::Lifecycle> lifecycle_)
      : arbitration_id(arbitration_id_),
        dispatcher(dispatcher_),
        lifecycle(std::move(lifecycle_)) {
    if (!lifecycle) {
      throw ValidationError("lifecycle must not be null");
    }
    subscription_id = dispatcher.subscribe([this](const CanMessage& msg) { on_message(msg); });
    lifecycle_subscription_id = lifecycle->subscribe([this] { notify_waiters(); });
  }

  ~Impl() {
    try {
      stop_streaming();
    } catch (...) {
    }
    try {
      lifecycle->unsubscribe(lifecycle_subscription_id);
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

  AngleData get_angles_blocking(double timeout_ms) {
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
      try {
        send_sense_request();
      } catch (...) {
        std::lock_guard<std::mutex> lock(waiters_mutex);
        waiters.erase(
            std::remove_if(
                waiters.begin(),
                waiters.end(),
                [&](const auto& w) { return w.get() == waiter.get(); }),
            waiters.end());
        throw;
      }
    }

    const auto timeout =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double, std::milli>(timeout_ms));

    std::unique_lock<std::mutex> waiter_lock(waiter->mutex);
    const bool ok = waiter->cv.wait_for(
        waiter_lock, timeout, [&] { return waiter->ready || lifecycle->state() != linkerhand::LifecycleState::Open; });
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

    // If lifecycle is no longer open, throw StateError before TimeoutError
    lifecycle->ensure_open();
    throw TimeoutError("No angle data received within " + std::to_string(timeout_ms) + "ms");
  }

  std::optional<AngleData> get_current_angles() const {
    std::lock_guard<std::mutex> lock(latest_mutex);
    return latest_data;
  }

  IterableQueue<AngleData> stream(double interval_ms, std::size_t maxsize) {
    if (interval_ms <= 0) {
      throw ValidationError("interval_ms must be positive");
    }
    if (maxsize == 0) {
      throw ValidationError("maxsize must be positive");
    }
    if (!dispatcher.is_running()) {
      throw StateError("CAN dispatcher is stopped");
    }

    IterableQueue<AngleData> queue(maxsize);
    {
      std::lock_guard<std::mutex> lock(streaming_mutex);
      if (streaming_queue.has_value() || streaming_thread.joinable()) {
        throw StateError("Streaming is already active. Call stop_streaming() first.");
      }
      streaming_queue = queue;
      streaming_running.store(true);
      try {
        streaming_thread = std::thread([this, interval_ms, queue] {
          try {
            streaming_loop(interval_ms);
          } catch (...) {
          }
          queue.close();
          streaming_running.store(false);
        });
      } catch (...) {
        streaming_running.store(false);
        streaming_queue.reset();
        throw;
      }
    }

    return queue;
  }

  void stop_streaming() {
    std::lock_guard<std::mutex> lock(streaming_mutex);
    if (!streaming_queue.has_value()) {
      if (streaming_thread.joinable()) {
        streaming_running.store(false);
        streaming_thread.join();
      }
      return;
    }

    streaming_running.store(false);
    streaming_queue->close();
    streaming_queue.reset();
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

  void streaming_loop(double interval_ms) {
    const auto sleep_duration =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double, std::milli>(interval_ms));

    while (streaming_running.load()) {
      send_sense_request();
      std::this_thread::sleep_for(sleep_duration);
    }
  }

  void notify_waiters() {
    std::vector<std::shared_ptr<AngleWaiter>> waiters_copy;
    {
      std::lock_guard<std::mutex> lock(waiters_mutex);
      waiters_copy = waiters;
    }
    for (const auto& waiter : waiters_copy) {
      waiter->cv.notify_one();
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
      waiter->data = data;
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
    } catch (const StateError&) {
      return;
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
  std::shared_ptr<linkerhand::Lifecycle> lifecycle;
  std::size_t lifecycle_subscription_id = 0;

  mutable std::mutex latest_mutex;
  std::optional<AngleData> latest_data;

  mutable std::mutex waiters_mutex;
  std::vector<std::shared_ptr<AngleWaiter>> waiters;

  mutable std::mutex streaming_mutex;
  std::optional<IterableQueue<AngleData>> streaming_queue;
  std::atomic<bool> streaming_running{false};
  std::thread streaming_thread;
};

AngleManager::AngleManager(std::uint32_t arbitration_id, CANMessageDispatcher& dispatcher)
    : AngleManager(arbitration_id, dispatcher, std::make_shared<linkerhand::Lifecycle>("AngleManager")) {}

AngleManager::AngleManager(
    std::uint32_t arbitration_id,
    CANMessageDispatcher& dispatcher,
    std::shared_ptr<linkerhand::Lifecycle> lifecycle)
    : impl_(std::make_unique<Impl>(arbitration_id, dispatcher, lifecycle)),
      lifecycle_(std::move(lifecycle)) {}

AngleManager::~AngleManager() = default;

void AngleManager::set_angles(const std::array<int, 6>& angles) {
  lifecycle_->ensure_open();
  impl_->set_angles(angles);
}
void AngleManager::set_angles(const std::vector<int>& angles) {
  lifecycle_->ensure_open();
  impl_->set_angles(angles);
}

AngleData AngleManager::get_angles_blocking(double timeout_ms) {
  lifecycle_->ensure_open();
  return impl_->get_angles_blocking(timeout_ms);
}

std::optional<AngleData> AngleManager::get_current_angles() const {
  lifecycle_->ensure_open();
  return impl_->get_current_angles();
}

IterableQueue<AngleData> AngleManager::stream(double interval_ms, std::size_t maxsize) {
  lifecycle_->ensure_open();
  return impl_->stream(interval_ms, maxsize);
}

void AngleManager::stop_streaming() {
  // stop_streaming is a cleanup operation, allowed in any lifecycle state
  impl_->stop_streaming();
}

}  // namespace linkerhand::hand::l6
