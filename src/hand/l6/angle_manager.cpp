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
    detail::validate_fixed_u8_array(angles, 255, "Angle");

    CanMessage msg{};
    msg.arbitration_id = arbitration_id;
    msg.dlc = static_cast<std::uint8_t>(1 + kAngleCount);
    msg.data[0] = kControlCmd;
    for (std::size_t i = 0; i < kAngleCount; ++i) {
      msg.data[1 + i] = static_cast<std::uint8_t>(angles[i]);
    }
    dispatcher.send(msg);
  }

  AngleData get_angles_blocking(std::chrono::milliseconds timeout) {
    if (timeout <= std::chrono::milliseconds::zero()) {
      throw ValidationError("timeout must be positive");
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
        remove_waiter(waiter);
        throw;
      }
    }

    std::unique_lock<std::mutex> waiter_lock(waiter->mutex);
    const bool ok = waiter->cv.wait_for(
        waiter_lock, timeout,
        [&] { return waiter->ready || lifecycle->state() != linkerhand::LifecycleState::Open; });
    if (ok && waiter->data.has_value()) {
      return *waiter->data;
    }

    remove_waiter(waiter);
    lifecycle->ensure_open();
    throw TimeoutError("No angle data received within timeout");
  }

  std::optional<AngleData> get_current_angles() const {
    std::lock_guard<std::mutex> lock(latest_mutex);
    return latest_data;
  }

  IterableQueue<AngleData> stream(std::chrono::milliseconds interval, std::size_t maxsize) {
    if (interval <= std::chrono::milliseconds::zero()) {
      throw ValidationError("interval must be positive");
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
      streaming_running.store(true, std::memory_order_release);
      try {
        streaming_thread = std::thread([this, interval, queue] {
          try {
            streaming_loop(interval);
          } catch (...) {
          }
          queue.close();
          streaming_running.store(false, std::memory_order_release);
        });
      } catch (...) {
        streaming_running.store(false, std::memory_order_release);
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
        streaming_running.store(false, std::memory_order_release);
        streaming_thread.join();
      }
      return;
    }

    streaming_running.store(false, std::memory_order_release);
    streaming_queue->close();
    streaming_queue.reset();
    if (streaming_thread.joinable()) {
      streaming_thread.join();
    }
  }

  void send_sense_request() {
    CanMessage msg{};
    msg.arbitration_id = arbitration_id;
    msg.dlc = 1;
    msg.data[0] = kControlCmd;
    dispatcher.send(msg);
  }

  void streaming_loop(std::chrono::milliseconds interval) {
    while (streaming_running.load(std::memory_order_acquire)) {
      send_sense_request();
      std::this_thread::sleep_for(interval);
    }
  }

  void remove_waiter(const std::shared_ptr<AngleWaiter>& waiter) {
    std::lock_guard<std::mutex> lock(waiters_mutex);
    waiters.erase(
        std::remove_if(
            waiters.begin(), waiters.end(),
            [&](const auto& w) { return w.get() == waiter.get(); }),
        waiters.end());
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
    if (static_cast<std::size_t>(msg.dlc - 1) != kAngleCount) {
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
      waiters_copy = std::move(waiters);
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
    if (q.has_value()) {
      q->force_put(data);
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

AngleData AngleManager::get_angles_blocking(std::chrono::milliseconds timeout) {
  lifecycle_->ensure_open();
  return impl_->get_angles_blocking(timeout);
}

std::optional<AngleData> AngleManager::get_current_angles() const {
  lifecycle_->ensure_open();
  return impl_->get_current_angles();
}

IterableQueue<AngleData> AngleManager::stream(std::chrono::milliseconds interval, std::size_t maxsize) {
  lifecycle_->ensure_open();
  return impl_->stream(interval, maxsize);
}

void AngleManager::stop_streaming() {
  impl_->stop_streaming();
}

}  // namespace linkerhand::hand::l6
