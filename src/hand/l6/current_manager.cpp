#include "linkerhand/hand/l6/current_manager.hpp"

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

constexpr std::uint8_t kSenseCmd = 0x36;
constexpr std::size_t kCurrentCount = 6;

struct CurrentWaiter {
  std::mutex mutex;
  std::condition_variable cv;
  bool ready = false;
  std::optional<CurrentData> data;
};

}  // namespace

struct CurrentManager::Impl {
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

  CurrentData get_currents_blocking(double timeout_ms) {
    if (timeout_ms <= 0) {
      throw ValidationError("timeout_ms must be positive");
    }

    auto waiter = std::make_shared<CurrentWaiter>();
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

    throw TimeoutError("No current data received within " + std::to_string(timeout_ms) + "ms");
  }

  std::optional<CurrentData> get_current_currents() const {
    std::lock_guard<std::mutex> lock(latest_mutex);
    return latest_data;
  }

  IterableQueue<CurrentData> stream(double interval_ms, std::size_t maxsize) {
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
      streaming_queue = IterableQueue<CurrentData>(maxsize);
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
    std::optional<IterableQueue<CurrentData>> queue_to_close;
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
    msg.data[0] = kSenseCmd;
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
    if (msg.dlc < 2 || msg.data[0] != kSenseCmd) {
      return;
    }

    const std::size_t payload_len = msg.dlc - 1;
    if (payload_len != kCurrentCount) {
      return;
    }

    CurrentData data{};
    for (std::size_t i = 0; i < kCurrentCount; ++i) {
      data.currents[i] = static_cast<int>(msg.data[1 + i]);
    }
    data.timestamp = detail::now_unix_seconds();

    on_complete_data(data);
  }

  void on_complete_data(const CurrentData& data) {
    {
      std::lock_guard<std::mutex> lock(latest_mutex);
      latest_data = data;
    }

    std::vector<std::shared_ptr<CurrentWaiter>> waiters_copy;
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

    std::optional<IterableQueue<CurrentData>> q;
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
  std::optional<CurrentData> latest_data;

  mutable std::mutex waiters_mutex;
  std::vector<std::shared_ptr<CurrentWaiter>> waiters;

  mutable std::mutex streaming_mutex;
  std::optional<IterableQueue<CurrentData>> streaming_queue;
  std::optional<double> streaming_interval_ms;
  std::atomic<bool> streaming_running{false};
  std::thread streaming_thread;
};

CurrentManager::CurrentManager(std::uint32_t arbitration_id, CANMessageDispatcher& dispatcher)
    : impl_(std::make_unique<Impl>(arbitration_id, dispatcher)) {}

CurrentManager::~CurrentManager() = default;

CurrentData CurrentManager::get_currents_blocking(double timeout_ms) {
  return impl_->get_currents_blocking(timeout_ms);
}

std::optional<CurrentData> CurrentManager::get_current_currents() const {
  return impl_->get_current_currents();
}

IterableQueue<CurrentData> CurrentManager::stream(double interval_ms, std::size_t maxsize) {
  return impl_->stream(interval_ms, maxsize);
}

void CurrentManager::stop_streaming() { impl_->stop_streaming(); }

}  // namespace linkerhand::hand::l6
