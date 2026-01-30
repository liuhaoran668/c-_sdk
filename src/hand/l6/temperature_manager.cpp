#include "linkerhand/hand/l6/temperature_manager.hpp"

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

constexpr std::uint8_t kSenseCmd = 0x33;
constexpr std::size_t kTemperatureCount = 6;

struct TemperatureWaiter {
  std::mutex mutex;
  std::condition_variable cv;
  bool ready = false;
  std::optional<TemperatureData> data;
};

}  // namespace

struct TemperatureManager::Impl {
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

  TemperatureData get_temperatures_blocking(double timeout_ms) {
    if (timeout_ms <= 0) {
      throw ValidationError("timeout_ms must be positive");
    }

    auto waiter = std::make_shared<TemperatureWaiter>();
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

    throw TimeoutError(
        "No temperature data received within " + std::to_string(timeout_ms) + "ms");
  }

  std::optional<TemperatureData> get_current_temperatures() const {
    std::lock_guard<std::mutex> lock(latest_mutex);
    return latest_data;
  }

  IterableQueue<TemperatureData> stream(double interval_ms, std::size_t maxsize) {
    if (interval_ms <= 0) {
      throw ValidationError("interval_ms must be positive");
    }
    if (maxsize == 0) {
      throw ValidationError("maxsize must be positive");
    }
    if (!dispatcher.is_running()) {
      throw StateError("CAN dispatcher is stopped");
    }

    IterableQueue<TemperatureData> queue(maxsize);
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
    msg.data[0] = kSenseCmd;
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

  void on_message(const CanMessage& msg) {
    if (msg.arbitration_id != arbitration_id) {
      return;
    }
    if (msg.dlc < 2 || msg.data[0] != kSenseCmd) {
      return;
    }

    const std::size_t payload_len = msg.dlc - 1;
    if (payload_len != kTemperatureCount) {
      return;
    }

    TemperatureData data{};
    for (std::size_t i = 0; i < kTemperatureCount; ++i) {
      data.temperatures[i] = static_cast<int>(msg.data[1 + i]);
    }
    data.timestamp = detail::now_unix_seconds();

    on_complete_data(data);
  }

  void on_complete_data(const TemperatureData& data) {
    {
      std::lock_guard<std::mutex> lock(latest_mutex);
      latest_data = data;
    }

    std::vector<std::shared_ptr<TemperatureWaiter>> waiters_copy;
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

    std::optional<IterableQueue<TemperatureData>> q;
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

  mutable std::mutex latest_mutex;
  std::optional<TemperatureData> latest_data;

  mutable std::mutex waiters_mutex;
  std::vector<std::shared_ptr<TemperatureWaiter>> waiters;

  mutable std::mutex streaming_mutex;
  std::optional<IterableQueue<TemperatureData>> streaming_queue;
  std::atomic<bool> streaming_running{false};
  std::thread streaming_thread;
};

TemperatureManager::TemperatureManager(std::uint32_t arbitration_id, CANMessageDispatcher& dispatcher)
    : impl_(std::make_unique<Impl>(arbitration_id, dispatcher)) {}

TemperatureManager::~TemperatureManager() = default;

TemperatureData TemperatureManager::get_temperatures_blocking(double timeout_ms) {
  return impl_->get_temperatures_blocking(timeout_ms);
}

std::optional<TemperatureData> TemperatureManager::get_current_temperatures() const {
  return impl_->get_current_temperatures();
}

IterableQueue<TemperatureData> TemperatureManager::stream(double interval_ms, std::size_t maxsize) {
  return impl_->stream(interval_ms, maxsize);
}

void TemperatureManager::stop_streaming() { impl_->stop_streaming(); }

}  // namespace linkerhand::hand::l6
