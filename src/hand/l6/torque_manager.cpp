#include "linkerhand/hand/l6/torque_manager.hpp"

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

constexpr std::uint8_t kControlCmd = 0x02;
constexpr std::size_t kTorqueCount = 6;

struct TorqueWaiter {
  std::mutex mutex;
  std::condition_variable cv;
  bool ready = false;
  std::optional<TorqueData> data;
};

}  // namespace

struct TorqueManager::Impl {
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

  void set_torques(const std::array<int, 6>& torques) {
    detail::validate_fixed_u8_array(torques, /*max_inclusive=*/255, "Torque");
    CanMessage msg{};
    msg.arbitration_id = arbitration_id;
    msg.is_extended_id = false;
    msg.dlc = 1 + kTorqueCount;
    msg.data[0] = kControlCmd;
    const auto bytes = detail::to_u8_array(torques);
    for (std::size_t i = 0; i < kTorqueCount; ++i) {
      msg.data[1 + i] = bytes[i];
    }
    dispatcher.send(msg);
  }

  void set_torques(const std::vector<int>& torques) {
    detail::validate_fixed_u8_vector(
        torques, kTorqueCount, /*max_inclusive=*/255, "torques", "Torque");
    CanMessage msg{};
    msg.arbitration_id = arbitration_id;
    msg.is_extended_id = false;
    msg.dlc = 1 + kTorqueCount;
    msg.data[0] = kControlCmd;
    const auto bytes = detail::to_u8_array_6(torques);
    for (std::size_t i = 0; i < kTorqueCount; ++i) {
      msg.data[1 + i] = bytes[i];
    }
    dispatcher.send(msg);
  }

  TorqueData get_torques_blocking(double timeout_ms) {
    if (timeout_ms <= 0) {
      throw ValidationError("timeout_ms must be positive");
    }

    auto waiter = std::make_shared<TorqueWaiter>();
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

    throw TimeoutError("No torque data received within " + std::to_string(timeout_ms) + "ms");
  }

  std::optional<TorqueData> get_current_torques() const {
    std::lock_guard<std::mutex> lock(latest_mutex);
    return latest_data;
  }

  IterableQueue<TorqueData> stream(double interval_ms, std::size_t maxsize) {
    if (interval_ms <= 0) {
      throw ValidationError("interval_ms must be positive");
    }
    if (maxsize == 0) {
      throw ValidationError("maxsize must be positive");
    }
    if (!dispatcher.is_running()) {
      throw StateError("CAN dispatcher is stopped");
    }

    IterableQueue<TorqueData> queue(maxsize);
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

  void on_message(const CanMessage& msg) {
    if (msg.arbitration_id != arbitration_id) {
      return;
    }
    if (msg.dlc < 2 || msg.data[0] != kControlCmd) {
      return;
    }

    const std::size_t payload_len = msg.dlc - 1;
    if (payload_len != kTorqueCount) {
      return;
    }

    TorqueData data{};
    for (std::size_t i = 0; i < kTorqueCount; ++i) {
      data.torques[i] = static_cast<int>(msg.data[1 + i]);
    }
    data.timestamp = detail::now_unix_seconds();

    on_complete_data(data);
  }

  void on_complete_data(const TorqueData& data) {
    {
      std::lock_guard<std::mutex> lock(latest_mutex);
      latest_data = data;
    }

    std::vector<std::shared_ptr<TorqueWaiter>> waiters_copy;
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

    std::optional<IterableQueue<TorqueData>> q;
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
  std::optional<TorqueData> latest_data;

  mutable std::mutex waiters_mutex;
  std::vector<std::shared_ptr<TorqueWaiter>> waiters;

  mutable std::mutex streaming_mutex;
  std::optional<IterableQueue<TorqueData>> streaming_queue;
  std::atomic<bool> streaming_running{false};
  std::thread streaming_thread;
};

TorqueManager::TorqueManager(std::uint32_t arbitration_id, CANMessageDispatcher& dispatcher)
    : impl_(std::make_unique<Impl>(arbitration_id, dispatcher)) {}

TorqueManager::~TorqueManager() = default;

void TorqueManager::set_torques(const std::array<int, 6>& torques) { impl_->set_torques(torques); }
void TorqueManager::set_torques(const std::vector<int>& torques) { impl_->set_torques(torques); }

TorqueData TorqueManager::get_torques_blocking(double timeout_ms) {
  return impl_->get_torques_blocking(timeout_ms);
}

std::optional<TorqueData> TorqueManager::get_current_torques() const {
  return impl_->get_current_torques();
}

IterableQueue<TorqueData> TorqueManager::stream(double interval_ms, std::size_t maxsize) {
  return impl_->stream(interval_ms, maxsize);
}

void TorqueManager::stop_streaming() { impl_->stop_streaming(); }

}  // namespace linkerhand::hand::l6
